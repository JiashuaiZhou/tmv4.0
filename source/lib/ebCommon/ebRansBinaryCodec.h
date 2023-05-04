// The copyright in this software is being made available under the BSD
// Licence, included below.  This software may be subject to other third
// party and contributor rights, including patent rights, and no such
// rights are granted under this licence.
// 
// Copyright (c) 2022, InterDigital
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//  
// * Neither the name of the InterDigital nor the names of its contributors
//   may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//
// This software was originally published under the Apache 2.0 License 
// terms in 2016 by The Draco Authors. See license at the end of file.
//
// This original software has been modified.
// *****************************************************************

#ifndef _EB_RANSBINARYCODEC_H_
#define _EB_RANSBINARYCODEC_H_

#include <vector>
#include "ebRansCodecUtils.h"

namespace eb {
    namespace rans {

        constexpr uint32_t ANS_BINARY_PRECISION = 256;
        constexpr uint32_t ANS_L_BASE = 4096;

        class RansBinaryEncoder {
        public:
            RansBinaryEncoder() : _numBits(0), _numOnes(0) {}
            ~RansBinaryEncoder() {}

            // append bit values to the _packedBits buffer
            void append(bool value) {
                const auto shift = (_numBits++ & 0x1f);
                if (shift == 0)
                    _packedBits.push_back(value);
                else if (value)
                    _packedBits.back() |= (1 << shift);
                if (value) _numOnes++;
            }

            // encode _numBits values from _packedBits to encodedBuffer
            void encode(std::vector<uint8_t>& encodedBuffer) {

                // probability interval [0,1] mapped to [0, 256], then clamped to [1, 255] where 128 exactly corresponds to p=0.5
                const uint32_t p0r = static_cast<uint32_t>(((_numBits - _numOnes) / static_cast<double>(_numBits) * 256.0) + 0.5);
                uint8_t p0 = (p0r > 255) ? 255 : (p0r == 0) ? 1 : p0r;

                // preallocate to upper limit (to refine)
                std::vector<uint8_t> buffer((_packedBits.size() + 1) << 2);

                uint32_t offset = 0;
                uint32_t state = ANS_L_BASE;
                const uint8_t p1 = ANS_BINARY_PRECISION - p0;
                const uint32_t renorm = ((ANS_L_BASE / ANS_BINARY_PRECISION) << 8);

                // loop though bits to encode
                auto shift = ((_numBits - 1) & 0x1f);
                for (auto it = _packedBits.rbegin(); it != _packedBits.rend(); ++it, shift = 31) {
                    const uint32_t value = *it;
                    for (int i = shift; i >= 0; --i) {
                        //rabs_write(&ans_coder, value & (1 << i), p0);
                        const bool bit = value & (1 << i);
                        const unsigned p = bit ? p1 : p0;
                        unsigned quotient, remainder;
                        if (state >= renorm * p) { // renorm is a shift
                            buffer[offset++] = state & 0xff;
                            state >>= 8;
                        }
                        quotient = state / p; // TODO implement fast div - possible with one less operation see rans implem ?!
                        remainder = state % p;
                        state = quotient * ANS_BINARY_PRECISION + remainder + (bit ? 0 : p1);
                    }
                }
                // flush state
                state -= ANS_L_BASE;
                uint8_t bytes = 0;
                while (state >= (1 << 6))
                {
                    buffer[offset++] = state & 0xff;
                    state >>= 8;
                    bytes++;
                }
                buffer[offset++] = (bytes << 6) | state;

                // append info for decoding - p0 + size - to encode buffer
                encodedBuffer.insert(encodedBuffer.end(), 1, p0);
                EncodeVarUint32(offset, encodedBuffer);
                // then append coded bytes
                encodedBuffer.insert(encodedBuffer.end(), buffer.begin(), buffer.begin() + offset);
            }
            const uint32_t numBits() const { return _numBits; }

        private:
            uint32_t _numBits;
            uint32_t _numOnes;
            std::vector<uint32_t> _packedBits;
        };

        class RansBinaryDecoder {
        public:
            RansBinaryDecoder() {}
            ~RansBinaryDecoder() {}

            void init(std::vector<uint8_t>& encodedBuffer) {
                _encodedBuffer = &encodedBuffer;
                // get p0 and byte length from first bytes
                _baseOffset = 0;
                _p0 = encodedBuffer[_baseOffset++];
                _p1 = ANS_BINARY_PRECISION - _p0; // _p0 + _p1 = ANS_BINARY_PRECISION
                uint32_t sizeInBytes;
                DecodeVarUint32(sizeInBytes, encodedBuffer, _baseOffset);

                // reverse read from buffer end
                _readOffset = sizeInBytes - 1;

                // extract final state
                const uint32_t value = encodedBuffer[_baseOffset + _readOffset--];
                uint8_t prefix = value >> 6; // 2 msbs for byte length
                _state = value & 0x3F;
                while (prefix-- > 0) {
                    _state <<= 8; _state |= encodedBuffer[_baseOffset + _readOffset--];
                }
                _state += ANS_L_BASE;
            }
            bool decodeNextBit() {
                // state renormalization
                if (_state < ANS_L_BASE && _readOffset >= 0) {
                    _state = (_state << 8) | _encodedBuffer->at(_baseOffset + _readOffset--); // caution - no protection when wraps to 0xffffffff
                }

                const uint32_t quotient = _state / ANS_BINARY_PRECISION;
                const uint32_t remainder = _state % ANS_BINARY_PRECISION;
                const uint32_t xn = quotient * _p1;
                // decode from "lut"
                const bool val = remainder < _p1;
                // update state - use cum prob(0)=0 and quot*(_p0+_p1)=quot*ANS_BINARY_PRECISION, 
                _state = (val) ? xn + remainder : _state - xn - _p1;
                // return decoded bit
                return val;
            }

        private:
            uint32_t _readOffset;
            uint32_t _baseOffset;
            uint32_t _state;

            uint8_t _p0, _p1;

            std::vector<uint8_t>* _encodedBuffer = nullptr;
        };

    }  // namespace rans
} // namespace mm

#endif

// License of the original software
//
// Copyright 2016 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//