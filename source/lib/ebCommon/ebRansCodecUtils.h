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

#ifndef _EB_RANSCODECUTILS_H_
#define _EB_RANSCODECUTILS_H_

#include <vector>
#include <algorithm>

namespace eb {
    namespace rans {

        inline void EncodeVarUint32(uint32_t val, std::vector<uint8_t>& buffer) {
            // byte msb set signals extension - 7 useful bits per byte
            const uint8_t out = val & 0x7f;
            if (val >= 0x80) {
                buffer.push_back(out | 0x80);
                EncodeVarUint32(val >> 7, buffer);
                return;
            }
            buffer.push_back(out);
        }

        inline void DecodeVarUint32(uint32_t& value, const std::vector<uint8_t>& buffer, uint32_t& offset) {
            uint8_t in = buffer[offset++];
            if (in & 0x80) { // decode next byte if available
                DecodeVarUint32(value, buffer, offset);
                // then append decoded info from this byte.
                value <<= 7; value |= in & 0x7f;
            }
            else { // last byte            
                value = in;
            }
        }

        // https://graphics.stanford.edu/~seander/bithacks.html
        inline uint32_t reverse32(uint32_t v) {
            v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);  // swap odd and even bits      
            v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);  // swap consecutive pairs
            v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);  // swap nibbles ... 
            v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);  // swap bytes
            return (v >> 16) | (v << 16);                           // swap 2-byte long pairs
        }

        inline uint32_t count32(uint32_t v) {
            v = v - ((v >> 1) & 0x55555555);                        // reuse input as temporary
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);         // temp
            return ((v + (v >> 4) & 0xF0F0F0F) * 0x1010101) >> 24;  // count
        }

        inline int MostSignificantBit(uint32_t n) {
#if defined(__GNUC__)
            return 31 ^ __builtin_clz(n);
#elif defined(_MSC_VER)
            unsigned long where;
            _BitScanReverse(&where, n);
            return (int)where;
#else
            int msb = 0;
            while (n >>= 1) msb++;
            return msb;
#endif
        }

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