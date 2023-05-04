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

#ifndef _EB_RANSSYMBOLENCODER_H_
#define _EB_RANSSYMBOLENCODER_H_

#include <vector>
#include <algorithm>
#include "ebRansCodecUtils.h"

namespace eb {
    namespace rans {

        constexpr int ANS_MAX_SYMBOL_ENCODING_BITLENGTH = 18;
        constexpr int ANS_MIN_PRECISION_BITS = 8; // 12 orig , downto 6 for low poly ?? add an option for this

        struct rans_sym {
            uint32_t prob;
            uint32_t cum_prob;  // not-inclusive.
        };

        struct ProbabilityLess {
            explicit ProbabilityLess(const std::vector<rans_sym>* probs)
                : probabilities(probs) {}
            bool operator()(int i, int j) const {
                return probabilities->at(i).prob < probabilities->at(j).prob;
            }
            const std::vector<rans_sym>* probabilities;
        };

        class RansSymbolEncoder {
        public:
            RansSymbolEncoder() {}
            ~RansSymbolEncoder() {}

            void encode(const uint32_t* symbols, int num_values, std::vector<uint8_t>& outputBuffer, uint8_t resolution_offset = 2) {

                std::vector<uint8_t> encodedBuffer;

                // compute max value
                uint32_t max_value = 0;
                for (int i = 0; i < num_values; ++i) {
                    // Get the maximum value for a given entry across all attribute components.
                    if (symbols[i] > max_value)
                        max_value = symbols[i];
                }

                // Count the frequency of each entry value.
                std::vector<uint64_t> frequencies(max_value + 1, 0);
                for (int i = 0; i < num_values; ++i) {
                    ++frequencies[symbols[i]];
                }

                // Get the number of unique symbols
                uint32_t num_unique_symbols = 0;
                for (uint32_t i = 0; i < max_value + 1; ++i) {
                    if (frequencies[i] > 0)
                        ++num_unique_symbols;
                }

                // get the number of bits required to encode a mapping of unique symbols
                int symbol_bits = num_unique_symbols ? MostSignificantBit(num_unique_symbols) : 0;
                int unique_symbols_bit_length = symbol_bits + 1; // should not exceed ANS_MAX_SYMBOL_ENCODING_BITLENGTH      

                // adjust resolution according to compression level set and number of unique symbols - and clamp to valid range
                unique_symbols_bit_length += resolution_offset; // depends on compression level in dracon for cl = 10, add 2 - maybe not always optimal
                unique_symbols_bit_length = std::min(std::max(1, unique_symbols_bit_length), ANS_MAX_SYMBOL_ENCODING_BITLENGTH);

                // indicate base used to compute precision in bitstream
                outputBuffer.push_back(static_cast<uint8_t>(unique_symbols_bit_length));
                // define precision based on the maximum symbol bit length (twicked by resolution_offset)
                // when templated enables perfs improvement by maximizing constexpr usage
                auto precision = (3 * unique_symbols_bit_length) / 2;
                // is it optimal !!OM!! check6 for low poly ?, 12 is draco value 
                const uint8_t rans_precision_bits = precision < ANS_MIN_PRECISION_BITS ? ANS_MIN_PRECISION_BITS : (precision > 20 ? 20 : precision);
                _rans_precision = 1 << rans_precision_bits;
                _l_rans_base = _rans_precision * 4;
                _state = _l_rans_base;

                // create freqs tables to generate probability tables to be encoded in the buffer

                // compute the total of the input frequencies.
                uint32_t num_symbols = static_cast<uint32_t>(frequencies.size());
                uint64_t total_freq = 0;
                int max_valid_symbol = 0;
                for (uint32_t i = 0; i < num_symbols; ++i) {
                    total_freq += frequencies[i];
                    if (frequencies[i] > 0)
                        max_valid_symbol = i;
                }
                num_symbols = max_valid_symbol + 1; // number of symbols in the input alphabet
                _probability_table.resize(num_symbols);
                const double total_freq_d = static_cast<double>(total_freq);
                const double rans_precision_d = static_cast<double>(_rans_precision);
                // Compute probabilities by rescaling the normalized frequencies into interval [1, rans_precision - 1
                // The total probability needs to be equal to rans_precision.
                uint32_t total_rans_prob = 0;
                for (uint32_t i = 0; i < num_symbols; ++i) {
                    const uint64_t freq = frequencies[i];
                    const double prob = static_cast<double>(freq) / total_freq_d;               // normalized probability                
                    uint32_t rans_prob = static_cast<uint32_t>(prob * rans_precision_d + 0.5f); // rans probability in range of [1, rans_precision - 1].
                    if (rans_prob == 0 && freq > 0)
                        rans_prob = 1;
                    _probability_table[i].prob = rans_prob;
                    total_rans_prob += rans_prob;
                }
                // because of rounding errors, the total precision may not be exactly accurate, adjustment required to ensure total match _rans_precision
                if (total_rans_prob != _rans_precision) {
                    std::vector<int> sorted_probabilities(num_symbols);
                    for (uint32_t i = 0; i < num_symbols; ++i)
                        sorted_probabilities[i] = i;
                    std::stable_sort(sorted_probabilities.begin(), sorted_probabilities.end(), ProbabilityLess(&_probability_table));
                    if (total_rans_prob < _rans_precision) { // This happens rather infrequently, just add the extra needed precision to the most frequent symbol.                        
                        _probability_table[sorted_probabilities.back()].prob += _rans_precision - total_rans_prob;
                    }
                    else {
                        // We have over-allocated the precision, which is quite common. Rescale the probabilities of all symbols.
                        int32_t error = total_rans_prob - _rans_precision;
                        while (error > 0) {
                            const double act_total_prob_d = static_cast<double>(total_rans_prob);
                            const double act_rel_error_d = rans_precision_d / act_total_prob_d;
                            for (int j = num_symbols - 1; j > 0; --j) {
                                int symbol_id = sorted_probabilities[j];
                                if (_probability_table[symbol_id].prob <= 1)
                                    break;
                                const int32_t new_prob = static_cast<int32_t>(floor(act_rel_error_d * static_cast<double>(_probability_table[symbol_id].prob)));
                                int32_t fix = _probability_table[symbol_id].prob - new_prob;
                                if (fix == 0u)
                                    fix = 1;
                                if (fix >= static_cast<int32_t>(_probability_table[symbol_id].prob))
                                    fix = _probability_table[symbol_id].prob - 1;
                                if (fix > error)
                                    fix = error;
                                _probability_table[symbol_id].prob -= fix;
                                total_rans_prob -= fix;
                                error -= fix;
                                if (total_rans_prob == _rans_precision)
                                    break;
                            }
                        }
                    }
                }

                // compute the cumulative probabilities
                uint32_t total_prob = 0;
                for (uint32_t i = 0; i < num_symbols; ++i) {
                    _probability_table[i].cum_prob = total_prob;
                    total_prob += _probability_table[i].prob;
                }
                // here total_prob should match _rans_precision

                // estimate the number of bits needed to encode the input using shannon -sum(Ni*log2(Pi))
                double num_bits = 0;
                for (uint32_t i = 0; i < num_symbols; ++i) {
                    if (_probability_table[i].prob == 0)
                        continue;
                    const double norm_prob = static_cast<double>(_probability_table[i].prob) / rans_precision_d;
                    num_bits += static_cast<double>(frequencies[i]) * log2(norm_prob);
                }

                // resize the buffer before encoding
                const uint64_t num_expected_bits = static_cast<uint64_t>(ceil(-num_bits));
                const uint64_t required_bits = 2 * num_expected_bits + 32; // significant margin there
                const int64_t required_bytes = (required_bits + 7) / 8;
                encodedBuffer.reserve(encodedBuffer.size() + required_bytes + 8);

                // encode the probability table
                EncodeVarUint32(num_symbols, outputBuffer);
                // use varint encoding for the probabilities (first two bits represent the number of bytes used - 1 or indicate skip to next non zero prob).
                for (uint32_t i = 0; i < num_symbols; ++i) {
                    const uint32_t prob = _probability_table[i].prob; // here prob should never equal or exceed 1 << 22
                    int num_extra_bytes = (prob < (1 << 6)) ? 0 : ((prob < (1 << 14)) ? 1 : 2);
                    if (prob == 0) { // use '11' identifier when 0 prob symbol and 6 bits offset to next symbol (which may be 0 prob again)
                        uint32_t offset = 0;
                        for (; offset < (1 << 6) - 1; ++offset) {
                            if (_probability_table[i + offset + 1].prob > 0)
                                break; // break on non zero probability
                        }
                        outputBuffer.push_back(static_cast<uint8_t>((offset << 2) | 3));
                        i += offset;
                    }
                    else { // Encode the first byte (including the number of extra bytes).
                        outputBuffer.push_back(static_cast<uint8_t>((prob << 2) | (num_extra_bytes & 3)));
                        for (int b = 0; b < num_extra_bytes; ++b) { // Encode the extra bytes.
                            outputBuffer.push_back(static_cast<uint8_t>(prob >> (8 * (b + 1) - 2)));
                        }
                    }
                }

                // encode all values.
                const uint32_t renorm = ((_l_rans_base / _rans_precision) << 8);
                //const uint8_t renormShift = MostSignificantBit(renorm); // _l_rans_base / _rans_precision is fixed to 4 here - is a shift
                for (int i = num_values - 1; i >= 0; --i) {
                    const struct rans_sym* const sym = &_probability_table[symbols[i]];
                    const uint32_t p = sym->prob;
                    //while (_state >= (p << renormShift)) { // to test - not sure this is better
                    while (_state >= renorm * p) {
                        encodedBuffer.push_back(_state & 0xff);
                        _state >>= 8;
                    }
                    _state = (_state / p) * _rans_precision + _state % p + sym->cum_prob;
                }

                // finalize - flush state
                _state -= _l_rans_base;
                uint8_t bytes = 0;
                while (_state >= (1 << 6)) {
                    encodedBuffer.push_back(_state & 0xff);
                    _state >>= 8;
                    bytes++;
                }
                encodedBuffer.push_back((bytes << 6) | _state);

                // write total size in output buffer
                const uint32_t bytes_written = static_cast<uint32_t>(encodedBuffer.size());
                EncodeVarUint32(bytes_written, outputBuffer);

                // then append coded data 
                outputBuffer.reserve(outputBuffer.size() + encodedBuffer.size());
                outputBuffer.insert(outputBuffer.end(), encodedBuffer.begin(), encodedBuffer.end());
            }

        private:
            // no need for private data - encode function could be static
            uint32_t _rans_precision;
            uint32_t _l_rans_base;
            uint32_t _state;

            std::vector<rans_sym> _probability_table;
        };

        class RansSymbolDecoder {
        public:
            RansSymbolDecoder() {}
            ~RansSymbolDecoder() {}

            void init(std::vector<uint8_t>& encodedBuffer) {

                // store a ref to encoded buffer
                _encodedBuffer = &encodedBuffer;
                _baseOffset = 0;

                const uint8_t unique_symbols_bit_length = encodedBuffer[_baseOffset++];
                // define presicion
                const uint8_t precision = (3 * unique_symbols_bit_length) / 2;
                // is it optimal !!OM!! check6 for low poly ?,
                const uint8_t rans_precision_bits = precision < ANS_MIN_PRECISION_BITS ? ANS_MIN_PRECISION_BITS : (precision > 20 ? 20 : precision);
                _rans_precision = 1 << rans_precision_bits;
                _l_rans_base = _rans_precision * 4;

                // get num of symbols
                uint32_t num_symbols;
                DecodeVarUint32(num_symbols, encodedBuffer, _baseOffset);
                // decode temp prob table
                std::vector<uint32_t> probability_table;
                probability_table.resize(num_symbols);
                for (uint32_t i = 0; i < num_symbols; ++i) {
                    uint8_t prob_data = 0;
                    // the 2 LSBs define 0-2 extra bytes, or when set to 3 the offset to next symbol proba description (which may again be a zerp prob symbol)
                    prob_data = encodedBuffer[_baseOffset++];
                    const int token = prob_data & 3;
                    if (token == 3) { // zero probability for all symbols in the specified range.
                        const uint32_t offset = prob_data >> 2;
                        for (uint32_t j = 0; j < offset + 1; ++j) {
                            probability_table[i + j] = 0; // no check of (i + offset >= _num_symbols) limit to acces probability_table
                        }
                        i += offset;
                    }
                    else {
                        const int extra_bytes = token;
                        uint32_t prob = prob_data >> 2;
                        for (int b = 0; b < extra_bytes; ++b) {
                            // shift 8 bits for each extra byte and subtract 2 for the two first bits
                            prob |= static_cast<uint32_t>(encodedBuffer[_baseOffset++]) << (8 * (b + 1) - 2);
                        }
                        probability_table[i] = prob;
                    }
                }

                // build probability table and lut
                _lut_table.resize(_rans_precision);
                _probability_table.resize(num_symbols);
                uint32_t cum_prob = 0;
                uint32_t act_prob = 0;
                for (uint32_t i = 0; i < num_symbols; ++i) {
                    _probability_table[i].prob = probability_table[i];
                    _probability_table[i].cum_prob = cum_prob;
                    cum_prob += probability_table[i]; // here cum_prob shoud not exceed _rans_precision
                    for (uint32_t j = act_prob; j < cum_prob; ++j) {
                        _lut_table[j] = i;
                    }
                    act_prob = cum_prob;
                }
                // here cum_prob should match exactly rans_precision

                // get num of symbols
                uint32_t sizeInBytes; // should it be extended to 64b
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
                _state += _l_rans_base;
            }

            uint32_t decodeNextSymbol() {
                // state renormalization
                while (_state < _l_rans_base && _readOffset >= 0) {
                    _state = (_state << 8) | _encodedBuffer->at(_baseOffset + _readOffset--); // caution - no protection when wraps to 0xffffffff
                }
                // rans_precision is no longer a power of two compile time constant, and the below
                // division and modulo are not going to be optimized by the compiler. but should be powers of 2 so shifts based implem could be checked
                const uint32_t quotient = _state / _rans_precision;
                const uint32_t remainder = _state % _rans_precision;
                // decode symbol
                uint32_t symbol = _lut_table[remainder];
                uint32_t sym_prob = _probability_table[symbol].prob;
                uint32_t sym_cum_prob = _probability_table[symbol].cum_prob;
                //update state
                _state = quotient * sym_prob + remainder - sym_cum_prob;
                // return symbol value
                return symbol;
            }

        private:
            uint32_t _readOffset;
            uint32_t _baseOffset;
            uint32_t _state;

            std::vector<rans_sym> _probability_table;
            std::vector<uint32_t> _lut_table;

            uint32_t _rans_precision;
            uint32_t _l_rans_base;

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