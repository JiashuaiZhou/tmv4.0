/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2022, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <fstream>
#include <vector>

namespace eb {

    //============================================================================

    enum class Endianness {
        BIG = 0,
        LITTLE = 1
    };

    //----------------------------------------------------------------------------

    inline Endianness
        systemEndianness() {
        uint32_t num = 1;
        return (*(reinterpret_cast<char*>(&num)) == 1) ? Endianness::LITTLE
            : Endianness::BIG;
    }

    //============================================================================

    struct Bitstream {
        Bitstream() : endianness(systemEndianness()) {}

        size_t                size() const { return buffer.size(); }
        void                  resize(const size_t sz) { return buffer.resize(sz); }
        std::vector<uint8_t>& vector() { return buffer; }

        template<typename T>
        void write(const T u) {
            union {
                T       u;
                uint8_t u8[sizeof(T)];
            } source{};
            source.u = u;
            if (systemEndianness() == Endianness::LITTLE) {
                for (size_t k = 0; k < sizeof(T); k++) {
                    buffer.push_back(source.u8[k]);
                }
            }
            else {
                for (size_t k = 0; k < sizeof(T); k++) {
                    buffer.push_back(source.u8[sizeof(T) - k - 1]);
                }
            }
        }

        template<typename UintTypeT>
        void writeVarUint(const UintTypeT u) {
            // byte msb set signals extension - 7 useful bits per byte
            const uint8_t out = u & 0x7f;
            if (u >= 0x80) {
                write(static_cast<uint8_t>(out | 0x80));
                writeVarUint(u >> 7);
                return;
            }
            write(out);
        }

        template<typename UintTypeT>
        void readVarUint(UintTypeT& u, size_t& counter) const {
            uint8_t in;
            read(in, counter);
            if (in & 0x80) { // decode next byte if available
                readVarUint(u, counter);
                // then append decoded info from this byte.
                u <<= 7; u |= in & 0x7f;
            }
            else { // last byte            
                u = in;
            }
        }

        // TODO: bette handling of float for bounding box storage
        template<typename T>
        void writeRaw(const T u) {
            const uint8_t* data = (const uint8_t*)&u;
            for (size_t k = 0; k < sizeof(T); k++) {
                buffer.push_back(data[k]);
            }
        }

        // TODO: bette handling of float for bounding box storage
        template<typename T>
        void readRaw(T& u, size_t& counter) const {
            uint8_t* data = (uint8_t*)&u;
            for (size_t k = 0; k < sizeof(T); k++) {
                data[k] = buffer[counter++];
            }
        }

        template<typename T>
        bool read(T& u, size_t& counter) const {
            union {
                T       u;
                uint8_t u8[sizeof(T)];
            } dest;

            if (counter + sizeof(T) > size()) { return false; }

            if (systemEndianness() == Endianness::LITTLE) {
                for (size_t k = 0; k < sizeof(T); k++) {
                    dest.u8[k] = buffer[counter++];
                }
            }
            else {
                for (size_t k = 0; k < sizeof(T); k++) {
                    dest.u8[sizeof(T) - k - 1] = buffer[counter++];
                }
            }
            u = dest.u;
            return true;
        }

        /*
        void append(const uint8_t* data, const uint32_t byteCount) {
          const auto offset = buffer.size();
          buffer.resize(offset + byteCount);
          std::copy(data, data + byteCount, buffer.begin() + offset);
        }

        void append(const std::vector<uint8_t>& buff) {
            buffer.insert(buffer.end(), buff.begin(), buff.end());
        }*/

        bool append(const std::string& fileName, bool writeByteCountHeader) {
            std::ifstream file(fileName, std::ios::binary);
            return append(file, writeByteCountHeader);
        }

        bool append(std::ifstream& file, bool writeByteCountHeader) {
            if (!file.is_open()) { return false; }
            file.ignore(std::numeric_limits<std::streamsize>::max());
            std::streamsize length = file.gcount();
            file.clear();
            file.seekg(0, std::ios_base::beg);
            assert(length <= std::numeric_limits<uint32_t>::max());
            const auto byteCount = uint32_t(length);
            if (writeByteCountHeader) { write(byteCount); }
            const auto offset = buffer.size();
            buffer.resize(offset + byteCount);
            file.read(reinterpret_cast<char*>(buffer.data() + offset), byteCount);
            return true;
        }

        bool save(std::ofstream& file) const {
            if (!file.is_open()) { return false; }
            file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
            return true;
        }

        bool save(const std::string& fileName) const {
            std::ofstream file(fileName, std::ios::binary);
            return save(file);
        }

        bool load(std::ifstream& file) {
            if (!file.is_open()) { return false; }
            buffer.resize(0);
            return append(file, false);
        }

        bool load(const std::string& fileName) {
            std::ifstream file(fileName, std::ios::binary);
            return load(file);
        }

        const Endianness     endianness;
        std::vector<uint8_t> buffer;
    };

    struct BistreamPosition {
        uint64_t bytes_;
        uint8_t  bits_;
    };

    // bit parser used for qt/qp aligned values
    // TODO - replace with unified parser
    class BitstreamRef {
    public:
        BitstreamRef(std::vector<uint8_t>& data):data_(data)
        {
            position_.bytes_ = 0;
            position_.bits_ = 0;
        }
        ~BitstreamRef() {};
        void beginning() {
            position_.bits_ = 0;
            position_.bytes_ = 0;
        }
        uint8_t* buffer() { return data_.data(); }

        uint64_t& size() { return position_.bytes_; }
        BistreamPosition            getPosition() { return position_; }
        void       setPosition(BistreamPosition& val) { position_ = val; }
        bool byteAligned() { return (position_.bits_ == 0); }

        inline uint32_t read(uint8_t bits, bool bFullStream = false) {
            uint32_t code = read(bits, position_);
            return code;
        }
        template<typename T>
        void write(T value, uint8_t bits, bool bFullStream = false) {
            write(static_cast<uint32_t>(value), bits, position_);
        }

    private:
        inline uint32_t read(uint8_t bits, BistreamPosition& pos) {
            uint32_t value = 0;
            for (size_t i = 0; i < bits; i++) {
                value |= ((data_[pos.bytes_] >> (7 - pos.bits_)) & 1) << (bits - 1 - i);
                if (pos.bits_ == 7) {
                    pos.bytes_++;
                    pos.bits_ = 0;
                }
                else {
                    pos.bits_++;
                }
            }
            return value;
        }

        inline void write(uint32_t value, uint8_t bits, BistreamPosition& pos) {
            //if (pos.bytes_ + bits + 64 >= data_.size()) { realloc(); }
            for (size_t i = 0; i < bits; i++) {
                if (pos.bytes_ >= data_.size()) { data_.resize(pos.bytes_ + 1); } // for simplicity not perfs
                data_[pos.bytes_] |= ((value >> (bits - 1 - i)) & 1) << (7 - pos.bits_);
                if (pos.bits_ == 7) {
                    pos.bytes_++;
                    pos.bits_ = 0;
                }
                else {
                    pos.bits_++;
                }
            }
        }

        std::vector<uint8_t>& data_;
        BistreamPosition     position_;

    };


    //============================================================================

}  // namespace vmesh
