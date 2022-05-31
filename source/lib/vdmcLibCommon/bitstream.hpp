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

namespace vmesh {

//============================================================================

enum class Endianness
{
  BIG = 0,
  LITTLE = 1
};

//----------------------------------------------------------------------------

inline Endianness
systemEndianness()
{
  uint32_t num = 1;
  return (*(reinterpret_cast<char*>(&num)) == 1) ? Endianness::LITTLE
                                                 : Endianness::BIG;
}

//============================================================================

struct Bitstream {
  Bitstream() : endianness(systemEndianness()) {}

  size_t size() const { return buffer.size(); }
  void resize(const size_t sz) { return buffer.resize(sz); }

  template<typename T>
  void write(const T u)
  {
    union {
      T u;
      uint8_t u8[sizeof(T)];
    } source;
    source.u = u;
    if (systemEndianness() == Endianness::LITTLE) {
      for (size_t k = 0; k < sizeof(T); k++) {
        buffer.push_back(source.u8[k]);
      }
    } else {
      for (size_t k = 0; k < sizeof(T); k++) {
        buffer.push_back(source.u8[sizeof(T) - k - 1]);
      }
    }
  }

  template<typename T>
  bool read(T& u, size_t& counter) const
  {
    union {
      T u;
      uint8_t u8[sizeof(T)];
    } dest;

    if (counter + sizeof(T) > size()) {
      return true;
    }

    if (systemEndianness() == Endianness::LITTLE) {
      for (size_t k = 0; k < sizeof(T); k++) {
        dest.u8[k] = buffer[counter++];
      }
    } else {
      for (size_t k = 0; k < sizeof(T); k++) {
        dest.u8[sizeof(T) - k - 1] = buffer[counter++];
      }
    }
    u = dest.u;
    return false;
  }

  void append(const uint8_t* data, const uint32_t byteCount)
  {
    write(byteCount);
    const auto offset = buffer.size();
    buffer.resize(offset + byteCount);
    std::copy(data, data + byteCount, buffer.begin() + offset);
  }

  bool append(const std::string& fileName, bool writeByteCountHeader)
  {
    std::ifstream file(fileName, std::ios::binary);
    return append(file, writeByteCountHeader);
  }

  bool append(std::ifstream& file, bool writeByteCountHeader)
  {
    if (!file.is_open()) {
      return true;
    }
    file.ignore(std::numeric_limits<std::streamsize>::max());
    std::streamsize length = file.gcount();
    file.clear();
    file.seekg(0, std::ios_base::beg);
    assert(length <= std::numeric_limits<uint32_t>::max());
    const auto byteCount = uint32_t(length);
    if (writeByteCountHeader) {
      write(byteCount);
    }
    const auto offset = buffer.size();
    buffer.resize(offset + byteCount);
    file.read(reinterpret_cast<char*>(buffer.data() + offset), byteCount);
    return false;
  }

  void append(const std::vector<uint8_t>& buff)
  {
    buffer.insert(buffer.end(), buff.begin(), buff.end());
  }

  bool save(std::ofstream& file) const
  {
    if (!file.is_open()) {
      return true;
    }
    file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    return false;
  }

  bool save(const std::string& fileName) const
  {
    std::ofstream file(fileName, std::ios::binary);
    return save(file);
  }

  bool load(std::ifstream& file)
  {
    if (!file.is_open()) {
      return true;
    }
    buffer.resize(0);
    append(file, false);
    return false;
  }

  bool load(const std::string& fileName)
  {
    std::ifstream file(fileName, std::ios::binary);
    return load(file);
  }

  const Endianness endianness;
  std::vector<uint8_t> buffer;
};

//============================================================================

}  // namespace vmesh
