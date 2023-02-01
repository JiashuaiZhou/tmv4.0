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
#include <stdio.h>
#include "vmc.hpp"
#include "vmc.hpp"

namespace mm {
class Image;
class Model;
}  // namespace mm

namespace vmesh {

class Checksum {
public:
  Checksum()  = default;
  ~Checksum() = default;

  void add(const Sequence& sequence);

  void print();
  template<typename T>
  void print(const TriangleMesh<T>& mesh, std::string eString);
  void print(const Frame<uint8_t>& texture, std::string eString);

  bool read(const std::string& path);
  bool write(const std::string& path);

  template<typename T>
  std::string getChecksum(const TriangleMesh<T>& mesh);
  std::string getChecksum(const Frame<uint8_t>& texture);
  std::string getChecksum(const mm::Model& mesh);
  std::string getChecksum(const mm::Image& texture);

  bool operator!=(const Checksum& rhs) const { return !(*this == rhs); }

  bool operator==(const Checksum& rhs) const ;

private:
  template<typename T>
  void add(const TriangleMesh<T>& mesh);
  void add(const Frame<uint8_t>& texture);

  template<typename T>
  std::vector<uint8_t> compute(const TriangleMesh<T>& mesh);
  std::vector<uint8_t> compute(const Frame<uint8_t>& texture);
  std::vector<uint8_t> compute(const mm::Model& mesh);
  std::vector<uint8_t> compute(const mm::Image& texture);

  std::vector<std::vector<uint8_t>> mesh_;
  std::vector<std::vector<uint8_t>> texture_;
};

}  // namespace vmesh
