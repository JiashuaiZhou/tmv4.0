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

namespace vmesh {

class Checksum {
public:
  Checksum()  = default;
  ~Checksum() = default;

  template<typename T>
  void add(const vmesh::TriangleMesh<T>& mesh,
           const vmesh::Frame<uint8_t>&  texture);

  void print();
  template<typename T>
  void print(const vmesh::TriangleMesh<T>& mesh, std::string eString);
  void print(const vmesh::Frame<uint8_t>& texture, std::string eString);

  bool read(const std::string& path);
  bool write(const std::string& path);

  template<typename T>
  std::string getChecksum(const vmesh::TriangleMesh<T>& mesh);
  std::string getChecksum(const vmesh::Frame<uint8_t>& texture);

  bool operator!=(const Checksum& rhs) const { return !(*this == rhs); }

  bool operator==(const Checksum& rhs) const {
    size_t num   = (std::min)(mesh_.size(), rhs.mesh_.size());
    bool   equal = true;
    equal &= mesh_.size() == rhs.mesh_.size();
    equal &= texture_.size() == rhs.texture_.size();
    for (size_t i = 0; equal && (i < num); i++) {
      equal &= mesh_[i] == rhs.mesh_[i];
      equal &= texture_[i] == rhs.texture_[i];
      printf("Frame %4zu: [MD5GEO:", i);
      for (auto& c : mesh_[i]) { printf("%02x", c); }
      printf(",");
      for (auto& c : rhs.mesh_[i]) { printf("%02x", c); }
      printf("][MD5TEX:");
      for (auto& c : texture_[i]) { printf("%02x", c); }
      printf(",");
      for (auto& c : rhs.texture_[i]) { printf("%02x", c); }
      printf("][%s,%s]\n",
             mesh_[i] == rhs.mesh_[i] ? "EQUAL" : "DIFF",
             texture_[i] == rhs.texture_[i] ? "EQUAL" : "DIFF");
    }
    return equal;
  }

private:
  template<typename T>
  void add(const vmesh::TriangleMesh<T>& mesh);
  void add(const vmesh::Frame<uint8_t>& texture);
  template<typename T>
  std::vector<uint8_t> compute(const vmesh::TriangleMesh<T>& mesh);
  std::vector<uint8_t> compute(const vmesh::Frame<uint8_t>& texture);

  std::vector<std::vector<uint8_t>> mesh_;
  std::vector<std::vector<uint8_t>> texture_;
};

}  // namespace vmesh
