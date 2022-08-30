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
#include "checksum.hpp"
#include "MD5.h"

namespace vmesh {

//============================================================================

std::vector<uint8_t>
Checksum::compute(vmesh::TriangleMesh<double>& mesh) {
  std::vector<uint8_t> digest;
  md5::MD5             md5;
  auto&                positions = mesh.points();
  auto&                triangles = mesh.triangles();
  auto&                coords    = mesh.texCoords();
  md5.update(reinterpret_cast<uint8_t*>(positions.data()),
             positions.size() * sizeof(vmesh::Vec3<double>));
  if (triangles.size() != 0)
    md5.update(reinterpret_cast<uint8_t*>(triangles.data()),
               triangles.size() * sizeof(vmesh::Vec3<int>));
  if (coords.size() != 0)
    md5.update(reinterpret_cast<uint8_t*>(coords.data()),
               coords.size() * sizeof(vmesh::Vec2<double>));
  //  if ( withColors_ ) {
  //    md5.update( reinterpret_cast<uint8_t*>( colors_.data() ), colors_.size() * sizeof( PCCColor3B ) );
  //  }
  //  if ( withReflectances_ ) {
  //    md5.update( reinterpret_cast<uint8_t*>( reflectances_.data() ), reflectances_.size() * sizeof( uint16_t ) );
  //  }
  digest.resize(MD5_DIGEST_STRING_LENGTH);
  md5.finalize(digest.data());
  return digest;
}

//============================================================================
std::vector<uint8_t>
Checksum::compute(vmesh::Frame<uint8_t>& texture) {
  std::vector<uint8_t> digest;
  md5::MD5             md5;
  auto texturesize0 = texture.plane(0).width() * texture.plane(0).height();
  auto texturesize1 = texture.plane(1).width() * texture.plane(1).height();
  auto texturesize2 = texture.plane(2).width() * texture.plane(2).height();
  md5.update(reinterpret_cast<uint8_t*>(texture.plane(0).data()),
             texturesize0 * sizeof(uint8_t));
  md5.update(reinterpret_cast<uint8_t*>(texture.plane(1).data()),
             texturesize1 * sizeof(uint8_t));
  md5.update(reinterpret_cast<uint8_t*>(texture.plane(2).data()),
             texturesize2 * sizeof(uint8_t));
  digest.resize(MD5_DIGEST_STRING_LENGTH);
  md5.finalize(digest.data());
  return digest;
}

//============================================================================

void
Checksum::add(vmesh::TriangleMesh<double>& mesh) {
  auto md5 = compute(mesh);
  mesh_.push_back(md5);
}

//============================================================================

void
Checksum::add(vmesh::Frame<uint8_t>& texture) {
  auto md5 = compute(texture);
  texture_.push_back(md5);
}

//============================================================================

void
Checksum::add(vmesh::TriangleMesh<double>& mesh,
              vmesh::Frame<uint8_t>&       texture) {
  add(mesh);
  add(texture);
}

//============================================================================

bool
Checksum::read(const std::string& path) {
  std::ifstream fin(path, std::ios::in);
  if (!fin.is_open()) return false;
  std::string token;
  size_t      numberOfFrames = 0;
  size_t      sizeChecksum   = 0;
  fin >> numberOfFrames;
  fin >> sizeChecksum;
  mesh_.resize(numberOfFrames);
  texture_.resize(numberOfFrames);
  for (size_t i = 0; i < mesh_.size(); i++) {
    for (size_t j = 0; j < 2; j++) {
      auto& checksum = j == 0 ? mesh_[i] : texture_[i];
      checksum.resize(sizeChecksum, 0);
      for (auto& c : checksum) {
        uint8_t c0;
        uint8_t c1;
        fin >> c0 >> c1;
        c = ((c0 + (c0 > '9' ? 9 : 0)) & 0x0F) * 16
            + ((c1 + (c1 > '9' ? 9 : 0)) & 0x0F);
      }
    }
  }
  fin.close();
  return true;
}

//============================================================================

bool
Checksum::write(const std::string& path) {
  std::ofstream fout(path, std::ios::out);
  if (!fout.is_open()) { return false; }
  fout << mesh_.size() << std::endl;
  fout << (mesh_.empty() ? 0 : mesh_[0].size()) << std::endl;
  for (size_t i = 0; i < mesh_.size(); i++) {
    for (auto& c : mesh_[i])
      fout << std::hex << (c / 16) << std::hex << (c % 16);
    fout << " ";
    for (auto& c : texture_[i])
      fout << std::hex << (c / 16) << std::hex << (c % 16);
    fout << std::endl;
  }
  fout.close();
  return true;
}

//============================================================================

void
Checksum::print(TriangleMesh<double>& mesh, std::string eString) {
  auto checksum = compute(mesh);
  printf("Checksum %s: ", eString.c_str());
  for (auto& c : checksum) { printf("%02x", c); }
  printf("\n");
}

//============================================================================

void
Checksum::print(vmesh::Frame<uint8_t>& texture, std::string eString) {
  auto checksum = compute(texture);
  printf("Checksum %s: ", eString.c_str());
  for (auto& c : checksum) { printf("%02x", c); }
  printf("\n");
}

//============================================================================

void
Checksum::print() {
  for (size_t i = 0; i < mesh_.size(); i++) {
    printf("Frame %4zu: [MD5GEO:", i);
    for (auto& c : mesh_[i]) { printf("%02x", c); }
    printf("][MD5TEX:");
    for (auto& c : texture_[i]) { printf("%02x", c); }
    printf("] \n");
  }
}

}  // namespace vmesh
