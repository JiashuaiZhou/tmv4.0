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

#include "mmModel.h"
#include "mmImage.h"

namespace vmesh {

//============================================================================

template<typename T>
std::vector<uint8_t>
Checksum::compute(const TriangleMesh<T>& mesh) {
  std::vector<uint8_t> digest;
  md5::MD5             md5;
  const auto&          points            = mesh.points();
  const auto&          colours           = mesh.colours();
  const auto&          texCoords         = mesh.texCoords();
  const auto&          normals           = mesh.normals();
  const auto&          triangles         = mesh.triangles();
  const auto&          texCoordTriangles = mesh.texCoordTriangles();
  const auto&          normalTriangles   = mesh.normalTriangles();
  md5.update(reinterpret_cast<const uint8_t*>(points.data()),
             points.size() * sizeof(Vec3<T>));
  if (!colours.empty())
    md5.update(reinterpret_cast<const uint8_t*>(colours.data()),
               colours.size() * sizeof(Vec3<T>));
  if (!texCoords.empty())
    md5.update(reinterpret_cast<const uint8_t*>(texCoords.data()),
               texCoords.size() * sizeof(Vec2<T>));
  if (!normals.empty())
    md5.update(reinterpret_cast<const uint8_t*>(normals.data()),
               normals.size() * sizeof(Vec3<T>));
  if (!triangles.empty())
    md5.update(reinterpret_cast<const uint8_t*>(triangles.data()),
               triangles.size() * sizeof(Vec3<int>));
  if (!texCoordTriangles.empty())
    md5.update(reinterpret_cast<const uint8_t*>(texCoordTriangles.data()),
               texCoordTriangles.size() * sizeof(Vec3<int>));
  if (!normalTriangles.empty())
    md5.update(reinterpret_cast<const uint8_t*>(normalTriangles.data()),
               normalTriangles.size() * sizeof(Vec3<int>));
  digest.resize(MD5_DIGEST_STRING_LENGTH);
  md5.finalize(digest.data());
  return digest;
}

//============================================================================

std::vector<uint8_t>
Checksum::compute(const Frame<uint8_t>& texture) {
  std::vector<uint8_t> digest;
  md5::MD5             md5;
  auto size0 = texture.plane(0).width() * texture.plane(0).height();
  auto size1 = texture.plane(1).width() * texture.plane(1).height();
  auto size2 = texture.plane(2).width() * texture.plane(2).height();
  md5.update(reinterpret_cast<const uint8_t*>(texture.plane(0).data()),
             size0 * sizeof(uint8_t));
  md5.update(reinterpret_cast<const uint8_t*>(texture.plane(1).data()),
             size1 * sizeof(uint8_t));
  md5.update(reinterpret_cast<const uint8_t*>(texture.plane(2).data()),
             size2 * sizeof(uint8_t));
  digest.resize(MD5_DIGEST_STRING_LENGTH);
  md5.finalize(digest.data());
  return digest;
}

//============================================================================

std::vector<uint8_t>
Checksum::compute(const mm::Model& mesh) {
  std::vector<uint8_t> digest;
  md5::MD5             md5;
  md5.update(reinterpret_cast<const uint8_t*>(mesh.vertices.data()),
             mesh.vertices.size() * sizeof(float));
  if (mesh.hasColors())
    md5.update(reinterpret_cast<const uint8_t*>(mesh.colors.data()),
               mesh.colors.size() * sizeof(float));
  if (mesh.hasUvCoords())
    md5.update(reinterpret_cast<const uint8_t*>(mesh.uvcoords.data()),
               mesh.uvcoords.size() * sizeof(float));
  if (mesh.hasNormals())
    md5.update(reinterpret_cast<const uint8_t*>(mesh.normals.data()),
               mesh.normals.size() * sizeof(float));
  if (mesh.hasTriangleNormals())
    md5.update(reinterpret_cast<const uint8_t*>(mesh.faceNormals.data()),
               mesh.faceNormals.size() * sizeof(float));
  if (mesh.hasTriangles())
    md5.update(reinterpret_cast<const uint8_t*>(mesh.triangles.data()),
               mesh.triangles.size() * sizeof(int));
  if (mesh.hasUvCoords())
    md5.update(reinterpret_cast<const uint8_t*>(mesh.trianglesuv.data()),
               mesh.trianglesuv.size() * sizeof(int));
  digest.resize(MD5_DIGEST_STRING_LENGTH);
  md5.finalize(digest.data());
  return digest;
}

//============================================================================

std::vector<uint8_t>
Checksum::compute(const mm::Image& texture) {
  std::vector<uint8_t> digest;
  md5::MD5             md5;
  auto                 size = texture.width * texture.height * texture.nbc;
  md5.update(reinterpret_cast<const uint8_t*>(texture.data),
             size * sizeof(uint8_t));
  digest.resize(MD5_DIGEST_STRING_LENGTH);
  md5.finalize(digest.data());
  return digest;
}

//============================================================================
template<typename T>
void
Checksum::add(const TriangleMesh<T>& mesh) {
  auto md5 = compute(mesh);
  mesh_.push_back(md5);
}

//============================================================================

void
Checksum::add(const Frame<uint8_t>& texture) {
  auto md5 = compute(texture);
  texture_.push_back(md5);
}

//============================================================================

void
Checksum::add(const Sequence& sequence) {
  for (size_t i = 0; i < sequence.frameCount(); i++) {
    add(sequence.mesh(i));
    add(sequence.texture(i));
  }
}

//============================================================================

std::string
toString(const std::vector<uint8_t>& checksum) {
  std::string str;
  for (auto& c : checksum) {
    char data[8];
    snprintf(data, sizeof(data), "%02x", c);
    str += data;
  }
  return str;
}

//============================================================================

template<typename T>
std::string
Checksum::getChecksum(const TriangleMesh<T>& mesh) {
  return toString(compute(mesh));
}

//============================================================================

std::string
Checksum::getChecksum(const Frame<uint8_t>& texture) {
  return toString(compute(texture));
}

//============================================================================

std::string
Checksum::getChecksum(const mm::Model& mesh) {
  return toString(compute(mesh));
}

//============================================================================

std::string
Checksum::getChecksum(const mm::Image& texture) {
  return toString(compute(texture));
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

template<typename T>
void
Checksum::print(const TriangleMesh<T>& mesh, std::string eString) {
  printf("Checksum %s: %s \n", eString.c_str(), getChecksum(mesh).c_str());
}

//============================================================================

void
Checksum::print(const Frame<uint8_t>& texture, std::string eString) {
  printf("Checksum %s: %s \n", eString.c_str(), getChecksum(texture).c_str());
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

//============================================================================

template void Checksum::print<float>(const TriangleMesh<float>&, std::string);
template void Checksum::print<double>(const TriangleMesh<double>&,
                                      std::string);

template std::string
Checksum::getChecksum<float>(const TriangleMesh<float>& mesh);
template std::string
Checksum::getChecksum<double>(const TriangleMesh<double>& mesh);

template std::vector<uint8_t>
Checksum::compute<float>(const TriangleMesh<float>&);
template std::vector<uint8_t>
Checksum::compute<double>(const TriangleMesh<double>&);

template void Checksum::add<float>(const TriangleMesh<float>&);
template void Checksum::add<double>(const TriangleMesh<double>&);

//============================================================================

}  // namespace vmesh
