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

//============================================================================

#include <chrono>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <memory>
#include "encoder.hpp"
#include "textureParametrization.hpp"

// NB: these must come after the standard headers as they define prohibited
//     macros that conflict with the system implementation
#include <DirectXMath.h>
#include <DirectXMesh.h>

#include <UVAtlas.h>

namespace vmesh {

//============================================================================

HRESULT __cdecl UVAtlasCallback(float fPercentDone) {
  static auto prev = std::chrono::steady_clock::now();
  const auto  tick = std::chrono::steady_clock::now();

  if (tick - prev > std::chrono::seconds(1)) {
    std::cout << fPercentDone * 100. << "%   \r" << std::flush;
    prev = tick;
  }
  return S_OK;
}

//============================================================================

template<typename T>
bool
TextureParametrization::generate(const TriangleMesh<T>&      decimate,
                                 TriangleMesh<T>&            decimateTexture,
                                 const VMCEncoderParameters& params) {
  TriangleMesh<float> mesh;
  mesh.convert(decimate);

  // Remove unwanted mesh components
  mesh.displacements().clear();
  mesh.colours().clear();
  mesh.texCoords().clear();
  mesh.texCoordTriangles().clear();
  mesh.normals().clear();
  mesh.normalTriangles().clear();

  std::cout << mesh.pointCount() << " vertices, " << mesh.triangleCount()
            << " faces\n";

  if ((mesh.pointCount() == 0) || (mesh.triangleCount() == 0)) {
    std::cerr << "ERROR: Invalid mesh\n";
    return 1;
  }

  // Prepare mesh for processing
  const float           epsilon = 0.F;
  std::vector<uint32_t> adjacency(3 * mesh.triangleCount());
  auto                  hr = DirectX::GenerateAdjacencyAndPointReps(
    reinterpret_cast<uint32_t*>(mesh.triangles().data()),
    mesh.triangleCount(),
    reinterpret_cast<DirectX::XMFLOAT3*>(mesh.points().data()),
    mesh.pointCount(),
    epsilon,
    nullptr,
    adjacency.data());
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed generating adjacency (" << hr << ")\n";
    return 1;
  }

  // Validation
  std::wstring msgs;
  DirectX::Validate(reinterpret_cast<uint32_t*>(mesh.triangles().data()),
                    mesh.triangleCount(),
                    mesh.pointCount(),
                    adjacency.data(),
                    DirectX::VALIDATE_BACKFACING | DirectX::VALIDATE_BOWTIES,
                    &msgs);
  if (!msgs.empty()) {
    std::cerr << "WARNING: \n";
    std::wcerr << msgs;
  }

  // Clean
  std::vector<uint32_t> dups;
  bool                  breakBowties = true;
  hr = DirectX::Clean(reinterpret_cast<uint32_t*>(mesh.triangles().data()),
                      mesh.triangleCount(),
                      mesh.pointCount(),
                      adjacency.data(),
                      nullptr,
                      dups,
                      breakBowties);
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed mesh clean " << hr << '\n';
    return 1;
  }

  if (!dups.empty()) {
    std::cout << " [" << dups.size() << " vertex dups]\n";

    mesh.reservePoints(mesh.pointCount() + dups.size());
    for (auto dupIdx : dups) { mesh.addPoint(mesh.point(dupIdx)); }
  }

  // Perform UVAtlas isocharting
  std::cout << "Computing isochart atlas on mesh...\n";

  std::vector<DirectX::UVAtlasVertex> vb;
  std::vector<uint8_t>                ib;
  float                               outStretch = 0.F;
  size_t                              outCharts  = 0;
  std::vector<uint32_t>               facePartitioning;
  std::vector<uint32_t>               vertexRemapArray;
  const auto                          start = std::chrono::steady_clock::now();
  hr =
    UVAtlasCreate(reinterpret_cast<DirectX::XMFLOAT3*>(mesh.points().data()),
                  mesh.pointCount(),
                  reinterpret_cast<uint32_t*>(mesh.triangles().data()),
                  DXGI_FORMAT_R32_UINT,
                  mesh.triangleCount(),
                  params.maxCharts,
                  params.maxStretch,
                  params.width,
                  params.height,
                  params.gutter,
                  adjacency.data(),
                  nullptr,
                  nullptr,
                  UVAtlasCallback,
                  DirectX::UVATLAS_DEFAULT_CALLBACK_FREQUENCY,
                  params.uvOptions,
                  vb,
                  ib,
                  &facePartitioning,
                  &vertexRemapArray,
                  &outStretch,
                  &outCharts);
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed creating isocharts " << hr << '\n';
    return 1;
  }

  namespace chrono = std::chrono;
  auto end         = chrono::steady_clock::now();
  auto deltams     = chrono::duration_cast<chrono::milliseconds>(end - start);
  std::cout << "Processing time: " << deltams.count() << " ms\n";
  std::cout << "Output # of charts: " << outCharts << ", resulting stretching "
            << outStretch << ", " << vb.size() << " verts\n";

  assert(ib.size() == 3 * mesh.triangles().size() * sizeof(uint32_t));
  memcpy(mesh.triangles().data(), ib.data(), ib.size());

  assert(vertexRemapArray.size() == vb.size());
  std::vector<Vec3<float>> pos(vertexRemapArray.size());
  hr = DirectX::UVAtlasApplyRemap(
    reinterpret_cast<DirectX::XMFLOAT3*>(mesh.points().data()),
    sizeof(DirectX::XMFLOAT3),
    mesh.pointCount(),
    vertexRemapArray.size(),
    vertexRemapArray.data(),
    reinterpret_cast<DirectX::XMFLOAT3*>(pos.data()));
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed applying atlas vertex remap (" << hr << ")\n";
    return 1;
  }
  //  std::swap(mesh.points(), pos); // float to double conversion issues
  mesh.points().clear();
  mesh.points().resize(pos.size());
  for (size_t i = 0; i < pos.size(); i++) { mesh.point(i) = pos[i]; }

  msgs.clear();
  DirectX::Validate(reinterpret_cast<uint32_t*>(mesh.triangles().data()),
                    mesh.triangleCount(),
                    mesh.pointCount(),
                    adjacency.data(),
                    DirectX::VALIDATE_DEFAULT,
                    &msgs);
  if (!msgs.empty()) {
    std::cerr << "WARNING: \n";
    std::wcerr << msgs;
  }

  // Copy isochart UVs into mesh
  mesh.reserveTexCoords(vb.size());
  std::transform(vb.begin(),
                 vb.end(),
                 std::back_inserter(mesh.texCoords()),
                 [](DirectX::UVAtlasVertex& vtx) {
                   return Vec2<float>{vtx.uv.x, vtx.uv.y};
                 });

  mesh.texCoordTriangles() = mesh.triangles();

  decimateTexture.convert(mesh);

  return 0;
}

//============================================================================

template bool
TextureParametrization::generate<float>(const TriangleMesh<float>&,
                                        TriangleMesh<float>&,
                                        const VMCEncoderParameters&);

template bool
TextureParametrization::generate<double>(const TriangleMesh<double>&,
                                         TriangleMesh<double>&,
                                         const VMCEncoderParameters&);

//============================================================================

}  // namespace vmesh