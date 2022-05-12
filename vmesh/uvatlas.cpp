/* This code was originally based on UVAtlas.cpp,
 * Copyright (c) Microsoft Corporation.
 * Licensed under the MIT License.
 *
 * http://go.microsoft.com/fwlink/?LinkID=512686
 *
 * Subsequent modifications are made available under the following
 */
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

#include <chrono>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <memory>
#include <program-options-lite/program_options_lite.h>

#include "vmesh/util/mesh.hpp"
#include "vmesh/util/misc.hpp"

// NB: these must come after the standard headers as they define prohibited
//     macros that conflict with the system implementation
#include <DirectXMath.h>
#include <DirectXMesh.h>

#include <UVAtlas.h>

using namespace DirectX;

//============================================================================

namespace {
struct Parameters {
  // frame number for expansion in input/output filenames
  int fnum;

  std::string input;
  std::string output;
  size_t maxCharts;
  float maxStretch;
  float gutter;
  size_t width;
  size_t height;
  UVATLAS uvOptions;
};
}  // namespace

//============================================================================

namespace DirectX {
static std::istream&
operator>>(std::istream& in, UVATLAS& val)
{
  std::string str;
  in >> str;
  if (str == "DEFAULT")
    val = DirectX::UVATLAS_DEFAULT;
  else if (str == "FAST")
    val = DirectX::UVATLAS_GEODESIC_FAST;
  else if (str == "QUALITY")
    val = DirectX::UVATLAS_GEODESIC_QUALITY;
  else
    in.setstate(std::ios::failbit);
  return in;
}
}  // namespace DirectX

//----------------------------------------------------------------------------

namespace DirectX {
static std::ostream&
operator<<(std::ostream& out, UVATLAS val)
{
  switch (val) {
  case DirectX::UVATLAS_DEFAULT: out << "DEFAULT"; break;
  case DirectX::UVATLAS_GEODESIC_FAST: out << "FAST"; break;
  case DirectX::UVATLAS_GEODESIC_QUALITY: out << "QUALITY"; break;
  default: out << int(val) << " (unknown)";
  }
  return out;
}
}  // namespace DirectX

//============================================================================

static bool
parseParameters(int argc, char* argv[], Parameters& params)
try {
  namespace po = df::program_options_lite;

  bool print_help = false;

  /* clang-format off */
  po::Options opts;
  opts.addOptions()
  ("help", print_help, false, "This help text")
  ("config,c", po::parseConfigFile, "Configuration file name")

  (po::Section("Input/Output"))
  ("fnum",     params.fnum,   {}, "Frame number for %d expansion")
  ("input,i",  params.input,  {}, "Input mesh (OBJ format)")
  ("output,o", params.output, {}, "Output mesh (OBJ format)")

  (po::Section("Isochart"))
  ("quality,q",  params.uvOptions, UVATLAS_DEFAULT,
   "Quality level of DEFAULT, FAST or QUALITY")

  ("n",          params.maxCharts, size_t(),
   "Maximum number of charts to generate")

  ("stretch,st", params.maxStretch, 0.16667f,
   "Maximum amount of stretch 0 to 1")

  ("gutter,g",   params.gutter, 2.f,
   "Gutter width betwen charts in texels")

  ("width,w",    params.width, size_t(512),
   "texture width")

  ("height,h",   params.height, size_t(512),
   "texture height")
  ;
  /* clang-format on */

  po::setDefaults(opts);
  po::ErrorReporter err;
  const std::list<const char*>& argv_unhandled =
    po::scanArgv(opts, argc, (const char**)argv, err);

  for (const auto arg : argv_unhandled)
    err.warn() << "Unhandled argument ignored: " << arg << '\n';

  if (argc == 1 || print_help) {
    std::cout << "usage: " << argv[0] << " [arguments...] \n\n";
    po::doHelp(std::cout, opts, 78);
    return false;
  }

  if (params.input.empty())
    err.error() << "input mesh not specified\n";

  if (params.output.empty())
    err.error() << "output mesh not specified\n";

  if (err.is_errored)
    return false;

  // Dump the complete derived configuration
  std::cout << "+ Configuration parameters\n";
  po::dumpCfg(std::cout, opts, "Input/Output", 4);
  po::dumpCfg(std::cout, opts, "Isochart", 4);
  std::cout << '\n';

  return true;
}
catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

HRESULT __cdecl UVAtlasCallback(float fPercentDone)
{
  static auto prev = std::chrono::steady_clock::now();
  const auto tick = std::chrono::steady_clock::now();

  if (tick - prev > std::chrono::seconds(1)) {
    std::cout << fPercentDone * 100. << "%   \r" << std::flush;
    prev = tick;
  }

  return S_OK;
}

//============================================================================

int
main(int argc, char* argv[])
{
  Parameters params;
  if (!parseParameters(argc, argv, params)) {
    return 1;
  }

  vmesh::TriangleMesh<float> mesh;
  if (1) {
    const auto& name = vmesh::expandNum(params.input, params.fnum);
    if (!mesh.loadFromOBJ(name)) {
      std::cerr << "Error: can't load " << name << '\n';
      return 1;
    }
  }

  // Remove unwanted mesh components
  mesh.displacements().clear();
  mesh.colours().clear();
  mesh.texCoords().clear();
  mesh.texCoordTriangles().clear();
  mesh.normals().clear();
  mesh.normalTriangles().clear();

  std::cout << mesh.pointCount() << " vertices, " << mesh.triangleCount()
            << " faces\n";

  if (!mesh.pointCount() || !mesh.triangleCount()) {
    std::cerr << "ERROR: Invalid mesh\n";
    return 1;
  }

  // Prepare mesh for processing
  const float epsilon = 0.f;
  std::vector<uint32_t> adjacency(3 * mesh.triangleCount());
  auto hr = DirectX::GenerateAdjacencyAndPointReps(
    reinterpret_cast<uint32_t*>(mesh.triangles().data()), mesh.triangleCount(),
    reinterpret_cast<XMFLOAT3*>(mesh.points().data()), mesh.pointCount(),
    epsilon, nullptr, adjacency.data());
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed generating adjacency (" << hr << ")\n";
    return 1;
  }

  // Validation
  std::wstring msgs;
  DirectX::Validate(
    reinterpret_cast<uint32_t*>(mesh.triangles().data()), mesh.triangleCount(),
    mesh.pointCount(), adjacency.data(),
    VALIDATE_BACKFACING | VALIDATE_BOWTIES, &msgs);
  if (!msgs.empty()) {
    std::cerr << "WARNING: \n";
    std::wcerr << msgs;
  }

  // Clean
  std::vector<uint32_t> dups;
  bool breakBowties = true;
  hr = DirectX::Clean(
    reinterpret_cast<uint32_t*>(mesh.triangles().data()), mesh.triangleCount(),
    mesh.pointCount(), adjacency.data(), nullptr, dups, breakBowties);
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed mesh clean " << hr << '\n';
    return 1;
  }

  if (!dups.empty()) {
    std::cout << " [" << dups.size() << " vertex dups]\n";

    mesh.reservePoints(mesh.pointCount() + dups.size());
    for (auto dupIdx : dups)
      mesh.addPoint(mesh.point(dupIdx));
  }

  // Perform UVAtlas isocharting
  std::cout << "Computing isochart atlas on mesh...\n";

  std::vector<UVAtlasVertex> vb;
  std::vector<uint8_t> ib;
  float outStretch = 0.f;
  size_t outCharts = 0;
  std::vector<uint32_t> facePartitioning;
  std::vector<uint32_t> vertexRemapArray;
  const auto start = std::chrono::steady_clock::now();
  hr = UVAtlasCreate(
    reinterpret_cast<XMFLOAT3*>(mesh.points().data()), mesh.pointCount(),
    reinterpret_cast<uint32_t*>(mesh.triangles().data()), DXGI_FORMAT_R32_UINT,
    mesh.triangleCount(), params.maxCharts, params.maxStretch, params.width,
    params.height, params.gutter, adjacency.data(), nullptr, nullptr,
    UVAtlasCallback, UVATLAS_DEFAULT_CALLBACK_FREQUENCY, params.uvOptions, vb,
    ib, &facePartitioning, &vertexRemapArray, &outStretch, &outCharts);
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed creating isocharts " << hr << '\n';
    return 1;
  }

  namespace chrono = std::chrono;
  auto end = chrono::steady_clock::now();
  auto deltams = chrono::duration_cast<chrono::milliseconds>(end - start);
  std::cout << "Processing time: " << deltams.count() << " ms\n";
  std::cout << "Output # of charts: " << outCharts << ", resulting stretching "
            << outStretch << ", " << vb.size() << " verts\n";

  assert(ib.size() == 3 * mesh.triangles().size() * sizeof(uint32_t));
  memcpy(mesh.triangles().data(), ib.data(), ib.size());

  assert(vertexRemapArray.size() == vb.size());
  std::vector<vmesh::Vec3<float>> pos(vertexRemapArray.size());
  hr = DirectX::UVAtlasApplyRemap(
    reinterpret_cast<XMFLOAT3*>(mesh.points().data()), sizeof(XMFLOAT3),
    mesh.pointCount(), vertexRemapArray.size(), vertexRemapArray.data(),
    reinterpret_cast<XMFLOAT3*>(pos.data()));
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed applying atlas vertex remap (" << hr << ")\n";
    return 1;
  }
  std::swap(mesh.points(), pos);

  msgs.clear();
  DirectX::Validate(
    reinterpret_cast<uint32_t*>(mesh.triangles().data()), mesh.triangleCount(),
    mesh.pointCount(), adjacency.data(), VALIDATE_DEFAULT, &msgs);
  if (!msgs.empty()) {
    std::cerr << "WARNING: \n";
    std::wcerr << msgs;
  }

  // Copy isochart UVs into mesh
  mesh.reserveTexCoords(vb.size());
  std::transform(
    vb.begin(), vb.end(), std::back_inserter(mesh.texCoords()),
    [](UVAtlasVertex& vtx) {
      return vmesh::Vec2<float>{vtx.uv.x, vtx.uv.y};
    });

  mesh.texCoordTriangles() = mesh.triangles();

  if (1) {
    const auto& name = vmesh::expandNum(params.output, params.fnum);
    if (!mesh.saveToOBJ(name)) {
      std::cerr << "Error: can't save " << name << '\n';
      return 1;
    }
  }

  return 0;
}
