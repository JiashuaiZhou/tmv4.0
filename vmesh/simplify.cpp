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
#include <cstdint>
#include <string>
#include <program-options-lite/program_options_lite.h>

#include "vmesh/util/mesh.hpp"
#include "vmesh/util/misc.hpp"
#include "vmesh/util/verbose.hpp"
#include "vmesh/version.hpp"
#include "vmesh/simplifymesh.hpp"

using namespace std;
using namespace vmesh;

//============================================================================

namespace {
struct Parameters {
  bool verbose;

  // frame number for expansion in input/output filenames
  int fnum;

  std::string inputMeshPath;
  std::string decimatedMeshPath;
  std::string mappedMeshPath;
  std::string referenceMeshPath;
  int32_t texCoordQuantizationBits;
  int32_t minCCTriangleCount;
  double targetTriangleRatio;
  double triangleFlipThreshold = 0.3;
  double trackedTriangleFlipThreshold = 0.1;
  double trackedPointNormalFlipThreshold = 0.5;
};
}  // namespace

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
  ("verbose,v", params.verbose, true, "Verbose output")

  (po::Section("Input/Output"))
  ("fnum",      params.fnum, {}, "Frame number for %d expansion")
  ("input",     params.inputMeshPath,     {}, "Input mesh")
  ("decimated", params.decimatedMeshPath, {}, "Decimated mesh")
  ("mapped",    params.mappedMeshPath,    {}, "Mapped mesh")
  ("reference", params.referenceMeshPath, {}, "Reference mesh")

  (po::Section("Simplification"))
  ("target", params.targetTriangleRatio, 0.125,
   "Target triangle count ratio")

  ("qt", params.texCoordQuantizationBits, 0,
   "texture coordinate quantization bits")

  ("cctcount", params.minCCTriangleCount, 0,
   "minimum triangle count per connected component")
  ;
  /* clang-format on */

  po::setDefaults(opts);
  po::ErrorReporter err;
  const list<const char*>& argv_unhandled =
    po::scanArgv(opts, argc, (const char**)argv, err);

  for (const auto arg : argv_unhandled)
    err.warn() << "Unhandled argument ignored: " << arg << '\n';

  if (argc == 1 || print_help) {
    std::cout << "usage: " << argv[0] << " [arguments...] \n\n";
    po::doHelp(std::cout, opts, 78);
    return false;
  }

  if (params.inputMeshPath.empty())
    err.error() << "input mesh not specified\n";

  if (params.decimatedMeshPath.empty())
    err.error() << "decimated output mesh not specified\n";

  if (params.mappedMeshPath.empty())
    err.error() << "mapped output mesh not specified\n";

  if (params.referenceMeshPath.empty())
    err.error() << "reference mesh not specified\n";

  if (err.is_errored)
    return false;

  // Dump the complete derived configuration
  cout << "+ Configuration parameters\n";
  po::dumpCfg(cout, opts, "Input/Output", 4);
  po::dumpCfg(cout, opts, "Simplification", 4);
  cout << '\n';

  return true;
}
catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

bool
RemoveSmallConnectedComponents(
  TriangleMesh<double>& mesh, int32_t minCCTriangleCount)
{
  vout << "Removing small connected components...\n";
  auto start = chrono::steady_clock::now();

  const auto pointCount = mesh.pointCount();
  const auto triangleCount = mesh.triangleCount();
  vector<int32_t> partition;
  vector<shared_ptr<TriangleMesh<double>>> connectedComponents;
  const auto ccCount0 = ExtractConnectedComponents(
    mesh.triangles(), mesh.pointCount(), mesh, partition,
    &connectedComponents);
  mesh.clear();
  int32_t ccCounter = 0;
  for (int32_t c = 0; c < ccCount0; ++c) {
    const auto& connectedComponent = connectedComponents[c];

    vout << "\t CC " << c << " -> " << connectedComponent->pointCount() << "V "
         << connectedComponent->triangleCount() << "T\n";

    if (connectedComponent->triangleCount() <= minCCTriangleCount) {
      continue;
    }
    ++ccCounter;
    mesh.append(*connectedComponent);
  }

  auto end = chrono::steady_clock::now();
  auto deltams = chrono::duration_cast<chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  vout << "Input mesh: " << ccCount0 << "CC " << pointCount << "V "
       << triangleCount << "T\n";
  vout << "Cleaned up mesh: " << ccCounter << "CC " << mesh.pointCount()
       << "V " << mesh.triangleCount() << 'T'
       << "\t Removed " << (ccCount0 - ccCounter) << "CC "
       << (pointCount - mesh.pointCount()) << "V "
       << (triangleCount - mesh.triangleCount()) << "T\n";
  return true;
}

//----------------------------------------------------------------------------

bool
RemoveDuplicatedTriangles(TriangleMesh<double>& mesh)
{
  vout << "Removing duplicated triangles... ";
  auto start = chrono::steady_clock::now();

  const auto triangleCount = mesh.triangleCount();
  vector<Vec3<double>> triangleNormals;
  mesh.computeTriangleNormals(triangleNormals);
  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(
    mesh.triangles(), mesh.pointCount(), vertexToTriangle);
  vector<Triangle> trianglesOutput;
  if (!RemoveDuplicatedTriangles(
        mesh.triangles(), triangleNormals, vertexToTriangle,
        trianglesOutput)) {
    return false;
  }
  swap(mesh.triangles(), trianglesOutput);

  auto end = chrono::steady_clock::now();
  auto deltams = chrono::duration_cast<chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  vout << "Cleaned up mesh: " << mesh.pointCount() << "V "
       << mesh.triangleCount() << "T\n";
  vout << "\t Removed " << (triangleCount - mesh.triangleCount())
       << " triangles\n";
  return true;
}

//----------------------------------------------------------------------------

bool
UnifyVertices(TriangleMesh<double>& mesh)
{
  vout << "Unifying vertices... ";
  const auto pointCount0 = mesh.pointCount();
  const auto triangleCount0 = mesh.triangleCount();
  auto start = chrono::steady_clock::now();

  vector<Vec3<double>> upoints;
  vector<Triangle> utriangles;
  vector<int32_t> mapping;
  UnifyVertices(mesh.points(), mesh.triangles(), upoints, utriangles, mapping);
  std::swap(mesh.points(), upoints);
  std::swap(mesh.triangles(), utriangles);
  RemoveDegeneratedTriangles(mesh);

  auto end = chrono::steady_clock::now();
  auto deltams = chrono::duration_cast<chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  vout << "Unified mesh: " << mesh.pointCount() << "V " << mesh.triangleCount()
       << "T\n";
  vout << "\t Removed " << (pointCount0 - mesh.pointCount()) << " vertices "
       << (triangleCount0 - mesh.triangleCount()) << " triangles\n";
  return true;
}

//----------------------------------------------------------------------------

bool
SimplifyMesh(
  const TriangleMesh<double>& mesh,
  TriangleMesh<double>& dmesh,
  TriangleMesh<double>& mmesh,
  const Parameters& params)
{
  vout << "Simplifying Mesh... \n";
  auto start = chrono::steady_clock::now();

  TriangleMeshDecimatorParameters dparams;
  dparams.triangleCount =
    std::ceil(mesh.triangleCount() * params.targetTriangleRatio);
  dparams.triangleFlipThreshold = params.triangleFlipThreshold;
  dparams.trackedTriangleFlipThreshold = params.trackedTriangleFlipThreshold;
  dparams.trackedPointNormalFlipThreshold =
    params.trackedPointNormalFlipThreshold;
  TriangleMeshDecimator decimator;
  if (
    decimator.decimate(
      (const double*)mesh.points().data(), mesh.pointCount(),
      (const int32_t*)mesh.triangles().data(), mesh.triangleCount(), dparams)
    != Error::OK) {
    cerr << "Error: can't decimate model\n";
    return false;
  }

  dmesh.resizePoints(decimator.decimatedPointCount());
  dmesh.resizeTriangles(decimator.decimatedTriangleCount());
  if (
    decimator.decimatedMesh(
      (double*)dmesh.points().data(), dmesh.pointCount(),
      (int32_t*)dmesh.triangles().data(), dmesh.triangleCount())
    != Error::OK) {
    cerr << "Error: can't extract decimated model\n";
    return false;
  }

  mmesh.triangles() = mesh.triangles();
  mmesh.resizePoints(mesh.pointCount());
  decimator.trackedPoints(
    (double*)(mmesh.points().data()), nullptr, mesh.pointCount());

  auto end = chrono::steady_clock::now();
  auto deltams = chrono::duration_cast<chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  vout << "Decimated mesh: " << decimator.decimatedPointCount() << "V "
       << decimator.decimatedTriangleCount() << "T\n";
  return true;
}

//============================================================================

bool
LoadMesh(TriangleMesh<double>& mesh, const Parameters& params)
{
  vout << "Loading Mesh... ";

  if (!mesh.loadFromOBJ(expandNum(params.inputMeshPath, params.fnum))) {
    return false;
  }

  vout << " [done]\n";
  vout << "Original mesh: " << mesh.pointCount() << "V "
       << mesh.triangleCount() << "T\n";
  vout << " \t bbox = " << mesh.boundingBox() << '\n';
  vout << " \t texCoord bbox = " << mesh.texCoordBoundingBox() << '\n';
  return true;
}

//============================================================================

int
main(int argc, char* argv[])
{
  cout << "MPEG VMESH version " << ::vmesh::version << '\n';

  Parameters params;
  if (!parseParameters(argc, argv, params)) {
    return 1;
  }

  if (params.verbose)
    vmesh::vout.rdbuf(std::cout.rdbuf());

  TriangleMesh<double> mesh;
  if (!LoadMesh(mesh, params)) {
    cerr << "Error: can't load " << params.inputMeshPath << "!\n";
    return 1;
  }
  if (!UnifyVertices(mesh)) {
    cerr << "Error: can't unify vertices!\n";
    return 1;
  }

  TriangleMesh<double> dmesh;
  TriangleMesh<double> mmesh;
  if (!SimplifyMesh(mesh, dmesh, mmesh, params)) {
    cerr << "Error: can't simplify mesh!\n";
    return 1;
  }

  if (!RemoveDuplicatedTriangles(dmesh)) {
    cerr << "Error: can't remove duplicated triangles!\n";
    return 1;
  }

  if (!RemoveSmallConnectedComponents(dmesh, params.minCCTriangleCount)) {
    cerr << "Error: can't remove small connected components!\n";
    return 1;
  }

  if (!dmesh.saveToOBJ(expandNum(params.decimatedMeshPath, params.fnum))) {
    cerr << "Error: can't save decimated mesh\n";
    return 1;
  }

  if (!mmesh.saveToOBJ(expandNum(params.mappedMeshPath, params.fnum))) {
    cerr << "Error: can't save mapped mesh\n";
    return 1;
  }
  const auto scale = params.texCoordQuantizationBits > 0
    ? 1.0 / ((1 << params.texCoordQuantizationBits) - 1)
    : 1.0;

  if (!mesh.saveToOBJ(
        expandNum(params.referenceMeshPath, params.fnum), scale)) {
    cerr << "Error: can't save reference mesh\n";
    return 1;
  }

  return 0;
}
