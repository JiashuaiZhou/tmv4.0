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

#include <cassert>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <program-options-lite/program_options_lite.h>

#include "vmesh/util/image.hpp"
#include "vmesh/util/kdtree.hpp"
#include "vmesh/util/misc.hpp"
#include "vmesh/util/mesh.hpp"
#include "vmesh/util/vector.hpp"
#include "vmesh/util/verbose.hpp"
#include "vmesh/version.hpp"

using namespace std;
using namespace vmesh;

//============================================================================

enum class SmoothingMethod
{
  NONE = 0,
  VERTEX_CONSTRAINT = 1
};

//============================================================================

namespace {
struct Parameters {
  bool verbose;

  ///
  // Frame number.
  // Used in the expansion of input/output filenames.
  int fnum;

  ///
  // Reference frame number.
  // Used in the expansion of input filenames.
  int fnumRef;

  std::string targetMeshPath;
  std::string sourceMeshPath;
  std::string targetModifierMeshPath;
  std::string mappedMeshPath;
  std::string subdiv0MeshPath;
  std::string baseMeshPath;
  std::string subdivMeshPath;
  std::string nsubdivMeshPath;

  int geometrySamplingSubdivisionIterationCount;
  bool applyVertexUnification;

  int32_t geometryFittingIterationCount;
  double geometrySmoothingCoeffcient;
  double geometrySmoothingCoeffcientDecayRatio;
  double geometryMissedVerticesSmoothingCoeffcient;
  int32_t geometryMissedVerticesSmoothingIterationCount;

  SubdivisionMethod subdivisionMethod = SubdivisionMethod::MID_POINT;
  int32_t subdivisionIterationCount;
  bool fitSubdivisionSurface;

  double initialDeformNormalDeviationThreshold;
  int32_t initialDeformNNCount;
  bool initialDeformForceNormalDisplacement;

  bool applySmoothingDeform;
  SmoothingMethod smoothDeformSmoothingMethod;
  bool smoothDeformUpdateNormals;
  double smoothDeformTriangleNormalFlipThreshold;
  bool smoothingDeformUseInitialGeometry;
  bool smoothingDeformSmoothMotion;
};
}  // namespace

//============================================================================

static std::istream&
operator>>(std::istream& in, SmoothingMethod& val)
{
  unsigned int tmp;
  in >> tmp;
  val = SmoothingMethod(tmp);
  return in;
}

//----------------------------------------------------------------------------

static std::ostream&
operator<<(std::ostream& out, SmoothingMethod val)
{
  switch (val) {
  case SmoothingMethod::NONE: out << "0 (None)"; break;
  case SmoothingMethod::VERTEX_CONSTRAINT:
    out << "1 (Vertex constraint)";
    break;
  }
  return out;
}

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
  ("fnum",     params.fnum, {},    "Frame number for %d expansion")
  ("rnum",     params.fnumRef, -1, "Reference frame number for %d expansion")

  ("source,i", params.sourceMeshPath,  {}, "Source mesh")
  ("target",   params.targetMeshPath,  {}, "Target mesh")
  ("mtarget",  params.targetModifierMeshPath, {}, "Target modifier mesh")
  ("mapped",   params.mappedMeshPath,  {}, "Mapped mesh")
  ("subdiv0",  params.subdiv0MeshPath, {}, "Initial deform mesh")
  ("base",     params.baseMeshPath,    {}, "Base mesh")
  ("subdiv",   params.subdivMeshPath,  {}, "Subdivision mesh")
  ("nsubdiv",  params.nsubdivMeshPath, {}, "Normal subdivision mesh")

  (po::Section("Subdiv"))
  ("sdeform",
   params.applySmoothingDeform, true,
   "Apply deformation refinement stage")

  ("it",
   params.subdivisionIterationCount, 3,
   "Subdivision iteration count")

  ("forceNormalDisp",
   params.initialDeformForceNormalDisplacement, false,
   "Force displacements to aligned with the surface normals")

  ("unifyVertices",
   params.applyVertexUnification, true,
   "Unify duplicated vertices")

  ("deformNNCount",
   params.initialDeformNNCount, 1,
   "Number of nearest neighbours used during the initial deformation stage")

  ("deformNormalThres",
   params.initialDeformNormalDeviationThreshold, 0.1,
   "Maximum allowed normal deviation during the initial deformation stage")

  ("sampIt",
   params.geometrySamplingSubdivisionIterationCount, 3,
   "Number of subdivision iterations used for geometry sampling")

  ("fitIt",
   params.geometryFittingIterationCount, 16,
   "Number of iterations used during the deformation refinement stage")

  ("smoothCoeff",
   params.geometrySmoothingCoeffcient, 0.25,
   "Initial smoothing coefficient used to smooth the deformed mesh "
   "during deformation refinement")

  ("smoothDecay",
   params.geometrySmoothingCoeffcientDecayRatio, 0.75,
   "Decay factor applied to intial smoothing coefficient after every "
   "iteration of deformation refinement")

  ("smoothMissedCoeff",
   params.geometryMissedVerticesSmoothingCoeffcient, 0.1,
   "Smoothing coefficient applied to the missed vertices")

  ("smoothMissedIt",
   params.geometryMissedVerticesSmoothingIterationCount, 10,
   "Number of iterations when smoothing the positions of the missed vertices")

  ("smoothMethod",
   params.smoothDeformSmoothingMethod, SmoothingMethod::VERTEX_CONSTRAINT,
   "Smoothing method to be applied when smoothing the deformed mesh during"
   "the deformation refinement stage")

  ("deformUpdateNormals",
   params.smoothDeformUpdateNormals, true,
   "Recompute normals after each iteration of deformation refinement")

  ("deformFlipThres",
   params.smoothDeformTriangleNormalFlipThreshold, -0.5,
   "Threshold to detect triangle normals flip")

  ("useInitialGeom",
   params.smoothingDeformUseInitialGeometry, true,
   "Use the initial geometry during the the deformation refinement stage")

  ("fitSubdiv",
   params.fitSubdivisionSurface, true,
   "Update the positions of the decimated mesh to minimize displacements "
   "between the subdivided mesh and the deformed mesh")

  ("smoothMotion",
   params.smoothingDeformSmoothMotion, true,
   "Apply smoothing to motion instead of vertex positions")
  ;
  /* clang-format on */

  po::setDefaults(opts);
  po::ErrorReporter err;
  const list<const char*>& argv_unhandled =
    po::scanArgv(opts, argc, (const char**)argv, err);

  for (const auto arg : argv_unhandled)
    err.warn() << "Unhandled argument ignored: " << arg << "\n";

  if (argc == 1 || print_help) {
    std::cout << "usage: " << argv[0] << " [arguments...] \n\n";
    po::doHelp(std::cout, opts, 78);
    return false;
  }

  if (params.sourceMeshPath.empty())
    err.error() << "input mesh not specified\n";

  if (params.targetMeshPath.empty())
    err.error() << "target mesh not specified\n";

  if (params.subdivMeshPath.empty())
    err.error() << "subdivision mesh not specified\n";

  if (err.is_errored)
    return false;

  // If --rnum is not set, default to --fnum
  if (params.fnumRef < 0)
    params.fnumRef = params.fnum;

  // expand path names
  // if mtarget is specified, target is reference frame
  // otherwise,               target is current frame
  params.targetMeshPath = expandNum(params.targetMeshPath,
    params.targetModifierMeshPath.empty() ? params.fnum : params.fnumRef);

  params.targetModifierMeshPath =
    expandNum(params.targetModifierMeshPath, params.fnum);

  // reference frames use fnumRef
  params.sourceMeshPath = expandNum(params.sourceMeshPath, params.fnumRef);
  params.mappedMeshPath = expandNum(params.mappedMeshPath, params.fnumRef);
  params.subdiv0MeshPath = expandNum(params.subdiv0MeshPath, params.fnumRef);

  // output file names use fnum
  params.baseMeshPath = expandNum(params.baseMeshPath, params.fnum);
  params.nsubdivMeshPath = expandNum(params.nsubdivMeshPath, params.fnum);
  params.subdivMeshPath = expandNum(params.subdivMeshPath, params.fnum);

  // Dump the complete derived configuration
  cout << "+ Configuration parameters\n";
  po::dumpCfg(cout, opts, "Input/Output", 4);
  po::dumpCfg(cout, opts, "Subdiv", 4);
  cout << endl;

  return true;
}
catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

bool
CheckTriangleNormalInversion(
  const int32_t vindex,
  const TriangleMesh<double>& output,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  const vector<Vec3<double>>& initialTriangleNormals,
  const Parameters& params)
{
  const auto& tadj = vertexToTriangle.neighbours();
  const auto start = vertexToTriangle.neighboursStartIndex(vindex);
  const auto end = vertexToTriangle.neighboursEndIndex(vindex);
  for (int j = start; j < end; ++j) {
    const auto tindex = tadj[j];
    const auto& tri = output.triangle(tindex);
    const auto normal = computeTriangleNormal(
      output.point(tri[0]), output.point(tri[1]), output.point(tri[2]), true);
    if (
      normal * initialTriangleNormals[tindex]
      < params.smoothDeformTriangleNormalFlipThreshold) {
      return true;
    }
  }
  return false;
}

//----------------------------------------------------------------------------

void
FitMesh(
  const TriangleMesh<double>& target,
  const KdTree& kdtree,
  TriangleMesh<double>& output)
{
  const auto pointCount = output.pointCount();
  for (int32_t vindex = 0; vindex < pointCount; ++vindex) {
    int32_t index;
    double sqrDist;
    const auto point0 = output.point(vindex);
    const auto normal0 = output.normal(vindex);
    kdtree.query(point0.data(), 1, &index, &sqrDist);
    const auto delta = target.point(index) - point0;
    const auto d = delta * normal0;
    const auto point1 = point0 + d * normal0;
    output.setPoint(vindex, point1);
  }
}

//----------------------------------------------------------------------------

void
UpdateMissedVertices(
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  const vector<int8_t>& isBoundaryVertex,
  const vector<int32_t>& missedVertices,
  TriangleMesh<double>& output,
  vector<int32_t>& vadj,
  vector<int8_t>& vtags,
  vector<int8_t>& ttags,
  const Parameters& params)
{
  if (params.geometryMissedVerticesSmoothingIterationCount <= 0) {
    return;
  }
  const auto pointCount = output.pointCount();
  const auto triangleCount = output.triangleCount();
  const auto missedPointCount = int32_t(missedVertices.size());
  const auto alpha = params.geometryMissedVerticesSmoothingCoeffcient;
  std::vector<Vec3<double>> smoothedPositions(missedPointCount);
  vtags.resize(pointCount);
  ttags.resize(triangleCount);
  for (int32_t smoohtIt = 0;
       smoohtIt < params.geometryMissedVerticesSmoothingIterationCount;
       ++smoohtIt) {
    for (int i = 0; i < missedPointCount; ++i) {
      const auto vindex = missedVertices[i];
      ComputeAdjacentVertices(
        vindex, output.triangles(), vertexToTriangle, vtags, vadj);
      const auto neighbourCount = int32_t(vadj.size());
      if (!neighbourCount) {
        smoothedPositions[i] = output.point(vindex);
        continue;
      }
      const auto point0 = output.point(vindex);
      if (isBoundaryVertex[vindex]) {
        int32_t boundaryNeighbourCount = 0;
        Vec3<double> centroid(0.0);
        for (int j = 0; j < neighbourCount; ++j) {
          const auto vindex1 = vadj[j];
          if (isBoundaryVertex[vindex1]) {
            centroid += output.point(vindex1);
            ++boundaryNeighbourCount;
          }
        }
        if (boundaryNeighbourCount) {
          centroid /= boundaryNeighbourCount;
          smoothedPositions[i] = point0 + alpha * (centroid - point0);
        }
      } else {
        Vec3<double> centroid(0.0);
        for (int j = 0; j < neighbourCount; ++j) {
          centroid += output.point(vadj[j]);
        }
        centroid /= neighbourCount;
        smoothedPositions[i] = point0 + alpha * (centroid - point0);
      }
    }
    for (int i = 0; i < missedPointCount; ++i) {
      output.setPoint(missedVertices[i], smoothedPositions[i]);
    }
  }
}

//----------------------------------------------------------------------------

void
InitialDeform(
  const TriangleMesh<double>& target,
  const TriangleMesh<double>& mapped,
  const TriangleMesh<double>& motion,
  TriangleMesh<double>& output,
  const Parameters& params)
{
  StaticAdjacencyInformation<int32_t> vertexToTriangleMapped;
  ComputeVertexToTriangle(
    mapped.triangles(), mapped.pointCount(), vertexToTriangleMapped);
  KdTree kdtreeMapped(3, mapped.points(), 10);  // dim, cloud, max leaf
  const auto pointCount = output.pointCount();
  const auto* neighboursMapped = vertexToTriangleMapped.neighbours();
  const auto nnCount = params.initialDeformNNCount;
  std::vector<int32_t> indexes(nnCount);
  std::vector<double> sqrDists(nnCount);
  for (int32_t vindex = 0; vindex < pointCount; ++vindex) {
    const auto point0 = output.point(vindex);
    const auto normal0 = output.normal(vindex);
    kdtreeMapped.query(
      point0.data(), nnCount, indexes.data(), sqrDists.data());
    int32_t nindex = -1;
    for (int32_t n = 0; n < nnCount; ++n) {
      const auto index = indexes[n];
      const auto& normalTarget = target.normal(index);
      if (
        normal0 * normalTarget
        >= params.initialDeformNormalDeviationThreshold) {
        nindex = index;
        break;
      }
    }
    if (nindex == -1) {
      continue;
    }
    const auto start = vertexToTriangleMapped.neighboursStartIndex(nindex);
    const auto end = vertexToTriangleMapped.neighboursEndIndex(nindex);
    double minDist = std::numeric_limits<double>::max();
    auto minDistDisp = target.point(nindex) - mapped.point(nindex);
    auto minMotion = motion.point(nindex);
    auto minDistPoint = mapped.point(nindex);
    for (int32_t n = start; n < end; ++n) {
      const auto tindex = neighboursMapped[n];
      assert(tindex < target.triangleCount());
      const auto& tri = mapped.triangle(tindex);
      const auto& pt0 = mapped.point(tri[0]);
      const auto& pt1 = mapped.point(tri[1]);
      const auto& pt2 = mapped.point(tri[2]);
      Vec3<double> bcoord;
      const auto cpoint =
        ClosestPointInTriangle(point0, pt0, pt1, pt2, &bcoord);
      assert(bcoord[0] >= 0.0 && bcoord[1] >= 0.0 && bcoord[2] >= 0.0);
      assert(fabs(1.0 - bcoord[0] - bcoord[1] - bcoord[2]) < 0.000001);
      const auto dist = (point0 - cpoint).norm2();
      // compute closest point
      if (dist < minDist) {
        minDist = dist;
        minDistPoint = cpoint;
        const auto disp0 = target.point(tri[0]) - pt0;
        const auto disp1 = target.point(tri[1]) - pt1;
        const auto disp2 = target.point(tri[2]) - pt2;
        minDistDisp =
          bcoord[0] * disp0 + bcoord[1] * disp1 + bcoord[2] * disp2;
        const auto m0 = motion.point(tri[0]);
        const auto m1 = motion.point(tri[1]);
        const auto m2 = motion.point(tri[2]);
        minMotion = bcoord[0] * m0 + bcoord[1] * m1 + bcoord[2] * m2;
      }
    }
    if (params.initialDeformForceNormalDisplacement) {
      const auto delta = minDistPoint + minDistDisp - point0;
      const auto d = delta * normal0;
      output.setPoint(vindex, minMotion + point0 + d * normal0);
    } else {
      output.setPoint(vindex, minMotion + minDistPoint + minDistDisp);
    }
  }
}

//----------------------------------------------------------------------------

void
InitialDeform(
  const TriangleMesh<double>& target,
  TriangleMesh<double>& output,
  const Parameters& params)
{
  StaticAdjacencyInformation<int32_t> vertexToTriangleTarget;
  ComputeVertexToTriangle(
    target.triangles(), target.pointCount(), vertexToTriangleTarget);
  KdTree kdtreeTarget(3, target.points(), 10);  // dim, cloud, max leaf

  const auto* neighboursTarget = vertexToTriangleTarget.neighbours();
  vector<int32_t> missedVertices;
  const auto pointCount = output.pointCount();
  const auto nnCount = params.initialDeformNNCount;
  std::vector<int32_t> indexes(nnCount);
  std::vector<double> sqrDists(nnCount);
  for (int32_t vindex = 0; vindex < pointCount; ++vindex) {
    const auto point0 = output.point(vindex);
    const auto normal0 = output.normal(vindex);
    kdtreeTarget.query(
      point0.data(), nnCount, indexes.data(), sqrDists.data());
    int32_t nindex = -1;
    for (int32_t n = 0; n < nnCount; ++n) {
      const auto index = indexes[n];
      const auto& normalTarget = target.normal(index);
      if (
        normal0 * normalTarget
        >= params.initialDeformNormalDeviationThreshold) {
        nindex = index;
        break;
      }
    }
    if (nindex == -1) {
      missedVertices.push_back(vindex);
      continue;
    }
    const auto start = vertexToTriangleTarget.neighboursStartIndex(nindex);
    const auto end = vertexToTriangleTarget.neighboursEndIndex(nindex);
    double minDist = std::numeric_limits<double>::max();
    auto minDistDisp = target.point(nindex) - target.point(nindex);
    auto minDistPoint = target.point(nindex);
    for (int32_t n = start; n < end; ++n) {
      const auto tindex = neighboursTarget[n];
      assert(tindex < target.triangleCount());
      const auto& tri = target.triangle(tindex);
      const auto& pt0 = target.point(tri[0]);
      const auto& pt1 = target.point(tri[1]);
      const auto& pt2 = target.point(tri[2]);
      Vec3<double> bcoord;
      const auto cpoint =
        ClosestPointInTriangle(point0, pt0, pt1, pt2, &bcoord);
      assert(bcoord[0] >= 0.0 && bcoord[1] >= 0.0 && bcoord[2] >= 0.0);
      assert(fabs(1.0 - bcoord[0] - bcoord[1] - bcoord[2]) < 0.000001);
      const auto dist = (point0 - cpoint).norm2();
      // compute closest point
      if (dist < minDist) {
        minDist = dist;
        minDistPoint = cpoint;
        const auto disp0 = target.point(tri[0]) - pt0;
        const auto disp1 = target.point(tri[1]) - pt1;
        const auto disp2 = target.point(tri[2]) - pt2;
        minDistDisp =
          bcoord[0] * disp0 + bcoord[1] * disp1 + bcoord[2] * disp2;
      }
    }
    if (params.initialDeformForceNormalDisplacement) {
      const auto delta = minDistPoint + minDistDisp - point0;
      const auto d = delta * normal0;
      output.setPoint(vindex, point0 + d * normal0);
    } else {
      output.setPoint(vindex, minDistPoint + minDistDisp);
    }
  }
}

//----------------------------------------------------------------------------

void
Deform(
  const TriangleMesh<double>& target,
  const vector<Vec3<double>>& initialPositions,
  const vector<Vec3<double>>& initialTriangleNormals,
  TriangleMesh<double>& output,
  const Parameters& params)
{
  const auto pointCount = output.pointCount();
  const auto triangleCount = output.triangleCount();
  StaticAdjacencyInformation<int32_t> vertexToTriangleOutput;
  ComputeVertexToTriangle(
    output.triangles(), output.pointCount(), vertexToTriangleOutput);
  std::vector<int8_t> vtags(pointCount);
  std::vector<int8_t> ttags(triangleCount);
  std::vector<int8_t> isBoundaryVertex;
  ComputeBoundaryVertices(
    output.triangles(), vertexToTriangleOutput, vtags, ttags,
    isBoundaryVertex);
  KdTree kdtreeTarget(3, target.points(), 10);  // dim, cloud, max leaf

  vector<int32_t> missedVertices;
  std::vector<int32_t> vadj;
  double smoothCoeff = params.geometrySmoothingCoeffcient;
  for (int32_t it = 0; it < params.geometryFittingIterationCount; ++it) {
    if (params.smoothDeformUpdateNormals) {
      vout << "\t Iteration " << it;
      vout << "\t Compute Normals...";
      output.computeNormals();
      vout << "\t Fitting...";
    }

    FitMesh(target, kdtreeTarget, output);

    vout << "\t Updating missed vertices...";

    missedVertices.resize(0);
    for (int32_t vindex = 0; vindex < pointCount; ++vindex) {
      if (CheckTriangleNormalInversion(
            vindex, output, vertexToTriangleOutput, initialTriangleNormals,
            params)) {
        missedVertices.push_back(vindex);
      }
    }

    for (int i = 0; i < pointCount && params.smoothingDeformSmoothMotion;
         ++i) {
      output.setPoint(i, output.point(i) - initialPositions[i]);
    }
    UpdateMissedVertices(
      vertexToTriangleOutput, isBoundaryVertex, missedVertices, output, vadj,
      vtags, ttags, params);

    vout << "[" << missedVertices.size() << "]";
    vout << "\t Smoothing[" << smoothCoeff << "]...";

    if (
      params.smoothDeformSmoothingMethod
      == SmoothingMethod::VERTEX_CONSTRAINT) {
      SmoothWithVertexConstraints(
        output, vertexToTriangleOutput, isBoundaryVertex, vtags, ttags,
        smoothCoeff);
    }
    for (int i = 0; i < pointCount && params.smoothingDeformSmoothMotion;
         ++i) {
      output.setPoint(i, output.point(i) + initialPositions[i]);
    }

    vout << " done\n";

    smoothCoeff *= params.geometrySmoothingCoeffcientDecayRatio;
  }
}

//----------------------------------------------------------------------------

bool
Subdivide(TriangleMesh<double>& mesh, const Parameters& params)
{
  vout << "Subdividing Mesh...\n";
  auto start = chrono::steady_clock::now();

  mesh.computeNormals();
  if (params.subdivisionIterationCount) {
    vector<SubdivisionLevelInfo> infoLevelOfDetails;
    vector<int64_t> subdivEdges;
    mesh.subdivideMidPoint(
      params.subdivisionIterationCount, &infoLevelOfDetails, &subdivEdges);
    mesh.resizeNormals(mesh.pointCount());
    interpolateSubdivision(
      mesh.normals(), infoLevelOfDetails, subdivEdges, 0.5, 0.5, true);
  }

  auto end = chrono::steady_clock::now();
  auto deltams = chrono::duration_cast<chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  vout << "\t Subdivided mesh: " << mesh.pointCount() << "V "
       << mesh.triangleCount() << "T\n";
  return true;
}

//----------------------------------------------------------------------------

bool
FitMesh(
  const TriangleMesh<double>& target,
  TriangleMesh<double>& mapped,
  const TriangleMesh<double>& motion,
  TriangleMesh<double>& deformed,
  const Parameters& params)
{
  vout << "Deforming Mesh...\n";
  auto start = chrono::steady_clock::now();

  vector<Vec3<double>> initialPositions;
  vector<Vec3<double>> initialTriangleNormals;
  if (params.smoothingDeformUseInitialGeometry) {
    initialPositions = deformed.points();
    deformed.computeTriangleNormals(initialTriangleNormals);
  }
  if (mapped.pointCount() != target.pointCount()) {
    vout << "\t Computing mapping...\n";
    mapped = target;
    InitialDeform(deformed, mapped, params);
  }

  vout << "\t Computing initial deformation...\n";
  auto subdivTarget = target;
  auto subdivMotion = motion;
  auto subdivMapped = mapped;
  subdivTarget.subdivideMidPoint(
    params.geometrySamplingSubdivisionIterationCount);
  subdivMotion.subdivideMidPoint(
    params.geometrySamplingSubdivisionIterationCount);
  subdivMapped.subdivideMidPoint(
    params.geometrySamplingSubdivisionIterationCount);
  subdivTarget.computeNormals();
  InitialDeform(subdivTarget, subdivMapped, subdivMotion, deformed, params);

  if (params.applySmoothingDeform) {
    vout << "\t Refining deformation...\n";
    if (!params.smoothingDeformUseInitialGeometry) {
      initialPositions = deformed.points();
      deformed.computeTriangleNormals(initialTriangleNormals);
    }
    for (int32_t v = 0; v < subdivTarget.pointCount(); ++v) {
      subdivTarget.setPoint(v, subdivTarget.point(v) + subdivMotion.point(v));
    }
    subdivTarget.computeNormals();
    Deform(
      subdivTarget, initialPositions, initialTriangleNormals, deformed,
      params);
  }

  auto end = chrono::steady_clock::now();
  auto deltams = chrono::duration_cast<chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  return true;
}

//----------------------------------------------------------------------------

bool
RemoveDuplicatedTriangles(TriangleMesh<double>& mesh)
{
  vout << "Removing duplicated triangles... ";
  const auto triangleCount0 = mesh.triangleCount();
  auto start = chrono::steady_clock::now();

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
  vout << "\t Cleaned up mesh: " << mesh.pointCount() << "V "
       << mesh.triangleCount() << "T\n";
  vout << "\t\t Removed " << (triangleCount0 - mesh.triangleCount())
       << " triangles!\n";
  return true;
}

//----------------------------------------------------------------------------

bool
UnifyVertices(const TriangleMesh<double>& mesh, TriangleMesh<double>& umesh)
{
  cout << "Unifying vertices... ";
  const auto pointCount0 = mesh.pointCount();
  const auto triangleCount0 = mesh.triangleCount();
  auto start = chrono::steady_clock::now();

  vector<int32_t> mapping;
  UnifyVertices(
    mesh.points(), mesh.triangles(), umesh.points(), umesh.triangles(),
    mapping);
  RemoveDegeneratedTriangles(umesh);

  auto end = chrono::steady_clock::now();
  auto deltams = chrono::duration_cast<chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  vout << "\t Unified mesh: " << mesh.pointCount() << "V "
       << mesh.triangleCount() << "T\n";
  vout << "\t\t Removed " << (pointCount0 - umesh.pointCount()) << " vertices "
       << (triangleCount0 - umesh.triangleCount()) << " triangles\n";
  return true;
}

//----------------------------------------------------------------------------

bool
ComputeMotion(
  const TriangleMesh<double>& reference,
  const TriangleMesh<double>& target,
  TriangleMesh<double>& motion)
{
  vout << "Deforming geometry... ";

  const auto pointCountTarget = target.pointCount();
  const auto texCoordCountTarget = target.texCoordCount();
  const auto triangleCountTarget = target.triangleCount();
  assert(triangleCountTarget == target.texCoordTriangleCount());
  const auto pointCountReference = reference.pointCount();
  const auto texCoordCountReference = reference.texCoordCount();
  const auto triangleCountReference = reference.triangleCount();
  assert(triangleCountReference == reference.texCoordTriangleCount());
  if (texCoordCountTarget != texCoordCountReference) {
    return false;
  }

  std::vector<int32_t> mapPositionToTexCoordTarget(pointCountTarget, -1);
  for (int32_t t = 0; t < triangleCountTarget; ++t) {
    const auto& tri = target.triangle(t);
    const auto& texCoordTri = target.texCoordTriangle(t);
    for (int32_t k = 0; k < 3; ++k) {
      mapPositionToTexCoordTarget[tri[k]] = texCoordTri[k];
    }
  }
  std::vector<int32_t> mapTexCoordToPositionReference(
    texCoordCountReference, -1);
  for (int32_t t = 0; t < triangleCountReference; ++t) {
    const auto& tri = reference.triangle(t);
    const auto& texCoordTri = reference.texCoordTriangle(t);
    for (int32_t k = 0; k < 3; ++k) {
      mapTexCoordToPositionReference[texCoordTri[k]] = tri[k];
    }
  }
  motion.triangles() = target.triangles();
  motion.resizePoints(pointCountTarget);
  for (int32_t v = 0; v < pointCountTarget; ++v) {
    const auto i = mapPositionToTexCoordTarget[v];
    assert(i >= 0 && i < texCoordCountTarget);
    const auto j = mapTexCoordToPositionReference[i];
    if (j >= 0 && j < pointCountReference) {
      motion.setPoint(v, reference.point(j) - target.point(v));
    } else {
      assert(0);
    }
  }

  vout << " [done]\n";
  vout << "\t Reference mesh: " << reference.pointCount() << "V "
       << reference.triangleCount() << "T " << reference.texCoordCount()
       << "UV " << reference.texCoordTriangleCount() << "TUV\n";
  vout << "\t Target mesh: " << target.pointCount() << "V "
       << target.triangleCount() << "T " << target.texCoordCount() << "UV "
       << target.texCoordTriangleCount() << "TUV\n";
  return true;
}

//============================================================================

bool
LoadInputMeshes(
  TriangleMesh<double>& target,
  TriangleMesh<double>& source,
  TriangleMesh<double>& mapped,
  TriangleMesh<double>& mtarget,
  const Parameters& params)
{
  vout << "Loading Meshes... ";

  if (1) {
    // if mtarget is specified, target is reference frame
    // otherwise,               target is current frame
    auto tgtFnum = params.targetModifierMeshPath.empty()
        ? params.fnum : params.fnumRef;
    const auto& name = vmesh::expandNum(params.targetMeshPath, tgtFnum);
    if (!target.loadFromOBJ(name)) {
      cerr << "Error: can't load target mesh " << name << endl;
      return false;
    }
  }
  if (1) {
    const auto& name = vmesh::expandNum(params.sourceMeshPath, params.fnumRef);
    if (!source.loadFromOBJ(name)) {
      cerr << "Error: can't load source mesh " << name << endl;
      return false;
    }
  }
  if (!params.mappedMeshPath.empty()) {
    const auto& name = vmesh::expandNum(params.mappedMeshPath, params.fnumRef);
    if (!mapped.loadFromOBJ(name)) {
      cerr << "Error: can't load mapped mesh " << name << endl;
      return false;
    }
  }
  if (!params.targetModifierMeshPath.empty()) {
    const auto& name =
      vmesh::expandNum(params.targetModifierMeshPath, params.fnumRef);
    if (!mtarget.loadFromOBJ(name)) {
      cerr << "Error: can't load target modifier mesh " << name << endl;
      return false;
    }
  }

  source.resizeNormalTriangles(0);
  source.resizeNormals(0);

  vout << " [done]\n";
  vout << "\t Target mesh: " << target.pointCount() << "V "
       << target.triangleCount() << "T\n";
  vout << "\t Source mesh: " << source.pointCount() << "V "
       << source.triangleCount() << "T\n";
  vout << "\t Target modifier mesh: " << mtarget.pointCount() << "V "
       << mtarget.triangleCount() << "T\n";
  vout << "\t Mapped mesh: " << mapped.pointCount() << "V "
       << mapped.triangleCount() << "T\n";

  if (!params.mappedMeshPath.empty() &&
      (mapped.pointCount() != target.pointCount() ||
       mapped.triangleCount() != target.triangleCount())) {
    cerr << "Error: mapped has a different connectivity from target!" << endl;
    return false;
  }
  return true;
}

//============================================================================

int
main(int argc, char* argv[])
{
  cout << "MPEG VMESH version " << ::vmesh::version << endl;

  Parameters params;
  if (!parseParameters(argc, argv, params))
    return 1;

  if (params.verbose)
    vmesh::vout.rdbuf(std::cout.rdbuf());

  TriangleMesh<double> target, source, mapped, mtarget;
  if (!LoadInputMeshes(target, source, mapped, mtarget, params)) {
    cerr << "Error: can't load input meshes!" << endl;
    return 1;
  }
  target.computeNormals();

  TriangleMesh<double> motion;
  if (
    !params.targetModifierMeshPath.empty()
    && !ComputeMotion(mtarget, target, motion)) {
    cerr << "Error: target mesh not modified!" << endl;
    return 1;
  }

  if (
    motion.pointCount() != target.pointCount()
    || motion.triangles() != target.triangles()) {
    motion.triangles() = target.triangles();
    motion.resizePoints(target.pointCount());
  }

  TriangleMesh<double> usource = source;
  if (params.applyVertexUnification && !UnifyVertices(source, usource)) {
    cerr << "Error: can't unify vertices!" << endl;
    return 1;
  }

  TriangleMesh<double> deformed;
  if (!params.subdiv0MeshPath.empty()) {
    const auto& name = vmesh::expandNum(params.subdiv0MeshPath, params.fnum);
    if (!deformed.loadFromOBJ(name)) {
      cerr << "Error: can't load source mesh " << name << endl;
      return 1;
    }
  } else {
    deformed = usource;
    if (!Subdivide(deformed, params)) {
      cerr << "Error: can't subdivide mesh!" << endl;
      return 1;
    }
  }

  if (
    params.subdivisionIterationCount
    && !FitMesh(target, mapped, motion, deformed, params)) {
    cerr << "Error: can't fit mesh!" << endl;
    return 1;
  }

  auto base = usource;
  if (params.fitSubdivisionSurface && params.subdivisionIterationCount) {
    vout << "Fitting subdiv ...\n";
    FitMidPointSubdivision(deformed, base, params.subdivisionIterationCount);
  }

  auto subdiv = base;
  if (!Subdivide(subdiv, params)) {
    cerr << "Error: can't subdivide mesh!" << endl;
    return 1;
  }
  auto ndeformed = deformed;
  double errorNormal = 0.0;
  double errorTangent = 0.0;
  for (int32_t v = 0, count = deformed.pointCount(); v < count; ++v) {
    const auto p0 = deformed.point(v);
    const auto p1 = subdiv.point(v);
    const auto n = subdiv.normal(v);
    const auto delta = p0 - p1;
    const auto dot = delta * n;
    const auto nDelta = dot * n;
    const auto tDelta = delta - nDelta;
    errorNormal += nDelta * nDelta;
    errorTangent += tDelta.norm2();
    ndeformed.setPoint(v, p1 + nDelta);
  }
  errorNormal /= deformed.pointCount();
  errorTangent /= deformed.pointCount();
  std::cout << " normal rmse = " << std::sqrt(errorNormal) << std::endl;
  std::cout << " tangential rmse = " << std::sqrt(errorTangent) << std::endl;

  if (!params.baseMeshPath.empty()) {
    const auto& name = vmesh::expandNum(params.baseMeshPath, params.fnum);
    if (!base.saveToOBJ(name)) {
      cerr << "Error: can't save base mesh!" << endl;
      return 1;
    }
  }

  if (!params.nsubdivMeshPath.empty()) {
    const auto& name = vmesh::expandNum(params.nsubdivMeshPath, params.fnum);
    if (!ndeformed.saveToOBJ(name)) {
      cerr << "Error: can't save normal subdivision mesh!" << endl;
      return 1;
    }
  }

  if (1) {
    const auto& name = vmesh::expandNum(params.subdivMeshPath, params.fnum);
    if (!deformed.saveToOBJ(name)) {
      cerr << "Error: can't save subdivision mesh!" << endl;
      return 1;
    }
  }

  return 0;
}
