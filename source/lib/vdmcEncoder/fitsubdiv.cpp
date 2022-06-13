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

#include "fitsubdiv.hpp"
#include "vmc.hpp"
#include "simplifyer.hpp"

namespace vmesh {

//============================================================================

bool
FitSubDiv::generate(
  VMCFrame& frame,
  const VMCFitSubDivParameters& params,
  TriangleMesh<double>& mtarget,
  TriangleMesh<double>& subdiv0)
{
  auto& target = frame.reference;
  auto& source = frame.decimateTexture;
  auto& mapped = frame.mapped;
  auto& deformed = frame.subdiv;
  auto& ndeformed= frame.nsubdiv;
  auto& base = frame.base;

  target.computeNormals();
  source.resizeNormalTriangles(0);
  source.resizeNormals(0);

  std::cout << " [done]\n";
  std::cout << "\t Target mesh: " << target.pointCount() << "V "
       << target.triangleCount() << "T\n";
  std::cout << "\t Source mesh: " << source.pointCount() << "V "
       << source.triangleCount() << "T\n";
  std::cout << "\t Target modifier mesh: " << mtarget.pointCount() << "V "
       << mtarget.triangleCount() << "T\n";
  std::cout << "\t Mapped mesh: " << mapped.pointCount() << "V "
       << mapped.triangleCount() << "T\n";

  if ( mapped.pointCount() != 0 &&
      (mapped.pointCount() != target.pointCount() ||
       mapped.triangleCount() != target.triangleCount())) {
    std::cerr << "Error: mapped has a different connectivity from target!\n";
    return false;
  }

  TriangleMesh<double> motion;
  if ( mtarget.pointCount() != 0  
    && !ComputeMotion(mtarget, target, motion)) {
    std::cerr << "Error: target mesh not modified!\n";
    return false;
  }

  if (
    motion.pointCount() != target.pointCount()
    || motion.triangles() != target.triangles()) {
    motion.triangles() = target.triangles();
    motion.resizePoints(target.pointCount());
  }

  TriangleMesh<double> usource = source;
  Simplifyer simplifier;
  if (params.applyVertexUnification && !simplifier.unifyVertices(source, usource)) {
    std::cerr << "Error: can't unify vertices!\n";
    return false;
  }

  if ( subdiv0.pointCount() != 0 ) {
    deformed = subdiv0;
  } else {
    deformed = usource;
    if (!Subdivide(deformed, params)) {
      std::cerr << "Error: can't subdivide mesh!\n";
      return false;
    }
  }

  if (
    params.subdivisionIterationCount
    && !FitMesh(target, mapped, motion, deformed, params)) {
    std::cerr << "Error: can't fit mesh!\n";
    return false;
  }

  base = usource;
  if (params.fitSubdivisionSurface && params.subdivisionIterationCount) {
    vout << "Fitting subdiv ...\n";
    FitMidPointSubdivision(deformed, base, params.subdivisionIterationCount);
  }

  auto subdiv = base;
  if (!Subdivide(subdiv, params)) {
    std::cerr << "Error: can't subdivide mesh!\n";
    return false;
  }
  
  ndeformed = deformed;
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
  std::cout << " normal rmse = " << std::sqrt(errorNormal) << '\n';
  std::cout << " tangential rmse = " << std::sqrt(errorTangent) << '\n';
  return true;
}

//============================================================================

bool
FitSubDiv::CheckTriangleNormalInversion(
  const int32_t vindex,
  const TriangleMesh<double>& output,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  const std::vector<Vec3<double>>& initialTriangleNormals,
  const VMCFitSubDivParameters& params)
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
FitSubDiv::FitMesh(
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
FitSubDiv::UpdateMissedVertices(
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  const std::vector<int8_t>& isBoundaryVertex,
  const std::vector<int32_t>& missedVertices,
  TriangleMesh<double>& output,
  std::vector<int32_t>& vadj,
  std::vector<int8_t>& vtags,
  std::vector<int8_t>& ttags,
  const VMCFitSubDivParameters& params)
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
FitSubDiv::InitialDeform(
  const TriangleMesh<double>& target,
  const TriangleMesh<double>& mapped,
  const TriangleMesh<double>& motion,
  TriangleMesh<double>& output,
  const VMCFitSubDivParameters& params)
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
FitSubDiv::InitialDeform(
  const TriangleMesh<double>& target,
  TriangleMesh<double>& output,
  const VMCFitSubDivParameters& params)
{
  StaticAdjacencyInformation<int32_t> vertexToTriangleTarget;
  ComputeVertexToTriangle(
    target.triangles(), target.pointCount(), vertexToTriangleTarget);
  KdTree kdtreeTarget(3, target.points(), 10);  // dim, cloud, max leaf

  const auto* neighboursTarget = vertexToTriangleTarget.neighbours();
  std::vector<int32_t> missedVertices;
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
FitSubDiv::Deform(
  const TriangleMesh<double>& target,
  const std::vector<Vec3<double>>& initialPositions,
  const std::vector<Vec3<double>>& initialTriangleNormals,
  TriangleMesh<double>& output,
  const VMCFitSubDivParameters& params)
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

  std::vector<int32_t> missedVertices;
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

    vout << '[' << missedVertices.size() << ']';
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
FitSubDiv::Subdivide(TriangleMesh<double>& mesh, const VMCFitSubDivParameters& params)
{
  vout << "Subdividing Mesh...\n";
  auto start = std::chrono::steady_clock::now();

  mesh.computeNormals();
  if (params.subdivisionIterationCount) {
    std::vector<SubdivisionLevelInfo> infoLevelOfDetails;
    std::vector<int64_t> subdivEdges;
    mesh.subdivideMidPoint(
      params.subdivisionIterationCount, &infoLevelOfDetails, &subdivEdges);
    mesh.resizeNormals(mesh.pointCount());
    interpolateSubdivision(
      mesh.normals(), infoLevelOfDetails, subdivEdges, 0.5, 0.5, true);
  }

  auto end = std::chrono::steady_clock::now();
  auto deltams = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  vout << "\t Subdivided mesh: " << mesh.pointCount() << "V "
       << mesh.triangleCount() << "T\n";
  return true;
}

//----------------------------------------------------------------------------

bool
FitSubDiv::FitMesh(
  const TriangleMesh<double>& target,
  TriangleMesh<double>& mapped,
  const TriangleMesh<double>& motion,
  TriangleMesh<double>& deformed,
  const VMCFitSubDivParameters& params)
{
  vout << "Deforming Mesh...\n";
  auto start = std::chrono::steady_clock::now();

  std::vector<Vec3<double>> initialPositions;
  std::vector<Vec3<double>> initialTriangleNormals;
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

  auto end = std::chrono::steady_clock::now();
  auto deltams = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  vout << " [done] " << deltams.count() << " ms\n";
  return true;
}

//----------------------------------------------------------------------------

bool
FitSubDiv::ComputeMotion(
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

}  // namespace vmesh