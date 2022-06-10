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

#include "simplifyMesh.hpp"
#include "triangleMeshDecimator.hpp"

namespace vmeshenc {


//============================================================================

bool 
SimplifyMesh::simplify( vmesh::VMCFrame& frame, const VMCSimplifyParameters& params ) {
  if (!unifyVertices(frame.input)) {
    std::cerr << "Error: can't unify vertices!\n";
    return 1;
  }

  if (!decimate(
        frame.input, frame.decimate, frame.mapped, params )) {
    std::cerr << "Error: can't simplify mesh!\n";
    return 1;
  }

  if (!removeDuplicatedTriangles(frame.decimate)) {
    std::cerr << "Error: can't remove duplicated triangles!\n";
    return 1;
  }

  if (!removeSmallConnectedComponents(frame.decimate, params.minCCTriangleCount)) {
    std::cerr << "Error: can't remove small connected components!\n";
    return 1;
  }

  frame.reference = frame.input;
  frame.reference.scaleTextureCoordinates( params.texCoordQuantizationBits );
  return 0;
}

//============================================================================

bool
SimplifyMesh::removeSmallConnectedComponents(
  vmesh::TriangleMesh<double>& mesh, int32_t minCCTriangleCount)
{
  std::cout << "Removing small connected components...\n";

  const auto pointCount = mesh.pointCount();
  const auto triangleCount = mesh.triangleCount();
  std::vector<int32_t> partition;
  std::vector<std::shared_ptr<vmesh::TriangleMesh<double>>> connectedComponents;
  const auto ccCount0 = ExtractConnectedComponents(
    mesh.triangles(), mesh.pointCount(), mesh, partition,
    &connectedComponents);
  mesh.clear();
  int32_t ccCounter = 0;
  for (int32_t c = 0; c < ccCount0; ++c) {
    const auto& connectedComponent = connectedComponents[c];
    std::cout << "\t CC " << c << " -> " << connectedComponent->pointCount() << "V "
         << connectedComponent->triangleCount() << "T\n";
    if (connectedComponent->triangleCount() <= minCCTriangleCount) {
      continue;
    }
    ++ccCounter;
    mesh.append(*connectedComponent);
  }

  std::cout << "Input mesh: " << ccCount0 << "CC " << pointCount << "V "
       << triangleCount << "T\n";
  std::cout << "Cleaned up mesh: " << ccCounter << "CC " << mesh.pointCount()
       << "V " << mesh.triangleCount() << 'T' << "\t Removed "
       << (ccCount0 - ccCounter) << "CC " << (pointCount - mesh.pointCount())
       << "V " << (triangleCount - mesh.triangleCount()) << "T\n";
  return true;
}

//============================================================================

bool
SimplifyMesh::removeDuplicatedTriangles(vmesh::TriangleMesh<double>& mesh)
{
  std::cout << "Removing duplicated triangles... ";

  const auto triangleCount = mesh.triangleCount();
  std::vector<vmesh::Vec3<double>> triangleNormals;
  mesh.computeTriangleNormals(triangleNormals);
  vmesh::StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(
    mesh.triangles(), mesh.pointCount(), vertexToTriangle);
  std::vector<vmesh::Triangle> trianglesOutput;
  if (!vmesh::RemoveDuplicatedTriangles(
        mesh.triangles(), triangleNormals, vertexToTriangle,
        trianglesOutput)) {
    return false;
  }
  std::swap(mesh.triangles(), trianglesOutput);

  std::cout << "Cleaned up mesh: " << mesh.pointCount() << "V "
       << mesh.triangleCount() << "T\n";
  std::cout << "\t Removed " << (triangleCount - mesh.triangleCount())
       << " triangles\n";
  return true;
}

//============================================================================

bool
SimplifyMesh::unifyVertices(vmesh::TriangleMesh<double>& mesh)
{
  std::cout << "Unifying vertices... ";
  const auto pointCount0 = mesh.pointCount();
  const auto triangleCount0 = mesh.triangleCount();

  std::vector<vmesh::Vec3<double>> upoints;
  std::vector<vmesh::Triangle> utriangles;
  std::vector<int32_t> mapping;
  vmesh::UnifyVertices(mesh.points(), mesh.triangles(), upoints, utriangles, mapping);
  std::swap(mesh.points(), upoints);
  std::swap(mesh.triangles(), utriangles);
  RemoveDegeneratedTriangles(mesh);

  std::cout << "Unified mesh: " << mesh.pointCount() << "V " << mesh.triangleCount()
       << "T\n";
  std::cout << "\t Removed " << (pointCount0 - mesh.pointCount()) << " vertices "
       << (triangleCount0 - mesh.triangleCount()) << " triangles\n";
  return true;
}

//============================================================================

bool
SimplifyMesh::decimate(
  const vmesh::TriangleMesh<double>& mesh,
  vmesh::TriangleMesh<double>& dmesh,
  vmesh::TriangleMesh<double>& mmesh,
  const VMCSimplifyParameters& params )
{
  std::cout << "Simplifying Mesh... \n";

  TriangleMeshDecimatorParameters dparams;
  dparams.triangleCount =
    std::ceil(mesh.triangleCount() * params.targetTriangleRatio);
  dparams.triangleFlipThreshold =  params.triangleFlipThreshold;
  dparams.trackedTriangleFlipThreshold =  params.trackedTriangleFlipThreshold;
  dparams.trackedPointNormalFlipThreshold =  params.trackedPointNormalFlipThreshold;
  TriangleMeshDecimator decimator;
  if (
    decimator.decimate(
      (const double*)mesh.points().data(), mesh.pointCount(),
      (const int32_t*)mesh.triangles().data(), mesh.triangleCount(), dparams)
    != vmesh::Error::OK) {
    std::cerr << "Error: can't decimate model\n";
    return false;
  }

  dmesh.resizePoints(decimator.decimatedPointCount());
  dmesh.resizeTriangles(decimator.decimatedTriangleCount());
  if (
    decimator.decimatedMesh(
      (double*)dmesh.points().data(), dmesh.pointCount(),
      (int32_t*)dmesh.triangles().data(), dmesh.triangleCount())
    != vmesh::Error::OK) {
    std::cerr << "Error: can't extract decimated model\n";
    return false;
  }

  mmesh.triangles() = mesh.triangles();
  mmesh.resizePoints(mesh.pointCount());
  decimator.trackedPoints(
    (double*)(mmesh.points().data()), nullptr, mesh.pointCount());

  std::cout << "Decimated mesh: " << decimator.decimatedPointCount() << "V "
       << decimator.decimatedTriangleCount() << "T\n";
  return true;
}


}  // namespace vmeshenc