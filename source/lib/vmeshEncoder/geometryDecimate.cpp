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

#include "util/misc.hpp"
#include "util/mesh.hpp"
#include "util/verbose.hpp"
#include "util/vector.hpp"
#include "vmc.hpp"

#include "encoder.hpp"
#include "geometryDecimate.hpp"
#include "triangleMeshDecimator.hpp"

#include <chrono>

namespace vmesh {

//============================================================================
template<typename T>
bool
GeometryDecimate::generate(const TriangleMesh<T>&      input,
                           TriangleMesh<T>&            reference,
                           TriangleMesh<T>&            decimate,
                           TriangleMesh<T>&            mapped,
                           const VMCEncoderParameters& params) {
  reference = input;
  if (!unifyVertices(reference)) {
    std::cerr << "Error: can't unify vertices!\n";
    return 1;
  }

  if (!generate(reference, decimate, mapped, params)) {
    std::cerr << "Error: can't simplify mesh!\n";
    return 1;
  }

  if (!removeDuplicatedTriangles(decimate)) {
    std::cerr << "Error: can't remove duplicated triangles!\n";
    return 1;
  }

  if (!removeSmallConnectedComponents(decimate, params.minCCTriangleCount)) {
    std::cerr << "Error: can't remove small connected components!\n";
    return 1;
  }
  reference.scaleTextureCoordinates(params.texCoordQuantizationBits);
  return 0;
}

//============================================================================

template<typename T>
bool
GeometryDecimate::removeSmallConnectedComponents(TriangleMesh<T>& mesh,
                                                 int minCCTriangleCount) {
  std::cout << "Removing small connected components...\n";

  const auto           pointCount    = mesh.pointCount();
  const auto           triangleCount = mesh.triangleCount();
  std::vector<int32_t> partition;
  std::vector<std::shared_ptr<TriangleMesh<T>>> connectedComponents;
  const auto ccCount0 = ExtractConnectedComponents(mesh.triangles(),
                                                   mesh.pointCount(),
                                                   mesh,
                                                   partition,
                                                   &connectedComponents);
  mesh.clear();
  int32_t ccCounter = 0;
  for (int32_t c = 0; c < ccCount0; ++c) {
    const auto& connectedComponent = connectedComponents[c];
    std::cout << "\t CC " << c << " -> " << connectedComponent->pointCount()
              << "V " << connectedComponent->triangleCount() << "T\n";
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
            << (ccCount0 - ccCounter) << "CC "
            << (pointCount - mesh.pointCount()) << "V "
            << (triangleCount - mesh.triangleCount()) << "T\n";
  return true;
}

//============================================================================

template<typename T>
bool
GeometryDecimate::removeDuplicatedTriangles(TriangleMesh<T>& mesh) {
  std::cout << "Removing duplicated triangles... ";

  const auto           triangleCount = mesh.triangleCount();
  std::vector<Vec3<T>> triangleNormals;
  mesh.computeTriangleNormals(triangleNormals);
  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(
    mesh.triangles(), mesh.pointCount(), vertexToTriangle);
  std::vector<Triangle> trianglesOutput;
  if (!RemoveDuplicatedTriangles(mesh.triangles(),
                                 triangleNormals,
                                 vertexToTriangle,
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
template<typename T>
bool
GeometryDecimate::unifyVertices(const TriangleMesh<T>& mesh,
                                TriangleMesh<T>&       umesh) {
  std::cout << "Unifying vertices... ";
  const auto pointCount0    = mesh.pointCount();
  const auto triangleCount0 = mesh.triangleCount();
  auto       start          = std::chrono::steady_clock::now();

  std::vector<int32_t> mapping;
  UnifyVertices(mesh.points(),
                mesh.triangles(),
                umesh.points(),
                umesh.triangles(),
                mapping);
  RemoveDegeneratedTriangles(umesh);

  auto end = std::chrono::steady_clock::now();
  auto deltams =
    std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << " [done] " << deltams.count() << " ms\n";
  std::cout << "\t Unified mesh: " << mesh.pointCount() << "V "
            << mesh.triangleCount() << "T\n";
  std::cout << "\t\t Removed " << (pointCount0 - umesh.pointCount())
            << " vertices " << (triangleCount0 - umesh.triangleCount())
            << " triangles\n";
  return true;
}

//============================================================================

template<typename T>
bool
GeometryDecimate::unifyVertices(TriangleMesh<T>& mesh) {
  std::cout << "Unifying vertices... ";
  const auto pointCount0    = mesh.pointCount();
  const auto triangleCount0 = mesh.triangleCount();

  std::vector<Vec3<T>>  upoints;
  std::vector<Triangle> utriangles;
  std::vector<int32_t>  mapping;
  UnifyVertices(mesh.points(), mesh.triangles(), upoints, utriangles, mapping);
  std::swap(mesh.points(), upoints);
  std::swap(mesh.triangles(), utriangles);
  RemoveDegeneratedTriangles(mesh);

  std::cout << "Unified mesh: " << mesh.pointCount() << "V "
            << mesh.triangleCount() << "T\n";
  std::cout << "\t Removed " << (pointCount0 - mesh.pointCount())
            << " vertices " << (triangleCount0 - mesh.triangleCount())
            << " triangles\n";
  return true;
}

//============================================================================

template<typename T>
bool
GeometryDecimate::generate(const TriangleMesh<T>&      meshT,
                           TriangleMesh<T>&            dmeshT,
                           TriangleMesh<T>&            mmeshT,
                           const VMCEncoderParameters& params) {
  std::cout << "Simplifying Mesh... \n";

  TriangleMesh<double> mesh, dmesh, mmesh;
  mesh.convert(meshT);

  TriangleMeshDecimatorParameters dparams;
  dparams.triangleCount =
    std::ceil(mesh.triangleCount() * params.targetTriangleRatio);
  dparams.triangleFlipThreshold        = params.triangleFlipThreshold;
  dparams.trackedTriangleFlipThreshold = params.trackedTriangleFlipThreshold;
  dparams.trackedPointNormalFlipThreshold =
    params.trackedPointNormalFlipThreshold;
  TriangleMeshDecimator decimator;
  if (decimator.decimate((const double*)mesh.points().data(),
                         mesh.pointCount(),
                         (const int32_t*)mesh.triangles().data(),
                         mesh.triangleCount(),
                         dparams)
      != Error::OK) {
    std::cerr << "Error: can't decimate model\n";
    return false;
  }
  printf("decimatedMesh \n");
  fflush(stdout);

  dmesh.resizePoints(decimator.decimatedPointCount());
  dmesh.resizeTriangles(decimator.decimatedTriangleCount());
  if (decimator.decimatedMesh((double*)dmesh.points().data(),
                              dmesh.pointCount(),
                              (int32_t*)dmesh.triangles().data(),
                              dmesh.triangleCount())
      != Error::OK) {
    std::cerr << "Error: can't extract decimated model\n";
    return false;
  }

  mmesh.triangles() = mesh.triangles();
  printf("mesh.triangleCount()  = %d \n", mesh.triangleCount());
  printf("mesh.pointCount()     = %d \n", mesh.pointCount());
  printf("mmesh.triangleCount() = %d \n", mmesh.triangleCount());
  printf("mmesh.pointCount()    = %d \n", mmesh.pointCount());
  printf("resizePoints \n");
  fflush(stdout);
  mmesh.resizePoints(mesh.pointCount());
  printf("trackedPoints \n");
  fflush(stdout);
  decimator.trackedPoints(
    (double*)(mmesh.points().data()), nullptr, mesh.pointCount());

  std::cout << "Decimated mesh: " << decimator.decimatedPointCount() << "V "
            << decimator.decimatedTriangleCount() << "T\n";

  dmeshT.convert(dmesh);
  mmeshT.convert(mmesh);
  return true;
}

//============================================================================

template bool GeometryDecimate::generate<float>(const TriangleMesh<float>&,
                                                TriangleMesh<float>&,
                                                TriangleMesh<float>&,
                                                TriangleMesh<float>&,
                                                const VMCEncoderParameters&);
template bool GeometryDecimate::generate<double>(const TriangleMesh<double>&,
                                                 TriangleMesh<double>&,
                                                 TriangleMesh<double>&,
                                                 TriangleMesh<double>&,
                                                 const VMCEncoderParameters&);

template bool GeometryDecimate::unifyVertices<float>(TriangleMesh<float>&);
template bool GeometryDecimate::unifyVertices<double>(TriangleMesh<double>&);

template bool
GeometryDecimate::unifyVertices<float>(const TriangleMesh<float>&,
                                       TriangleMesh<float>&);
template bool
GeometryDecimate::unifyVertices<double>(const TriangleMesh<double>&,
                                        TriangleMesh<double>&);

//============================================================================

}  // namespace vmesh