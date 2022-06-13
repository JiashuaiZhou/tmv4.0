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

#pragma once

#include "misc.hpp"
#include "verbose.hpp"
#include "vector.hpp"
#include "version.hpp"

#include "mesh.hpp"
#include "kdtree.hpp"
#include "vmc.hpp"

namespace vmesh {

//============================================================================

enum class SmoothingMethod
{
  NONE = 0,
  VERTEX_CONSTRAINT = 1
};

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

struct VMCFitSubDivParameters {
  int geometrySamplingSubdivisionIterationCount = 3;
  bool applyVertexUnification = true;

  int32_t geometryFittingIterationCount = 16;
  double geometrySmoothingCoeffcient = 0.25;
  double geometrySmoothingCoeffcientDecayRatio = 0.75;
  double geometryMissedVerticesSmoothingCoeffcient = 0.1;
  int32_t geometryMissedVerticesSmoothingIterationCount = 10;

  SubdivisionMethod subdivisionMethod = SubdivisionMethod::MID_POINT;
  int32_t subdivisionIterationCount = 3;
  bool fitSubdivisionSurface = true;

  double initialDeformNormalDeviationThreshold = 0.1;
  int32_t initialDeformNNCount = 1;
  bool initialDeformForceNormalDisplacement = false;

  bool applySmoothingDeform = true;
  SmoothingMethod smoothDeformSmoothingMethod =
    SmoothingMethod::VERTEX_CONSTRAINT;
  bool smoothDeformUpdateNormals = true;
  double smoothDeformTriangleNormalFlipThreshold = -0.5;
  bool smoothingDeformUseInitialGeometry = true;
  bool smoothingDeformSmoothMotion = true;
};

//============================================================================

class FitSubDiv {
public:
  FitSubDiv() = default;
  ~FitSubDiv() = default;

  bool generate(
    VMCFrame& frame,
    const VMCFitSubDivParameters& params,
    TriangleMesh<double>& mtarget,
    TriangleMesh<double>& subdiv0);

private:
  bool CheckTriangleNormalInversion(
    const int32_t vindex,
    const TriangleMesh<double>& output,
    const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
    const std::vector<Vec3<double>>& initialTriangleNormals,
    const VMCFitSubDivParameters& params);

  void FitMesh(
    const TriangleMesh<double>& target,
    const KdTree& kdtree,
    TriangleMesh<double>& output);

  void UpdateMissedVertices(
    const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
    const std::vector<int8_t>& isBoundaryVertex,
    const std::vector<int32_t>& missedVertices,
    TriangleMesh<double>& output,
    std::vector<int32_t>& vadj,
    std::vector<int8_t>& vtags,
    std::vector<int8_t>& ttags,
    const VMCFitSubDivParameters& params);

  void InitialDeform(
    const TriangleMesh<double>& target,
    const TriangleMesh<double>& mapped,
    const TriangleMesh<double>& motion,
    TriangleMesh<double>& output,
    const VMCFitSubDivParameters& params);

  void InitialDeform(
    const TriangleMesh<double>& target,
    TriangleMesh<double>& output,
    const VMCFitSubDivParameters& params);

  void Deform(
    const TriangleMesh<double>& target,
    const std::vector<Vec3<double>>& initialPositions,
    const std::vector<Vec3<double>>& initialTriangleNormals,
    TriangleMesh<double>& output,
    const VMCFitSubDivParameters& params);

  bool Subdivide(TriangleMesh<double>& mesh, const VMCFitSubDivParameters& params);

  bool FitMesh(
    const TriangleMesh<double>& target,
    TriangleMesh<double>& mapped,
    const TriangleMesh<double>& motion,
    TriangleMesh<double>& deformed,
    const VMCFitSubDivParameters& params);

  bool RemoveDuplicatedTriangles(TriangleMesh<double>& mesh);

  bool ComputeMotion(
    const TriangleMesh<double>& reference,
    const TriangleMesh<double>& target,
    TriangleMesh<double>& motion);
};

}  // namespace vmesh