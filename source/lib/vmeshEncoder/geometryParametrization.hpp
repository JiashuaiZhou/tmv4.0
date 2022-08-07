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

#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "util/vector.hpp"
#include "version.hpp"

#include "util/mesh.hpp"
#include "util/kdtree.hpp"
#include "vmc.hpp"

namespace vmesh {

struct GeometryParametrizationParameters;

//============================================================================

class GeometryParametrization {
public:
  GeometryParametrization()  = default;
  ~GeometryParametrization() = default;

  bool generate(TriangleMesh<double>&                    target,
                TriangleMesh<double>&                    source,
                TriangleMesh<double>&                    mapped,
                const TriangleMesh<double>&              mtarget,
                const TriangleMesh<double>&              subdiv0,
                const GeometryParametrizationParameters& params,
                TriangleMesh<double>&                    base,
                TriangleMesh<double>&                    deformed,
                TriangleMesh<double>&                    ndeformed);

private:
  static bool CheckTriangleNormalInversion(
    int32_t                                    vindex,
    const TriangleMesh<double>&                output,
    const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
    const std::vector<Vec3<double>>&           initialTriangleNormals,
    const GeometryParametrizationParameters&   params);

  static void FitMesh(const TriangleMesh<double>& target,
                      const KdTree&               kdtree,
                      TriangleMesh<double>&       output);

  static void UpdateMissedVertices(
    const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
    const std::vector<int8_t>&                 isBoundaryVertex,
    const std::vector<int32_t>&                missedVertices,
    TriangleMesh<double>&                      output,
    std::vector<int32_t>&                      vadj,
    std::vector<int8_t>&                       vtags,
    std::vector<int8_t>&                       ttags,
    const GeometryParametrizationParameters&   params);

  static void InitialDeform(const TriangleMesh<double>&              target,
                            const TriangleMesh<double>&              mapped,
                            const TriangleMesh<double>&              motion,
                            TriangleMesh<double>&                    output,
                            const GeometryParametrizationParameters& params);

  static void InitialDeform(const TriangleMesh<double>&              target,
                            TriangleMesh<double>&                    output,
                            const GeometryParametrizationParameters& params);

  static void Deform(const TriangleMesh<double>&      target,
                     const std::vector<Vec3<double>>& initialPositions,
                     const std::vector<Vec3<double>>& initialTriangleNormals,
                     TriangleMesh<double>&            output,
                     const GeometryParametrizationParameters& params);

  static bool Subdivide(TriangleMesh<double>&                    mesh,
                        const GeometryParametrizationParameters& params);

  static bool FitMesh(const TriangleMesh<double>&              target,
                      TriangleMesh<double>&                    mapped,
                      const TriangleMesh<double>&              motion,
                      TriangleMesh<double>&                    deformed,
                      const GeometryParametrizationParameters& params);

  bool RemoveDuplicatedTriangles(TriangleMesh<double>& mesh);

  static bool ComputeMotion(const TriangleMesh<double>& reference,
                            const TriangleMesh<double>& target,
                            TriangleMesh<double>&       motion);
};

}  // namespace vmesh