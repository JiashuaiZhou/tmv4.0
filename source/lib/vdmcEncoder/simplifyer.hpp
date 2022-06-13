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

namespace vmesh {

struct VMCFrame; 
template<typename T>
class TriangleMesh;

//============================================================================

struct VMCSimplifyParameters {  
  int32_t texCoordQuantizationBits;
  int32_t minCCTriangleCount;
  double targetTriangleRatio;
  double triangleFlipThreshold = 0.3;
  double trackedTriangleFlipThreshold = 0.1;
  double trackedPointNormalFlipThreshold = 0.5;
};

//============================================================================

class Simplifyer {
public:
  Simplifyer() = default;
  ~Simplifyer() = default;

  bool simplify(VMCFrame& frame, const VMCSimplifyParameters& params);

  bool removeSmallConnectedComponents(
    TriangleMesh<double>& mesh, int32_t minCCTriangleCount);

  bool removeDuplicatedTriangles(TriangleMesh<double>& mesh);

  bool unifyVertices(TriangleMesh<double>& mesh);

  bool unifyVertices(const TriangleMesh<double>& mesh, TriangleMesh<double>& umesh);

  bool decimate(
    const TriangleMesh<double>& mesh,
    TriangleMesh<double>& dmesh,
    TriangleMesh<double>& mmesh,
    const VMCSimplifyParameters& params);
private:
};

}  // namespace vmesh