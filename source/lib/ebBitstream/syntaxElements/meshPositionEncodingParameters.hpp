/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <vector>

namespace eb {

// 1.2.3	Mesh position encoding parameters syntax
class MeshPositionEncodingParameters {
public:
  MeshPositionEncodingParameters() {}
  ~MeshPositionEncodingParameters() {}
  MeshPositionEncodingParameters&
  operator=(const MeshPositionEncodingParameters&) = default;

  auto getMeshPositionBitDepthMinus1() const { return meshPositionBitDepthMinus1_; }
  auto getMeshClersSymbolsEncodingMethod() const {
    return meshClersSymbolsEncodingMethod_;
  }
  auto getMeshPositionPredictionMethod() const {
    return meshPositionPredictionMethod_;
  }
  auto getMeshPositionResidualsEncodingMethod() const {
    return meshPositionResidualsEncodingMethod_;
  }
  auto getMeshPositionDeduplicateMethod() const {
    return meshPositionDeduplicateMethod_;
  }

  auto& getMeshPositionBitDepthMinus1() { return meshPositionBitDepthMinus1_; }
  auto& getMeshClersSymbolsEncodingMethod() {
    return meshClersSymbolsEncodingMethod_;
  }
  auto& getMeshPositionPredictionMethod() {
    return meshPositionPredictionMethod_;
  }
  auto& getMeshPositionResidualsEncodingMethod() {
    return meshPositionResidualsEncodingMethod_;
  }
  auto& getMeshPositionDeduplicateMethod() {
    return meshPositionDeduplicateMethod_;
  }

private:
  uint32_t                            meshPositionBitDepthMinus1_             = 0;
  MeshClersSymbolsEncodingMethod      meshClersSymbolsEncodingMethod_      = MeshClersSymbolsEncodingMethod::MESH_CLERS_AC_DEFAULT;
  MeshPositionPredictionMethod        meshPositionPredictionMethod_        = MeshPositionPredictionMethod::MESH_POSITION_MPARA;
  MeshPositionResidualsEncodingMethod meshPositionResidualsEncodingMethod_ = MeshPositionResidualsEncodingMethod::MESH_POSITION_AC_DEFAULT;
  MeshPositionDeduplicateMethod       meshPositionDeduplicateMethod_       = MeshPositionDeduplicateMethod::MESH_POSITION_DEDUP_NONE;
};

};  // namespace eb
