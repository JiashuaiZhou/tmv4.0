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

#include "meshPositionDeduplicateInformation.hpp"

namespace eb {

// 1.2.7	Mesh position coding payload syntax
class MeshPositionCodingPayload {
public:
  MeshPositionCodingPayload() {}
  ~MeshPositionCodingPayload() {}

  MeshPositionCodingPayload&
  operator=(const MeshPositionCodingPayload&) = default;

  auto& getMeshVertexCount() const { return meshVertexCount_; }
  auto& getMeshClersCount() const { return meshClersCount_; }
  auto& getMeshCcCount() const { return meshCcCount_; }
  auto& getMeshVirtualVertexCount() const { return meshVirtualVertexCount_; }
  auto& getMeshVirtualIndexDelta() const { return meshVirtualIndexDelta_; }
  auto& getMeshCcWithHandlesCount() const { return meshCcWithHandlesCount_; }
  auto& getMeshHandlesCcOffset() const { return meshHandlesCcOffset_; }
  auto& getMeshHandlesCount() const { return meshHandlesCount_; }
  auto& getMeshHandleIndexFirstDelta() const {
    return meshHandleIndexFirstDelta_;
  }
  auto& getMeshHandleIndexSecondDelta() const {
    return meshHandleIndexSecondDelta_;
  }
  auto& getMeshCodedHandleIndexSecondShiftSize() const {
    return meshCodedHandleIndexSecondShiftSize_;
  }
  auto& getMeshHandleIndexSecondShift() const {
    return meshHandleIndexSecondShift_;
  }
  auto& getMeshCodedClersSymbolsSize() const {
    return meshCodedClersSymbolsSize_;
  }
  auto& getMeshClersSymbol() const { return meshClersSymbol_; }
  auto& getMeshPositionStart() const { return meshPositionStart_; }
  auto& getMeshCodedPositionResidualsSize() const {
    return meshCodedPositionResidualsSize_;
  }
  auto& getMeshPositionResidual() const { return meshPositionResidual_; }
  auto& getMeshPositionDeduplicateInformation() const {
    return meshPositionDeduplicateInformation_;
  }

  auto& getMeshVertexCount() { return meshVertexCount_; }
  auto& getMeshClersCount() { return meshClersCount_; }
  auto& getMeshCcCount() { return meshCcCount_; }
  auto& getMeshVirtualVertexCount() { return meshVirtualVertexCount_; }
  auto& getMeshVirtualIndexDelta() { return meshVirtualIndexDelta_; }
  auto& getMeshCcWithHandlesCount() { return meshCcWithHandlesCount_; }
  auto& getMeshHandlesCcOffset() { return meshHandlesCcOffset_; }
  auto& getMeshHandlesCount() { return meshHandlesCount_; }
  auto& getMeshHandleIndexFirstDelta() { return meshHandleIndexFirstDelta_; }
  auto& getMeshHandleIndexSecondDelta() { return meshHandleIndexSecondDelta_; }
  auto& getMeshCodedHandleIndexSecondShiftSize() {
    return meshCodedHandleIndexSecondShiftSize_;
  }
  auto& getMeshHandleIndexSecondShift() { return meshHandleIndexSecondShift_; }
  auto& getMeshCodedClersSymbolsSize() { return meshCodedClersSymbolsSize_; }
  auto& getMeshClersSymbol() { return meshClersSymbol_; }
  auto& getMeshPositionStart() { return meshPositionStart_; }
  auto& getMeshCodedPositionResidualsSize() {
    return meshCodedPositionResidualsSize_;
  }
  auto& getMeshPositionResidual() { return meshPositionResidual_; }
  auto& getMeshPositionDeduplicateInformation() {
    return meshPositionDeduplicateInformation_;
  }

private:
  uint32_t                           meshVertexCount_        = 0;
  uint32_t                           meshClersCount_         = 0;
  uint32_t                           meshCcCount_            = 0;
  uint32_t                           meshVirtualVertexCount_ = 0;
  std::vector<uint32_t>              meshVirtualIndexDelta_;
  uint32_t                           meshCcWithHandlesCount_ = 0;
  std::vector<uint32_t>              meshHandlesCcOffset_;
  std::vector<uint32_t>              meshHandlesCount_;
  std::vector<int32_t>               meshHandleIndexFirstDelta_;
  std::vector<int32_t>               meshHandleIndexSecondDelta_;
  uint32_t                           meshCodedHandleIndexSecondShiftSize_ = 0;
  std::vector<uint8_t>               meshHandleIndexSecondShift_;
  uint32_t                           meshCodedClersSymbolsSize_ = 0;
  std::vector<uint8_t>               meshClersSymbol_;
  std::vector<std::vector<uint32_t>> meshPositionStart_;
  uint32_t                           meshCodedPositionResidualsSize_ = 0;
  std::vector<uint8_t>  meshPositionResidual_;

  MeshPositionDeduplicateInformation meshPositionDeduplicateInformation_;
};

};  // namespace eb
