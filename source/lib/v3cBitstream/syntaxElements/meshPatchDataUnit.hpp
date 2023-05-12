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

#include "v3cCommon.hpp"

namespace vmesh {

// 8.3.7.X  Mesh Patch data unit syntax
class MeshPatchDataUnit {
public:
  MeshPatchDataUnit() {
    clearSubPatches();
  }
  ~MeshPatchDataUnit() {
    clearSubPatches();
  }

  MeshPatchDataUnit& operator=(const MeshPatchDataUnit&) = default;

  auto  getSubmeshId() const { return submeshId_; }
  //orthoAtlas
  auto  getProjectionTextcoordFrameScale() const { return projectionTextcoordFrameScale_; }
  auto  getProjectionTextcoordSubpatchCountMinus1() const { return projectionTextcoordSubpatchCountMinus1_; }
  auto  getProjectionTextcoordProjectionId(size_t index) const { return projectionTextcoordProjectionId_[index]; }
  auto  getProjectionTextcoordOrientationId(size_t index) const { return projectionTextcoordOrientationId_[index]; }
  auto  getProjectionTextcoord2dPosX(size_t index) const { return projectionTextcoord2dPosX_[index]; }
  auto  getProjectionTextcoord2dPosY(size_t index) const { return projectionTextcoord2dPosY_[index]; }
  auto  getProjectionTextcoord2dSizeXMinus1(size_t index) const { return projectionTextcoord2dSizeXMinus1_[index]; }
  auto  getProjectionTextcoord2dSizeYMinus1(size_t index) const { return projectionTextcoord2dSizeYMinus1_[index]; }
  auto  getProjectionTextcoordScalePresentFlag(size_t index) const { return projectionTextcoordScalePresentFlag_[index]; }
  auto  getProjectionTextcoordSubpatchScale(size_t index) const { return projectionTextcoordSubpatchScale_[index]; }

  auto  getTileOrder() const { return tileOrder_; }
  auto  getPatchIndex() const { return patchIndex_; }
  auto  getFrameIndex() const { return frameIndex_; }

  auto& getSubmeshId() { return submeshId_; }
  //orthoAtlas
  auto& getProjectionTextcoordFrameScale() { return projectionTextcoordFrameScale_; }
  //auto& getProjectionTextcoordSubpatchCountMinus1() { return projectionTextcoordSubpatchCountMinus1_; }
  auto& getProjectionTextcoordProjectionId(size_t index) { return projectionTextcoordProjectionId_[index]; }
  auto& getProjectionTextcoordOrientationId(size_t index) { return projectionTextcoordOrientationId_[index]; }
  auto& getProjectionTextcoord2dPosX(size_t index) { return projectionTextcoord2dPosX_[index]; }
  auto& getProjectionTextcoord2dPosY(size_t index) { return projectionTextcoord2dPosY_[index]; }
  auto& getProjectionTextcoord2dSizeXMinus1(size_t index) { return projectionTextcoord2dSizeXMinus1_[index]; }
  auto& getProjectionTextcoord2dSizeYMinus1(size_t index) { return projectionTextcoord2dSizeYMinus1_[index]; }
  auto& getProjectionTextcoordScalePresentFlag(size_t index) { return projectionTextcoordScalePresentFlag_[index]; }
  auto& getProjectionTextcoordSubpatchScale(size_t index) { return projectionTextcoordSubpatchScale_[index]; }

  auto& getTileOrder() { return tileOrder_; }
  auto& getPatchIndex() { return patchIndex_; }
  auto& getFrameIndex() { return frameIndex_; }

  void allocateSubPatches(size_t subPatchCount) {
      projectionTextcoordSubpatchCountMinus1_ = subPatchCount - 1;
      projectionTextcoordProjectionId_.resize(subPatchCount, 0);
      projectionTextcoordOrientationId_.resize(subPatchCount, 0);
      projectionTextcoord2dPosX_.resize(subPatchCount, 0);
      projectionTextcoord2dPosY_.resize(subPatchCount, 0);
      projectionTextcoord2dSizeXMinus1_.resize(subPatchCount, 0);
      projectionTextcoord2dSizeYMinus1_.resize(subPatchCount, 0);
      projectionTextcoordScalePresentFlag_.resize(subPatchCount, false);
      projectionTextcoordSubpatchScale_.resize(subPatchCount, 0);
  }
  void clearSubPatches() {
      projectionTextcoordProjectionId_.clear();
      projectionTextcoordOrientationId_.clear();
      projectionTextcoord2dPosX_.clear();
      projectionTextcoord2dPosY_.clear();
      projectionTextcoord2dSizeXMinus1_.clear();
      projectionTextcoord2dSizeYMinus1_.clear();
      projectionTextcoordScalePresentFlag_.clear();
      projectionTextcoordSubpatchScale_.clear();
  }

private:

  uint32_t              submeshId_ = 0;
  // orthoAtlas 
  double                projectionTextcoordFrameScale_ = 1.0;
  size_t                projectionTextcoordSubpatchCountMinus1_ = 0;
  std::vector<size_t>   projectionTextcoordProjectionId_;
  std::vector<size_t>   projectionTextcoordOrientationId_;
  std::vector<size_t>   projectionTextcoord2dPosX_;
  std::vector<size_t>   projectionTextcoord2dPosY_;
  std::vector<uint64_t> projectionTextcoord2dSizeXMinus1_;
  std::vector<uint64_t> projectionTextcoord2dSizeYMinus1_;
  std::vector<uint8_t>  projectionTextcoordScalePresentFlag_;
  std::vector<size_t>   projectionTextcoordSubpatchScale_;

  size_t   patchIndex_       = 0;
  size_t   frameIndex_       = 0;
  size_t   tileOrder_        = 0;
};

};  // namespace vmesh
