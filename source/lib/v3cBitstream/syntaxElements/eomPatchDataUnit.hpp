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

// 8.3.7.8  EOM patch data unit syntax
class EOMPatchDataUnit {
public:
  EOMPatchDataUnit() {
    associatedPatchesIdx_.clear();
    points_.clear();
  };
  ~EOMPatchDataUnit(){};
  EOMPatchDataUnit& operator=(const EOMPatchDataUnit&) = default;

  auto getPatchInAuxiliaryVideoFlag() const {
    return patchInAuxiliaryVideoFlag_;
  }
  auto get2dPosX() const { return pos2dX_; }
  auto get2dPosY() const { return pos2dY_; }
  auto get2dSizeXMinus1() const { return size2dXMinus1_; }
  auto get2dSizeYMinus1() const { return size2dYMinus1_; }
  auto getPatchCountMinus1() const { return patchCountMinus1_; }
  auto getAssociatedPatchesIdx(size_t index) const {
    return associatedPatchesIdx_[index];
  }
  auto getPoints(size_t index) const { return points_[index]; }
  auto getPatchIndex() const { return patchIndex_; }
  auto getFrameIndex() const { return frameIndex_; }
  auto getTileIndex() const { return tileIndex_; }

  auto& getPatchInAuxiliaryVideoFlag() { return patchInAuxiliaryVideoFlag_; }
  auto& get2dPosX() { return pos2dX_; }
  auto& get2dPosY() { return pos2dY_; }
  auto& get2dSizeXMinus1() { return size2dXMinus1_; }
  auto& get2dSizeYMinus1() { return size2dYMinus1_; }
  auto& getPatchCountMinus1() { return patchCountMinus1_; }
  auto& getAssociatedPatchesIdx(size_t index) {
    return associatedPatchesIdx_[index];
  }
  auto& getPoints(size_t index) { return points_[index]; }
  auto& getPatchIndex() { return patchIndex_; }
  auto& getFrameIndex() { return frameIndex_; }
  auto& getTileIndex() { return tileIndex_; }

  void allocPatchCountMinus1(uint32_t value) {
    patchCountMinus1_ = value;
    associatedPatchesIdx_.resize(patchCountMinus1_ + 1);
    points_.resize(patchCountMinus1_ + 1);
  }

private:
  bool                patchInAuxiliaryVideoFlag_ = 0;
  size_t              pos2dX_                    = 0;
  size_t              pos2dY_                    = 0;
  size_t              size2dXMinus1_             = 0;
  size_t              size2dYMinus1_             = 0;
  size_t              patchCountMinus1_          = 0;
  size_t              patchIndex_                = 0;
  size_t              frameIndex_                = 0;
  size_t              tileIndex_                 = 0;
  std::vector<size_t> associatedPatchesIdx_;
  std::vector<size_t> points_;
};

};  // namespace vmesh
