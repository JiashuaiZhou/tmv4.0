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
#include "plrData.hpp"

namespace vmesh {

// 8.3.7.3  Patch data unit syntax
class PatchDataUnit {
public:
  PatchDataUnit() {}
  ~PatchDataUnit() {}

  PatchDataUnit& operator=(const PatchDataUnit&) = default;

  auto  get2dPosX() const { return pos2dX_; }
  auto  get2dPosY() const { return pos2dY_; }
  auto  get2dSizeXMinus1() const { return size2dXMinus1_; }
  auto  get2dSizeYMinus1() const { return size2dYMinus1_; }
  auto  get3dOffsetU() const { return pos3dOffsetU_; }
  auto  get3dOffsetV() const { return pos3dOffsetV_; }
  auto  get3dOffsetD() const { return pos3dOffsetD_; }
  auto  get3dRangeD() const { return pos3dRangeD_; }
  auto  getProjectionId() const { return projectionId_; }
  auto  getOrientationIndex() const { return orientationIndex_; }
  auto  getLodEnableFlag() const { return lodEnableFlag_; }
  auto  getLodScaleXMinus1() const { return lodScaleXminus1_; }
  auto  getLodScaleYIdc() const { return lodScaleYIdc_; }
  auto  getTileOrder() const { return tileOrder_; }
  auto  getPatchIndex() const { return patchIndex_; }
  auto  getFrameIndex() const { return frameIndex_; }
  auto& getPLRData() const { return pointLocalReconstructionData_; }

  auto& get2dPosX() { return pos2dX_; }
  auto& get2dPosY() { return pos2dY_; }
  auto& get2dSizeXMinus1() { return size2dXMinus1_; }
  auto& get2dSizeYMinus1() { return size2dYMinus1_; }
  auto& get3dOffsetU() { return pos3dOffsetU_; }
  auto& get3dOffsetV() { return pos3dOffsetV_; }
  auto& get3dOffsetD() { return pos3dOffsetD_; }
  auto& get3dRangeD() { return pos3dRangeD_; }
  auto& getProjectionId() { return projectionId_; }
  auto& getOrientationIndex() { return orientationIndex_; }
  auto& getLodEnableFlag() { return lodEnableFlag_; }
  auto& getLodScaleXMinus1() { return lodScaleXminus1_; }
  auto& getLodScaleYIdc() { return lodScaleYIdc_; }
  auto& getTileOrder() { return tileOrder_; }
  auto& getPatchIndex() { return patchIndex_; }
  auto& getFrameIndex() { return frameIndex_; }
  auto& getPLRData() { return pointLocalReconstructionData_; }

private:
  size_t   pos2dX_           = 0;
  size_t   pos2dY_           = 0;
  uint64_t size2dXMinus1_    = 0;
  uint64_t size2dYMinus1_    = 0;
  size_t   pos3dOffsetU_     = 0;
  size_t   pos3dOffsetV_     = 0;
  size_t   pos3dOffsetD_     = 0;
  size_t   pos3dRangeD_      = 0;
  size_t   projectionId_     = 0;
  size_t   orientationIndex_ = 0;
  bool     lodEnableFlag_    = 0;
  uint8_t  lodScaleXminus1_  = 0;
  uint8_t  lodScaleYIdc_     = 0;
  size_t   patchIndex_       = 0;
  size_t   frameIndex_       = 0;
  size_t   tileOrder_        = 0;
  PLRData  pointLocalReconstructionData_;
};

};  // namespace vmesh
