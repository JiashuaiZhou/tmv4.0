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

// 8.3.7.7  raw patch data unit syntax
class RawPatchDataUnit {
public:
  RawPatchDataUnit() {}
  ~RawPatchDataUnit() {}
  RawPatchDataUnit& operator=(const RawPatchDataUnit&) = default;

  auto getPatchInAuxiliaryVideoFlag() const {
    return patchInAuxiliaryVideoFlag_;
  }
  auto get2dPosX() const { return pos2dX_; }
  auto get2dPosY() const { return pos2dY_; }
  auto get2dSizeXMinus1() const { return size2dXMinus1_; }
  auto get2dSizeYMinus1() const { return size2dYMinus1_; }
  auto get3dOffsetU() const { return pos3dOffsetU_; }
  auto get3dOffsetV() const { return pos3dOffsetV_; }
  auto get3dOffsetD() const { return pos3dOffsetD_; }
  auto getRawPointsMinus1() const { return rawPointsMinus1_; }
  auto getPatchIndex() const { return patchIndex_; }
  auto getFrameIndex() const { return frameIndex_; }
  auto getTileOrder() const { return tileOrder_; }

  auto& getPatchInAuxiliaryVideoFlag() { return patchInAuxiliaryVideoFlag_; }
  auto& get2dPosX() { return pos2dX_; }
  auto& get2dPosY() { return pos2dY_; }
  auto& get2dSizeXMinus1() { return size2dXMinus1_; }
  auto& get2dSizeYMinus1() { return size2dYMinus1_; }
  auto& get3dOffsetU() { return pos3dOffsetU_; }
  auto& get3dOffsetV() { return pos3dOffsetV_; }
  auto& get3dOffsetD() { return pos3dOffsetD_; }
  auto& getRawPointsMinus1() { return rawPointsMinus1_; }
  auto& getPatchIndex() { return patchIndex_; }
  auto& getFrameIndex() { return frameIndex_; }
  auto& getTileOrder() { return tileOrder_; }

private:
  bool     patchInAuxiliaryVideoFlag_ = false;
  size_t   pos2dX_                    = 0;
  size_t   pos2dY_                    = 0;
  uint64_t size2dXMinus1_             = 0;
  uint64_t size2dYMinus1_             = 0;
  size_t   pos3dOffsetU_              = 0;
  size_t   pos3dOffsetV_              = 0;
  size_t   pos3dOffsetD_              = 0;
  uint32_t rawPointsMinus1_           = 0;
  size_t   patchIndex_                = 0;
  size_t   frameIndex_                = 0;
  size_t   tileOrder_                 = 0;
};

};  // namespace vmesh
