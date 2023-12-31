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
#include "refListStruct.hpp"

namespace vmesh {

// 8.3.6.11  Atlas tile header syntax
class AtlasTileHeader {
public:
  AtlasTileHeader() {
    additionalAfocLsbPresentFlag_.resize(1, 0);
    additionalAfocLsbVal_.resize(1, 0);
  };
  ~AtlasTileHeader() {
    additionalAfocLsbPresentFlag_.clear();
    additionalAfocLsbVal_.clear();
  };
  AtlasTileHeader& operator=(const AtlasTileHeader&) = default;

  auto& getRefListStruct() { return refListStruct_; }
  auto  getFrameIndex() const { return frameIndex_; }
  auto  getNoOutputOfPriorAtlasFramesFlag() const {
    return noOutputOfPriorAtlasFramesFlag_;
  }
  auto getAtlasFrameParameterSetId() const {
    return atlasFrameParameterSetId_;
  }
  auto getAtlasAdaptationParameterSetId() const {
    return atlasAdaptationParameterSetId_;
  }
  auto getId() const { return id_; }
  auto getType() const { return type_; }
  auto getAtlasOutputFlag() const { return atlasOutputFlag_; }
  auto getAtlasFrmOrderCntLsb() const { return atlasFrmOrderCntLsb_; }
  auto getRefAtlasFrameListSpsFlag() const {
    return refAtlasFrameListSpsFlag_;
  }
  auto getRefAtlasFrameListIdx() const { return refAtlasFrameListIdx_; }
  auto getPosMinDQuantizer() const { return posMinDQuantizer_; }
  auto getPosDeltaMaxDQuantizer() const { return posDeltaMaxDQuantizer_; }
  auto getPatchSizeXinfoQuantizer() const { return patchSizeXinfoQuantizer_; }
  auto getPatchSizeYinfoQuantizer() const { return patchSizeYinfoQuantizer_; }
  auto getRaw3dOffsetAxisBitCountMinus1() const {
    return raw3dOffsetAxisBitCountMinus1_;
  }
  bool getNumRefIdxActiveOverrideFlag() const {
    return numRefIdxActiveOverrideFlag_;
  }
  auto  getNumRefIdxActiveMinus1() const { return numRefIdxActiveMinus1_; }
  auto& getAdditionalAfocLsbPresentFlag() {
    return additionalAfocLsbPresentFlag_;
  }
  auto& getAdditionalAfocLsbVal() const { return additionalAfocLsbVal_; }
  auto  getAdditionalAfocLsbPresentFlag(size_t idx) const {
    return additionalAfocLsbPresentFlag_[idx];
  }
  auto getAdditionalAfocLsbVal(size_t idx) const {
    return additionalAfocLsbVal_[idx];
  }
  auto getTileNaluTypeInfo() const { return tileNaluTypeInfo_; }

  auto& getFrameIndex() { return frameIndex_; }
  auto& getNoOutputOfPriorAtlasFramesFlag() {
    return noOutputOfPriorAtlasFramesFlag_;
  }
  auto& getAtlasFrameParameterSetId() { return atlasFrameParameterSetId_; }
  auto& getAtlasAdaptationParameterSetId() {
    return atlasAdaptationParameterSetId_;
  }
  auto& getId() { return id_; }
  auto& getType() { return type_; }
  auto& getAtlasOutputFlag() { return atlasOutputFlag_; }
  auto& getAtlasFrmOrderCntLsb() { return atlasFrmOrderCntLsb_; }
  bool& getRefAtlasFrameListSpsFlag() { return refAtlasFrameListSpsFlag_; }
  auto& getRefAtlasFrameListIdx() { return refAtlasFrameListIdx_; }
  auto& getPosMinDQuantizer() { return posMinDQuantizer_; }
  auto& getPosDeltaMaxDQuantizer() { return posDeltaMaxDQuantizer_; }
  auto& getPatchSizeXinfoQuantizer() { return patchSizeXinfoQuantizer_; }
  auto& getPatchSizeYinfoQuantizer() { return patchSizeYinfoQuantizer_; }
  auto& getRaw3dOffsetAxisBitCountMinus1() {
    return raw3dOffsetAxisBitCountMinus1_;
  }
  bool& getNumRefIdxActiveOverrideFlag() {
    return numRefIdxActiveOverrideFlag_;
  }
  auto& getNumRefIdxActiveMinus1() { return numRefIdxActiveMinus1_; }
  auto& getAdditionalAfocLsbPresentFlag(size_t idx) {
    return additionalAfocLsbPresentFlag_[idx];
  }
  auto& getAdditionalAfocLsbVal(size_t idx) {
    return additionalAfocLsbVal_[idx];
  }
  auto& getTileNaluTypeInfo() { return tileNaluTypeInfo_; }

private:
  bool                 noOutputOfPriorAtlasFramesFlag_ = false;
  uint8_t              frameIndex_                     = 0;
  uint8_t              atlasFrameParameterSetId_       = 0;
  uint8_t              atlasAdaptationParameterSetId_  = 0;
  uint32_t             id_                             = 0;
  TileType             type_                           = TileType(0);
  bool                 atlasOutputFlag_                = false;
  size_t               atlasFrmOrderCntLsb_            = 0;
  bool                 refAtlasFrameListSpsFlag_       = false;
  uint8_t              refAtlasFrameListIdx_           = 0;
  std::vector<uint8_t> additionalAfocLsbPresentFlag_;
  std::vector<uint8_t> additionalAfocLsbVal_;
  uint8_t              posMinDQuantizer_              = 0;
  uint8_t              posDeltaMaxDQuantizer_         = 0;
  uint8_t              patchSizeXinfoQuantizer_       = 0;
  uint8_t              patchSizeYinfoQuantizer_       = 0;
  uint8_t              raw3dOffsetAxisBitCountMinus1_ = 0;
  bool                 numRefIdxActiveOverrideFlag_   = false;
  uint8_t              numRefIdxActiveMinus1_         = 0;
  RefListStruct        refListStruct_;
  uint8_t              tileNaluTypeInfo_;
};

};  // namespace vmesh
