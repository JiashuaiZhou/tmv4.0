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

// Base mesh tile header syntax
class BaseMeshTileHeader {
public:
  BaseMeshTileHeader() {
    additionalAfocLsbPresentFlag_.resize(1, 0);
    additionalAfocLsbVal_.resize(1, 0);
  };
  ~BaseMeshTileHeader() {
    additionalAfocLsbPresentFlag_.clear();
    additionalAfocLsbVal_.clear();
  };
  BaseMeshTileHeader& operator=(const BaseMeshTileHeader&) = default;

  auto& getRefListStruct() const { return refListStruct_; }
  auto  getFrameIndex() const { return frameIndex_; }
  auto  getBaseMeshFrameParameterSetId() const {
    return baseMeshFrameParameterSetId_;
  }
  auto getBaseMeshAdaptationParameterSetId() const {
    return baseMeshAdaptationParameterSetId_;
  }
  auto getNoOutputOfPriorBaseMeshFramesFlag() const {
    return noOutputOfPriorBaseMeshFramesFlag_;
  }
  auto getBaseMeshId() const { return baseMeshId_; }
  auto getBaseMeshType() const { return baseMeshType_; }
  auto getBaseMeshOutputFlag() const { return baseMeshOutputFlag_; }

  auto getBaseMeshFrmOrderCntLsb() const { return baseMeshFrmOrderCntLsb_; }
  auto getRefBaseMeshFrameListSpsFlag() const {
    return refBaseMeshFrameListSpsFlag_;
  }
  auto getRefBaseMeshFrameListIdx() const { return refBaseMeshFrameListIdx_; }
  auto getNumRefIdxActiveOverrideFlag() const {
    return numRefIdxActiveOverrideFlag_;
  }
  auto  getNumRefIdxActiveMinus1() const { return numRefIdxActiveMinus1_; }
  auto& getAdditionalAfocLsbPresentFlag() const {
    return additionalAfocLsbPresentFlag_;
  }
  auto& getAdditionalAfocLsbVal() const { return additionalAfocLsbVal_; }
  auto  getAdditionalAfocLsbPresentFlag(size_t idx) const {
    return additionalAfocLsbPresentFlag_[idx];
  }
  auto getAdditionalAfocLsbVal(size_t idx) const {
    return additionalAfocLsbVal_[idx];
  }

  auto& getRefListStruct() { return refListStruct_; }
  auto& getFrameIndex() { return frameIndex_; }
  auto& getBaseMeshFrameParameterSetId() {
    return baseMeshFrameParameterSetId_;
  }
  auto& getBaseMeshAdaptationParameterSetId() {
    return baseMeshAdaptationParameterSetId_;
  }
  bool& getNoOutputOfPriorBaseMeshFramesFlag() {
    return noOutputOfPriorBaseMeshFramesFlag_;
  }
  auto& getBaseMeshId() { return baseMeshId_; }
  auto& getBaseMeshType() { return baseMeshType_; }
  auto& getBaseMeshOutputFlag() { return baseMeshOutputFlag_; }
  auto& getBaseMeshFrmOrderCntLsb() { return baseMeshFrmOrderCntLsb_; }
  auto& getRefBaseMeshFrameListSpsFlag() {
    return refBaseMeshFrameListSpsFlag_;
  }
  auto& getRefBaseMeshFrameListIdx() { return refBaseMeshFrameListIdx_; }
  auto& getNumRefIdxActiveOverrideFlag() {
    return numRefIdxActiveOverrideFlag_;
  }
  auto& getNumRefIdxActiveMinus1() { return numRefIdxActiveMinus1_; }
  auto& getAdditionalAfocLsbPresentFlag() {
    return additionalAfocLsbPresentFlag_;
  }
  auto& getAdditionalAfocLsbVal() { return additionalAfocLsbVal_; }
  auto& getAdditionalAfocLsbPresentFlag(size_t idx) {
    return additionalAfocLsbPresentFlag_[idx];
  }
  auto& getAdditionalAfocLsbVal(size_t idx) {
    return additionalAfocLsbVal_[idx];
  }

private:
  bool                 noOutputOfPriorBaseMeshFramesFlag_ = false;
  uint8_t              frameIndex_                        = 0;
  uint8_t              baseMeshFrameParameterSetId_       = 0;
  uint8_t              baseMeshAdaptationParameterSetId_  = 0;
  uint32_t             baseMeshId_                        = 0;
  BaseMeshType         baseMeshType_                      = I_BASEMESH;
  bool                 baseMeshOutputFlag_                = false;
  size_t               baseMeshFrmOrderCntLsb_            = 0;
  bool                 refBaseMeshFrameListSpsFlag_       = false;
  uint8_t              refBaseMeshFrameListIdx_           = 0;
  bool                 numRefIdxActiveOverrideFlag_       = false;
  uint8_t              numRefIdxActiveMinus1_             = 0;
  RefListStruct        refListStruct_;
  std::vector<uint8_t> additionalAfocLsbPresentFlag_;
  std::vector<uint8_t> additionalAfocLsbVal_;
};

};  // namespace vmesh
