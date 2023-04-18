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
#include "vuiParameters.hpp"
#include "refListStruct.hpp"
#include "plrInformation.hpp"
#include "vpccExtension.hpp"

namespace vmesh {

// General base mesh sequence parameter set Rbsp
class BaseMeshSequenceParameterSetRbsp {
public:
  BaseMeshSequenceParameterSetRbsp()  = default;
  ~BaseMeshSequenceParameterSetRbsp() = default;

  BaseMeshSequenceParameterSetRbsp&
  operator=(const BaseMeshSequenceParameterSetRbsp&) = default;

  void allocateRefListStruct() {
    refListStruct_.resize(numRefBaseMeshFrameListsInAsps_);
  }

  auto getBaseMeshSequenceParameterSetId() const {
    return baseMeshSequenceParameterSetId_;
  }
  auto getLog2MaxBaseMeshFrameOrderCntLsbMinus4() const {
    return log2MaxBaseMeshFrameOrderCntLsbMinus4_;
  }
  auto getMaxDecBaseMeshFrameBufferingMinus1() const {
    return maxDecBaseMeshFrameBufferingMinus1_;
  }
  auto getLongTermRefBaseMeshFramesFlag() const {
    return longTermRefBaseMeshFramesFlag_;
  }
  auto getNumRefBaseMeshFrameListsInAsps() const {
    return numRefBaseMeshFrameListsInAsps_;
  }
  auto getBaseMeshCodecId() const { return baseMeshCodecId_; }
  auto getQpPositionMinus1() const { return qpPositionMinus1_; }
  auto getQpTexCoordMinus1() const { return qpTexCoordMinus1_; }

  auto& getBaseMeshSequenceParameterSetId() {
    return baseMeshSequenceParameterSetId_;
  }
  auto& getLog2MaxBaseMeshFrameOrderCntLsbMinus4() {
    return log2MaxBaseMeshFrameOrderCntLsbMinus4_;
  }
  auto& getMaxDecBaseMeshFrameBufferingMinus1() {
    return maxDecBaseMeshFrameBufferingMinus1_;
  }
  auto& getLongTermRefBaseMeshFramesFlag() {
    return longTermRefBaseMeshFramesFlag_;
  }
  auto& getNumRefBaseMeshFrameListsInAsps() {
    return numRefBaseMeshFrameListsInAsps_;
  }
  auto& getBaseMeshCodecId() { return baseMeshCodecId_; }
  auto& getQpPositionMinus1() { return qpPositionMinus1_; }
  auto& getQpTexCoordMinus1() { return qpTexCoordMinus1_; }

  auto& getRefListStruct(uint8_t index) { return refListStruct_[index]; }
  void  addRefListStruct(RefListStruct value) {
    refListStruct_.push_back(value);
  }
  auto& addRefListStruct() {
    RefListStruct refListStruct;
    refListStruct_.push_back(refListStruct);
    return refListStruct_.back();
  }

private:
  uint8_t                    baseMeshSequenceParameterSetId_        = 0;
  uint8_t                    log2MaxBaseMeshFrameOrderCntLsbMinus4_ = 4;
  uint8_t                    maxDecBaseMeshFrameBufferingMinus1_    = 0;
  bool                       longTermRefBaseMeshFramesFlag_         = 0;
  uint8_t                    numRefBaseMeshFrameListsInAsps_        = 0;
  uint8_t                    baseMeshCodecId_                       = 0;
  uint8_t                    qpPositionMinus1_                      = 0;
  uint8_t                    qpTexCoordMinus1_                      = 0;
  std::vector<RefListStruct> refListStruct_;
};

};  // namespace vmesh
