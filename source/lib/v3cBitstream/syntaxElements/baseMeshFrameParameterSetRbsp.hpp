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

#include "baseMeshFrameTileInformation.hpp"

namespace vmesh {

// 8.3.6.2 BaseMesh frame parameter set Rbsp syntax
// 8.3.6.2.1 General baseMesh frame parameter set Rbsp syntax
class BaseMeshFrameParameterSetRbsp {
public:
  BaseMeshFrameParameterSetRbsp()  = default;
  ~BaseMeshFrameParameterSetRbsp() = default;

  BaseMeshFrameParameterSetRbsp&
  operator=(const BaseMeshFrameParameterSetRbsp&) = default;

  auto getBaseMeshFrameParameterSetId() const {
    return baseMeshFrameParameterSetId_;
  }
  auto getBaseMeshSequenceParameterSetId() const {
    return baseMeshSequenceParameterSetId_;
  }
  auto getOutputFlagPresentFlag() const { return outputFlagPresentFlag_; }
  auto getNumRefIdxDefaultActiveMinus1() const {
    return numRefIdxDefaultActiveMinus1_;
  }
  auto getAdditionalLtAfocLsbLen() const { return additionalLtAfocLsbLen_; }

  auto getMotionGroupSize() const { return motionGroupSize; }
  auto getExtensionFlag() const { return extensionFlag_; }
  auto getExtension8Bits() const { return extension8Bits_; }

  auto& getBaseMeshFrameParameterSetId() {
    return baseMeshFrameParameterSetId_;
  }
  auto& getBaseMeshSequenceParameterSetId() {
    return baseMeshSequenceParameterSetId_;
  }
  auto& getOutputFlagPresentFlag() { return outputFlagPresentFlag_; }
  auto& getNumRefIdxDefaultActiveMinus1() {
    return numRefIdxDefaultActiveMinus1_;
  }
  auto& getAdditionalLtAfocLsbLen() { return additionalLtAfocLsbLen_; }
  auto& getMotionGroupSize() { return motionGroupSize; }
  auto& getExtensionFlag() { return extensionFlag_; }
  auto& getExtension8Bits() { return extension8Bits_; }

  auto& getBaseMeshFrameTileInformation() const {
    return baseMeshFrameTileInformation_;
  }
  auto& getBaseMeshFrameTileInformation() {
    return baseMeshFrameTileInformation_;
  }

  void copyFrom(BaseMeshFrameParameterSetRbsp& ref) {
    baseMeshSequenceParameterSetId_ = ref.getBaseMeshSequenceParameterSetId();
    numRefIdxDefaultActiveMinus1_   = ref.getNumRefIdxDefaultActiveMinus1();
    additionalLtAfocLsbLen_         = ref.getAdditionalLtAfocLsbLen();
  }

private:
  uint8_t baseMeshFrameParameterSetId_    = 0;
  uint8_t baseMeshSequenceParameterSetId_ = 0;
  bool    outputFlagPresentFlag_          = false;
  uint8_t numRefIdxDefaultActiveMinus1_   = 0;
  uint8_t additionalLtAfocLsbLen_         = 0;
  uint8_t motionGroupSize                 = 16;
  bool    extensionFlag_                  = false;
  uint8_t extension8Bits_                 = 0;

  BaseMeshFrameTileInformation baseMeshFrameTileInformation_;
};

};  // namespace vmesh
