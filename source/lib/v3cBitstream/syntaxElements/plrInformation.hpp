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

typedef struct PointLocalReconstructionMode {
  bool    interpolate_ = false;
  bool    filling_     = false;
  uint8_t minD1_       = 0;
  uint8_t neighbor_    = 0;
} PointLocalReconstructionMode;

// 8.3.6.1.2 Point local reconstruction information syntax
class PLRInformation {
public:
  PLRInformation() {
    minimumDepth_.clear();
    neighbourMinus1_.clear();
    interpolateFlag_.clear();
    fillingFlag_.clear();
  };
  ~PLRInformation() {
    minimumDepth_.clear();
    neighbourMinus1_.clear();
    interpolateFlag_.clear();
    fillingFlag_.clear();
  };

  PLRInformation& operator=(const PLRInformation&) = default;

  void allocate() {
    minimumDepth_.resize(numberOfModesMinus1_ + 1, 0);
    neighbourMinus1_.resize(numberOfModesMinus1_ + 1, 0);
    interpolateFlag_.resize(numberOfModesMinus1_ + 1, false);
    fillingFlag_.resize(numberOfModesMinus1_ + 1, false);
  }

  auto getMapEnabledFlag() const { return mapEnabledFlag_; }
  auto getNumberOfModesMinus1() const { return numberOfModesMinus1_; }
  auto getBlockThresholdPerPatchMinus1() const {
    return blockThresholdPerPatchMinus1_;
  }
  auto getMinimumDepth(size_t index) const { return minimumDepth_[index]; }
  auto getNeighbourMinus1(size_t index) const {
    return neighbourMinus1_[index];
  }
  auto getInterpolateFlag(size_t index) const {
    return interpolateFlag_[index];
  }
  auto getFillingFlag(size_t index) const { return fillingFlag_[index]; }

  auto& getMapEnabledFlag() { return mapEnabledFlag_; }
  auto& getNumberOfModesMinus1() { return numberOfModesMinus1_; }
  auto& getBlockThresholdPerPatchMinus1() {
    return blockThresholdPerPatchMinus1_;
  }
  auto& getMinimumDepth(size_t index) { return minimumDepth_[index]; }
  auto& getNeighbourMinus1(size_t index) { return neighbourMinus1_[index]; }
  auto& getInterpolateFlag(size_t index) { return interpolateFlag_[index]; }
  auto& getFillingFlag(size_t index) { return fillingFlag_[index]; }

private:
  bool                 mapEnabledFlag_               = false;
  uint8_t              numberOfModesMinus1_          = 0;
  uint8_t              blockThresholdPerPatchMinus1_ = 0;
  std::vector<uint8_t> interpolateFlag_;
  std::vector<uint8_t> fillingFlag_;
  std::vector<uint8_t> minimumDepth_;
  std::vector<uint8_t> neighbourMinus1_;
};

};  // namespace vmesh