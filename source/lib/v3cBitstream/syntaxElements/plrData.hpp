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

// 8.3.7.9 Point local reconstruction data syntax
class PLRData {
public:
  PLRData() {
    blockPresentFlag_.clear();
    blockModeMinus1_.clear();
  }
  ~PLRData() {
    blockPresentFlag_.clear();
    blockModeMinus1_.clear();
  }

  PLRData& operator=(const PLRData&) = default;

  void allocate(size_t blockToPatchMapWidth, size_t blockToPatchMapHeight) {
    blockToPatchMapWidth_  = blockToPatchMapWidth;
    blockToPatchMapHeight_ = blockToPatchMapHeight;
    blockPresentFlag_.resize(blockToPatchMapWidth * blockToPatchMapHeight, 0);
    blockModeMinus1_.resize(blockToPatchMapWidth * blockToPatchMapHeight, 0);
  }
  auto getBlockToPatchMapHeight() const { return blockToPatchMapHeight_; }
  auto getBlockToPatchMapWidth() const { return blockToPatchMapWidth_; }
  auto getLevelFlag() const { return levelFlag_; }
  auto getPresentFlag() const { return presentFlag_; }
  auto getModeMinus1() const { return modeMinus1_; }
  auto getBlockPresentFlag(size_t index) const {
    return blockPresentFlag_[index];
  }
  auto getBlockModeMinus1(size_t index) const {
    return blockModeMinus1_[index];
  }

  auto& getBlockToPatchMapHeight() { return blockToPatchMapHeight_; }
  auto& getBlockToPatchMapWidth() { return blockToPatchMapWidth_; }
  auto& getLevelFlag() { return levelFlag_; }
  auto& getPresentFlag() { return presentFlag_; }
  auto& getModeMinus1() { return modeMinus1_; }
  auto& getBlockPresentFlag(size_t index) { return blockPresentFlag_[index]; }
  auto& getBlockModeMinus1(size_t index) { return blockModeMinus1_[index]; }

private:
  size_t               blockToPatchMapHeight_ = 0;
  size_t               blockToPatchMapWidth_  = 0;
  bool                 levelFlag_             = 0;
  bool                 presentFlag_           = 0;
  uint8_t              modeMinus1_            = 0;
  std::vector<uint8_t> blockPresentFlag_;
  std::vector<uint8_t> blockModeMinus1_;
};

};  // namespace vmesh
