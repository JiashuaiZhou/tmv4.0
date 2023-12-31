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

// 8.3.6.2.2 BaseMesh frame tile information syntax
class BaseMeshFrameTileInformation {
public:
  BaseMeshFrameTileInformation(){};
  ~BaseMeshFrameTileInformation(){};

  BaseMeshFrameTileInformation&
       operator=(const BaseMeshFrameTileInformation&) = default;
  bool operator==(const BaseMeshFrameTileInformation& other) const {
    if (singleTileInBaseMeshFrameFlag_ != other.singleTileInBaseMeshFrameFlag_)
      return false;
    if (singlePartitionPerTileFlag_ != other.singlePartitionPerTileFlag_)
      return false;
    if (numTilesInBaseMeshFrameMinus1_ != other.numTilesInBaseMeshFrameMinus1_)
      return false;
    if (signalledTileIdFlag_ != other.signalledTileIdFlag_) return false;
    if (signalledTileIdLengthMinus1_ != other.signalledTileIdLengthMinus1_)
      return false;
    if (tileId_.size() != other.tileId_.size()) return false;
    for (size_t i = 0; i <= numTilesInBaseMeshFrameMinus1_; i++)
      if (tileId_[i] != other.tileId_[i]) return false;
    return true;
  }

  auto getSingleTileInBaseMeshFrameFlag() const {
    return singleTileInBaseMeshFrameFlag_;
  }
  auto getNumTilesInBaseMeshFrameMinus1() const {
    return numTilesInBaseMeshFrameMinus1_;
  }
  auto getSignalledTileIdFlag() const { return signalledTileIdFlag_; }
  auto getSignalledTileIdLengthMinus1() const {
    return signalledTileIdLengthMinus1_;
  }
  auto getTileId(size_t index) const { return tileId_[index]; }

  auto& getSingleTileInBaseMeshFrameFlag() {
    return singleTileInBaseMeshFrameFlag_;
  }
  auto& getNumTilesInBaseMeshFrameMinus1() {
    return numTilesInBaseMeshFrameMinus1_;
  }
  auto& getSinglePartitionPerTileFlag() { return singlePartitionPerTileFlag_; }
  auto& getSignalledTileIdFlag() { return signalledTileIdFlag_; }
  auto& getSignalledTileIdLengthMinus1() {
    return signalledTileIdLengthMinus1_;
  }
  auto& getTileId(size_t index) {
    if (index <= tileId_.size()) tileId_.resize(tileId_.size() + 1);
    return tileId_[index];
  }

private:
  bool                  singleTileInBaseMeshFrameFlag_ = false;
  uint32_t              singlePartitionPerTileFlag_    = 0;
  uint32_t              numTilesInBaseMeshFrameMinus1_ = 0;
  bool                  signalledTileIdFlag_           = false;
  uint32_t              signalledTileIdLengthMinus1_   = 0;
  std::vector<uint32_t> tileId_;
};

};  // namespace vmesh
