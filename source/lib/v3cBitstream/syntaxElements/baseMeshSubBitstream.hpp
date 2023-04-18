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
#include "meshBitstream.hpp"

#include "baseMeshSequenceParameterSetRbsp.hpp"
#include "baseMeshFrameParameterSetRbsp.hpp"
#include "baseMeshTileLayer.hpp"

namespace vmesh {

class BitstreamStat;

// used for syntax handling
class BaseMeshSubBitstream {
public:
  BaseMeshSubBitstream() {}
  ~BaseMeshSubBitstream() {}

  // BMSPS related functions
  auto& getBaseMeshSequenceParameterSet(size_t setId) {
    return baseMeshSequenceParameterSet_[setId];
  }
  auto& getBaseMeshSequenceParameterSet(size_t setId) const {
    return baseMeshSequenceParameterSet_[setId];
  }
  auto& addBaseMeshSequenceParameterSet() {
    BaseMeshSequenceParameterSetRbsp asps;
    asps.getBaseMeshSequenceParameterSetId() =
      baseMeshSequenceParameterSet_.size();
    baseMeshSequenceParameterSet_.push_back(asps);
    return baseMeshSequenceParameterSet_.back();
  }
  auto& addBaseMeshSequenceParameterSet(uint8_t setId) {
    BaseMeshSequenceParameterSetRbsp asps;
    asps.getBaseMeshSequenceParameterSetId() = setId;
    if (baseMeshSequenceParameterSet_.size() < setId + 1) {
      baseMeshSequenceParameterSet_.resize(setId + 1);
    }
    baseMeshSequenceParameterSet_[setId] = asps;
    return baseMeshSequenceParameterSet_[setId];
  }
  auto& getBaseMeshSequenceParameterSetList() {
    return baseMeshSequenceParameterSet_;
  }

  // reference list, defined in BMSPS
  void setNumOfRefBaseMeshFrameList(size_t value) {
    refBaseMeshFrameList_.resize(value);
  }
  void setSizeOfRefBaseMeshFrameList(size_t listIndex, size_t listSize) {
    refBaseMeshFrameList_[listIndex].resize(listSize);
  }
  void setRefBaseMeshFrame(size_t listIndex, size_t refIndex, int32_t value) {
    refBaseMeshFrameList_[listIndex][refIndex] = value;
  }
  void setRefBaseMeshFrameList(std::vector<std::vector<int32_t>>& list) {
    size_t listSize = (std::min)(refBaseMeshFrameList_.size(), list.size());
    for (size_t i = 0; i < listSize; i++) {
      size_t refSize =
        (std::min)(refBaseMeshFrameList_[i].size(), list[i].size());
      for (size_t j = 0; j < refSize; j++)
        refBaseMeshFrameList_[i][j] = list[i][j];
    }
  }
  auto getNumOfRefBaseMeshFrameList() { return refBaseMeshFrameList_.size(); }
  auto getSizeOfRefBaseMeshFrameList(size_t listIndex) {
    return refBaseMeshFrameList_[listIndex].size();
  }
  auto getRefBaseMeshFrame(size_t listIndex, size_t refIndex) {
    return refBaseMeshFrameList_[listIndex][refIndex];
  }
  auto& getRefBaseMeshFrameList(size_t listIndex) {
    return refBaseMeshFrameList_[listIndex];
  }
  auto getMaxNumRefBaseMeshFrame(size_t listIndex) {
    return maxNumRefBaseMeshFrame_;
  }

  // BMFPS related functions
  auto& getBaseMeshFrameParameterSet(size_t setId) {
    return baseMeshFrameParameterSet_[setId];
  }
  auto& getBaseMeshFrameParameterSet(size_t setId) const {
    return baseMeshFrameParameterSet_[setId];
  }
  auto& getBaseMeshFrameParameterSetList() {
    return baseMeshFrameParameterSet_;
  }
  auto& getBaseMeshFrameParameterSetList() const {
    return baseMeshFrameParameterSet_;
  }
  auto& addBaseMeshFrameParameterSet() {
    BaseMeshFrameParameterSetRbsp afps;
    afps.getBaseMeshFrameParameterSetId() = baseMeshFrameParameterSet_.size();
    baseMeshFrameParameterSet_.push_back(afps);
    return baseMeshFrameParameterSet_.back();
  }
  auto& addBaseMeshFrameParameterSet(BaseMeshFrameParameterSetRbsp& refAfps) {
    size_t                        setId = baseMeshFrameParameterSet_.size();
    BaseMeshFrameParameterSetRbsp afps;
    afps.copyFrom(refAfps);
    afps.getBaseMeshFrameParameterSetId() = setId;
    baseMeshFrameParameterSet_.push_back(afps);
    return baseMeshFrameParameterSet_.back();
  }
  auto& addBaseMeshFrameParameterSet(uint8_t setId) {
    BaseMeshFrameParameterSetRbsp afps;
    afps.getBaseMeshFrameParameterSetId() = setId;
    if (baseMeshFrameParameterSet_.size() < setId + 1) {
      baseMeshFrameParameterSet_.resize(setId + 1);
    }
    baseMeshFrameParameterSet_[setId] = afps;
    return baseMeshFrameParameterSet_[setId];
  }

  // Mesh data
  auto& getBaseMeshTileLayerList() { return baseMeshTileLayer_; }

  BaseMeshTileLayer& getBaseMeshTileLayer(size_t index) {
    return baseMeshTileLayer_[index];
  }
  const BaseMeshTileLayer& getBaseMeshTileLayer(size_t index) const {
    return baseMeshTileLayer_[index];
  }
  auto& addBaseMeshTileLayer() {
    BaseMeshTileLayer bmtl;
    baseMeshTileLayer_.push_back(bmtl);
    return baseMeshTileLayer_.back();
  }

private:
  std::vector<BaseMeshSequenceParameterSetRbsp> baseMeshSequenceParameterSet_;
  std::vector<std::vector<int32_t>>             refBaseMeshFrameList_;
  size_t                                        maxNumRefBaseMeshFrame_ = 0;
  std::vector<BaseMeshFrameParameterSetRbsp>    baseMeshFrameParameterSet_;
  std::vector<BaseMeshTileLayer>                baseMeshTileLayer_;
};

};  // namespace vmesh
