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
#include "videoBitstream.hpp"
#include "meshBitstream.hpp"
#include "bitstream.hpp"

#include "sei.hpp"
#include "v3cUnit.hpp"
#include "sampleStreamNalUnit.hpp"
#include "sampleStreamV3CUnit.hpp"
#include "atlasTileHeader.hpp"
#include "v3cParameterSet.hpp"
#include "v3cUnitHeader.hpp"
#include "atlasTileLayerRbsp.hpp"
#include "atlasSequenceParameterSetRbsp.hpp"
#include "atlasFrameParameterSetRbsp.hpp"
#include "atlasTileLayerRbsp.hpp"

namespace vmesh {

class BitstreamStat;

// used for syntax handling
class AtlasSubBitstream {
public:
  AtlasSubBitstream() {}
  ~AtlasSubBitstream() { videoBitstream_.clear(); }

  // video data related functions
  VideoBitstream& createVideoBitstream(VideoType type) {
    videoBitstream_.push_back(VideoBitstream(type));
    return videoBitstream_.back();
  }
  size_t getVideoBitstreamCount() { return videoBitstream_.size(); }

  // const auto& getDisplacement() const {
  //   return getVideoBitstream(VIDEO_GEOMETRY).vector();
  // }
  // const auto& getAttribute() const {
  //   return getVideoBitstream(VIDEO_ATTRIBUTE).vector();
  // }
  auto& getDisplacement() {
    return getVideoBitstream(VIDEO_GEOMETRY).vector();
  }
  auto& getAttribute() { return getVideoBitstream(VIDEO_ATTRIBUTE).vector(); }

  const auto& getDisplacement() const {
    return getVideoBitstream(VIDEO_GEOMETRY).vector();
  }
  const auto& getAttribute() const {
    return getVideoBitstream(VIDEO_ATTRIBUTE).vector();
  }

  VideoBitstream& getVideoBitstream(size_t index) {
    return videoBitstream_[index];
  }
  VideoBitstream& getVideoBitstream(VideoType type) {
    for (auto& value : videoBitstream_) {
      if (value.type() == type) { return value; }
    }
    printf("ERROR: can't get video bitstream of type %s \n",
           toString(type).c_str());
    fflush(stdout);
    assert(0);
    exit(-1);
  }
  const VideoBitstream& getVideoBitstream(VideoType type) const {
    for (const auto& value : videoBitstream_) {
      if (value.type() == type) { return value; }
    }
    printf("ERROR: can't get video bitstream of type %s \n",
           toString(type).c_str());
    fflush(stdout);
    assert(0);
    exit(-1);
  }

  inline void printVideoBitstream() const {
    size_t index = 0;
    printf("VideoBitstream list: \n");
    for (auto& videoBitstream : videoBitstream_) {
      printf("  * %zu / %zu: ", index, videoBitstream_.size());
      videoBitstream.trace();
      index++;
    }
    fflush(stdout);
  }

  // ASPS related functions
  AtlasSequenceParameterSetRbsp& getAtlasSequenceParameterSet(size_t setId) {
    return atlasSequenceParameterSet_[setId];
  }
  const AtlasSequenceParameterSetRbsp&
  getAtlasSequenceParameterSet(size_t setId) const {
    return atlasSequenceParameterSet_[setId];
  }
  AtlasSequenceParameterSetRbsp& addAtlasSequenceParameterSet() {
    AtlasSequenceParameterSetRbsp asps;
    asps.getAtlasSequenceParameterSetId() = atlasSequenceParameterSet_.size();
    atlasSequenceParameterSet_.push_back(asps);
    return atlasSequenceParameterSet_.back();
  }
  AtlasSequenceParameterSetRbsp& addAtlasSequenceParameterSet(uint8_t setId) {
    AtlasSequenceParameterSetRbsp asps;
    asps.getAtlasSequenceParameterSetId() = setId;
    if (atlasSequenceParameterSet_.size() < setId + 1) {
      atlasSequenceParameterSet_.resize(setId + 1);
    }
    atlasSequenceParameterSet_[setId] = asps;
    return atlasSequenceParameterSet_[setId];
  }
  std::vector<AtlasSequenceParameterSetRbsp>&
  getAtlasSequenceParameterSetList() {
    return atlasSequenceParameterSet_;
  }

  // reference list, defined in ASPS
  void setNumOfRefAtlasFrameList(size_t value) {
    refAtlasFrameList_.resize(value);
  }
  void setSizeOfRefAtlasFrameList(size_t listIndex, size_t listSize) {
    refAtlasFrameList_[listIndex].resize(listSize);
  }
  void setRefAtlasFrame(size_t listIndex, size_t refIndex, int32_t value) {
    refAtlasFrameList_[listIndex][refIndex] = value;
  }
  void setRefAtlasFrameList(std::vector<std::vector<int32_t>>& list) {
    size_t listSize = (std::min)(refAtlasFrameList_.size(), list.size());
    for (size_t i = 0; i < listSize; i++) {
      size_t refSize =
        (std::min)(refAtlasFrameList_[i].size(), list[i].size());
      for (size_t j = 0; j < refSize; j++)
        refAtlasFrameList_[i][j] = list[i][j];
    }
  }
  size_t getNumOfRefAtlasFrameList() { return refAtlasFrameList_.size(); }
  size_t getSizeOfRefAtlasFrameList(size_t listIndex) {
    return refAtlasFrameList_[listIndex].size();
  }
  int32_t getRefAtlasFrame(size_t listIndex, size_t refIndex) {
    return refAtlasFrameList_[listIndex][refIndex];
  }
  std::vector<int32_t>& getRefAtlasFrameList(size_t listIndex) {
    return refAtlasFrameList_[listIndex];
  }
  size_t getNumRefIdxActive(AtlasTileHeader& ath) {
    size_t afpsId          = ath.getAtlasFrameParameterSetId();
    auto&  afps            = getAtlasFrameParameterSet(afpsId);
    size_t numRefIdxActive = 0;
    if (ath.getType() == P_TILE || ath.getType() == SKIP_TILE) {
      if (ath.getNumRefIdxActiveOverrideFlag()) {
        numRefIdxActive = ath.getNumRefIdxActiveMinus1() + 1;
      } else {
        auto& asps =
          getAtlasSequenceParameterSet(afps.getAtlasSequenceParameterSetId());
        auto& refList =
          ath.getRefAtlasFrameListSpsFlag()
            ? asps.getRefListStruct(ath.getRefAtlasFrameListIdx())
            : ath.getRefListStruct();
        numRefIdxActive = static_cast<size_t>((std::min)(
          static_cast<int>(refList.getNumRefEntries()),
          static_cast<int>(afps.getNumRefIdxDefaultActiveMinus1()) + 1));
      }
    }
    return numRefIdxActive;
  }
  size_t getMaxNumRefAtlasFrame(size_t listIndex) {
    return maxNumRefAtlasFrame_;
  }
  void setMaxNumRefAtlasFrame(size_t value) { maxNumRefAtlasFrame_ = value; }

  // point local recosntruction, defined in ASPS
  void
  addPointLocalReconstructionMode(const PointLocalReconstructionMode& mode) {
    pointLocalReconstructionMode_.push_back(mode);
  }
  PointLocalReconstructionMode& getPointLocalReconstructionMode(size_t index) {
    return pointLocalReconstructionMode_[index];
  }
  size_t getPointLocalReconstructionModeNumber() {
    return pointLocalReconstructionMode_.size();
  }

  // MeshSequenceDataUnit& getMeshSequenceDataUnit(size_t setId) {
  //   return meshSequenceDataUnit_[setId];
  // }
  // const MeshSequenceDataUnit& getMeshSequenceDataUnit(size_t setId) const {
  //   return meshSequenceDataUnit_[setId];
  // }

  // MeshSequenceDataUnit& addMeshSequenceDataUnit() {
  //   MeshSequenceDataUnit msdu;
  //   meshSequenceDataUnit_.push_back(msdu);
  //   return meshSequenceDataUnit_.back();
  // }

  // AFPS related functions
  AtlasFrameParameterSetRbsp& getAtlasFrameParameterSet(size_t setId) {
    return atlasFrameParameterSet_[setId];
  }
  const AtlasFrameParameterSetRbsp&
  getAtlasFrameParameterSet(size_t setId) const {
    return atlasFrameParameterSet_[setId];
  }
  std::vector<AtlasFrameParameterSetRbsp>& getAtlasFrameParameterSetList() {
    return atlasFrameParameterSet_;
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet() {
    AtlasFrameParameterSetRbsp afps;
    afps.getAtlasFrameParameterSetId() = atlasFrameParameterSet_.size();
    atlasFrameParameterSet_.push_back(afps);
    return atlasFrameParameterSet_.back();
  }
  AtlasFrameParameterSetRbsp&
  addAtlasFrameParameterSet(AtlasFrameParameterSetRbsp& refAfps) {
    size_t                     setId = atlasFrameParameterSet_.size();
    AtlasFrameParameterSetRbsp afps;
    afps.copyFrom(refAfps);
    afps.getAtlasFrameParameterSetId() = setId;
    atlasFrameParameterSet_.push_back(afps);
    return atlasFrameParameterSet_.back();
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet(uint8_t setId) {
    AtlasFrameParameterSetRbsp afps;
    afps.getAtlasFrameParameterSetId() = setId;
    if (atlasFrameParameterSet_.size() < setId + 1) {
      atlasFrameParameterSet_.resize(setId + 1);
    }
    atlasFrameParameterSet_[setId] = afps;
    return atlasFrameParameterSet_[setId];
  }

  // ATGL related functions
  AtlasTileLayerRbsp& addAtlasTileLayer() {
    AtlasTileLayerRbsp atgl;
    atgl.getTileOrder()               = atlasTileLayer_.size();
    atgl.getDataUnit().getTileOrder() = atlasTileLayer_.size();
    atgl.getEncFrameIndex()           = (std::numeric_limits<size_t>::max)();
    atgl.getEncTileIndex()            = (std::numeric_limits<size_t>::max)();
    atlasTileLayer_.push_back(atgl);
    return atlasTileLayer_.back();
  }
  AtlasTileLayerRbsp& addAtlasTileLayer(
    size_t frameIdx,
    size_t
      tileIdx) {  // ajt::how is tileIdx is used, should it also do setTileOrder(tileIdx)?
    AtlasTileLayerRbsp atgl;
    atgl.getEncFrameIndex()       = frameIdx;
    atgl.getEncTileIndex()        = tileIdx;
    atgl.getAtlasFrmOrderCntVal() = frameIdx;
    atlasTileLayer_.push_back(atgl);
    return atlasTileLayer_.back();
  }
  void traceAtlasTileLayer() {
    printf("traceAtlasTileLayer: atlasTileLayer_.size() = %zu \n",
           atlasTileLayer_.size());
    for (size_t atglIndex = 0; atglIndex < atlasTileLayer_.size();
         atglIndex++) {
      printf("  atgl %3zu: EncFrameIndex = %zu EncTileIndex = %zu \n",
             atglIndex,
             atlasTileLayer_[atglIndex].getEncFrameIndex(),
             atlasTileLayer_[atglIndex].getEncTileIndex());
    }
    fflush(stdout);
  }
  size_t getAtlasTileLayerIndex(size_t frameIndex, size_t tileIndex) {
    for (size_t atglIndex = 0; atglIndex < atlasTileLayer_.size();
         atglIndex++) {
      if (atlasTileLayer_[atglIndex].getEncFrameIndex() == frameIndex
          && atlasTileLayer_[atglIndex].getEncTileIndex() == tileIndex) {
        return atglIndex;
      }
    }
    return 0;
  }
  std::vector<AtlasTileLayerRbsp>& getAtlasTileLayerList() {
    return atlasTileLayer_;
  }
  AtlasTileLayerRbsp& getAtlasTileLayer(size_t atglIndex) {
    return atlasTileLayer_[atglIndex];
  }
  AtlasTileLayerRbsp& getAtlasTileLayer(size_t frameIndex, size_t tileIndex) {
    return atlasTileLayer_[getAtlasTileLayerIndex(frameIndex, tileIndex)];
  }
  const AtlasTileLayerRbsp& getAtlasTileLayer(size_t atglIndex) const {
      return atlasTileLayer_[atglIndex];
  }
  size_t getFrameCount(const AtlasSequenceParameterSetRbsp& asps,
                       const AtlasFrameParameterSetRbsp&    afps) const {
    size_t frameCount          = 0;
    size_t atlasFrmOrderCntMsb = 0;
    size_t atlasFrmOrderCntVal = 0;
    for (size_t i = 0; i < atlasTileLayer_.size(); i++) {
      size_t afocVal = calculateAFOCval(
        asps, afps, i, atlasFrmOrderCntMsb, atlasFrmOrderCntVal);
      frameCount = std::max(frameCount, (afocVal + 1));
    }
    return frameCount;
  }

  // 8.2.3.1 Atlas frame order count derivation process
  size_t calculateAFOCval(const AtlasSequenceParameterSetRbsp& asps,
                          const AtlasFrameParameterSetRbsp&    afps,
                          size_t                               atglIndex,
                          size_t& atlasFrmOrderCntMsb,
                          size_t& atlasFrmOrderCntVal) const {
    auto& atgh = atlasTileLayer_[atglIndex].getHeader();
    if (atglIndex == 0) {
      // atlasTileLayer_[atglIndex].setAtlasFrmOrderCntMsb(0);
      // atlasTileLayer_[atglIndex].setAtlasFrmOrderCntVal(
      //   atlh.getAtlasFrmOrderCntLsb());
      return atgh.getAtlasFrmOrderCntLsb();
    }

    size_t prevAtlasFrmOrderCntMsb = atlasFrmOrderCntMsb;
    size_t newAtlasFrmOrderCntMsb  = 0;
    // auto& afps = getAtlasFrameParameterSet(atgh.getAtlasFrameParameterSetId());
    // auto& asps =
    //  getAtlasSequenceParameterSet(afps.getAtlasSequenceParameterSetId());
    size_t maxAtlasFrmOrderCntLsb =
      size_t(1) << (asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4);
    size_t afocLsb = atgh.getAtlasFrmOrderCntLsb();
    size_t prevAtlasFrmOrderCntLsb =
      atlasTileLayer_[atglIndex - 1].getHeader().getAtlasFrmOrderCntLsb();
    if ((afocLsb < prevAtlasFrmOrderCntLsb)
        && ((prevAtlasFrmOrderCntLsb - afocLsb)
            >= (maxAtlasFrmOrderCntLsb / 2))) {
      newAtlasFrmOrderCntMsb =
        prevAtlasFrmOrderCntMsb + maxAtlasFrmOrderCntLsb;
    } else if ((afocLsb > prevAtlasFrmOrderCntLsb)
               && ((afocLsb - prevAtlasFrmOrderCntLsb)
                   > (maxAtlasFrmOrderCntLsb / 2))) {
      newAtlasFrmOrderCntMsb =
        prevAtlasFrmOrderCntMsb - maxAtlasFrmOrderCntLsb;
    } else {
      newAtlasFrmOrderCntMsb = prevAtlasFrmOrderCntMsb;
    }
    atlasFrmOrderCntMsb = newAtlasFrmOrderCntMsb;
    atlasFrmOrderCntVal = newAtlasFrmOrderCntMsb + afocLsb;
    return atlasFrmOrderCntMsb + afocLsb;
  }

private:
  size_t                                     maxNumRefAtlasFrame_;
  std::vector<VideoBitstream>                videoBitstream_;
  std::vector<AtlasSequenceParameterSetRbsp> atlasSequenceParameterSet_;
  std::vector<std::vector<int32_t>>          refAtlasFrameList_;
  std::vector<PointLocalReconstructionMode>  pointLocalReconstructionMode_;
  std::vector<AtlasFrameParameterSetRbsp>    atlasFrameParameterSet_;
  std::vector<AtlasTileLayerRbsp>            atlasTileLayer_;
  std::vector<int32_t>                       refAFOCList_;
};

};  // namespace vmesh
