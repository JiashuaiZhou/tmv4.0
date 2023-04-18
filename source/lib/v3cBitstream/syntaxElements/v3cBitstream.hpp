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
#include "atlasSubBitstream.hpp"
#include "baseMeshSubBitstream.hpp"

namespace vmesh {

class BitstreamStat;

class V3cBitstream {
public:
  V3cBitstream() {}
  ~V3cBitstream() {}

  // bitstream statistic related functions
  void setBitstreamStat(BitstreamStat& bitstreamStat) {
    bitstreamStat_ = &bitstreamStat;
  }
  BitstreamStat& getBitstreamStat() { return *bitstreamStat_; }

  auto& getV3CUnitHeader(V3CUnitType type) {
    return v3cUnitHeader_[size_t(type)];
  }

  // VPS related functions
  V3CParameterSet&       getVps() { return getVps(activeVPS_); }
  const V3CParameterSet& getVps() const { return getVps(activeVPS_); }
  V3CParameterSet&       getVps(uint8_t vpsId) {
    for (auto& vps : vpccParameterSets_)
      if (vps.getV3CParameterSetId() == vpsId) return vps;
    printf("ERROR: can't get vps with id %d \n", vpsId);
    fflush(stdout);
    exit(-1);
  }
  const V3CParameterSet& getVps(uint8_t vpsId) const {
    for (auto& vps : vpccParameterSets_)
      if (vps.getV3CParameterSetId() == vpsId) return vps;
    printf("ERROR: can't get vps with id %d \n", vpsId);
    fflush(stdout);
    exit(-1);
  }
  auto& getVpsList() { return vpccParameterSets_; }
  auto& getActiveVpsId() { return activeVPS_; }
  auto& addV3CParameterSet(uint8_t index) {
    V3CParameterSet sps;
    sps.getV3CParameterSetId() = index;
    vpccParameterSets_.push_back(sps);
    return vpccParameterSets_.back();
  }
  auto& addV3CParameterSet() {
    return addV3CParameterSet(vpccParameterSets_.size());
  }

  void allocateAtlas(size_t size) {
    atlasHLS_.resize(size);
    atlasIndex_ = 0;
  }
  void allocateBaseMesh(size_t size) {
    baseMeshHLS_.resize(size);
    baseMeshIndex_ = 0;
  }

  auto& getAtlasIndex() const { return atlasIndex_; }
  auto& getBaseMeshIndex() const { return baseMeshIndex_; }
  auto& getAtlas() const { return atlasHLS_[atlasIndex_]; }
  auto& getBaseMesh() const { return baseMeshHLS_[baseMeshIndex_]; }
  auto  getOccupancyPrecision() const { return occupancyPrecision_; }
  auto  getLog2PatchQuantizerSizeX() const { return log2PatchQuantizerSizeX_; }
  auto  getLog2PatchQuantizerSizeY() const { return log2PatchQuantizerSizeY_; }
  auto  getEnablePatchSizeQuantization() const {
    return enablePatchSizeQuantization_;
  }
  auto getPrefilterLossyOM() const { return prefilterLossyOM_; }
  auto getOffsetLossyOM() const { return offsetLossyOM_; }
  auto getGeometry3dCoordinatesBitdepth() const {
    return geometry3dCoordinatesBitdepth_;
  }
  auto getSingleLayerMode() const { return singleLayerMode_; }

  auto& getAtlasIndex() { return atlasIndex_; }
  auto& getBaseMeshIndex() { return baseMeshIndex_; }
  auto& getAtlas() { return atlasHLS_[atlasIndex_]; }
  auto& getBaseMesh() { return baseMeshHLS_[baseMeshIndex_]; }
  auto& getOccupancyPrecision() { return occupancyPrecision_; }
  auto& getLog2PatchQuantizerSizeX() { return log2PatchQuantizerSizeX_; }
  auto& getLog2PatchQuantizerSizeY() { return log2PatchQuantizerSizeY_; }
  auto& getEnablePatchSizeQuantization() {
    return enablePatchSizeQuantization_;
  }
  auto& getPrefilterLossyOM() { return prefilterLossyOM_; }
  auto& getOffsetLossyOM() { return offsetLossyOM_; }
  auto& getGeometry3dCoordinatesBitdepth() {
    return geometry3dCoordinatesBitdepth_;
  }
  auto& getSingleLayerMode() { return singleLayerMode_; }

  VideoBitstream& createVideoBitstream(V3CUnitType v3cUnitType) {
    auto&  vps        = getVps();
    auto&  vuh        = getV3CUnitHeader(v3cUnitType);
    size_t atlasIndex = vuh.getAtlasId();
    auto&  atlas      = getAtlas();
    auto   type       = getVideoBitstreamType(v3cUnitType);
    return atlas.createVideoBitstream(type);
  }

  VideoBitstream& getVideoBitstream(V3CUnitType v3cUnitType) {
    auto&  vps        = getVps();
    auto&  vuh        = getV3CUnitHeader(v3cUnitType);
    size_t atlasIndex = vuh.getAtlasId();
    auto&  atlas      = getAtlas();
    auto   type       = getVideoBitstreamType(v3cUnitType);
    return atlas.getVideoBitstream(type);
  }

private:
  VideoType getVideoBitstreamType(V3CUnitType v3cUnitType) {
    auto&  vps        = getVps();
    auto&  vuh        = getV3CUnitHeader(v3cUnitType);
    size_t atlasIndex = vuh.getAtlasId();
    auto&  atlas      = getAtlas();
    if (v3cUnitType == V3C_OVD) {
      return VIDEO_OCCUPANCY;
    } else if (v3cUnitType == V3C_GVD) {
      if (vuh.getAuxiliaryVideoFlag()) {
        return VIDEO_GEOMETRY_RAW;
      } else {
        if (vps.getMapCountMinus1(atlasIndex) > 0
            && vps.getMultipleMapStreamsPresentFlag(atlasIndex)) {
          return static_cast<VideoType>(VIDEO_GEOMETRY_D0 + vuh.getMapIndex());
        } else {
          return VIDEO_GEOMETRY;
        }
      }
    } else if (v3cUnitType == V3C_AVD) {
      if (vps.getAttributeInformation(atlasIndex).getAttributeCount() > 0) {
        if (vuh.getAuxiliaryVideoFlag()) {
          return static_cast<VideoType>(VIDEO_ATTRIBUTE_RAW
                                        + vuh.getAttributeDimensionIndex());
        } else {
          if (vps.getMapCountMinus1(atlasIndex) > 0
              && vps.getMultipleMapStreamsPresentFlag(atlasIndex)) {
            return static_cast<VideoType>(
              VIDEO_ATTRIBUTE_T0 + vuh.getMapIndex() * MAX_NUM_ATTR_PARTITIONS
              + vuh.getAttributeDimensionIndex());
          } else {
            return static_cast<VideoType>(VIDEO_ATTRIBUTE
                                          + vuh.getAttributeDimensionIndex());
          }
        }
      }
    }
    return NUM_VIDEO_TYPE;
  }

  uint8_t                           activeVPS_                     = 0;
  uint8_t                           occupancyPrecision_            = 0;
  uint8_t                           log2PatchQuantizerSizeX_       = 0;
  uint8_t                           log2PatchQuantizerSizeY_       = 0;
  bool                              enablePatchSizeQuantization_   = 0;
  bool                              prefilterLossyOM_              = 0;
  size_t                            offsetLossyOM_                 = 0;
  size_t                            geometry3dCoordinatesBitdepth_ = 0;
  bool                              singleLayerMode_               = 0;
  size_t                            atlasIndex_                    = 0;
  size_t                            baseMeshIndex_                 = 0;
  V3CUnitHeader                     v3cUnitHeader_[NUM_V3C_UNIT_TYPE];
  std::vector<V3CParameterSet>      vpccParameterSets_;
  std::vector<AtlasSubBitstream>    atlasHLS_;
  std::vector<BaseMeshSubBitstream> baseMeshHLS_;
  std::vector<VideoBitstream>       videoBitstream_;
  BitstreamStat*                    bitstreamStat_;
};

};  // namespace vmesh
