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

// G.2  VUI syntax
// G.2.3  Sub-layer HRD parameters syntax
class HrdSubLayerParameters {
public:
  HrdSubLayerParameters() {}
  ~HrdSubLayerParameters() {
    bitRateValueMinus1_.clear();
    cabSizeValueMinus1_.clear();
    cbrFlag_.clear();
  }
  HrdSubLayerParameters& operator=(const HrdSubLayerParameters&) = default;
  void                   allocate(size_t size) {
    bitRateValueMinus1_.resize(size, 0);
    cabSizeValueMinus1_.resize(size, 0);
    cbrFlag_.resize(size, false);
  }
  auto size() { return bitRateValueMinus1_.size(); }
  auto getCbrFlag(size_t i) const { return cbrFlag_[i]; }
  auto getBitRateValueMinus1(size_t i) const { return bitRateValueMinus1_[i]; }
  auto getCabSizeValueMinus1(size_t i) const { return cabSizeValueMinus1_[i]; }

  auto& getCbrFlag(size_t i) { return cbrFlag_[i]; }
  auto& getBitRateValueMinus1(size_t i) { return bitRateValueMinus1_[i]; }
  auto& getCabSizeValueMinus1(size_t i) { return cabSizeValueMinus1_[i]; }

private:
  std::vector<uint32_t> bitRateValueMinus1_;
  std::vector<uint32_t> cabSizeValueMinus1_;
  std::vector<uint8_t>  cbrFlag_;
};

// G.2.2  HRD parameters syntax
class HrdParameters {
public:
  HrdParameters() {
    fixedAtlasRateGeneralFlag_.resize(maxNumSubLayersMinus1_);
    fixedAtlasRateWithinCasFlag_.resize(maxNumSubLayersMinus1_);
    elementalDurationInTcMinus1_.resize(maxNumSubLayersMinus1_);
    lowDelayFlag_.resize(maxNumSubLayersMinus1_);
    cabCntMinus1_.resize(maxNumSubLayersMinus1_);
    subLayerParameters_[0].resize(maxNumSubLayersMinus1_);
    subLayerParameters_[1].resize(maxNumSubLayersMinus1_);
  }
  ~HrdParameters() {
    fixedAtlasRateGeneralFlag_.clear();
    fixedAtlasRateWithinCasFlag_.clear();
    elementalDurationInTcMinus1_.clear();
    lowDelayFlag_.clear();
    cabCntMinus1_.clear();
    subLayerParameters_[0].clear();
    subLayerParameters_[1].clear();
  }
  HrdParameters& operator=(const HrdParameters&) = default;

  auto getMaxNumSubLayersMinus1() { return maxNumSubLayersMinus1_; }
  auto getNalParametersPresentFlag() const {
    return nalParametersPresentFlag_;
  }
  auto getAclParametersPresentFlag() const {
    return aclParametersPresentFlag_;
  }
  auto getBitRateScale() const { return bitRateScale_; }
  auto getCabSizeScale() const { return cabSizeScale_; }
  auto getFixedAtlasRateGeneralFlag(size_t i) const {
    return fixedAtlasRateGeneralFlag_[i];
  }
  auto getFixedAtlasRateWithinCasFlag(size_t i) const {
    return fixedAtlasRateWithinCasFlag_[i];
  }
  auto getElementalDurationInTcMinus1(size_t i) const {
    return elementalDurationInTcMinus1_[i];
  }
  auto  getLowDelayFlag(size_t i) const { return lowDelayFlag_[i]; }
  auto  getCabCntMinus1(size_t i) const { return cabCntMinus1_[i]; }
  auto& getHdrSubLayerParameters(size_t i, size_t j) const {
    return subLayerParameters_[i][j];
  }

  auto& getNalParametersPresentFlag() { return nalParametersPresentFlag_; }
  auto& getAclParametersPresentFlag() { return aclParametersPresentFlag_; }
  auto& getBitRateScale() { return bitRateScale_; }
  auto& getCabSizeScale() { return cabSizeScale_; }
  auto& getFixedAtlasRateGeneralFlag(size_t i) {
    return fixedAtlasRateGeneralFlag_[i];
  }
  auto& getFixedAtlasRateWithinCasFlag(size_t i) {
    return fixedAtlasRateWithinCasFlag_[i];
  }
  auto& getElementalDurationInTcMinus1(size_t i) {
    return elementalDurationInTcMinus1_[i];
  }
  auto& getLowDelayFlag(size_t i) { return lowDelayFlag_[i]; }
  auto& getCabCntMinus1(size_t i) { return cabCntMinus1_[i]; }
  auto& getHdrSubLayerParameters(size_t i, size_t j) {
    return subLayerParameters_[i][j];
  }

private:
  static const int32_t               maxNumSubLayersMinus1_    = 0;
  bool                               nalParametersPresentFlag_ = 0;
  bool                               aclParametersPresentFlag_ = 0;
  uint8_t                            bitRateScale_             = 0;
  uint8_t                            cabSizeScale_             = 0;
  std::vector<uint8_t>               fixedAtlasRateGeneralFlag_;
  std::vector<uint8_t>               fixedAtlasRateWithinCasFlag_;
  std::vector<uint32_t>              elementalDurationInTcMinus1_;
  std::vector<uint8_t>               lowDelayFlag_;
  std::vector<uint32_t>              cabCntMinus1_;
  std::vector<HrdSubLayerParameters> subLayerParameters_[2];
};

// G.2.4 Maximum coded video resolution syntax
class MaxCodedVideoResolution {
public:
  MaxCodedVideoResolution() {}
  ~MaxCodedVideoResolution() {}
  MaxCodedVideoResolution& operator=(const MaxCodedVideoResolution&) = default;

  auto getOccupancyResolutionPresentFlag() const {
    return occupancyResolutionPresentFlag_;
  }
  auto getGeometryResolutionPresentFlag() const {
    return geometryResolutionPresentFlag_;
  }
  auto getAttributeResolutionPresentFlag() const {
    return attributeResolutionPresentFlag_;
  }
  auto getOccupancyWidth() const { return occupancyWidth_; }
  auto getOccupancyHeight() const { return occupancyHeight_; }
  auto getGeometryWidth() const { return geometryWidth_; }
  auto getGeometryHeight() const { return geometryHeight_; }
  auto getAttributeWidth() const { return attributeWidth_; }
  auto getAttributeHeight() const { return attributeHeight_; }

  auto& getOccupancyResolutionPresentFlag() {
    return occupancyResolutionPresentFlag_;
  }
  auto& getGeometryResolutionPresentFlag() {
    return geometryResolutionPresentFlag_;
  }
  auto& getAttributeResolutionPresentFlag() {
    return attributeResolutionPresentFlag_;
  }
  auto& getOccupancyWidth() { return occupancyWidth_; }
  auto& getOccupancyHeight() { return occupancyHeight_; }
  auto& getGeometryWidth() { return geometryWidth_; }
  auto& getGeometryHeight() { return geometryHeight_; }
  auto& getAttributeWidth() { return attributeWidth_; }
  auto& getAttributeHeight() { return attributeHeight_; }

private:
  bool     occupancyResolutionPresentFlag_ = false;
  bool     geometryResolutionPresentFlag_  = false;
  bool     attributeResolutionPresentFlag_ = false;
  uint32_t occupancyWidth_                 = 0;
  uint32_t occupancyHeight_                = 0;
  uint32_t geometryWidth_                  = 0;
  uint32_t geometryHeight_                 = 0;
  uint32_t attributeWidth_                 = 0;
  uint32_t attributeHeight_                = 0;
};

// G.2.5 Coordinate system parameters syntax
class CoordinateSystemParameters {
public:
  CoordinateSystemParameters() {}
  ~CoordinateSystemParameters() {}
  CoordinateSystemParameters&
  operator=(const CoordinateSystemParameters&) = default;

  auto getForwardAxis() const { return forwardAxis_; }
  auto getDeltaLeftAxis() const { return deltaLeftAxis_; }
  auto getForwardSign() const { return forwardSign_; }
  auto getLeftSign() const { return leftSign_; }
  auto getUpSign() const { return upSign_; }

  auto& getForwardAxis() { return forwardAxis_; }
  auto& getDeltaLeftAxis() { return deltaLeftAxis_; }
  auto& getForwardSign() { return forwardSign_; }
  auto& getLeftSign() { return leftSign_; }
  auto& getUpSign() { return upSign_; }

private:
  uint8_t forwardAxis_   = 0;
  uint8_t deltaLeftAxis_ = 0;
  uint8_t forwardSign_   = 0;
  uint8_t leftSign_      = 0;
  uint8_t upSign_        = 0;
};

// G.2.1  VUI parameters syntax
class VUIParameters {
public:
  VUIParameters() {}

  ~VUIParameters() {}
  VUIParameters& operator=(const VUIParameters&) = default;

  auto& getHrdParameters() const { return hrdParameters_; }
  auto& getCoordinateSystemParameters() const {
    return coordinateSystemParameters_;
  }
  auto& getMaxCodedVideoResolution() const { return maxCodedVideoResolution_; }
  auto  getTimingInfoPresentFlag() const { return timingInfoPresentFlag_; }
  auto  getNumUnitsInTick() const { return numUnitsInTick_; }
  auto  getTimeScale() const { return timeScale_; }
  auto  getPocProportionalToTimingFlag() const {
    return pocProportionalToTimingFlag_;
  }
  auto getNumTicksPocDiffOneMinus1() const {
    return numTicksPocDiffOneMinus1_;
  }
  auto getHrdParametersPresentFlag() const {
    return hrdParametersPresentFlag_;
  }
  auto getTileRestrictionsPresentFlag() const {
    return tileRestrictionsPresentFlag_;
  }
  auto getFixedAtlasTileStructureFlag() const {
    return fixedAtlasTileStructureFlag_;
  }
  auto getFixedVideoTileStructureFlag() const {
    return fixedVideoTileStructureFlag_;
  }
  auto getConstrainedTilesAcrossV3cComponentsIdc() const {
    return constrainedTilesAcrossV3cComponentsIdc_;
  }
  auto getMaxNumTilesPerAtlasMinus1() const {
    return maxNumTilesPerAtlasMinus1_;
  }
  auto getMaxCodedVideoResolutionPresentFlag() const {
    return maxCodedVideoResolutionPresentFlag_;
  }
  auto getCoordinateSystemParametersPresentFlag() const {
    return coordinateSystemParametersPresentFlag_;
  }
  auto getUnitInMetresFlag() const { return unitInMetresFlag_; }
  auto getDisplayBoxInfoPresentFlag() const {
    return displayBoxInfoPresentFlag_;
  }
  auto getDisplayBoxOrigin(size_t i) const { return displayBoxOrigin_[i]; }
  auto getDisplayBoxSize(size_t i) const { return displayBoxSize_[i]; }
  auto getAnchorPointPresentFlag() const { return anchorPointPresentFlag_; }
  auto getAnchorPoint(size_t i) const { return anchorPoint_[i]; }

  auto& getHrdParameters() { return hrdParameters_; }
  auto& getCoordinateSystemParameters() { return coordinateSystemParameters_; }
  auto& getMaxCodedVideoResolution() { return maxCodedVideoResolution_; }
  auto& getTimingInfoPresentFlag() { return timingInfoPresentFlag_; }
  auto& getNumUnitsInTick() { return numUnitsInTick_; }
  auto& getTimeScale() { return timeScale_; }
  auto& getPocProportionalToTimingFlag() {
    return pocProportionalToTimingFlag_;
  }
  auto& getNumTicksPocDiffOneMinus1() { return numTicksPocDiffOneMinus1_; }
  auto& getHrdParametersPresentFlag() { return hrdParametersPresentFlag_; }
  auto& getTileRestrictionsPresentFlag() {
    return tileRestrictionsPresentFlag_;
  }
  auto& getFixedAtlasTileStructureFlag() {
    return fixedAtlasTileStructureFlag_;
  }
  auto& getFixedVideoTileStructureFlag() {
    return fixedVideoTileStructureFlag_;
  }
  auto& getConstrainedTilesAcrossV3cComponentsIdc() {
    return constrainedTilesAcrossV3cComponentsIdc_;
  }
  auto& getMaxNumTilesPerAtlasMinus1() { return maxNumTilesPerAtlasMinus1_; }
  auto& getMaxCodedVideoResolutionPresentFlag() {
    return maxCodedVideoResolutionPresentFlag_;
  }
  auto& getCoordinateSystemParametersPresentFlag() {
    return coordinateSystemParametersPresentFlag_;
  }
  auto& getUnitInMetresFlag() { return unitInMetresFlag_; }
  auto& getDisplayBoxInfoPresentFlag() { return displayBoxInfoPresentFlag_; }
  auto& getDisplayBoxOrigin(size_t i) { return displayBoxOrigin_[i]; }
  auto& getDisplayBoxSize(size_t i) { return displayBoxSize_[i]; }
  auto& getAnchorPointPresentFlag() { return anchorPointPresentFlag_; }
  auto& getAnchorPoint(size_t i) { return anchorPoint_[i]; }

private:
  bool     timingInfoPresentFlag_                  = 0;
  uint32_t numUnitsInTick_                         = 1001;
  uint32_t timeScale_                              = 60000;
  bool     pocProportionalToTimingFlag_            = false;
  uint32_t numTicksPocDiffOneMinus1_               = 0;
  bool     hrdParametersPresentFlag_               = false;
  bool     tileRestrictionsPresentFlag_            = false;
  bool     fixedAtlasTileStructureFlag_            = false;
  bool     fixedVideoTileStructureFlag_            = false;
  uint32_t constrainedTilesAcrossV3cComponentsIdc_ = false;
  uint32_t maxNumTilesPerAtlasMinus1_              = 0;
  bool     maxCodedVideoResolutionPresentFlag_     = false;
  bool     coordinateSystemParametersPresentFlag_  = false;
  bool     unitInMetresFlag_                       = false;
  bool     displayBoxInfoPresentFlag_              = false;
  uint32_t displayBoxOrigin_[3]                    = {0, 0, 0};
  uint32_t displayBoxSize_[3]                      = {0, 0, 0};
  bool     anchorPointPresentFlag_                 = false;
  float    anchorPoint_[3]                         = {0.f, 0.f, 0.f};

  HrdParameters              hrdParameters_;
  CoordinateSystemParameters coordinateSystemParameters_;
  MaxCodedVideoResolution    maxCodedVideoResolution_;
};

};  // namespace vmesh
