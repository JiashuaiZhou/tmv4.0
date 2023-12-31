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
#include "atlasFrameMeshInformation.hpp"
#include "vmc.hpp"

namespace vmesh {

// VPS V-DMC extension syntax
class VpsVdmcExtension {
public:
  VpsVdmcExtension() {}
  ~VpsVdmcExtension() {}
  VpsVdmcExtension& operator=(const VpsVdmcExtension&) = default;
};

// ASPS V-DMC extension syntax
class AspsVdmcExtension {
public:
  AspsVdmcExtension() {}
  ~AspsVdmcExtension() {}
  AspsVdmcExtension& operator=(const AspsVdmcExtension&) = default;

  auto& getEncodeDisplacements() const { return encodeDisplacements; }
  auto& getWidthDispVideo() const { return widthDispVideo; }
  auto& getHeightDispVideo() const { return heightDispVideo; }
  auto& getAddReconstructedNormals() const { return addReconstructedNormals; }
  auto& getSubdivisionMethod() const { return subdivisionMethod; }
  auto& getSubdivisionIterationCount() const {
    return subdivisionIterationCount;
  }
  auto& getLiftingQPs() const { return liftingQPs; }
  auto& getLiftingQPs(uint8_t index) const { return liftingQPs[index]; }
  auto& getLiftingLevelOfDetailInverseScale() const {
    return liftingLevelOfDetailInverseScale;
  }
  auto& getLiftingLevelOfDetailInverseScale(uint8_t index) const {
    return liftingLevelOfDetailInverseScale[index];
  }
  auto& getLiftingQuantizationParametersPerLevelOfDetails() const {
    return liftingQuantizationParametersPerLevelOfDetails;
  }
  auto& getInterpolateDisplacementNormals() const {
    return interpolateDisplacementNormals;
  }
  auto& getDisplacementReversePacking() const {
    return displacementReversePacking;
  }
  auto& getDisplacement1D() const {
      return displacement1D;
  }
  auto& getLodDisplacementQuantizationFlag() const {
    return lodDisplacementQuantizationFlag;
  }
  auto& getSubBlockSize() const { return subBlockSize; }

  auto& getEncodeDisplacements() { return encodeDisplacements; }
  auto& getWidthDispVideo() { return widthDispVideo; }
  auto& getHeightDispVideo() { return heightDispVideo; }
  auto& getAddReconstructedNormals() { return addReconstructedNormals; }
  auto& getSubdivisionMethod() { return subdivisionMethod; }
  auto& getSubdivisionIterationCount() { return subdivisionIterationCount; }
  auto& getLiftingQPs() { return liftingQPs; }
  auto& getLiftingQPs(uint8_t index) { return liftingQPs[index]; }
  auto& getLiftingLevelOfDetailInverseScale() {
    return liftingLevelOfDetailInverseScale;
  }
  auto& getLiftingLevelOfDetailInverseScale(uint8_t index) {
    return liftingLevelOfDetailInverseScale[index];
  }
  auto& getLiftingQuantizationParametersPerLevelOfDetails() {
    return liftingQuantizationParametersPerLevelOfDetails;
  }
  auto& getInterpolateDisplacementNormals() {
    return interpolateDisplacementNormals;
  }
  auto& getDisplacementReversePacking() { return displacementReversePacking; }
  auto& getDisplacement1D() { return displacement1D; }
  auto& getLodDisplacementQuantizationFlag() {
    return lodDisplacementQuantizationFlag;
  }
  auto& getSubBlockSize() { return subBlockSize; }

  auto& getMaxNumNeighborsMotion() const {
      return maxNumNeighborsMotion;
  }
  auto& getMaxNumNeighborsMotion() { return maxNumNeighborsMotion; }

  auto getDisplacementCoordinateSystem() const {
    return displacementCoordinateSystem;
  }

  auto& getLiftingSkipUpdate() const { return liftingSkipUpdate; }
  auto& getLiftingUpdateWeight() const { return liftingUpdateWeight; }
  auto& getLiftingPredictionWeight() const { return liftingPredictionWeight; }

  //orthoAtlas
  auto& getProjectionTextCoordEnableFlag() const { return   projectionTextCoordEnableFlag; }
  auto& getProjectionTextCoordMappingMethod() const { return projectionTextCoordMappingMethod; }
  auto& getProjectionTextCoordScaleFactor() const { return projectionTextCoordScaleFactor; }
  auto& getProjectionTextCoordEnableFlag() { return   projectionTextCoordEnableFlag; }
  auto& getProjectionTextCoordMappingMethod() { return projectionTextCoordMappingMethod; }
  auto& getProjectionTextCoordScaleFactor() { return projectionTextCoordScaleFactor; }

private:
  // Displacement
  uint8_t  encodeDisplacements        = 0;
  uint16_t widthDispVideo             = 0;
  uint16_t heightDispVideo            = 0;
  bool     displacementReversePacking = false;
  bool     displacement1D             = true;
  bool     addReconstructedNormals    = true;
  bool     lodDisplacementQuantizationFlag = true;
  uint16_t subBlockSize               = 0;

  // Motion
  int32_t maxNumNeighborsMotion       = 3;

  // Subdivision
  uint8_t subdivisionIterationCount      = 0;
  bool    interpolateDisplacementNormals = false;
  int32_t liftingQPs[3]                  = {0, 0, 0};
  std::vector<std::array<int32_t, 3>>
    liftingQuantizationParametersPerLevelOfDetails = {{16, 28, 28},
                                                      {22, 34, 34},
                                                      {28, 40, 40}};

  // Not coded
  SubdivisionMethod subdivisionMethod = SubdivisionMethod::MID_POINT;
  double            liftingLevelOfDetailInverseScale[3] = {2.0, 2.0, 2.0};
  double            liftingUpdateWeight                 = 0.125;
  double            liftingPredictionWeight             = 0.5;
  bool              liftingSkipUpdate                   = false;
  DisplacementCoordinateSystem displacementCoordinateSystem =
    DisplacementCoordinateSystem::LOCAL;

  // orthoAtlas
  bool    projectionTextCoordEnableFlag = false;
  uint8_t projectionTextCoordMappingMethod = 0;
  double  projectionTextCoordScaleFactor = 0.0;

};

// AFPS V-DMC extension syntax
class AfpsVdmcExtension {
public:
  AfpsVdmcExtension() {}
  ~AfpsVdmcExtension() {
      projectionTextcoordPresentFlag_.clear();
      projectionTextcoordWidth_.clear();
      projectionTextcoordHeight_.clear();
      projectionTextcoordGutter_.clear();
  }
  AfpsVdmcExtension& operator=(const AfpsVdmcExtension&) = default;

  auto& getAtlasFrameMeshInformation() { return atlasFrameMeshInformation_; }
  auto& getProjectionTextcoordPresentFlag() { return projectionTextcoordPresentFlag_; }
  auto& getProjectionTextcoordWidth() { return projectionTextcoordWidth_; }
  auto& getProjectionTextcoordHeight() { return projectionTextcoordHeight_; }
  auto& getProjectionTextcoordGutter() { return projectionTextcoordGutter_; }

  auto& getProjectionTextcoordPresentFlag(int index) { 
      if (index >= projectionTextcoordPresentFlag_.size()) projectionTextcoordPresentFlag_.resize(projectionTextcoordPresentFlag_.size() + 1);
      return projectionTextcoordPresentFlag_[index]; }
  auto& getProjectionTextcoordWidth(int index) { 
      if (index >= projectionTextcoordWidth_.size()) projectionTextcoordWidth_.resize(projectionTextcoordWidth_.size() + 1);
      return projectionTextcoordWidth_[index]; }
  auto& getProjectionTextcoordHeight(int index) { 
      if (index >= projectionTextcoordHeight_.size()) projectionTextcoordHeight_.resize(projectionTextcoordHeight_.size() + 1);
      return projectionTextcoordHeight_[index]; }
  auto& getProjectionTextcoordGutter(int index) { 
      if (index >= projectionTextcoordGutter_.size()) projectionTextcoordGutter_.resize(projectionTextcoordGutter_.size() + 1);
      return projectionTextcoordGutter_[index]; }
  const auto& getProjectionTextcoordPresentFlag(int index) const {
      return projectionTextcoordPresentFlag_[index];
  }
  const auto& getProjectionTextcoordWidth(int index) const {
      return projectionTextcoordWidth_[index];
  }
  const auto& getProjectionTextcoordHeight(int index) const {
      return projectionTextcoordHeight_[index];
  }
  const auto& getProjectionTextcoordGutter(int index) const {
      return projectionTextcoordGutter_[index];
  }
  void allocateStructures(int size) {
      projectionTextcoordPresentFlag_.resize(size);
      projectionTextcoordWidth_.resize(size);
      projectionTextcoordHeight_.resize(size);
      projectionTextcoordGutter_.resize(size);
  }

private:

    //orthoAtlas
    AtlasFrameMeshInformation atlasFrameMeshInformation_;
    std::vector<uint8_t> projectionTextcoordPresentFlag_;
    std::vector<uint32_t> projectionTextcoordWidth_;
    std::vector<uint32_t> projectionTextcoordHeight_;
    std::vector<uint32_t> projectionTextcoordGutter_;
};

// AAPS V-DMC extension syntax
class AapsVdmcExtension {
public:
  AapsVdmcExtension() {}
  ~AapsVdmcExtension() {}
  AapsVdmcExtension& operator=(const AapsVdmcExtension&) = default;

private:
};

};  // namespace vmesh
