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

// Annex E: Supplemental enhancement information
// F.2.1	General SEI message syntax  <=> 7.3.8	Supplemental enhancement
// information message syntax
class SEI {
public:
  SEI() {
    payloadSize_ = 0;
    byteStrData_.clear();
  }
  virtual ~SEI() { byteStrData_.clear(); }
  virtual SeiPayloadType getPayloadType() = 0;

  virtual std::vector<uint8_t>& getMD5ByteStrData() { return byteStrData_; }

  size_t getPayloadSize() { return payloadSize_; }
  void   setPayloadSize(size_t value) { payloadSize_ = value; }

protected:
  std::vector<uint8_t> byteStrData_;

private:
  size_t payloadSize_;
};

// F.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
class SEIUserDataRegisteredItuTT35 : public SEI {
public:
  SEIUserDataRegisteredItuTT35() {}

  ~SEIUserDataRegisteredItuTT35() { payloadByte_.clear(); }
  SEIUserDataRegisteredItuTT35&
  operator=(const SEIUserDataRegisteredItuTT35&) = default;

  SeiPayloadType getPayloadType() { return USER_DATAREGISTERED_ITUTT35; }

  auto getCountryCode() const { return countryCode_; }
  auto getCountryCodeExtensionByte() const {
    return countryCodeExtensionByte_;
  }
  auto& getPayloadByte() const { return payloadByte_; }

  auto& getCountryCode() { return countryCode_; }
  auto& getCountryCodeExtensionByte() { return countryCodeExtensionByte_; }
  auto& getPayloadByte() { return payloadByte_; }

private:
  uint8_t              countryCode_              = 0;
  uint8_t              countryCodeExtensionByte_ = 0;
  std::vector<uint8_t> payloadByte_;
};

// F.2.4  User data unregistered SEI message syntax
class SEIUserDataUnregistered : public SEI {
public:
  SEIUserDataUnregistered() {
    for (size_t i = 0; i < 16; i++) { uuidIsoIec11578_[i] = 0; }
  }

  ~SEIUserDataUnregistered() { userDataPayloadByte_.clear(); }
  SEIUserDataUnregistered& operator=(const SEIUserDataUnregistered&) = default;

  SeiPayloadType getPayloadType() { return USER_DATA_UNREGISTERED; }

  auto getUuidIsoIec11578(size_t i) const { return uuidIsoIec11578_[i]; }
  auto getUserDataPayloadByte(size_t i) const {
    return userDataPayloadByte_[i];
  }
  auto& getUserDataPayloadByte() const { return userDataPayloadByte_; }

  auto& getUuidIsoIec11578(size_t i) { return uuidIsoIec11578_[i]; }
  auto& getUserDataPayloadByte(size_t i) { return userDataPayloadByte_[i]; }
  auto& getUserDataPayloadByte() { return userDataPayloadByte_; }

private:
  uint8_t              uuidIsoIec11578_[16];
  std::vector<uint8_t> userDataPayloadByte_;
};

// F.2.5  Recovery point SEI message syntax
class SEIRecoveryPoint : public SEI {
public:
  SEIRecoveryPoint() {}
  ~SEIRecoveryPoint() {}
  SEIRecoveryPoint& operator=(const SEIRecoveryPoint&) = default;

  SeiPayloadType getPayloadType() { return RECOVERY_POINT; }

  auto  getRecoveryAfocCnt() const { return recoveryAfocCnt_; }
  auto  getExactMatchFlag() const { return exactMatchFlag_; }
  auto  getBrokenLinkFlag() const { return brokenLinkFlag_; }
  auto& getRecoveryAfocCnt() { return recoveryAfocCnt_; }
  auto& getExactMatchFlag() { return exactMatchFlag_; }
  auto& getBrokenLinkFlag() { return brokenLinkFlag_; }

private:
  int32_t recoveryAfocCnt_ = 0;
  uint8_t exactMatchFlag_  = 0;
  uint8_t brokenLinkFlag_  = 0;
};

// F.2.6  No display SEI message syntax
class SEINoDisplay : public SEI {
public:
  SEINoDisplay() {}
  ~SEINoDisplay() {}
  SEINoDisplay& operator=(const SEINoDisplay&) = default;

  SeiPayloadType getPayloadType() { return NO_RECONSTRUCTION; }

private:
};

// F.2.7  Reserved SEI message syntax
class SEIReservedSeiMessage : public SEI {
public:
  SEIReservedSeiMessage() {}
  ~SEIReservedSeiMessage() { payloadByte_.clear(); }
  SEIReservedSeiMessage& operator=(const SEIReservedSeiMessage&) = default;

  SeiPayloadType getPayloadType() { return RESERVED_SEI_MESSAGE; }

  auto& getPayloadByte() const { return payloadByte_; }
  auto  getPayloadByte(size_t i) const { return payloadByte_[i]; }

  auto& getPayloadByte() { return payloadByte_; }
  auto& getPayloadByte(size_t i) { return payloadByte_[i]; }

private:
  std::vector<uint8_t> payloadByte_;
};

// F.2.8  SEI manifest SEI message syntax
class SEIManifest : public SEI {
public:
  SEIManifest() {}
  ~SEIManifest() {
    seiPayloadType_.clear();
    seiDescription_.clear();
  }
  SEIManifest& operator=(const SEIManifest&) = default;

  SeiPayloadType getPayloadType() { return SEI_MANIFEST; }

  void allocate() {
    seiPayloadType_.resize(numSeiMsgTypes_, 0);
    seiDescription_.resize(numSeiMsgTypes_, 0);
  }
  auto  getNumSeiMsgTypes() const { return numSeiMsgTypes_; }
  auto& getSeiPayloadType() const { return seiPayloadType_; }
  auto& getSeiDescription() const { return seiDescription_; }
  auto  getSeiPayloadType(size_t i) const { return seiPayloadType_[i]; }
  auto  getSeiDescription(size_t i) const { return seiDescription_[i]; }

  auto& getNumSeiMsgTypes() { return numSeiMsgTypes_; }
  auto& getSeiPayloadType() { return seiPayloadType_; }
  auto& getSeiDescription() { return seiDescription_; }
  auto& getSeiPayloadType(size_t i) { return seiPayloadType_[i]; }
  auto& getSeiDescription(size_t i) { return seiDescription_[i]; }

private:
  uint16_t              numSeiMsgTypes_ = 0;
  std::vector<uint16_t> seiPayloadType_;
  std::vector<uint8_t>  seiDescription_;
};

// F.2.9  SEI prefix indication SEI message syntax
class SEIPrefixIndication : public SEI {
public:
  SEIPrefixIndication() {}

  ~SEIPrefixIndication() {
    numBitsInPrefixIndicationMinus1_.clear();
    for (auto& element : seiPrefixDataBit_) { element.clear(); }
    seiPrefixDataBit_.clear();
  }
  SEIPrefixIndication& operator=(const SEIPrefixIndication&) = default;

  SeiPayloadType getPayloadType() { return SEI_PREFIX_INDICATION; }

  auto getPrefixSeiPayloadType() const { return prefixSeiPayloadType_; }
  auto getNumSeiPrefixIndicationsMinus1() const {
    return numSeiPrefixIndicationsMinus1_;
  }
  auto& getNumBitsInPrefixIndicationMinus1() const {
    return numBitsInPrefixIndicationMinus1_;
  }
  auto& getSeiPrefixDataBit() const { return seiPrefixDataBit_; }
  auto  getNumBitsInPrefixIndicationMinus1(size_t i) const {
    return numBitsInPrefixIndicationMinus1_[i];
  }
  auto getSeiPrefixDataBit(size_t i) const { return seiPrefixDataBit_[i]; }
  auto getSeiPrefixDataBit(size_t i, size_t j) const {
    return seiPrefixDataBit_[i][j];
  }

  auto& getPrefixSeiPayloadType() { return prefixSeiPayloadType_; }
  auto& getNumSeiPrefixIndicationsMinus1() {
    return numSeiPrefixIndicationsMinus1_;
  }
  auto& getNumBitsInPrefixIndicationMinus1() {
    return numBitsInPrefixIndicationMinus1_;
  }
  auto& getSeiPrefixDataBit() { return seiPrefixDataBit_; }
  auto& getNumBitsInPrefixIndicationMinus1(size_t i) {
    return numBitsInPrefixIndicationMinus1_[i];
  }
  auto& getSeiPrefixDataBit(size_t i) { return seiPrefixDataBit_[i]; }
  auto& getSeiPrefixDataBit(size_t i, size_t j) {
    return seiPrefixDataBit_[i][j];
  }

private:
  uint16_t                          prefixSeiPayloadType_          = 0;
  uint8_t                           numSeiPrefixIndicationsMinus1_ = 0;
  std::vector<uint16_t>             numBitsInPrefixIndicationMinus1_;
  std::vector<std::vector<uint8_t>> seiPrefixDataBit_;
};

// F.2.10  Attribute transformation parameters SEI message syntax
class SEIAttributeTransformationParams : public SEI {
public:
  SEIAttributeTransformationParams() {}

  ~SEIAttributeTransformationParams() {
    attributeIdx_.clear();
    dimensionMinus1_.clear();
    for (auto& element : scaleParamsEnabledFlag_) { element.clear(); }
    for (auto& element : offsetParamsEnabledFlag_) { element.clear(); }
    for (auto& element : attributeScale_) { element.clear(); }
    for (auto& element : attributeOffset_) { element.clear(); }
    scaleParamsEnabledFlag_.clear();
    offsetParamsEnabledFlag_.clear();
    attributeScale_.clear();
    attributeOffset_.clear();
  }
  SEIAttributeTransformationParams&
  operator=(const SEIAttributeTransformationParams&) = default;

  SeiPayloadType getPayloadType() { return ATTRIBUTE_TRANSFORMATION_PARAMS; }
  void           allocate() {
    attributeIdx_.resize(numAttributeUpdates_ + 1, 0);
    dimensionMinus1_.resize(256 + 1, 0);
    scaleParamsEnabledFlag_.resize(256);
    offsetParamsEnabledFlag_.resize(256);
    attributeScale_.resize(256);
    attributeOffset_.resize(256);
  }
  void allocate(size_t i) {
    scaleParamsEnabledFlag_[i].resize(dimensionMinus1_[i] + 1, false);
    offsetParamsEnabledFlag_[i].resize(dimensionMinus1_[i] + 1, false);
    attributeScale_[i].resize(dimensionMinus1_[i] + 1, 0);
    attributeOffset_[i].resize(dimensionMinus1_[i] + 1, 0);
  }

  auto  getCancelFlag() const { return cancelFlag_; }
  auto  getNumAttributeUpdates() const { return numAttributeUpdates_; }
  auto& AttributeIdx() const { return attributeIdx_; }
  auto& getDimensionMinus() const { return dimensionMinus1_; }
  auto& getScaleParamsEnabledFlag() const { return scaleParamsEnabledFlag_; }
  auto& getOffsetParamsEnabledFlag() const { return offsetParamsEnabledFlag_; }
  auto& getAttributeScale() const { return attributeScale_; }
  auto& getAttributeOffset() const { return attributeOffset_; }
  auto  getPersistenceFlag() const { return persistenceFlag_; }
  auto  getAttributeIdx(size_t i) const { return attributeIdx_[i]; }
  auto  getDimensionMinus1(size_t i) const { return dimensionMinus1_[i]; }
  auto  getScaleParamsEnabledFlag(size_t i, size_t j) const {
    return scaleParamsEnabledFlag_[i][j];
  }
  auto getOffsetParamsEnabledFlag(size_t i, size_t j) const {
    return offsetParamsEnabledFlag_[i][j];
  }
  auto getAttributeScale(size_t i, size_t j) const {
    return attributeScale_[i][j];
  }
  auto getAttributeOffset(size_t i, size_t j) const {
    return attributeOffset_[i][j];
  }

  auto& getCancelFlag() { return cancelFlag_; }
  auto& getNumAttributeUpdates() { return numAttributeUpdates_; }
  auto& AttributeIdx() { return attributeIdx_; }
  auto& getDimensionMinus() { return dimensionMinus1_; }
  auto& getScaleParamsEnabledFlag() { return scaleParamsEnabledFlag_; }
  auto& getOffsetParamsEnabledFlag() { return offsetParamsEnabledFlag_; }
  auto& getAttributeScale() { return attributeScale_; }
  auto& getAttributeOffset() { return attributeOffset_; }
  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getAttributeIdx(size_t i) { return attributeIdx_[i]; }
  auto& getDimensionMinus1(size_t i) { return dimensionMinus1_[i]; }
  auto& getScaleParamsEnabledFlag(size_t i, size_t j) {
    return scaleParamsEnabledFlag_[i][j];
  }
  auto& getOffsetParamsEnabledFlag(size_t i, size_t j) {
    return offsetParamsEnabledFlag_[i][j];
  }
  auto& getAttributeScale(size_t i, size_t j) { return attributeScale_[i][j]; }
  auto& getAttributeOffset(size_t i, size_t j) {
    return attributeOffset_[i][j];
  }

private:
  bool                               cancelFlag_          = false;
  int32_t                            numAttributeUpdates_ = 0;
  std::vector<uint8_t>               attributeIdx_;
  std::vector<uint8_t>               dimensionMinus1_;
  std::vector<std::vector<uint8_t>>  scaleParamsEnabledFlag_;
  std::vector<std::vector<uint8_t>>  offsetParamsEnabledFlag_;
  std::vector<std::vector<uint32_t>> attributeScale_;
  std::vector<std::vector<int32_t>>  attributeOffset_;
  bool                               persistenceFlag_ = true;
};

// F.2.10  Active substreams SEI message syntax
class SEIActiveSubBitstreams : public SEI {
public:
  SEIActiveSubBitstreams() {}
  ~SEIActiveSubBitstreams() {
    activeAttributeIdx_.clear();
    activeMapIdx_.clear();
  }
  SEIActiveSubBitstreams& operator=(const SEIActiveSubBitstreams&) = default;

  SeiPayloadType getPayloadType() { return ACTIVE_SUB_BITSTREAMS; }

  auto getActiveSubBitstreamsCancelFlag() const {
    return activeSubBitstreamsCancelFlag_;
  }
  auto getActiveAttributesChangesFlag() const {
    return activeAttributesChangesFlag_;
  }
  auto getActiveMapsChangesFlag() const { return activeMapsChangesFlag_; }
  auto getAuxiliarySubstreamsActiveFlag() const {
    return auxiliarySubstreamsActiveFlag_;
  }
  auto getAllAttributesActiveFlag() const { return allAttributesActiveFlag_; }
  auto getAllMapsActiveFlag() const { return allMapsActiveFlag_; }
  auto getActiveAttributeCountMinus1() const {
    return activeAttributeCountMinus1_;
  }
  auto  getActiveMapCountMinus1() const { return activeMapCountMinus1_; }
  auto& getActiveAttributeIdx() const { return activeAttributeIdx_; }
  auto& getActiveMapIdx() const { return activeMapIdx_; }
  auto getActiveAttributeIdx(size_t i) const { return activeAttributeIdx_[i]; }
  auto getActiveMapIdx(size_t i) const { return activeMapIdx_[i]; }

  auto& getActiveSubBitstreamsCancelFlag() {
    return activeSubBitstreamsCancelFlag_;
  }
  auto& getActiveAttributesChangesFlag() {
    return activeAttributesChangesFlag_;
  }
  auto& getActiveMapsChangesFlag() { return activeMapsChangesFlag_; }
  auto& getAuxiliarySubstreamsActiveFlag() {
    return auxiliarySubstreamsActiveFlag_;
  }
  auto& getAllAttributesActiveFlag() { return allAttributesActiveFlag_; }
  auto& getAllMapsActiveFlag() { return allMapsActiveFlag_; }
  auto& getActiveAttributeCountMinus1() { return activeAttributeCountMinus1_; }
  auto& getActiveMapCountMinus1() { return activeMapCountMinus1_; }
  auto& getActiveAttributeIdx() { return activeAttributeIdx_; }
  auto& getActiveMapIdx() { return activeMapIdx_; }
  auto& getActiveAttributeIdx(size_t i) { return activeAttributeIdx_[i]; }
  auto& getActiveMapIdx(size_t i) { return activeMapIdx_[i]; }

private:
  bool                 activeSubBitstreamsCancelFlag_ = false;
  bool                 activeAttributesChangesFlag_   = false;
  bool                 activeMapsChangesFlag_         = false;
  bool                 auxiliarySubstreamsActiveFlag_ = false;
  bool                 allAttributesActiveFlag_       = false;
  bool                 allMapsActiveFlag_             = false;
  uint8_t              activeAttributeCountMinus1_    = 0;
  uint8_t              activeMapCountMinus1_          = 0;
  std::vector<uint8_t> activeAttributeIdx_;
  std::vector<uint8_t> activeMapIdx_;
};

// F.2.11  Component codec mapping SEI message syntax
class SEIComponentCodecMapping : public SEI {
public:
  SEIComponentCodecMapping() {}
  ~SEIComponentCodecMapping() {
    codecId_.clear();
    codec4cc_.clear();
  }
  SEIComponentCodecMapping&
  operator=(const SEIComponentCodecMapping&) = default;

  SeiPayloadType getPayloadType() { return COMPONENT_CODEC_MAPPING; }

  void allocate() {
    codecId_.resize(codecMappingsCountMinus1_ + 1, 0);
    codec4cc_.resize(256);
  }
  auto getComponentCodecCancelFlag() const {
    return componentCodecCancelFlag_;
  }
  auto getCodecMappingsCountMinus1() const {
    return codecMappingsCountMinus1_;
  }
  auto getCodecId(size_t i) const { return codecId_[i]; }
  auto getCodec4cc(size_t i) const { return codec4cc_[i]; }

  auto& getComponentCodecCancelFlag() { return componentCodecCancelFlag_; }
  auto& getCodecMappingsCountMinus1() { return codecMappingsCountMinus1_; }
  auto& getCodecId(size_t i) { return codecId_[i]; }
  auto& getCodec4cc(size_t i) { return codec4cc_[i]; }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = componentCodecCancelFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = codecMappingsCountMinus1_ & 0xFF;
    byteStrData_.push_back(val);
    for (int i = 0; i < codecId_.size(); i++) {
      val = codecId_[i] & 0xFF;
      byteStrData_.push_back(val);
    }
    for (int i = 0; i < codecId_.size(); i++) {
      char const* strByte = codec4cc_[i].c_str();
      for (int j = 0; j < codec4cc_[i].length(); j++) {
        val = strByte[j] & 0xFF;
        byteStrData_.push_back(val);
      }
    }
    return byteStrData_;
  }

private:
  uint8_t                  componentCodecCancelFlag_ = 0;
  uint8_t                  codecMappingsCountMinus1_ = 0;
  std::vector<uint8_t>     codecId_;
  std::vector<std::string> codec4cc_;
};

// F.2.12.1	Scene object information SEI message syntax
class SEISceneObjectInformation : public SEI {
public:
  SEISceneObjectInformation() {}
  ~SEISceneObjectInformation() {}
  SEISceneObjectInformation&
       operator=(const SEISceneObjectInformation&) = default;
  void allocateObjectIdx() { objectIdx_.resize(numObjectUpdates_); }
  void allocateObjectNumDependencies(size_t i, size_t objectNumDependencies) {
    objectDependencyIdx_[i].resize(objectNumDependencies);
  }
  void allocate(size_t size) {
    if (objectCancelFlag_.size() < size) {
      objectCancelFlag_.resize(size);
      objectLabelUpdateFlag_.resize(size);
      objectLabelIdx_.resize(size);
      priorityUpdateFlag_.resize(size);
      priorityValue_.resize(size);
      objectHiddenFlag_.resize(size);
      objectDependencyUpdateFlag_.resize(size);
      objectNumDependencies_.resize(size);
      visibilityConesUpdateFlag_.resize(size);
      directionX_.resize(size);
      directionY_.resize(size);
      directionZ_.resize(size);
      angle_.resize(size);
      boundingBoxUpdateFlag_.resize(size);
      boundingBoxX_.resize(size);
      boundingBoxY_.resize(size);
      boundingBoxZ_.resize(size);
      boundingBoxDeltaX_.resize(size);
      boundingBoxDeltaY_.resize(size);
      boundingBoxDeltaZ_.resize(size);
      collisionShapeUpdateFlag_.resize(size);
      collisionShapeId_.resize(size);
      pointStyleUpdateFlag_.resize(size);
      pointShapeId_.resize(size);
      pointSize_.resize(size);
      materialIdUpdateFlag_.resize(size);
      materialId_.resize(size);
      objectDependencyIdx_.resize(size);  // 16
    }
  }

  SeiPayloadType getPayloadType() { return SCENE_OBJECT_INFORMATION; }

  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getResetFlag() const { return resetFlag_; }
  auto getNumObjectUpdates() const { return numObjectUpdates_; }
  auto getSimpleObjectsFlag() const { return soiSimpleObjectsFlag_; }
  auto getObjectLabelPresentFlag() const { return objectLabelPresentFlag_; }
  auto getPriorityPresentFlag() const { return priorityPresentFlag_; }
  auto getObjectHiddenPresentFlag() const { return objectHiddenPresentFlag_; }
  auto getObjectDependencyPresentFlag() const {
    return objectDependencyPresentFlag_;
  }
  auto getVisibilityConesPresentFlag() const {
    return visibilityConesPresentFlag_;
  }
  auto get3dBoundingBoxPresentFlag() const { return boundingBoxPresentFlag_; }
  auto getCollisionShapePresentFlag() const {
    return collisionShapePresentFlag_;
  }
  auto getPointStylePresentFlag() const { return pointStylePresentFlag_; }
  auto getMaterialIdPresentFlag() const { return materialIdPresentFlag_; }
  auto getExtensionPresentFlag() const { return extensionPresentFlag_; }
  auto get3dBoundingBoxScaleLog2() const { return boundingBoxScaleLog2_; }
  auto get3dBoundingBoxPrecisionMinus8() const {
    return boundingBoxPrecisionMinus8_;
  }
  auto getLog2MaxObjectIdxUpdated() const { return log2MaxObjectIdxUpdated_; }
  auto getLog2MaxObjectDependencyIdx() const {
    return log2MaxObjectDependencyIdx_;
  }
  auto getObjectIdx(size_t i) const { return objectIdx_[i]; }
  auto getObjectCancelFlag(size_t i) const { return objectCancelFlag_[i]; }
  auto getObjectLabelUpdateFlag(size_t i) const {
    return objectLabelUpdateFlag_[i];
  }
  auto getObjectLabelIdx(size_t i) const { return objectLabelIdx_[i]; }
  auto getPriorityUpdateFlag(size_t i) const { return priorityUpdateFlag_[i]; }
  auto getPriorityValue(size_t i) const { return priorityValue_[i]; }
  auto getObjectHiddenFlag(size_t i) const { return objectHiddenFlag_[i]; }
  auto getObjectDependencyUpdateFlag(size_t i) const {
    return objectDependencyUpdateFlag_[i];
  }
  auto getObjectNumDependencies(size_t i) const {
    return objectNumDependencies_[i];
  }
  auto getObjectDependencyIdx(size_t i, size_t objnum) const {
    return objectDependencyIdx_[i][objnum];
  }
  auto getVisibilityConesUpdateFlag(size_t i) const {
    return visibilityConesUpdateFlag_[i];
  }
  auto getDirectionX(size_t i) const { return directionX_[i]; }
  auto getDirectionY(size_t i) const { return directionY_[i]; }
  auto getDirectionZ(size_t i) const { return directionZ_[i]; }
  auto getAngle(size_t i) const { return angle_[i]; }
  auto get3dBoundingBoxUpdateFlag(size_t i) const {
    return boundingBoxUpdateFlag_[i];
  }
  auto get3dBoundingBoxX(size_t i) const { return boundingBoxX_[i]; }
  auto get3dBoundingBoxY(size_t i) const { return boundingBoxY_[i]; }
  auto get3dBoundingBoxZ(size_t i) const { return boundingBoxZ_[i]; }
  auto get3dBoundingBoxDeltaX(size_t i) const { return boundingBoxDeltaX_[i]; }
  auto get3dBoundingBoxDeltaY(size_t i) const { return boundingBoxDeltaY_[i]; }
  auto get3dBoundingBoxDeltaZ(size_t i) const { return boundingBoxDeltaZ_[i]; }
  auto getCollisionShapeUpdateFlag(size_t i) const {
    return collisionShapeUpdateFlag_[i];
  }
  auto getCollisionShapeId(size_t i) const { return collisionShapeId_[i]; }
  auto getPointStyleUpdateFlag(size_t i) const {
    return pointStyleUpdateFlag_[i];
  }
  auto getPointShapeId(size_t i) const { return pointShapeId_[i]; }
  auto getPointSize(size_t i) const { return pointSize_[i]; }
  auto getMaterialIdUpdateFlag(size_t i) const {
    return materialIdUpdateFlag_[i];
  }
  auto getMaterialId(size_t i) const { return materialId_[i]; }

  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getResetFlag() { return resetFlag_; }
  auto& getNumObjectUpdates() { return numObjectUpdates_; }
  auto& getSimpleObjectsFlag() { return soiSimpleObjectsFlag_; }
  auto& getObjectLabelPresentFlag() { return objectLabelPresentFlag_; }
  auto& getPriorityPresentFlag() { return priorityPresentFlag_; }
  auto& getObjectHiddenPresentFlag() { return objectHiddenPresentFlag_; }
  auto& getObjectDependencyPresentFlag() {
    return objectDependencyPresentFlag_;
  }
  auto& getVisibilityConesPresentFlag() { return visibilityConesPresentFlag_; }
  auto& get3dBoundingBoxPresentFlag() { return boundingBoxPresentFlag_; }
  auto& getCollisionShapePresentFlag() { return collisionShapePresentFlag_; }
  auto& getPointStylePresentFlag() { return pointStylePresentFlag_; }
  auto& getMaterialIdPresentFlag() { return materialIdPresentFlag_; }
  auto& getExtensionPresentFlag() { return extensionPresentFlag_; }
  auto& get3dBoundingBoxScaleLog2() { return boundingBoxScaleLog2_; }
  auto& get3dBoundingBoxPrecisionMinus8() {
    return boundingBoxPrecisionMinus8_;
  }
  auto& getLog2MaxObjectIdxUpdated() { return log2MaxObjectIdxUpdated_; }
  auto& getLog2MaxObjectDependencyIdx() { return log2MaxObjectDependencyIdx_; }
  auto& getObjectIdx() { return objectIdx_; }
  auto& getObjectIdx(size_t i) { return objectIdx_[i]; }
  auto& getObjectCancelFlag(size_t i) { return objectCancelFlag_[i]; }
  auto& getObjectLabelUpdateFlag(size_t i) {
    return objectLabelUpdateFlag_[i];
  }
  auto& getObjectLabelIdx(size_t i) { return objectLabelIdx_[i]; }
  auto& getPriorityUpdateFlag(size_t i) { return priorityUpdateFlag_[i]; }
  auto& getPriorityValue(size_t i) { return priorityValue_[i]; }
  auto& getObjectHiddenFlag(size_t i) { return objectHiddenFlag_[i]; }
  auto& getObjectDependencyUpdateFlag(size_t i) {
    return objectDependencyUpdateFlag_[i];
  }
  auto& getObjectNumDependencies(size_t i) {
    return objectNumDependencies_[i];
  }
  auto& getObjectDependencyIdx(size_t i, size_t objnum) {
    return objectDependencyIdx_[i][objnum];
  }
  auto& getVisibilityConesUpdateFlag(size_t i) {
    return visibilityConesUpdateFlag_[i];
  }
  auto& getDirectionX(size_t i) { return directionX_[i]; }
  auto& getDirectionY(size_t i) { return directionY_[i]; }
  auto& getDirectionZ(size_t i) { return directionZ_[i]; }
  auto& getAngle(size_t i) { return angle_[i]; }
  auto& get3dBoundingBoxUpdateFlag(size_t i) {
    return boundingBoxUpdateFlag_[i];
  }
  auto& get3dBoundingBoxX(size_t i) { return boundingBoxX_[i]; }
  auto& get3dBoundingBoxY(size_t i) { return boundingBoxY_[i]; }
  auto& get3dBoundingBoxZ(size_t i) { return boundingBoxZ_[i]; }
  auto& get3dBoundingBoxDeltaX(size_t i) { return boundingBoxDeltaX_[i]; }
  auto& get3dBoundingBoxDeltaY(size_t i) { return boundingBoxDeltaY_[i]; }
  auto& get3dBoundingBoxDeltaZ(size_t i) { return boundingBoxDeltaZ_[i]; }
  auto& getCollisionShapeUpdateFlag(size_t i) {
    return collisionShapeUpdateFlag_[i];
  }
  auto& getCollisionShapeId(size_t i) { return collisionShapeId_[i]; }
  auto& getPointStyleUpdateFlag(size_t i) { return pointStyleUpdateFlag_[i]; }
  auto& getPointShapeId(size_t i) { return pointShapeId_[i]; }
  auto& getPointSize(size_t i) { return pointSize_[i]; }
  auto& getMaterialIdUpdateFlag(size_t i) { return materialIdUpdateFlag_[i]; }
  auto& getMaterialId(size_t i) { return materialId_[i]; }

private:
  bool                               persistenceFlag_             = true;
  bool                               resetFlag_                   = true;
  uint32_t                           numObjectUpdates_            = 0;
  bool                               soiSimpleObjectsFlag_        = false;
  bool                               objectLabelPresentFlag_      = false;
  bool                               priorityPresentFlag_         = false;
  bool                               objectHiddenPresentFlag_     = false;
  bool                               objectDependencyPresentFlag_ = false;
  bool                               visibilityConesPresentFlag_  = false;
  bool                               boundingBoxPresentFlag_      = false;
  bool                               collisionShapePresentFlag_   = false;
  bool                               pointStylePresentFlag_       = false;
  bool                               materialIdPresentFlag_       = false;
  bool                               extensionPresentFlag_        = false;
  uint8_t                            boundingBoxScaleLog2_        = 0;
  uint8_t                            boundingBoxPrecisionMinus8_  = 0;
  uint8_t                            log2MaxObjectIdxUpdated_     = 0;
  uint8_t                            log2MaxObjectDependencyIdx_  = 0;
  std::vector<uint32_t>              objectIdx_;
  std::vector<uint8_t>               objectCancelFlag_;
  std::vector<uint8_t>               objectLabelUpdateFlag_;
  std::vector<uint32_t>              objectLabelIdx_;
  std::vector<uint8_t>               priorityUpdateFlag_;
  std::vector<uint8_t>               priorityValue_;
  std::vector<uint8_t>               objectHiddenFlag_;
  std::vector<uint8_t>               objectDependencyUpdateFlag_;
  std::vector<uint8_t>               objectNumDependencies_;
  std::vector<std::vector<uint32_t>> objectDependencyIdx_;  // 16
  std::vector<uint8_t>               visibilityConesUpdateFlag_;
  std::vector<uint32_t>              directionX_;
  std::vector<uint32_t>              directionY_;
  std::vector<uint32_t>              directionZ_;
  std::vector<uint16_t>              angle_;
  std::vector<uint8_t>               boundingBoxUpdateFlag_;
  std::vector<uint32_t>              boundingBoxX_;
  std::vector<uint32_t>              boundingBoxY_;
  std::vector<uint32_t>              boundingBoxZ_;
  std::vector<uint32_t>              boundingBoxDeltaX_;
  std::vector<uint32_t>              boundingBoxDeltaY_;
  std::vector<uint32_t>              boundingBoxDeltaZ_;
  std::vector<uint8_t>               collisionShapeUpdateFlag_;
  std::vector<uint16_t>              collisionShapeId_;
  std::vector<uint8_t>               pointStyleUpdateFlag_;
  std::vector<uint8_t>               pointShapeId_;
  std::vector<uint16_t>              pointSize_;
  std::vector<uint8_t>               materialIdUpdateFlag_;
  std::vector<uint16_t>              materialId_;
};

// F.2.12.2 Object label information SEI message syntax
class SEIObjectLabelInformation : public SEI {
public:
  SEIObjectLabelInformation() {}
  ~SEIObjectLabelInformation() {}
  SEIObjectLabelInformation&
  operator=(const SEIObjectLabelInformation&) = default;

  SeiPayloadType getPayloadType() { return OBJECT_LABEL_INFORMATION; }

  void allocate() { labelIdx_.resize(numLabelUpdates_, 0); }

  auto getCancelFlag() const { return cancelFlag_; }
  auto getLabelLanguagePresentFlag() const {
    return labelLanguagePresentFlag_;
  }
  auto& getLabelLanguage() const { return labelLanguage_; }
  auto  getNumLabelUpdates() const { return numLabelUpdates_; }
  auto  getLabelIdx(size_t i) const { return labelIdx_[i]; }
  auto  getLabelCancelFlag() const { return labelCancelFlag_; }
  auto  getBitEqualToZero() const { return bitEqualToZero_; }
  auto& getLabel(size_t i) const { return label_[i]; }
  auto  getPersistenceFlag() const { return persistenceFlag_; }

  auto& getCancelFlag() { return cancelFlag_; }
  auto& getLabelLanguagePresentFlag() { return labelLanguagePresentFlag_; }
  auto& getLabelLanguage() { return labelLanguage_; }
  auto& getNumLabelUpdates() { return numLabelUpdates_; }
  auto& getLabelIdx(size_t i) { return labelIdx_[i]; }
  auto& getLabelCancelFlag() { return labelCancelFlag_; }
  auto& getBitEqualToZero() { return bitEqualToZero_; }
  auto& getLabel(size_t i) { return label_[i]; }
  auto& getPersistenceFlag() { return persistenceFlag_; }

private:
  bool                     cancelFlag_               = false;
  bool                     labelLanguagePresentFlag_ = false;
  std::string              labelLanguage_;
  uint32_t                 numLabelUpdates_ = 0;
  std::vector<uint32_t>    labelIdx_;
  bool                     labelCancelFlag_ = false;
  bool                     bitEqualToZero_  = false;
  std::vector<std::string> label_;
  bool                     persistenceFlag_ = false;
};

// F.2.12.3 Patch information SEI message syntax
class SEIPatchInformation : public SEI {
public:
  SEIPatchInformation() {}
  ~SEIPatchInformation() {}
  SEIPatchInformation& operator=(const SEIPatchInformation&) = default;
  SeiPayloadType       getPayloadType() { return PATCH_INFORMATION; }

  void allocate() {}  // TODO: allocate data

  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getResetFlag() const { return resetFlag_; }
  auto getNumTileUpdates() const { return numTileUpdates_; }
  auto getLog2MaxObjectIdxTracked() const { return log2MaxObjectIdxTracked_; }
  auto getLog2MaxPatchIdxUpdated() const { return log2MaxPatchIdxUpdated_; }
  auto getTileId(size_t i) const { return tileId_[i]; }
  auto getTileCancelFlag(size_t i) const { return tileCancelFlag_[i]; }
  auto getNumPatchUpdates(size_t i) const { return numPatchUpdates_[i]; }
  auto getPatchIdx(size_t i, size_t j) const { return patchIdx_[i][j]; }
  auto getPatchCancelFlag(size_t i, size_t j) const {
    return patchCancelFlag_[i][j];
  }
  auto getPatchNumberOfObjectsMinus1(size_t i, size_t j) const {
    return patchNumberOfObjectsMinus1_[i][j];
  }
  auto getPatchObjectIdx(size_t i, size_t j, size_t k) const {
    return patchObjectIdx_[i][j][k];
  }

  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getResetFlag() { return resetFlag_; }
  auto& getNumTileUpdates() { return numTileUpdates_; }
  auto& getLog2MaxObjectIdxTracked() { return log2MaxObjectIdxTracked_; }
  auto& getLog2MaxPatchIdxUpdated() { return log2MaxPatchIdxUpdated_; }
  auto& getTileId(size_t i) { return tileId_[i]; }
  auto& getTileCancelFlag(size_t i) { return tileCancelFlag_[i]; }
  auto& getNumPatchUpdates(size_t i) { return numPatchUpdates_[i]; }
  auto& getPatchIdx(size_t i, size_t j) { return patchIdx_[i][j]; }
  auto& getPatchCancelFlag(size_t i, size_t j) {
    return patchCancelFlag_[i][j];
  }
  auto& getPatchNumberOfObjectsMinus1(size_t i, size_t j) {
    return patchNumberOfObjectsMinus1_[i][j];
  }
  auto& getPatchObjectIdx(size_t i, size_t j, size_t k) {
    return patchObjectIdx_[i][j][k];
  }

private:
  bool                                            persistenceFlag_ = false;
  bool                                            resetFlag_       = false;
  uint32_t                                        numTileUpdates_  = 0;
  uint8_t                                         log2MaxObjectIdxTracked_ = 0;
  uint8_t                                         log2MaxPatchIdxUpdated_  = 0;
  std::vector<uint32_t>                           tileId_;
  std::vector<uint8_t>                            tileCancelFlag_;
  std::vector<uint32_t>                           numPatchUpdates_;
  std::vector<std::vector<uint32_t>>              patchIdx_;
  std::vector<std::vector<uint8_t>>               patchCancelFlag_;
  std::vector<std::vector<uint32_t>>              patchNumberOfObjectsMinus1_;
  std::vector<std::vector<std::vector<uint32_t>>> patchObjectIdx_;
};

// F.2.12.4 Volumetric rectangle information SEI message syntax
class SEIVolumetricRectangleInformation : public SEI {
public:
  SEIVolumetricRectangleInformation() {}
  ~SEIVolumetricRectangleInformation() {}
  SEIVolumetricRectangleInformation&
       operator=(const SEIVolumetricRectangleInformation&) = default;
  void allocate(size_t size) {
    boundingBoxUpdateFlag_.resize(size);
    boundingBoxTop_.resize(size);
    boundingBoxLeft_.resize(size);
    boundingBoxWidth_.resize(size);
    boundingBoxHeight_.resize(size);
    rectangleObjectIdx_.resize(size);
  }
  void allocateRectangleObjectIdx(size_t i, size_t size) {
    rectangleObjectIdx_[i].resize(size);
  }

  SeiPayloadType getPayloadType() { return VOLUMETRIC_RECTANGLE_INFORMATION; }

  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getResetFlag() const { return resetFlag_; }
  auto getNumRectanglesUpdates() const { return numRectanglesUpdates_; }
  auto getLog2MaxObjectIdxTracked() const { return log2MaxObjectIdxTracked_; }
  auto getLog2MaxRectangleIdxUpdated() const {
    return log2MaxRectangleIdxUpdated_;
  }
  auto getRectangleIdx(size_t i) const { return rectangleIdx_[i]; }
  auto getRectangleCancelFlag(size_t i) const {
    return rectangleCancelFlag_[i];
  }
  auto getBoundingBoxUpdateFlag(size_t i) const {
    return boundingBoxUpdateFlag_[i];
  }
  auto getBoundingBoxTop(size_t i) const { return boundingBoxTop_[i]; }
  auto getBoundingBoxLeft(size_t i) const { return boundingBoxLeft_[i]; }
  auto getBoundingBoxWidth(size_t i) const { return boundingBoxWidth_[i]; }
  auto getBoundingBoxHeight(size_t i) const { return boundingBoxHeight_[i]; }
  auto getRectangleNumberOfObjectsMinus1(size_t i) const {
    return rectangleNumberOfObjectsMinus1_[i];
  }
  auto getRectangleObjectIdx(size_t i, size_t j) const {
    return rectangleObjectIdx_[i][j];
  }
  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getResetFlag() { return resetFlag_; }
  auto& getNumRectanglesUpdates() { return numRectanglesUpdates_; }
  auto& getLog2MaxObjectIdxTracked() { return log2MaxObjectIdxTracked_; }
  auto& getLog2MaxRectangleIdxUpdated() { return log2MaxRectangleIdxUpdated_; }
  auto& getRectangleIdx(size_t i) { return rectangleIdx_[i]; }
  auto& getRectangleCancelFlag(size_t i) { return rectangleCancelFlag_[i]; }
  auto& getBoundingBoxUpdateFlag(size_t i) {
    return boundingBoxUpdateFlag_[i];
  }
  auto& getBoundingBoxTop(size_t i) { return boundingBoxTop_[i]; }
  auto& getBoundingBoxLeft(size_t i) { return boundingBoxLeft_[i]; }
  auto& getBoundingBoxWidth(size_t i) { return boundingBoxWidth_[i]; }
  auto& getBoundingBoxHeight(size_t i) { return boundingBoxHeight_[i]; }
  auto& getRectangleNumberOfObjectsMinus1(size_t i) {
    return rectangleNumberOfObjectsMinus1_[i];
  }
  auto& getRectangleObjectIdx(size_t i, size_t j) {
    return rectangleObjectIdx_[i][j];
  }

private:
  bool                               persistenceFlag_            = false;
  bool                               resetFlag_                  = false;
  uint32_t                           numRectanglesUpdates_       = 0;
  uint8_t                            log2MaxObjectIdxTracked_    = 0;
  uint8_t                            log2MaxRectangleIdxUpdated_ = 0;
  std::vector<uint32_t>              rectangleIdx_;
  std::vector<uint8_t>               rectangleCancelFlag_;
  std::vector<uint8_t>               boundingBoxUpdateFlag_;
  std::vector<uint32_t>              boundingBoxTop_;
  std::vector<uint32_t>              boundingBoxLeft_;
  std::vector<uint32_t>              boundingBoxWidth_;
  std::vector<uint32_t>              boundingBoxHeight_;
  std::vector<uint32_t>              rectangleNumberOfObjectsMinus1_;
  std::vector<std::vector<uint32_t>> rectangleObjectIdx_;
};

// F.2.13  Buffering period SEI message syntax
class SEIBufferingPeriod : public SEI {
public:
  SEIBufferingPeriod() {}
  ~SEIBufferingPeriod() {
    for (auto& element : nalInitialCabRemovalDelay_) { element.clear(); }
    for (auto& element : nalInitialCabRemovalOffset_) { element.clear(); }
    for (auto& element : aclInitialCabRemovalDelay_) { element.clear(); }
    for (auto& element : aclInitialCabRemovalOffset_) { element.clear(); }
    for (auto& element : nalInitialAltCabRemovalDelay_) { element.clear(); }
    for (auto& element : nalInitialAltCabRemovalOffset_) { element.clear(); }
    for (auto& element : aclInitialAltCabRemovalDelay_) { element.clear(); }
    for (auto& element : aclInitialAltCabRemovalOffset_) { element.clear(); }
    hrdCabCntMinus1_.clear();
    nalInitialCabRemovalDelay_.clear();
    nalInitialCabRemovalOffset_.clear();
    aclInitialCabRemovalDelay_.clear();
    aclInitialCabRemovalOffset_.clear();
    nalInitialAltCabRemovalDelay_.clear();
    nalInitialAltCabRemovalOffset_.clear();
    aclInitialAltCabRemovalDelay_.clear();
    aclInitialAltCabRemovalOffset_.clear();
  }
  SEIBufferingPeriod& operator=(const SEIBufferingPeriod&) = default;

  SeiPayloadType getPayloadType() { return BUFFERING_PERIOD; }
  void           allocate() {
    hrdCabCntMinus1_.resize(maxSubLayersMinus1_ + 1);
    nalInitialCabRemovalDelay_.resize(maxSubLayersMinus1_ + 1);
    nalInitialCabRemovalOffset_.resize(maxSubLayersMinus1_ + 1);
    aclInitialCabRemovalDelay_.resize(maxSubLayersMinus1_ + 1);
    aclInitialCabRemovalOffset_.resize(maxSubLayersMinus1_ + 1);
    nalInitialAltCabRemovalDelay_.resize(maxSubLayersMinus1_ + 1);
    nalInitialAltCabRemovalOffset_.resize(maxSubLayersMinus1_ + 1);
    aclInitialAltCabRemovalDelay_.resize(maxSubLayersMinus1_ + 1);
    aclInitialAltCabRemovalOffset_.resize(maxSubLayersMinus1_ + 1);
  }
  void allocate(size_t i) {
    nalInitialCabRemovalDelay_[i].resize(hrdCabCntMinus1_[i] + 1);
    nalInitialCabRemovalOffset_[i].resize(hrdCabCntMinus1_[i] + 1);
    aclInitialCabRemovalDelay_[i].resize(hrdCabCntMinus1_[i] + 1);
    aclInitialCabRemovalOffset_[i].resize(hrdCabCntMinus1_[i] + 1);
    nalInitialAltCabRemovalDelay_[i].resize(hrdCabCntMinus1_[i] + 1);
    nalInitialAltCabRemovalOffset_[i].resize(hrdCabCntMinus1_[i] + 1);
    aclInitialAltCabRemovalDelay_[i].resize(hrdCabCntMinus1_[i] + 1);
    aclInitialAltCabRemovalOffset_[i].resize(hrdCabCntMinus1_[i] + 1);
  }

  auto getNalHrdParamsPresentFlag() const { return nalHrdParamsPresentFlag_; }
  auto getAclHrdParamsPresentFlag() const { return aclHrdParamsPresentFlag_; }
  auto getInitialCabRemovalDelayLengthMinus1() const {
    return initialCabRemovalDelayLengthMinus1_;
  }
  auto getAuCabRemovalDelayLengthMinus1() const {
    return auCabRemovalDelayLengthMinus1_;
  }
  auto getDabOutputDelayLengthMinus1() const {
    return dabOutputDelayLengthMinus1_;
  }
  auto getIrapCabParamsPresentFlag() const {
    return irapCabParamsPresentFlag_;
  }
  auto getConcatenationFlag() const { return concatenationFlag_; }
  auto getAtlasSequenceParameterSetId() const {
    return atlasSequenceParameterSetId_;
  }
  auto getCabDelayOffset() const { return cabDelayOffset_; }
  auto getDabDelayOffset() const { return dabDelayOffset_; }
  auto getAtlasCabRemovalDelayDeltaMinus1() const {
    return atlasCabRemovalDelayDeltaMinus1_;
  }
  auto  getMaxSubLayersMinus1() const { return maxSubLayersMinus1_; }
  auto& getHrdCabCntMinus1() const { return hrdCabCntMinus1_; }
  auto& getNalInitialCabRemovalDelay() const {
    return nalInitialCabRemovalDelay_;
  }
  auto& getNalInitialCabRemovalOffse() const {
    return nalInitialCabRemovalOffset_;
  }
  auto& getNalInitialAltCabRemovalDelay() const {
    return nalInitialAltCabRemovalDelay_;
  }
  auto& getNalInitialAltCabRemovalOffset() const {
    return nalInitialAltCabRemovalOffset_;
  }
  auto& getAclInitialCabRemovalDelay() const {
    return aclInitialCabRemovalDelay_;
  }
  auto& getAclInitialCabRemovalOffset() const {
    return aclInitialCabRemovalOffset_;
  }
  auto& getAclInitialAltCabRemovalDelay() const {
    return aclInitialAltCabRemovalDelay_;
  }
  auto& getAclInitialAltCabRemovalOffset() const {
    return aclInitialAltCabRemovalOffset_;
  }
  auto getHrdCabCntMinus1(size_t i) const { return hrdCabCntMinus1_[i]; }
  auto getNalInitialCabRemovalDelay(size_t i, size_t j) const {
    return nalInitialCabRemovalDelay_[i][j];
  }
  auto getNalInitialCabRemovalOffset(size_t i, size_t j) const {
    return nalInitialCabRemovalOffset_[i][j];
  }
  auto getAclInitialCabRemovalDelay(size_t i, size_t j) const {
    return aclInitialCabRemovalDelay_[i][j];
  }
  auto getAclInitialCabRemovalOffset(size_t i, size_t j) const {
    return aclInitialCabRemovalOffset_[i][j];
  }
  auto getNalInitialAltCabRemovalDelay(size_t i, size_t j) const {
    return nalInitialAltCabRemovalDelay_[i][j];
  }
  auto getNalInitialAltCabRemovalOffset(size_t i, size_t j) const {
    return nalInitialAltCabRemovalOffset_[i][j];
  }
  auto getAclInitialAltCabRemovalDelay(size_t i, size_t j) const {
    return aclInitialAltCabRemovalDelay_[i][j];
  }
  auto getAclInitialAltCabRemovalOffset(size_t i, size_t j) const {
    return aclInitialAltCabRemovalOffset_[i][j];
  }
  auto& getNalInitialCabRemovalDelay(size_t i) const {
    return nalInitialCabRemovalDelay_[i];
  }
  auto& getNalInitialCabRemovalOffsey(size_t i) const {
    return nalInitialCabRemovalOffset_[i];
  }
  auto& getAclInitialCabRemovalDelay(size_t i) const {
    return aclInitialCabRemovalDelay_[i];
  }
  auto& getAclInitialCabRemovalOffset(size_t i) const {
    return aclInitialCabRemovalOffset_[i];
  }
  auto& getNalInitialAltCabRemovalDelay(size_t i) const {
    return nalInitialAltCabRemovalDelay_[i];
  }
  auto& getNalInitialAltCabRemovalOffset(size_t i) const {
    return nalInitialAltCabRemovalOffset_[i];
  }
  auto& getAclInitialAltCabRemovalDelay(size_t i) const {
    return aclInitialAltCabRemovalDelay_[i];
  }
  auto& getAclInitialAltCabRemovalOffset(size_t i) const {
    return aclInitialAltCabRemovalOffset_[i];
  }

  auto& getNalHrdParamsPresentFlag() { return nalHrdParamsPresentFlag_; }
  auto& getAclHrdParamsPresentFlag() { return aclHrdParamsPresentFlag_; }
  auto& getInitialCabRemovalDelayLengthMinus1() {
    return initialCabRemovalDelayLengthMinus1_;
  }
  auto& getAuCabRemovalDelayLengthMinus1() {
    return auCabRemovalDelayLengthMinus1_;
  }
  auto& getDabOutputDelayLengthMinus1() { return dabOutputDelayLengthMinus1_; }
  auto& getIrapCabParamsPresentFlag() { return irapCabParamsPresentFlag_; }
  auto& getConcatenationFlag() { return concatenationFlag_; }
  auto& getAtlasSequenceParameterSetId() {
    return atlasSequenceParameterSetId_;
  }
  auto& getCabDelayOffset() { return cabDelayOffset_; }
  auto& getDabDelayOffset() { return dabDelayOffset_; }
  auto& getAtlasCabRemovalDelayDeltaMinus1() {
    return atlasCabRemovalDelayDeltaMinus1_;
  }
  auto& getMaxSubLayersMinus1() { return maxSubLayersMinus1_; }
  auto& getHrdCabCntMinus1() { return hrdCabCntMinus1_; }
  auto& getNalInitialCabRemovalDelay() { return nalInitialCabRemovalDelay_; }
  auto& getNalInitialCabRemovalOffse() { return nalInitialCabRemovalOffset_; }
  auto& getNalInitialAltCabRemovalDelay() {
    return nalInitialAltCabRemovalDelay_;
  }
  auto& getNalInitialAltCabRemovalOffset() {
    return nalInitialAltCabRemovalOffset_;
  }
  auto& getAclInitialCabRemovalDelay() { return aclInitialCabRemovalDelay_; }
  auto& getAclInitialCabRemovalOffset() { return aclInitialCabRemovalOffset_; }
  auto& getAclInitialAltCabRemovalDelay() {
    return aclInitialAltCabRemovalDelay_;
  }
  auto& getAclInitialAltCabRemovalOffset() {
    return aclInitialAltCabRemovalOffset_;
  }
  auto& getHrdCabCntMinus1(size_t i) { return hrdCabCntMinus1_[i]; }
  auto& getNalInitialCabRemovalDelay(size_t i, size_t j) {
    return nalInitialCabRemovalDelay_[i][j];
  }
  auto& getNalInitialCabRemovalOffset(size_t i, size_t j) {
    return nalInitialCabRemovalOffset_[i][j];
  }
  auto& getAclInitialCabRemovalDelay(size_t i, size_t j) {
    return aclInitialCabRemovalDelay_[i][j];
  }
  auto& getAclInitialCabRemovalOffset(size_t i, size_t j) {
    return aclInitialCabRemovalOffset_[i][j];
  }
  auto& getNalInitialAltCabRemovalDelay(size_t i, size_t j) {
    return nalInitialAltCabRemovalDelay_[i][j];
  }
  auto& getNalInitialAltCabRemovalOffset(size_t i, size_t j) {
    return nalInitialAltCabRemovalOffset_[i][j];
  }
  auto& getAclInitialAltCabRemovalDelay(size_t i, size_t j) {
    return aclInitialAltCabRemovalDelay_[i][j];
  }
  auto& getAclInitialAltCabRemovalOffset(size_t i, size_t j) {
    return aclInitialAltCabRemovalOffset_[i][j];
  }
  auto& getNalInitialCabRemovalDelay(size_t i) {
    return nalInitialCabRemovalDelay_[i];
  }
  auto& getNalInitialCabRemovalOffsey(size_t i) {
    return nalInitialCabRemovalOffset_[i];
  }
  auto& getAclInitialCabRemovalDelay(size_t i) {
    return aclInitialCabRemovalDelay_[i];
  }
  auto& getAclInitialCabRemovalOffset(size_t i) {
    return aclInitialCabRemovalOffset_[i];
  }
  auto& getNalInitialAltCabRemovalDelay(size_t i) {
    return nalInitialAltCabRemovalDelay_[i];
  }
  auto& getNalInitialAltCabRemovalOffset(size_t i) {
    return nalInitialAltCabRemovalOffset_[i];
  }
  auto& getAclInitialAltCabRemovalDelay(size_t i) {
    return aclInitialAltCabRemovalDelay_[i];
  }
  auto& getAclInitialAltCabRemovalOffset(size_t i) {
    return aclInitialAltCabRemovalOffset_[i];
  }

private:
  bool                  nalHrdParamsPresentFlag_            = false;
  bool                  aclHrdParamsPresentFlag_            = false;
  bool                  initialCabRemovalDelayLengthMinus1_ = false;
  bool                  auCabRemovalDelayLengthMinus1_      = false;
  bool                  dabOutputDelayLengthMinus1_         = false;
  bool                  irapCabParamsPresentFlag_           = false;
  bool                  concatenationFlag_                  = false;
  uint8_t               atlasSequenceParameterSetId_        = 0;
  uint32_t              cabDelayOffset_                     = 0;
  uint32_t              dabDelayOffset_                     = 0;
  uint32_t              atlasCabRemovalDelayDeltaMinus1_    = 0;
  uint32_t              maxSubLayersMinus1_                 = 0;
  std::vector<uint32_t> hrdCabCntMinus1_;
  std::vector<std::vector<uint32_t>> nalInitialCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> nalInitialCabRemovalOffset_;
  std::vector<std::vector<uint32_t>> nalInitialAltCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> nalInitialAltCabRemovalOffset_;
  std::vector<std::vector<uint32_t>> aclInitialCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> aclInitialCabRemovalOffset_;
  std::vector<std::vector<uint32_t>> aclInitialAltCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> aclInitialAltCabRemovalOffset_;
};

// F.2.14  Atlas frame timing SEI message syntax
class SEIAtlasFrameTiming : public SEI {
public:
  SEIAtlasFrameTiming() {}
  ~SEIAtlasFrameTiming() {
    cabRemovalDelayMinus1_.clear();
    dabOutputDelay_.clear();
  }
  SEIAtlasFrameTiming& operator=(const SEIAtlasFrameTiming&) = default;

  SeiPayloadType getPayloadType() { return ATLAS_FRAME_TIMING; }

  void allocate(int32_t size) {
    cabRemovalDelayMinus1_.resize(size, 0);
    dabOutputDelay_.resize(size, 0);
  }

  auto& getAftCabRemovalDelayMinus1() const { return cabRemovalDelayMinus1_; }
  auto& getAftDabOutputDelay() const { return dabOutputDelay_; }
  auto  getAftCabRemovalDelayMinus1(uint32_t i) const {
    return cabRemovalDelayMinus1_[i];
  }
  auto getAftDabOutputDelay(uint32_t i) const { return dabOutputDelay_[i]; }

  auto& getAftCabRemovalDelayMinus1() { return cabRemovalDelayMinus1_; }
  auto& getAftDabOutputDelay() { return dabOutputDelay_; }
  auto& getAftCabRemovalDelayMinus1(uint32_t i) {
    return cabRemovalDelayMinus1_[i];
  }
  auto& getAftDabOutputDelay(uint32_t i) { return dabOutputDelay_[i]; }

private:
  std::vector<uint32_t> cabRemovalDelayMinus1_;
  std::vector<uint32_t> dabOutputDelay_;
};

// F.2.12.5  Atlas object information  SEI message syntax
class SEIAtlasInformation : public SEI {
public:
  SEIAtlasInformation() {}
  ~SEIAtlasInformation() {
    atlasId_.clear();
    objectIdx_.clear();
    for (auto& element : objectInAtlasPresentFlag_) { element.clear(); }
    objectInAtlasPresentFlag_.clear();
  }
  SEIAtlasInformation& operator=(const SEIAtlasInformation&) = default;

  SeiPayloadType getPayloadType() { return ATLAS_OBJECT_INFORMATION; }

  void allocateAltasId() { atlasId_.resize(numAtlasesMinus1_ + 1, 0); }
  void allocateObjectIdx() {
    objectIdx_.resize(numUpdates_, 0);
    objectInAtlasPresentFlag_.resize(numUpdates_);
    for (auto& element : objectInAtlasPresentFlag_) {
      element.resize(numAtlasesMinus1_ + 1, false);
    }
  }

  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getResetFlag() const { return resetFlag_; }
  auto getNumAtlasesMinus1() const { return numAtlasesMinus1_; }
  auto getNumUpdates() const { return numUpdates_; }
  auto getLog2MaxObjectIdxTracked() const { return log2MaxObjectIdxTracked_; }
  auto getAtlasId(size_t i) const { return atlasId_[i]; }
  auto getObjectIdx(size_t i) const { return objectIdx_[i]; }
  auto getObjectInAtlasPresentFlag(size_t i, size_t j) const {
    return objectInAtlasPresentFlag_[i][j];
  }

  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getResetFlag() { return resetFlag_; }
  auto& getNumAtlasesMinus1() { return numAtlasesMinus1_; }
  auto& getNumUpdates() { return numUpdates_; }
  auto& getLog2MaxObjectIdxTracked() { return log2MaxObjectIdxTracked_; }
  auto& getAtlasId(size_t i) { return atlasId_[i]; }
  auto& getObjectIdx(size_t i) { return objectIdx_[i]; }
  auto& getObjectInAtlasPresentFlag(size_t i, size_t j) {
    return objectInAtlasPresentFlag_[i][j];
  }

private:
  bool                              persistenceFlag_         = false;
  bool                              resetFlag_               = false;
  uint32_t                          numAtlasesMinus1_        = 0;
  uint32_t                          numUpdates_              = 0;
  uint32_t                          log2MaxObjectIdxTracked_ = 0;
  std::vector<uint32_t>             atlasId_;
  std::vector<uint32_t>             objectIdx_;
  std::vector<std::vector<uint8_t>> objectInAtlasPresentFlag_;
};

// F.2.15.1	Viewport camera parameters SEI messages syntax
class SEIViewportCameraParameters : public SEI {
public:
  SEIViewportCameraParameters() {}
  ~SEIViewportCameraParameters() {}
  SEIViewportCameraParameters&
  operator=(const SEIViewportCameraParameters&) = default;

  SeiPayloadType getPayloadType() { return VIEWPORT_CAMERA_PARAMETERS; }

  auto getCameraId() const { return cameraId_; }
  auto getCancelFlag() const { return cancelFlag_; }
  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getCameraType() const { return cameraType_; }
  auto getErpHorizontalFov() const { return erpHorizontalFov_; }
  auto getErpVerticalFov() const { return erpVerticalFov_; }
  auto getPerspectiveAspectRatio() const { return perspectiveAspectRatio_; }
  auto getPerspectiveHorizontalFov() const {
    return perspectiveHorizontalFov_;
  }
  auto getOrthoAspectRatio() const { return orthoAspectRatio_; }
  auto getOrthoHorizontalSize() const { return orthoHorizontalSize_; }
  auto getClippingNearPlane() const { return clippingNearPlane_; }
  auto getClippingFarPlane() const { return clippingFarPlane_; }

  auto& getCameraId() { return cameraId_; }
  auto& getCancelFlag() { return cancelFlag_; }
  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getCameraType() { return cameraType_; }
  auto& getErpHorizontalFov() { return erpHorizontalFov_; }
  auto& getErpVerticalFov() { return erpVerticalFov_; }
  auto& getPerspectiveAspectRatio() { return perspectiveAspectRatio_; }
  auto& getPerspectiveHorizontalFov() { return perspectiveHorizontalFov_; }
  auto& getOrthoAspectRatio() { return orthoAspectRatio_; }
  auto& getOrthoHorizontalSize() { return orthoHorizontalSize_; }
  auto& getClippingNearPlane() { return clippingNearPlane_; }
  auto& getClippingFarPlane() { return clippingFarPlane_; }

private:
  uint32_t cameraId_                 = 0;
  bool     cancelFlag_               = false;
  bool     persistenceFlag_          = false;
  uint8_t  cameraType_               = 0;
  float    erpHorizontalFov_         = 0.f;
  float    erpVerticalFov_           = 0.f;
  float    perspectiveAspectRatio_   = 0.f;
  float    perspectiveHorizontalFov_ = 0.f;
  float    orthoAspectRatio_         = 0.f;
  float    orthoHorizontalSize_      = 0.f;
  float    clippingNearPlane_        = 0.f;
  float    clippingFarPlane_         = 0.f;
};

// F.2.15.2	Viewport position SEI messages syntax
class SEIViewportPosition : public SEI {
public:
  SEIViewportPosition() {}
  ~SEIViewportPosition() {}
  SEIViewportPosition& operator=(const SEIViewportPosition&) = default;

  SeiPayloadType getPayloadType() { return VIEWPORT_POSITION; }

  auto getViewportId() const { return viewportId_; }
  auto getCameraParametersPresentFlag() const {
    return cameraParametersPresentFlag_;
  }
  auto getVcpCameraId() const { return CameraId_; }
  auto getCancelFlag() const { return cancelFlag_; }
  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getPosition(size_t i) const { return position_[i]; }
  auto getRotationQX() const { return rotationQX_; }
  auto getRotationQY() const { return rotationQY_; }
  auto getRotationQZ() const { return rotationQZ_; }
  auto getCenterViewFlag() const { return centerViewFlag_; }
  auto getLeftViewFlag() const { return leftViewFlag_; }

  auto& getViewportId() { return viewportId_; }
  auto& getCameraParametersPresentFlag() {
    return cameraParametersPresentFlag_;
  }
  auto& getVcpCameraId() { return CameraId_; }
  auto& getCancelFlag() { return cancelFlag_; }
  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getPosition(size_t i) { return position_[i]; }
  auto& getRotationQX() { return rotationQX_; }
  auto& getRotationQY() { return rotationQY_; }
  auto& getRotationQZ() { return rotationQZ_; }
  auto& getCenterViewFlag() { return centerViewFlag_; }
  auto& getLeftViewFlag() { return leftViewFlag_; }

private:
  uint32_t viewportId_                  = 0;
  bool     cameraParametersPresentFlag_ = false;
  uint8_t  CameraId_                    = 0;
  bool     cancelFlag_                  = false;
  bool     persistenceFlag_             = false;
  float    position_[3]                 = {0.0f, 0.0f, 0.0f};
  float    rotationQX_                  = 0.f;
  float    rotationQY_                  = 0.f;
  float    rotationQZ_                  = 0.f;
  bool     centerViewFlag_              = false;
  bool     leftViewFlag_                = false;
};

// F.2.16 Decoded Atlas Information Hash SEI message syntax
// F.2.16.1 Decoded high level hash unit syntax
class SEIDecodedAtlasInformationHash : public SEI {
public:
  SEIDecodedAtlasInformationHash() {
    tileId_.resize(numTilesMinus1_ + 1);
    highLevelMd5_.resize(16);
    atlasMd5_.resize(16);
    atlasB2pMd5_.resize(16);
    atlasTilesCrc_.resize(numTilesMinus1_ + 1);
    atlasTilesChecksum_.resize(numTilesMinus1_ + 1);
    ;
    atlasTilesMd5_.resize(numTilesMinus1_ + 1);
    atlasTilesB2pCrc_.resize(numTilesMinus1_ + 1);
    atlasTilesB2pChecksum_.resize(numTilesMinus1_ + 1);
    atlasTilesB2pMd5_.resize(numTilesMinus1_ + 1);
    for (auto& element : atlasTilesMd5_) { element.resize(16); }
    for (auto& element : atlasTilesB2pMd5_) { element.resize(16); }
  }

  ~SEIDecodedAtlasInformationHash() {
    highLevelMd5_.clear();
    atlasMd5_.clear();
    atlasB2pMd5_.clear();
    tileId_.clear();
    atlasTilesCrc_.clear();
    atlasTilesChecksum_.clear();
    atlasTilesB2pCrc_.clear();
    atlasTilesB2pChecksum_.clear();
    for (auto& element : atlasTilesMd5_) { element.clear(); }
    for (auto& element : atlasTilesB2pMd5_) { element.clear(); }
    atlasTilesMd5_.clear();
    atlasTilesB2pMd5_.clear();
  }
  SEIDecodedAtlasInformationHash&
  operator=(const SEIDecodedAtlasInformationHash&) = default;

  SeiPayloadType getPayloadType() { return DECODED_ATLAS_INFORMATION_HASH; }

  void allocateAtlasTilesHash(size_t numTiles) {
    assert(numTiles > 0);
    numTilesMinus1_ = numTiles - 1;
    tileId_.resize(numTiles);
    if (highLevelMd5_.size() != 16) highLevelMd5_.resize(16);
    if (atlasMd5_.size() != 16) atlasMd5_.resize(16);
    if (atlasB2pMd5_.size() != 16) atlasB2pMd5_.resize(16);
    atlasTilesMd5_.resize(numTiles);
    atlasTilesCrc_.resize(numTiles);
    atlasTilesChecksum_.resize(numTiles);
    atlasTilesB2pMd5_.resize(numTiles);
    atlasTilesB2pCrc_.resize(numTiles);
    atlasTilesB2pChecksum_.resize(numTiles);
    for (auto& element : atlasTilesMd5_) {
      if (element.size() == 0) element.resize(16);
    }
    for (auto& element : atlasTilesB2pMd5_) {
      if (element.size() == 0) element.resize(16);
    }
  }

  auto getCancelFlag() const { return cancelFlag_; }
  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getHashType() const { return hashType_; }
  auto getDecodedHighLevelHashPresentFlag() const {
    return decodedHighLevelHashPresentFlag_;
  }
  auto getDecodedAtlasHashPresentFlag() const {
    return decodedAtlasHashPresentFlag_;
  }
  auto getDecodedAtlasB2pHashPresentFlag() const {
    return decodedAtlasB2pHashPresentFlag_;
  }
  auto getDecodedAtlasTilesHashPresentFlag() const {
    return decodedAtlasTilesHashPresentFlag_;
  }
  auto getDecodedAtlasTilesB2pHashPresentFlag() const {
    return decodedAtlasTilesB2pHashPresentFlag_;
  }
  auto getHighLevelCrc() const { return highLevelCrc_; }
  auto getAtlasCrc() const { return atlasCrc_; }
  auto getHighLevelCheckSum() const { return highLevelChecksum_; }
  auto getAtlasCheckSum() const { return atlasChecksum_; }
  auto getAtlasB2pCrc() const { return atlasB2pCrc_; }
  auto getAtlasB2pCheckSum() const { return atlasB2pChecksum_; }
  auto getNumTilesMinus1() const { return numTilesMinus1_; }
  auto getTileIdLenMinus1() const { return tileIdLenMinus1_; }
  auto getTileId(size_t i) const { return tileId_[i]; }
  auto getHighLevelMd5(size_t i) const { return highLevelMd5_[i]; }
  auto getAtlasMd5(size_t i) const { return atlasMd5_[i]; }
  auto getAtlasB2pMd5(size_t i) const { return atlasB2pMd5_[i]; }
  auto getAtlasTilesCrc(size_t i) const { return atlasTilesCrc_[i]; }
  auto getAtlasTilesCheckSum(size_t i) const { return atlasTilesChecksum_[i]; }
  auto getAtlasTilesB2pCrc(size_t i) const { return atlasTilesB2pCrc_[i]; }
  auto getAtlasTilesB2pCheckSum(size_t i) const {
    return atlasTilesB2pChecksum_[i];
  }
  auto getAtlasTilesMd5(size_t i, size_t j) const {
    return atlasTilesMd5_[i][j];
  }
  auto getAtlasTilesB2pMd5(size_t i, size_t j) const {
    return atlasTilesB2pMd5_[i][j];
  }

  auto& getCancelFlag() { return cancelFlag_; }
  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getHashType() { return hashType_; }
  auto& getDecodedHighLevelHashPresentFlag() {
    return decodedHighLevelHashPresentFlag_;
  }
  auto& getDecodedAtlasHashPresentFlag() {
    return decodedAtlasHashPresentFlag_;
  }
  auto& getDecodedAtlasB2pHashPresentFlag() {
    return decodedAtlasB2pHashPresentFlag_;
  }
  auto& getDecodedAtlasTilesHashPresentFlag() {
    return decodedAtlasTilesHashPresentFlag_;
  }
  auto& getDecodedAtlasTilesB2pHashPresentFlag() {
    return decodedAtlasTilesB2pHashPresentFlag_;
  }
  auto& getHighLevelCrc() { return highLevelCrc_; }
  auto& getAtlasCrc() { return atlasCrc_; }
  auto& getHighLevelCheckSum() { return highLevelChecksum_; }
  auto& getAtlasCheckSum() { return atlasChecksum_; }
  auto& getAtlasB2pCrc() { return atlasB2pCrc_; }
  auto& getAtlasB2pCheckSum() { return atlasB2pChecksum_; }
  auto& getNumTilesMinus1() { return numTilesMinus1_; }
  auto& getTileIdLenMinus1() { return tileIdLenMinus1_; }
  auto& getTileId(size_t i) { return tileId_[i]; }
  auto& getHighLevelMd5(size_t i) { return highLevelMd5_[i]; }
  auto& getAtlasMd5(size_t i) { return atlasMd5_[i]; }
  auto& getAtlasB2pMd5(size_t i) { return atlasB2pMd5_[i]; }
  auto& getAtlasTilesCrc(size_t i) { return atlasTilesCrc_[i]; }
  auto& getAtlasTilesCheckSum(size_t i) { return atlasTilesChecksum_[i]; }
  auto& getAtlasTilesB2pCrc(size_t i) { return atlasTilesB2pCrc_[i]; }
  auto& getAtlasTilesB2pCheckSum(size_t i) {
    return atlasTilesB2pChecksum_[i];
  }
  auto& getAtlasTilesMd5(size_t i, size_t j) { return atlasTilesMd5_[i][j]; }
  auto& getAtlasTilesB2pMd5(size_t i, size_t j) {
    return atlasTilesB2pMd5_[i][j];
  }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = (uint8_t)cancelFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)persistenceFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = hashType_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)decodedHighLevelHashPresentFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)decodedAtlasHashPresentFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)decodedAtlasB2pHashPresentFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)decodedAtlasTilesHashPresentFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)decodedAtlasTilesB2pHashPresentFlag_ & 0xFF;
    byteStrData_.push_back(val);
    if (decodedHighLevelHashPresentFlag_) {
      switch (hashType_) {
      case 0:
        for (int i = 0; i < 16; i++) {
          val = highLevelMd5_[i] & 0xFF;
          byteStrData_.push_back(val);
        }
        break;
      case 1:
        val = highLevelCrc_ & 0xFF;
        byteStrData_.push_back(val);
        val = (highLevelCrc_ >> 8) & 0xFF;
        byteStrData_.push_back(val);
        break;
      case 2:
        val = highLevelChecksum_ & 0xFF;
        byteStrData_.push_back(val);
        val = (highLevelChecksum_ >> 8) & 0xFF;
        byteStrData_.push_back(val);
        val = (highLevelChecksum_ >> 16) & 0xFF;
        byteStrData_.push_back(val);
        val = (highLevelChecksum_ >> 24) & 0xFF;
        byteStrData_.push_back(val);
        break;
      default: std::cerr << " Undefined Hash Type " << std::endl;
      }
      if (decodedAtlasHashPresentFlag_) {
        switch (hashType_) {
        case 0:
          for (int i = 0; i < 16; i++) {
            val = atlasMd5_[i] & 0xFF;
            byteStrData_.push_back(val);
          }
          break;
        case 1:
          val = atlasCrc_ & 0xFF;
          byteStrData_.push_back(val);
          val = (atlasCrc_ >> 8) & 0xFF;
          byteStrData_.push_back(val);
          break;
        case 2:;
          val = atlasChecksum_ & 0xFF;
          byteStrData_.push_back(val);
          val = (atlasChecksum_ >> 8) & 0xFF;
          byteStrData_.push_back(val);
          val = (atlasChecksum_ >> 16) & 0xFF;
          byteStrData_.push_back(val);
          val = (atlasChecksum_ >> 24) & 0xFF;
          byteStrData_.push_back(val);
          break;
        default: std::cerr << " Undefined Hash Type " << std::endl;
        }
      }
      if (decodedAtlasB2pHashPresentFlag_) {
        switch (hashType_) {
        case 0:
          for (int i = 0; i < 16; i++) {
            val = atlasB2pMd5_[i] & 0xFF;
            byteStrData_.push_back(val);
          }
          break;
        case 1:
          val = atlasB2pCrc_ & 0xFF;
          byteStrData_.push_back(val);
          val = (atlasB2pCrc_ >> 8) & 0xFF;
          byteStrData_.push_back(val);
          break;
        case 2:;
          val = atlasChecksum_ & 0xFF;
          byteStrData_.push_back(val);
          val = (atlasB2pChecksum_ >> 8) & 0xFF;
          byteStrData_.push_back(val);
          val = (atlasB2pChecksum_ >> 16) & 0xFF;
          byteStrData_.push_back(val);
          val = (atlasB2pChecksum_ >> 24) & 0xFF;
          byteStrData_.push_back(val);
          break;
        default: std::cerr << " Undefined Hash Type " << std::endl;
        }
      }
      if (decodedAtlasTilesHashPresentFlag_
          || decodedAtlasTilesB2pHashPresentFlag_) {
        val = numTilesMinus1_ & 0XFF;
        byteStrData_.push_back(val);
        val = (numTilesMinus1_ >> 8) & 0XFF;
        byteStrData_.push_back(val);
        val = (numTilesMinus1_ >> 16) & 0XFF;
        byteStrData_.push_back(val);
        val = (numTilesMinus1_ >> 24) & 0XFF;
        byteStrData_.push_back(val);
        val = tileIdLenMinus1_ & 0XFF;
        byteStrData_.push_back(val);
        val = (tileIdLenMinus1_ >> 8) & 0XFF;
        byteStrData_.push_back(val);
        val = (tileIdLenMinus1_ >> 16) & 0XFF;
        byteStrData_.push_back(val);
        val = (tileIdLenMinus1_ >> 24) & 0XFF;
        byteStrData_.push_back(val);
        for (int t = 0; t < numTilesMinus1_ + 1; t++) {
          val = tileId_[t] & 0XFF;
          byteStrData_.push_back(val);
          val = (tileId_[t] >> 8) & 0XFF;
          byteStrData_.push_back(val);
          val = (tileId_[t] >> 16) & 0XFF;
          byteStrData_.push_back(val);
          val = (tileId_[t] >> 24) & 0XFF;
          byteStrData_.push_back(val);
        }
        for (int t = 0; t < numTilesMinus1_ + 1; t++) {
          uint32_t j = tileId_[t];
          if (decodedAtlasTilesHashPresentFlag_) {
            switch (hashType_) {
            case 0:
              for (int i = 0; i < 16; i++) {
                val = atlasTilesMd5_[j][i] & 0xFF;
                byteStrData_.push_back(val);
              }
              break;
            case 1:
              val = atlasTilesCrc_[j] & 0xFF;
              byteStrData_.push_back(val);
              val = (atlasTilesCrc_[j] >> 8) & 0xFF;
              byteStrData_.push_back(val);
              break;
            case 2:;
              val = atlasTilesChecksum_[j] & 0xFF;
              byteStrData_.push_back(val);
              val = (atlasTilesChecksum_[j] >> 8) & 0xFF;
              byteStrData_.push_back(val);
              val = (atlasTilesChecksum_[j] >> 16) & 0xFF;
              byteStrData_.push_back(val);
              val = (atlasTilesChecksum_[j] >> 24) & 0xFF;
              byteStrData_.push_back(val);
              break;
            default: std::cerr << " Undefined Hash Type " << std::endl;
            }
          }
          if (decodedAtlasTilesB2pHashPresentFlag_) {
            switch (hashType_) {
            case 0:
              for (int i = 0; i < 16; i++) {
                val = atlasTilesB2pMd5_[j][i] & 0xFF;
                byteStrData_.push_back(val);
              }
              break;
            case 1:
              val = atlasTilesB2pCrc_[j] & 0xFF;
              byteStrData_.push_back(val);
              val = (atlasTilesB2pCrc_[j] >> 8) & 0xFF;
              byteStrData_.push_back(val);
              break;
            case 2:;
              val = atlasTilesB2pChecksum_[j] & 0xFF;
              byteStrData_.push_back(val);
              val = (atlasTilesB2pChecksum_[j] >> 8) & 0xFF;
              byteStrData_.push_back(val);
              val = (atlasTilesB2pChecksum_[j] >> 16) & 0xFF;
              byteStrData_.push_back(val);
              val = (atlasTilesB2pChecksum_[j] >> 24) & 0xFF;
              byteStrData_.push_back(val);
              break;
            default: std::cerr << " Undefined Hash Type " << std::endl;
            }
          }
        }
      }
    }
    return byteStrData_;
  }

private:
  bool                  cancelFlag_                          = false;
  bool                  persistenceFlag_                     = false;
  uint8_t               hashType_                            = 1;
  bool                  decodedHighLevelHashPresentFlag_     = true;
  bool                  decodedAtlasHashPresentFlag_         = false;
  bool                  decodedAtlasB2pHashPresentFlag_      = false;
  bool                  decodedAtlasTilesHashPresentFlag_    = false;
  bool                  decodedAtlasTilesB2pHashPresentFlag_ = false;
  uint32_t              numTilesMinus1_                      = 0;
  uint32_t              tileIdLenMinus1_                     = 0;
  uint16_t              highLevelCrc_                        = 0;
  uint32_t              highLevelChecksum_                   = 0;
  uint16_t              atlasCrc_                            = 0;
  uint32_t              atlasChecksum_                       = 0;
  uint16_t              atlasB2pCrc_                         = 0;
  uint32_t              atlasB2pChecksum_                    = 0;
  std::vector<uint32_t> tileId_;
  std::vector<uint8_t>  highLevelMd5_;
  std::vector<uint8_t>  atlasMd5_;
  std::vector<uint8_t>  atlasB2pMd5_;
  std::vector<uint16_t> atlasTilesCrc_;
  std::vector<uint32_t> atlasTilesChecksum_;
  std::vector<std::vector<uint32_t>> atlasTilesMd5_;
  std::vector<uint16_t>              atlasTilesB2pCrc_;
  std::vector<uint32_t>              atlasTilesB2pChecksum_;
  std::vector<std::vector<uint32_t>> atlasTilesB2pMd5_;
};

// H.20.2.18 Occupancy synthesis SEI message syntax
class SEIOccupancySynthesis : public SEI {
public:
  SEIOccupancySynthesis() {}
  ~SEIOccupancySynthesis() {
    instanceIndex_.clear();
    instanceCancelFlag_.clear();
    methodType_.clear();
    pbfLog2ThresholdMinus1_.clear();
    pbfPassesCountMinus1_.clear();
    pbfFilterSizeMinus1_.clear();
  }
  SEIOccupancySynthesis& operator=(const SEIOccupancySynthesis&) = default;

  SeiPayloadType getPayloadType() { return OCCUPANCY_SYNTHESIS; }
  void           allocate() {
    instanceIndex_.resize(instancesUpdated_, 0);
    instanceCancelFlag_.resize(instancesUpdated_, 0);
    methodType_.resize(instancesUpdated_, 0);
    pbfLog2ThresholdMinus1_.resize(instancesUpdated_, 0);
    pbfPassesCountMinus1_.resize(instancesUpdated_, 0);
    pbfFilterSizeMinus1_.resize(instancesUpdated_, 0);
  }

  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getResetFlag() const { return resetFlag_; }
  auto getInstancesUpdated() const { return instancesUpdated_; }
  auto getInstanceIndex(size_t i) const { return instanceIndex_[i]; }
  auto getInstanceCancelFlag(size_t i) const { return instanceCancelFlag_[i]; }
  auto getMethodType(size_t i) const { return methodType_[i]; }
  auto getPbfLog2ThresholdMinus1(size_t i) const {
    return pbfLog2ThresholdMinus1_[i];
  }
  auto getPbfPassesCountMinus1(size_t i) const {
    return pbfPassesCountMinus1_[i];
  }
  auto getPbfFilterSizeMinus1(size_t i) const {
    return pbfFilterSizeMinus1_[i];
  }

  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getResetFlag() { return resetFlag_; }
  auto& getInstancesUpdated() { return instancesUpdated_; }
  auto& getInstanceIndex(size_t i) { return instanceIndex_[i]; }
  auto& getInstanceCancelFlag(size_t i) { return instanceCancelFlag_[i]; }
  auto& getMethodType(size_t i) { return methodType_[i]; }
  auto& getPbfLog2ThresholdMinus1(size_t i) {
    return pbfLog2ThresholdMinus1_[i];
  }
  auto& getPbfPassesCountMinus1(size_t i) { return pbfPassesCountMinus1_[i]; }
  auto& getPbfFilterSizeMinus1(size_t i) { return pbfFilterSizeMinus1_[i]; }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = (uint8_t)persistenceFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)resetFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = instancesUpdated_ & 0xFF;
    byteStrData_.push_back(val);
    for (int i = 0; i < instancesUpdated_; i++) {
      uint8_t k = instanceIndex_[i];
      val       = instanceIndex_[i] & 0xFF;
      byteStrData_.push_back(val);
      val = (uint8_t)instanceCancelFlag_[k] & 0xFF;
      byteStrData_.push_back(val);
      val = methodType_[k] & 0xFF;
      byteStrData_.push_back(val);
      if (methodType_[k] == 1) {
        val = pbfLog2ThresholdMinus1_[k] & 0xFF;
        byteStrData_.push_back(val);
        val = pbfPassesCountMinus1_[k] & 0xFF;
        byteStrData_.push_back(val);
        val = pbfFilterSizeMinus1_[k] & 0xFF;
        byteStrData_.push_back(val);
      }
    }
    return byteStrData_;
  }

private:
  bool                 persistenceFlag_  = false;
  bool                 resetFlag_        = false;
  uint8_t              instancesUpdated_ = 0;
  std::vector<uint8_t> instanceIndex_;
  std::vector<uint8_t> instanceCancelFlag_;
  std::vector<uint8_t> methodType_;
  std::vector<uint8_t> pbfLog2ThresholdMinus1_;
  std::vector<uint8_t> pbfPassesCountMinus1_;
  std::vector<uint8_t> pbfFilterSizeMinus1_;
};

// H.20.2.19 Geometry smoothing SEI message syntax
class SEIGeometrySmoothing : public SEI {
public:
  SEIGeometrySmoothing() {}
  ~SEIGeometrySmoothing() {
    instanceIndex_.clear();
    instanceCancelFlag_.clear();
    methodType_.clear();
    filterEomPointsFlag_.clear();
    gridSizeMinus2_.clear();
    threshold_.clear();
  }
  SEIGeometrySmoothing& operator=(const SEIGeometrySmoothing&) = default;

  SeiPayloadType getPayloadType() { return GEOMETRY_SMOOTHING; }
  void           allocate() {
    instanceIndex_.resize(instancesUpdated_, 0);
    instanceCancelFlag_.resize(instancesUpdated_, 0);
    methodType_.resize(instancesUpdated_, 0);
    filterEomPointsFlag_.resize(instancesUpdated_, 0);
    gridSizeMinus2_.resize(instancesUpdated_, 0);
    threshold_.resize(instancesUpdated_, 0);
  }

  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getResetFlag() const { return resetFlag_; }
  auto getInstancesUpdated() const { return instancesUpdated_; }
  auto getInstanceIndex(size_t i) const { return instanceIndex_[i]; }
  auto getInstanceCancelFlag(size_t i) const { return instanceCancelFlag_[i]; }
  auto getMethodType(size_t i) const { return methodType_[i]; }
  auto getFilterEomPointsFlag(size_t i) const {
    return filterEomPointsFlag_[i];
  }
  auto getGridSizeMinus2(size_t i) const { return gridSizeMinus2_[i]; }
  auto getThreshold(size_t i) const { return threshold_[i]; }

  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getResetFlag() { return resetFlag_; }
  auto& getInstancesUpdated() { return instancesUpdated_; }
  auto& getInstanceIndex(size_t i) { return instanceIndex_[i]; }
  auto& getInstanceCancelFlag(size_t i) { return instanceCancelFlag_[i]; }
  auto& getMethodType(size_t i) { return methodType_[i]; }
  auto& getFilterEomPointsFlag(size_t i) { return filterEomPointsFlag_[i]; }
  auto& getGridSizeMinus2(size_t i) { return gridSizeMinus2_[i]; }
  auto& getThreshold(size_t i) { return threshold_[i]; }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = (uint8_t)persistenceFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)resetFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = instancesUpdated_ & 0xFF;
    byteStrData_.push_back(val);
    for (int i = 0; i < instancesUpdated_; i++) {
      uint8_t k = instanceIndex_[i];
      val       = instanceIndex_[i] & 0xFF;
      byteStrData_.push_back(val);
      val = (uint8_t)instanceCancelFlag_[k] & 0xFF;
      byteStrData_.push_back(val);
      if (instanceCancelFlag_[k] != 1) {
        val = methodType_[k] & 0xFF;
        byteStrData_.push_back(val);
        if (methodType_[k] == 1) {
          val = (uint8_t)filterEomPointsFlag_[k] & 0xFF;
          byteStrData_.push_back(val);
          val = gridSizeMinus2_[k] & 0xFF;
          byteStrData_.push_back(val);
          val = threshold_[k] & 0xFF;
          byteStrData_.push_back(val);
        }
      }
    }
    return byteStrData_;
  }

private:
  bool                 persistenceFlag_  = false;
  bool                 resetFlag_        = false;
  uint8_t              instancesUpdated_ = 0;
  std::vector<uint8_t> instanceIndex_;
  std::vector<uint8_t> instanceCancelFlag_;
  std::vector<uint8_t> methodType_;
  std::vector<uint8_t> filterEomPointsFlag_;
  std::vector<uint8_t> gridSizeMinus2_;
  std::vector<uint8_t> threshold_;
};

// H.20.2.20 Attribute smoothing SEI message syntax
class SEIAttributeSmoothing : public SEI {
public:
  SEIAttributeSmoothing() {}
  ~SEIAttributeSmoothing() {
    attributeIdx_.clear();
    attributeSmoothingCancelFlag_.clear();
    instancesUpdated_.clear();
    for (auto& element : instanceIndex_) { element.clear(); }
    for (auto& element : instanceCancelFlag_) { element.clear(); }
    for (auto& element : methodType_) { element.clear(); }
    for (auto& element : gridSizeMinus2_) { element.clear(); }
    for (auto& element : threshold_) { element.clear(); }
    for (auto& element : thresholdVariation_) { element.clear(); }
    for (auto& element : thresholdDifference_) { element.clear(); }
    instanceIndex_.clear();
    instanceCancelFlag_.clear();
    methodType_.clear();
    filterEomPointsFlag_.clear();
    gridSizeMinus2_.clear();
    threshold_.clear();
    thresholdVariation_.clear();
    thresholdDifference_.clear();
  }
  SEIAttributeSmoothing& operator=(const SEIAttributeSmoothing&) = default;

  SeiPayloadType getPayloadType() { return ATTRIBUTE_SMOOTHING; }

  void allocate() {
    attributeIdx_.resize(numAttributesUpdated_, 0);
    attributeSmoothingCancelFlag_.resize(numAttributesUpdated_, false);
    instancesUpdated_.resize(numAttributesUpdated_, 0);
  }
  void allocate(size_t size, size_t dimension) {
    if (instanceIndex_.size() < size) {
      instanceIndex_.resize(size);
      instanceCancelFlag_.resize(size);
      methodType_.resize(size);
      filterEomPointsFlag_.resize(size);
      gridSizeMinus2_.resize(size);
      threshold_.resize(size);
      thresholdVariation_.resize(size);
      thresholdDifference_.resize(size);
    }
    instanceIndex_[size - 1].resize(dimension);
    instanceCancelFlag_[size - 1].resize(dimension);
    methodType_[size - 1].resize(dimension);
    filterEomPointsFlag_[size - 1].resize(dimension);
    gridSizeMinus2_[size - 1].resize(dimension);
    threshold_[size - 1].resize(dimension);
    thresholdVariation_[size - 1].resize(dimension);
    thresholdDifference_[size - 1].resize(dimension);
  }

  auto getPersistenceFlag() const { return persistenceFlag_; }
  auto getResetFlag() const { return resetFlag_; }
  auto getNumAttributesUpdated() const { return numAttributesUpdated_; }
  auto getAttributeIdx(size_t i) const { return attributeIdx_[i]; }
  auto getAttributeSmoothingCancelFlag(size_t i) const {
    return attributeSmoothingCancelFlag_[i];
  }
  auto getInstancesUpdated(size_t i) const { return instancesUpdated_[i]; }
  auto getInstanceIndex(size_t i, size_t j) const {
    return instanceIndex_[i][j];
  }
  auto getInstanceCancelFlag(size_t i, size_t j) const {
    return instanceCancelFlag_[i][j];
  }
  auto getMethodType(size_t i, size_t j) const { return methodType_[i][j]; }
  auto getFilterEomPointsFlag(size_t i, size_t j) const {
    return filterEomPointsFlag_[i][j];
  }
  auto getGridSizeMinus2(size_t i, size_t j) const {
    return gridSizeMinus2_[i][j];
  }
  auto getThreshold(size_t i, size_t j) const { return threshold_[i][j]; }
  auto getThresholdVariation(size_t i, size_t j) const {
    return thresholdVariation_[i][j];
  }
  auto getThresholdDifference(size_t i, size_t j) const {
    return thresholdDifference_[i][j];
  }

  auto& getPersistenceFlag() { return persistenceFlag_; }
  auto& getResetFlag() { return resetFlag_; }
  auto& getNumAttributesUpdated() { return numAttributesUpdated_; }
  auto& getAttributeIdx(size_t i) { return attributeIdx_[i]; }
  auto& getAttributeSmoothingCancelFlag(size_t i) {
    return attributeSmoothingCancelFlag_[i];
  }
  auto& getInstancesUpdated(size_t i) { return instancesUpdated_[i]; }
  auto& getInstanceIndex(size_t i, size_t j) { return instanceIndex_[i][j]; }
  auto& getInstanceCancelFlag(size_t i, size_t j) {
    return instanceCancelFlag_[i][j];
  }
  auto& getMethodType(size_t i, size_t j) { return methodType_[i][j]; }
  auto& getFilterEomPointsFlag(size_t i, size_t j) {
    return filterEomPointsFlag_[i][j];
  }
  auto& getGridSizeMinus2(size_t i, size_t j) { return gridSizeMinus2_[i][j]; }
  auto& getThreshold(size_t i, size_t j) { return threshold_[i][j]; }
  auto& getThresholdVariation(size_t i, size_t j) {
    return thresholdVariation_[i][j];
  }
  auto& getThresholdDifference(size_t i, size_t j) {
    return thresholdDifference_[i][j];
  }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = (uint8_t)persistenceFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = (uint8_t)resetFlag_ & 0xFF;
    byteStrData_.push_back(val);
    val = numAttributesUpdated_ & 0xFF;
    byteStrData_.push_back(val);
    for (int j = 0; j < numAttributesUpdated_; j++) {
      val = attributeIdx_[j] & 0xFF;
      byteStrData_.push_back(val);
      uint8_t k = attributeIdx_[j];
      val       = (uint8_t)attributeSmoothingCancelFlag_[k] & 0xFF;
      byteStrData_.push_back(val);
      val = instancesUpdated_[k] & 0xFF;
      byteStrData_.push_back(val);
      for (int i = 0; i < instancesUpdated_[k]; i++) {
        val = instanceIndex_[k][i] & 0xFF;
        byteStrData_.push_back(val);
        uint8_t m = instanceIndex_[k][i];
        val       = (uint8_t)instanceCancelFlag_[k][m] & 0xFF;
        byteStrData_.push_back(val);
        if (instanceCancelFlag_[k][m] != 1) {
          val = methodType_[k][m] & 0XFF;
          byteStrData_.push_back(val);
          if (methodType_[k][m] == 1) {
            val = (uint8_t)filterEomPointsFlag_[k][m] & 0XFF;
            byteStrData_.push_back(val);
            val = gridSizeMinus2_[k][m] & 0XFF;
            byteStrData_.push_back(val);
            val = threshold_[k][m] & 0XFF;
            byteStrData_.push_back(val);
            val = thresholdVariation_[k][m] & 0XFF;
            byteStrData_.push_back(val);
            val = thresholdDifference_[k][m] & 0XFF;
            byteStrData_.push_back(val);
          }
        }
      }
    }
    return byteStrData_;
  }

private:
  bool                              persistenceFlag_      = false;
  bool                              resetFlag_            = false;
  uint8_t                           numAttributesUpdated_ = 0;
  std::vector<uint8_t>              attributeIdx_;
  std::vector<uint8_t>              attributeSmoothingCancelFlag_;
  std::vector<uint8_t>              instancesUpdated_;
  std::vector<std::vector<uint8_t>> instanceIndex_;
  std::vector<std::vector<uint8_t>> instanceCancelFlag_;
  std::vector<std::vector<uint8_t>> methodType_;
  std::vector<std::vector<uint8_t>> filterEomPointsFlag_;
  std::vector<std::vector<uint8_t>> gridSizeMinus2_;
  std::vector<std::vector<uint8_t>> threshold_;
  std::vector<std::vector<uint8_t>> thresholdVariation_;
  std::vector<std::vector<uint8_t>> thresholdDifference_;
};

// F.2.17 Time code SEI message syntax
class SEITimeCode : public SEI {
public:
  SEITimeCode() {}
  ~SEITimeCode() {}
  SEITimeCode& operator=(const SEITimeCode&) = default;

  SeiPayloadType getPayloadType() { return TIME_CODE; }

  auto getNumUnitsInTick() const { return numUnitsInTick_; }
  auto getTimeScale() const { return timeScale_; }
  auto getCountingType() const { return countingType_; }
  auto getFullTimestampFlag() const { return fullTimestampFlag_; }
  auto getDiscontinuityFlag() const { return discontinuityFlag_; }
  auto getCntDroppedFlag() const { return cntDroppedFlag_; }
  auto getSecondFlag() const { return secondFlag_; }
  auto getMinutesFlag() const { return minutesFlag_; }
  auto getHoursFlag() const { return hoursFlag_; }
  auto getNFrames() const { return nFrames_; }
  auto getSecondsValue() const { return secondsValue_; }
  auto getMinutesValue() const { return minutesValue_; }
  auto getHoursValue() const { return hoursValue_; }
  auto getTimeOffsetLength() const { return timeOffsetLength_; }
  auto getTimeOffsetValue() const { return timeOffsetValue_; }

  auto& getNumUnitsInTick() { return numUnitsInTick_; }
  auto& getTimeScale() { return timeScale_; }
  auto& getCountingType() { return countingType_; }
  auto& getFullTimestampFlag() { return fullTimestampFlag_; }
  auto& getDiscontinuityFlag() { return discontinuityFlag_; }
  auto& getCntDroppedFlag() { return cntDroppedFlag_; }
  auto& getSecondFlag() { return secondFlag_; }
  auto& getMinutesFlag() { return minutesFlag_; }
  auto& getHoursFlag() { return hoursFlag_; }
  auto& getNFrames() { return nFrames_; }
  auto& getSecondsValue() { return secondsValue_; }
  auto& getMinutesValue() { return minutesValue_; }
  auto& getHoursValue() { return hoursValue_; }
  auto& getTimeOffsetLength() { return timeOffsetLength_; }
  auto& getTimeOffsetValue() { return timeOffsetValue_; }

private:
  uint32_t numUnitsInTick_    = 0;
  uint32_t timeScale_         = 0;
  uint16_t countingType_      = 0;
  bool     fullTimestampFlag_ = false;
  bool     discontinuityFlag_ = false;
  bool     cntDroppedFlag_    = false;
  bool     secondFlag_        = false;
  bool     minutesFlag_       = false;
  bool     hoursFlag_         = false;
  uint16_t nFrames_           = 0;
  uint16_t secondsValue_      = 0;
  uint16_t minutesValue_      = 0;
  uint16_t hoursValue_        = 0;
  uint16_t timeOffsetLength_  = 0;
  int16_t  timeOffsetValue_   = 0;
};

class PCCSEI {
public:
  PCCSEI() {
    seiPrefix_.clear();
    seiSuffix_.clear();
  }
  ~PCCSEI() {
    seiPrefix_.clear();
    seiSuffix_.clear();
  }

  SEI& addSei(AtlasNalUnitType nalUnitType, SeiPayloadType payloadType) {
    std::shared_ptr<SEI> sharedPtr;
    switch (payloadType) {
    case BUFFERING_PERIOD:
      sharedPtr = std::make_shared<SEIBufferingPeriod>();
      break;
    case ATLAS_FRAME_TIMING:
      sharedPtr = std::make_shared<SEIAtlasFrameTiming>();
      break;
    case FILLER_PAYLOAD: break;
    case USER_DATAREGISTERED_ITUTT35:
      sharedPtr = std::make_shared<SEIUserDataRegisteredItuTT35>();
      break;
    case USER_DATA_UNREGISTERED:
      sharedPtr = std::make_shared<SEIUserDataUnregistered>();
      break;
    case RECOVERY_POINT:
      sharedPtr = std::make_shared<SEIRecoveryPoint>();
      break;
    case NO_RECONSTRUCTION:
      sharedPtr = std::make_shared<SEINoDisplay>();
      break;
    case TIME_CODE: sharedPtr = std::make_shared<SEITimeCode>(); break;
    case SEI_MANIFEST: sharedPtr = std::make_shared<SEIManifest>(); break;
    case SEI_PREFIX_INDICATION:
      sharedPtr = std::make_shared<SEIPrefixIndication>();
      break;
    case ACTIVE_SUB_BITSTREAMS:
      sharedPtr = std::make_shared<SEIActiveSubBitstreams>();
      break;
    case COMPONENT_CODEC_MAPPING:
      sharedPtr = std::make_shared<SEIComponentCodecMapping>();
      break;
    case SCENE_OBJECT_INFORMATION:
      sharedPtr = std::make_shared<SEISceneObjectInformation>();
      break;
    case OBJECT_LABEL_INFORMATION:
      sharedPtr = std::make_shared<SEIObjectLabelInformation>();
      break;
    case PATCH_INFORMATION:
      sharedPtr = std::make_shared<SEIPatchInformation>();
      break;
    case VOLUMETRIC_RECTANGLE_INFORMATION:
      sharedPtr = std::make_shared<SEIVolumetricRectangleInformation>();
      break;
    case ATLAS_OBJECT_INFORMATION:
      sharedPtr = std::make_shared<SEIAtlasInformation>();
      break;
    case VIEWPORT_CAMERA_PARAMETERS:
      sharedPtr = std::make_shared<SEIViewportCameraParameters>();
      break;
    case VIEWPORT_POSITION:
      sharedPtr = std::make_shared<SEIViewportPosition>();
      break;
    case DECODED_ATLAS_INFORMATION_HASH:
      sharedPtr = std::make_shared<SEIDecodedAtlasInformationHash>();
      break;
    case ATTRIBUTE_TRANSFORMATION_PARAMS:
      sharedPtr = std::make_shared<SEIAttributeTransformationParams>();
      break;
    case OCCUPANCY_SYNTHESIS:
      sharedPtr = std::make_shared<SEIOccupancySynthesis>();
      break;
    case GEOMETRY_SMOOTHING:
      sharedPtr = std::make_shared<SEIGeometrySmoothing>();
      break;
    case ATTRIBUTE_SMOOTHING:
      sharedPtr = std::make_shared<SEIAttributeSmoothing>();
      break;
    case RESERVED_SEI_MESSAGE:
      sharedPtr = std::make_shared<SEIReservedSeiMessage>();
      break;
    default:
      fprintf(stderr, "SEI payload type not supported \n");
      exit(-1);
      break;
    }
    if (nalUnitType == ATLAS_NAL_PREFIX_ESEI
        || nalUnitType == ATLAS_NAL_PREFIX_NSEI) {
      seiPrefix_.push_back(sharedPtr);
      return *(seiPrefix_.back().get());
    } else if (nalUnitType == ATLAS_NAL_SUFFIX_ESEI
               || nalUnitType == ATLAS_NAL_SUFFIX_NSEI) {
      seiSuffix_.push_back(sharedPtr);
      return *(seiSuffix_.back().get());
    } else {
      fprintf(stderr, "Nal unit type of SEI not correct\n");
      exit(-1);
    }
    return *(sharedPtr);
  }
  bool seiIsPresent(AtlasNalUnitType nalUnitType, SeiPayloadType payloadType) {
    if (nalUnitType != ATLAS_NAL_PREFIX_ESEI
        && nalUnitType != ATLAS_NAL_SUFFIX_ESEI
        && nalUnitType != ATLAS_NAL_PREFIX_NSEI
        && nalUnitType != ATLAS_NAL_SUFFIX_NSEI) {
      return false;
    }
    for (auto& sei : nalUnitType == ATLAS_NAL_PREFIX_ESEI
                         || nalUnitType == ATLAS_NAL_PREFIX_NSEI
                       ? seiPrefix_
                       : seiSuffix_) {
      if (sei->getPayloadType() == payloadType) { return true; }
    }
    return false;
  }
  SEI* getSei(AtlasNalUnitType nalUnitType, SeiPayloadType payloadType) {
    auto& seis = (nalUnitType == ATLAS_NAL_PREFIX_ESEI
                  || nalUnitType == ATLAS_NAL_PREFIX_NSEI)
                   ? seiPrefix_
                   : seiSuffix_;
    for (auto& sei : seis) {
      if (sei->getPayloadType() == payloadType) { return sei.get(); }
    }
    assert(0);
    return (SEI*)nullptr;
  }

  SEI* getLastSei(AtlasNalUnitType nalUnitType, SeiPayloadType payloadType) {
    auto& seis = (nalUnitType == ATLAS_NAL_PREFIX_ESEI
                  || nalUnitType == ATLAS_NAL_PREFIX_NSEI)
                   ? seiPrefix_
                   : seiSuffix_;
    for (auto sei = seis.rbegin(); sei != seis.rend(); ++sei) {
      if (sei->get()->getPayloadType() == payloadType) { return sei->get(); }
    }
    assert(0);
    return (SEI*)nullptr;
  }
  SEI& addSeiPrefix(SeiPayloadType payloadType, bool essensial) {
    return addSei(essensial ? ATLAS_NAL_PREFIX_ESEI : ATLAS_NAL_PREFIX_NSEI,
                  payloadType);
  }
  SEI& addSeiSuffix(SeiPayloadType payloadType, bool essensial) {
    return addSei(essensial ? ATLAS_NAL_SUFFIX_ESEI : ATLAS_NAL_SUFFIX_NSEI,
                  payloadType);
  }
  void addSeiToSeiSuffix(SeiPayloadType                  payloadType,
                         bool                            essensial,
                         SEIDecodedAtlasInformationHash& seiContext) {
    seiSuffix_.push_back(
      std::make_shared<SEIDecodedAtlasInformationHash>(seiContext));
  }

  std::vector<std::shared_ptr<SEI>>& getSeiPrefix() { return seiPrefix_; }
  std::vector<std::shared_ptr<SEI>>& getSeiSuffix() { return seiSuffix_; }
  SEI& getSeiPrefix(size_t index) { return *(seiPrefix_[index]); }
  SEI& getSeiSuffix(size_t index) { return *(seiSuffix_[index]); }

private:
  std::vector<std::shared_ptr<SEI>> seiPrefix_;
  std::vector<std::shared_ptr<SEI>> seiSuffix_;
};

};  // namespace vmesh
