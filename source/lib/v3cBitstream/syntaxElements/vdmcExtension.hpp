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

  auto& getWidthDispVideo() const { return widthDispVideo; }
  auto& getHeightDispVideo() const { return heightDispVideo; }
  auto& getAddReconstructedNormals() const { return addReconstructedNormals; }
  auto& getSubdivisionMethod() const { return subdivisionMethod; }
  auto& getSubdivisionIterationCount() const {
    return subdivisionIterationCount;
  }
  auto& getLiftingQPs() const { return liftingQPs; }
  auto& getLiftingQPs(uint8_t index) const { return liftingQPs[index]; }
  auto& getInterpolateDisplacementNormals() const {
    return interpolateDisplacementNormals;
  }
  auto& getDisplacementReversePacking() const {
    return displacementReversePacking;
  }

  auto& getWidthDispVideo() { return widthDispVideo; }
  auto& getHeightDispVideo() { return heightDispVideo; }
  auto& getAddReconstructedNormals() { return addReconstructedNormals; }

  auto& getSubdivisionMethod() { return subdivisionMethod; }
  auto& getSubdivisionIterationCount() { return subdivisionIterationCount; }
  auto& getLiftingQPs() { return liftingQPs; }
  auto& getLiftingQPs(uint8_t index) { return liftingQPs[index]; }
  auto& getInterpolateDisplacementNormals() {
    return interpolateDisplacementNormals;
  }
  auto& getDisplacementReversePacking() { return displacementReversePacking; }

  auto getDisplacementCoordinateSystem() const {
    return displacementCoordinateSystem;
  }

  auto& getLiftingLevelOfDetailInverseScale() const {
    return liftingLevelOfDetailInverseScale;
  }

  auto& getLiftingSkipUpdate() const { return liftingSkipUpdate; }
  auto& getLiftingUpdateWeight() const { return liftingUpdateWeight; }
  auto& getLiftingPredictionWeight() const { return liftingPredictionWeight; }

private:
  // Displacement
  uint16_t widthDispVideo             = 0;
  uint16_t heightDispVideo            = 0;
  bool     displacementReversePacking = false;
  bool     addReconstructedNormals    = true;

  // Subdivition
  uint8_t subdivisionIterationCount      = 0;
  bool    interpolateDisplacementNormals = false;
  int32_t liftingQPs[3]                  = {0, 0, 0};

  // Not coded
  SubdivisionMethod subdivisionMethod = SubdivisionMethod::MID_POINT;
  double            liftingLevelOfDetailInverseScale[3] = {2.0, 2.0, 2.0};
  double            liftingUpdateWeight                 = 0.125;
  double            liftingPredictionWeight             = 0.5;
  bool              liftingSkipUpdate                   = false;
  DisplacementCoordinateSystem displacementCoordinateSystem =
    DisplacementCoordinateSystem::LOCAL;
};

// AFPS V-DMC extension syntax
class AfpsVdmcExtension {
public:
  AfpsVdmcExtension() {}
  ~AfpsVdmcExtension() {}
  AfpsVdmcExtension& operator=(const AfpsVdmcExtension&) = default;

private:
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
