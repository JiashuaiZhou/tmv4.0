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

#include <vector>

namespace eb {

// 1.2.5	Mesh attributes encoding parameters syntax
class MeshAttributesEncodingParameters {
public:
  MeshAttributesEncodingParameters() {}
  ~MeshAttributesEncodingParameters() {}
  
  MeshAttributesEncodingParameters&
  operator=(const MeshAttributesEncodingParameters&) = default;

  uint8_t getMeshAttributeBitDepthMinus1() const {
    return meshAttributeBitDepthMinus1_;
  }
  uint8_t getMeshAttributePerFaceFlag() const {
    return meshAttributePerFaceFlag_;
  }
  bool getMeshAttributeSeparateIndexFlag() const {
    return meshAttributeSeparateIndexFlag_;
  }
  uint8_t getMeshAttributeReferenceIndexPlus1() const {
    return meshAttributeReferenceIndexPlus1_;
  }
  uint8_t getMeshAttributePredictionMethod() const {
    return meshAttributePredictionMethod_;
  }
  uint8_t getMeshAttributeResidualsEncodingMethod() const {
    return meshAttributeResidualsEncodingMethod_;
  }

  uint8_t& getMeshAttributeBitDepthMinus1() { return meshAttributeBitDepthMinus1_; }
  bool&    getMeshAttributePerFaceFlag() { return meshAttributePerFaceFlag_; }
  bool&    getMeshAttributeSeparateIndexFlag() {
    return meshAttributeSeparateIndexFlag_;
  }
  uint8_t& getMeshAttributeReferenceIndexPlus1() {
    return meshAttributeReferenceIndexPlus1_;
  }
  uint8_t& getMeshAttributePredictionMethod() {
    return meshAttributePredictionMethod_;
  }
  uint8_t& getMeshAttributeResidualsEncodingMethod() {
    return meshAttributeResidualsEncodingMethod_;
  }

private:
  uint8_t meshAttributeBitDepthMinus1_             = 0;
  bool    meshAttributePerFaceFlag_             = false;
  bool    meshAttributeSeparateIndexFlag_       = true;
  uint8_t meshAttributeReferenceIndexPlus1_     = 0;
  uint8_t meshAttributePredictionMethod_        = 0;
  uint8_t meshAttributeResidualsEncodingMethod_ = 0;
};

};  // namespace eb
