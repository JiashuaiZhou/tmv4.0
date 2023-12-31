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

// Base mesh tile data unit syntax
class BaseMeshTileDataUnit {
public:
  BaseMeshTileDataUnit() {}
  ~BaseMeshTileDataUnit() {}
  BaseMeshTileDataUnit& operator=(const BaseMeshTileDataUnit&) = default;

  auto& getData() const { return data_; }
  auto  getMotionIntegrateMV() const { return motionIntegrateMV; }
  auto  getMotionSkipCount() const { return motionSkipCount; }
  auto  getMotionSkipFlag() const { return motionSkipFlag; }
  auto  getMotionSkipAll() const { return motionSkipAll; }
  auto  getMotionSkipVextexIndices() const { return motionSkipVextexIndices; }

  auto& getData() { return data_; }
  auto& getMotionIntegrateMV() { return motionIntegrateMV; }
  auto& getMotionSkipCount() { return motionSkipCount; }
  auto& getMotionSkipFlag() { return motionSkipFlag; }
  auto& getMotionSkipAll() { return motionSkipAll; }
  auto& getMotionSkipVextexIndices() { return motionSkipVextexIndices; }
  auto& getMotionSkipVextexIndices(size_t index) {
    return motionSkipVextexIndices[index];
  }

private:
  uint8_t              motionIntegrateMV = 0;
  bool                 motionSkipAll   = false;
  bool                 motionSkipFlag  = false;
  uint8_t              motionSkipCount = 0;
  std::vector<uint8_t> motionSkipVextexIndices;
  MeshBitstream        data_;
};

};  // namespace vmesh
