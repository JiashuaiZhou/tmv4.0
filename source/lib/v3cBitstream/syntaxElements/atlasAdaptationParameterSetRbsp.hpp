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

// 8.3.6.3.1	General atlas adaptation parameter set RBSP syntax
class AtlasAdaptationParameterSetRbsp {
public:
  AtlasAdaptationParameterSetRbsp() {}
  ~AtlasAdaptationParameterSetRbsp() {}
  AtlasAdaptationParameterSetRbsp&
  operator=(const AtlasAdaptationParameterSetRbsp&) = default;

  auto getAtlasAdaptationParameterSetId() const {
    return atlasAdaptationParameterSetId_;
  }
  auto  getExtensionFlag() const { return extensionFlag_; }
  auto  getVpccExtensionFlag() const { return vpccExtensionFlag_; }
  auto  getMivExtensionFlag() const { return mivExtensionFlag_; }
  auto  getVdmcExtensionFlag() const { return vdmcExtensionFlag_; }
  auto  getExtension5Bits() const { return extension5Bits_; }
  auto& getAapsVpccExtension() const { return aapsVpccExtension_; }
  auto& getAapsVdmcExtension() const { return aapsVdmcExtension_; }

  auto& getAtlasAdaptationParameterSetId() {
    return atlasAdaptationParameterSetId_;
  }
  auto& getExtensionFlag() { return extensionFlag_; }
  auto& getVpccExtensionFlag() { return vpccExtensionFlag_; }
  auto& getMivExtensionFlag() { return mivExtensionFlag_; }
  auto& getVdmcExtensionFlag() { return vdmcExtensionFlag_; }
  auto& getExtension5Bits() { return extension5Bits_; }
  auto& getAapsVpccExtension() { return aapsVpccExtension_; }
  auto& getAapsVdmcExtension() { return aapsVdmcExtension_; }

private:
  uint8_t           atlasAdaptationParameterSetId_ = 0;
  bool              extensionFlag_                 = false;
  bool              vpccExtensionFlag_             = false;
  bool              mivExtensionFlag_              = false;
  bool              vdmcExtensionFlag_             = false;
  uint8_t           extension5Bits_                = 0;
  AapsVpccExtension aapsVpccExtension_;
  AapsVdmcExtension aapsVdmcExtension_;
};

};  // namespace vmesh