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
#include "patchDataUnit.hpp"
#include "interPatchDataUnit.hpp"
#include "mergePatchDataUnit.hpp"
#include "mergePatchDataUnit.hpp"
#include "skipPatchDataUnit.hpp"
#include "rawPatchDataUnit.hpp"
#include "eomPatchDataUnit.hpp"

namespace vmesh {

// 8.3.7.2  Patch information data syntax (pid)
class PatchInformationData {
public:
  PatchInformationData(){};
  ~PatchInformationData(){};
  PatchInformationData& operator=(const PatchInformationData&) = default;

  auto  getTileOrder() const { return tileOrder_; }
  auto  getPatchIndex() const { return patchIndex_; }
  auto  getPatchMode() const { return patchMode_; }
  auto& getPatchDataUnit() const { return patchDataUnit_; }
  auto& getInterPatchDataUnit() const { return interPatchDataUnit_; }
  auto& getMergePatchDataUnit() const { return mergePatchDataUnit_; }
  auto& getSkipPatchDataUnit() const { return skipPatchDataUnit_; }
  auto& getRawPatchDataUnit() const { return rawPatchyDataUnit_; }
  auto& getEomPatchDataUnit() const { return eomPatchDataUnit_; }

  auto& getTileOrder() { return tileOrder_; }
  auto& getPatchIndex() { return patchIndex_; }
  auto& getPatchMode() { return patchMode_; }
  auto& getPatchDataUnit() { return patchDataUnit_; }
  auto& getInterPatchDataUnit() { return interPatchDataUnit_; }
  auto& getMergePatchDataUnit() { return mergePatchDataUnit_; }
  auto& getSkipPatchDataUnit() { return skipPatchDataUnit_; }
  auto& getRawPatchDataUnit() { return rawPatchyDataUnit_; }
  auto& getEomPatchDataUnit() { return eomPatchDataUnit_; }

private:
  size_t             tileOrder_  = 0;
  size_t             patchIndex_ = 0;
  uint8_t            patchMode_  = 0;
  PatchDataUnit      patchDataUnit_;
  InterPatchDataUnit interPatchDataUnit_;
  MergePatchDataUnit mergePatchDataUnit_;
  SkipPatchDataUnit  skipPatchDataUnit_;
  RawPatchDataUnit   rawPatchyDataUnit_;
  EOMPatchDataUnit   eomPatchDataUnit_;
};

};  // namespace vmesh
