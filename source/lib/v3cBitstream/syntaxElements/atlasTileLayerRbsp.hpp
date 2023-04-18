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
#include "atlasTileHeader.hpp"
#include "atlasTileDataUnit.hpp"

namespace vmesh {

// 8.3.6.10  Atlas tile layer RBSP syntax
class AtlasTileLayerRbsp {
public:
  AtlasTileLayerRbsp() {}
  ~AtlasTileLayerRbsp() {}

  AtlasTileLayerRbsp& operator=(const AtlasTileLayerRbsp&) = default;

  AtlasTileHeader&         getHeader() { return header_; }
  AtlasTileDataUnit&       getDataUnit() { return dataUnit_; }
  const AtlasTileHeader&   getHeader() const { return header_; }
  const AtlasTileDataUnit& getDataUnit() const { return dataUnit_; }

  auto getTileOrder() const { return tileOrder_; }
  auto getAtlasFrmOrderCntVal() const { return atlasFrmOrderCntVal_; }
  auto getAtlasFrmOrderCntMsb() const { return atlasFrmOrderCntMsb_; }
  auto getEncFrameIndex() const { return encFrameIndex_; }
  auto getEncTileIndex() const { return encTileIndex_; }

  auto& getTileOrder() { return tileOrder_; }
  auto& getAtlasFrmOrderCntVal() { return atlasFrmOrderCntVal_; }
  auto& getAtlasFrmOrderCntMsb() { return atlasFrmOrderCntMsb_; }
  auto& getEncFrameIndex() { return encFrameIndex_; }
  auto& getEncTileIndex() { return encTileIndex_; }

  auto& getSEI() { return sei_; }

private:
  AtlasTileHeader   header_;
  AtlasTileDataUnit dataUnit_;
  size_t            atlasFrmOrderCntVal_ = 0;
  size_t            atlasFrmOrderCntMsb_ = 0;
  size_t            tileOrder_           = 0;
  size_t            encFrameIndex_       = -1;
  size_t            encTileIndex_        = -1;
  PCCSEI            sei_;
};

};  // namespace vmesh
