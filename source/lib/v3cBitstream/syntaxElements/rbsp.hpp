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
#include "sei.hpp"

namespace vmesh {

// 8.3.6.4  Supplemental enhancement information Rbsp
class seiRbsp {
public:
  seiRbsp() {}
  ~seiRbsp() {}
  seiRbsp& operator=(const seiRbsp&) = default;

private:
  std::vector<SEI> sei_;
};

// 8.3.6.5  Access unit delimiter RBSP syntax
class AccessUnitDelimiterRbsp {
public:
  AccessUnitDelimiterRbsp() : aframeType_(0) {}
  ~AccessUnitDelimiterRbsp() {}
  AccessUnitDelimiterRbsp& operator=(const AccessUnitDelimiterRbsp&) = default;

  auto getAframeType() const { return aframeType_; }

  auto& getAframeType() { return aframeType_; }

private:
  uint8_t aframeType_;
};

// 8.3.6.6  End of sequence RBSP syntax
class EndOfSequenceRbsp {
public:
  EndOfSequenceRbsp() {}
  ~EndOfSequenceRbsp() {}
  EndOfSequenceRbsp& operator=(const EndOfSequenceRbsp&) = default;

private:
};

// 8.3.6.7 End of bitstream RBSP syntax
class EndOfAtlasSubBitstreamRbsp {
public:
  EndOfAtlasSubBitstreamRbsp() {}
  ~EndOfAtlasSubBitstreamRbsp() {}
  EndOfAtlasSubBitstreamRbsp&
  operator=(const EndOfAtlasSubBitstreamRbsp&) = default;

private:
};

// 8.3.6.8  Filler data RBSP syntax
class FillerDataRbsp {
public:
  FillerDataRbsp() {}
  ~FillerDataRbsp() {}
  FillerDataRbsp& operator=(const FillerDataRbsp&) = default;

private:
};
};  // namespace vmesh
