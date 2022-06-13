/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2022, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <cstdint>
#include <string>

#include "vmc.hpp"
#include "vmcstats.hpp"
#include "bitstream.hpp"

namespace vmesh {

//============================================================================


//============================================================================

struct VMCDecoderParameters {
  std::string geometryMeshDecoderPath;
  std::string geometryVideoDecoderPath;

  // texture video
  std::string textureVideoDecoderPath;
  std::string textureVideoHDRToolPath;
  std::string textureVideoHDRToolDecConfig;

  bool normalizeUV;

  // output
  std::string intermediateFilesPathPrefix = {};
  bool keepIntermediateFiles;
};

//============================================================================

class VMCDecoder {
public:
  VMCDecoder() = default;
  VMCDecoder(const VMCDecoder& rhs) = delete;
  VMCDecoder& operator=(const VMCDecoder& rhs) = delete;
  ~VMCDecoder() = default;

  int32_t decompress(
    const Bitstream& bitstream,
    VMCGroupOfFramesInfo& gofInfo,
    VMCGroupOfFrames& gof,
    size_t& byteCounter,
    const VMCDecoderParameters& params);

private:
  int32_t decodeSequenceHeader(const Bitstream& bitstream);
  int32_t
  decodeFrameHeader(const Bitstream& bitstream, VMCFrameInfo& frameInfo);
  int32_t decompressBaseMesh(
    const Bitstream& bitstream,
    const VMCGroupOfFrames& gof,
    VMCFrameInfo& frameInfo,
    VMCFrame& frame,
    VMCStats& stats,
    const VMCDecoderParameters& params);
  int32_t decompressMotion(
    const Bitstream& bitstream,
    const std::vector<Vec3<int32_t>>& triangles,
    const std::vector<Vec3<int32_t>>& reference,
    std::vector<Vec3<int32_t>>& current,
    const VMCDecoderParameters& params);
  int32_t decompressDisplacementsVideo(
    const Bitstream& bitstream, const VMCDecoderParameters& params);
  int32_t decompressTextureVideo(
    const Bitstream& bitstream,
    VMCGroupOfFrames& gof,
    const VMCDecoderParameters& params);

private:
  size_t _byteCounter = 0;
  VMCGroupOfFramesInfo _gofInfo;
  VMCSequenceParameterSet _sps;
  FrameSequence<uint16_t> _dispVideo;  // ColourSpace::YUV444p
};

//============================================================================

}  // namespace vmesh
