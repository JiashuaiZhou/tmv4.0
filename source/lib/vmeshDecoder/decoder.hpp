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
#include "vmcStats.hpp"
#include "util/bitstream.hpp"

namespace vmesh {

//============================================================================

//============================================================================

struct VMCDecoderParameters {
  std::string textureVideoHDRToolDecConfig = {};
  int32_t     textureVideoUpsampleFilter   = 0;
  bool        textureVideoFullRange        = false;
  bool        dequantizeUV                 = true;
  bool        keepIntermediateFiles        = false;
};

//============================================================================

class VMCDecoder {
public:
  VMCDecoder()                                 = default;
  VMCDecoder(const VMCDecoder& rhs)            = delete;
  VMCDecoder& operator=(const VMCDecoder& rhs) = delete;
  ~VMCDecoder()                                = default;

  bool decompress(const Bitstream&            bitstream,
                  VMCGroupOfFramesInfo&       gofInfo,
                  Sequence&                   reconstruct,
                  size_t&                     byteCounter,
                  const VMCDecoderParameters& params);

  uint32_t getBitDepthTexCoord() { return _sps.bitDepthTexCoord; }

  inline void setKeepFilesPathPrefix(const std::string& path) {
    _keepFilesPathPrefix = path;
  }
  VMCStats& stats() { return _stats; }

private:
  bool decodeSequenceHeader(const Bitstream& bitstream);
  bool decodeFrameHeader(const Bitstream& bitstream, VMCFrameInfo& frameInfo);
  bool decompressBaseMesh(const Bitstream&            bitstream,
                          const VMCGroupOfFrames&     gof,
                          VMCFrameInfo&               frameInfo,
                          VMCFrame&                   frame,
                          TriangleMesh<MeshType>&     rec,
                          const VMCDecoderParameters& params);
  bool decompressMotion(
    const Bitstream&                  bitstream,
    const std::vector<Vec3<int32_t>>& trianglesReference,
    const std::vector<Vec3<int32_t>>& referenceReference,
    const std::vector<Vec2<int32_t>>& baseIntegrateIndicesReference,
    const std::vector<Vec3<int32_t>>& trianglesBase,
    const std::vector<Vec3<int32_t>>& referenceBase,
    std::vector<Vec3<int32_t>>&       current,
    const VMCDecoderParameters&       params);
  bool decompressDisplacementsVideo(const Bitstream&            bitstream,
                                    FrameSequence<uint16_t>&    dispVideo,
                                    const VMCDecoderParameters& params);
  bool decompressTextureVideo(const Bitstream&            bitstream,
                              Sequence&                   reconsctruct,
                              const VMCDecoderParameters& params);

  size_t                  _byteCounter = 0;
  VMCSequenceParameterSet _sps;
  std::string             _keepFilesPathPrefix = {};
  VMCStats                _stats;
};

//============================================================================

}  // namespace vmesh
