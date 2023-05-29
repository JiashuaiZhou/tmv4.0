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
#include "v3cBitstream.hpp"

namespace vmesh {

//============================================================================

//============================================================================

struct VMCDecoderParameters {
  std::string textureVideoHDRToolDecConfig = {};
  int32_t     textureVideoUpsampleFilter   = 0;
  bool        textureVideoFullRange        = false;
  bool        dequantizeUV                 = true;
  bool        keepIntermediateFiles        = false;
  bool        reconstructNormals           = true;
};

//============================================================================

class VMCDecoder {
public:
  VMCDecoder()                                 = default;
  VMCDecoder(const VMCDecoder& rhs)            = delete;
  VMCDecoder& operator=(const VMCDecoder& rhs) = delete;
  ~VMCDecoder()                                = default;

  bool decompress(const V3cBitstream&         syntax,
                  VMCGroupOfFramesInfo&       gofInfo,
                  Sequence&                   reconstruct,
                  const VMCDecoderParameters& params);

  inline void setKeepFilesPathPrefix(const std::string& path) {
    _keepFilesPathPrefix = path;
  }

private:
  bool decompressBaseMesh(const V3cBitstream&         syntax,
                          const BaseMeshTileLayer&    bmtl,
                          const AtlasTileLayerRbsp&   atl,
                          const VMCGroupOfFrames&     gof,
                          VMCFrameInfo&               frameInfo,
                          VMCFrame&                   frame,
                          TriangleMesh<MeshType>&     rec,
                          const VMCDecoderParameters& params);
  bool decompressMotion(
    const V3cBitstream&               syntax,
    const BaseMeshTileLayer&          bmtl,
    const std::vector<Vec3<int32_t>>& trianglesReference,
    const std::vector<Vec3<int32_t>>& referenceReference,
    const std::vector<Vec2<int32_t>>& baseIntegrateIndicesReference,
    const std::vector<Vec3<int32_t>>& trianglesBase,
    const std::vector<Vec3<int32_t>>& referenceBase,
    std::vector<Vec3<int32_t>>&       current,
    const VMCDecoderParameters&       params);
  bool computeVertexAdjTableMotion(const std::vector<Vec3<int32_t>>& triangles,
                                   int32_t vertexCount,
                                   int32_t maxNumNeighborsMotion);
  bool addNeighbor(int32_t vertex,
                   int32_t vertexNeighbor,
                   int32_t maxNumNeighborsMotion);
  bool decompressDisplacementsVideo(const V3cBitstream&         syntax,
                                    FrameSequence<uint16_t>&    dispVideo,
                                    const VMCDecoderParameters& params);
  bool decompressDisplacementsAC(const V3cBitstream&   syntax,
                                 VMCGroupOfFrames&     gof,
                                 VMCGroupOfFramesInfo& gofInfo,
                                 int32_t               subBlockSize);
  bool decompressTextureVideo(const V3cBitstream&         syntax,
                              Sequence&                   reconsctruct,
                              const VMCDecoderParameters& params);

  std::string          _keepFilesPathPrefix = {};
  std::vector<int32_t> vertexAdjTableMotion;
  bool                 createVertexAdjTableMotion = true;
  std::vector<int32_t> numNeighborsMotion;
  std::vector<int32_t> umapping;
};

//============================================================================

}  // namespace vmesh
