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

#include "decoder.hpp"

#include <cstdio>
#include <sstream>

#include "util/bitstream.hpp"
#include "contexts.hpp"
#include "entropy.hpp"

#include "virtualGeometryDecoder.hpp"
#include "virtualVideoDecoder.hpp"
#include "virtualColourConverter.hpp"

namespace vmesh {

//============================================================================

//============================================================================

int32_t
VMCDecoder::decompressMotion(const Bitstream&                  bitstream,
                             const std::vector<Vec3<int32_t>>& triangles,
                             const std::vector<Vec3<int32_t>>& reference,
                             std::vector<Vec3<int32_t>>&       current,
                             const VMCDecoderParameters& /*params*/) {
  uint32_t byteCount = 0;
  bitstream.read(byteCount, _byteCounter);
  std::cout << "Motion byte count = " << byteCount << '\n';
  VMCMotionACContext ctx;
  EntropyDecoder     arithmeticDecoder;
  const auto* const  bufferPtr =
    reinterpret_cast<const char*>(bitstream.buffer.data() + _byteCounter);
  _byteCounter += byteCount;
  arithmeticDecoder.setBuffer(byteCount, bufferPtr);
  arithmeticDecoder.start();
  const auto                          pointCount = int32_t(reference.size());
  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(triangles, pointCount, vertexToTriangle);
  std::vector<int8_t>        available(pointCount, 0);
  std::vector<int8_t>        vtags(pointCount);
  std::vector<int32_t>       vadj;
  std::vector<int32_t>       tadj;
  std::vector<Vec3<int32_t>> motion(pointCount);
  current.resize(pointCount);
  for (int vindex0 = 0; vindex0 < pointCount; ++vindex0) {
    const auto    predIndex = arithmeticDecoder.decode(ctx.ctxPred);
    Vec3<int32_t> res{};
    for (int32_t k = 0; k < 3; ++k) {
      int32_t value = 0;
      if (arithmeticDecoder.decode(ctx.ctxCoeffGtN[0][k]) != 0) {
        const auto sign = arithmeticDecoder.decode(ctx.ctxSign[k]);
        ++value;
        if (arithmeticDecoder.decode(ctx.ctxCoeffGtN[1][k]) != 0) {
          value += 1
                   + arithmeticDecoder.decodeExpGolomb(
                     0, ctx.ctxCoeffRemPrefix[k], ctx.ctxCoeffRemSuffix[k]);
        }
        if (sign != 0) { value = -value; }
      }
      res[k] = value;
    }
    if (predIndex == 0) {
      motion[vindex0] = res;
    } else {
      ComputeAdjacentVertices(
        vindex0, triangles, vertexToTriangle, vtags, vadj);
      Vec3<int32_t> pred(0);
      int32_t       predCount = 0;
      for (int vindex1 : vadj) {
        if (available[vindex1] != 0) {
          const auto& mv1 = motion[vindex1];
          for (int32_t k = 0; k < 3; ++k) { pred[k] += mv1[k]; }
          ++predCount;
        }
      }
      if (predCount > 1) {
        const auto bias = predCount >> 1;
        for (int32_t k = 0; k < 3; ++k) {
          pred[k] = pred[k] >= 0 ? (pred[k] + bias) / predCount
                                 : -(-pred[k] + bias) / predCount;
        }
      }
      motion[vindex0] = res + pred;
    }
    available[vindex0] = 1;
  }
  for (int vindex = 0; vindex < pointCount; ++vindex) {
    current[vindex] = reference[vindex] + motion[vindex];
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decompressBaseMesh(const Bitstream&            bitstream,
                               const VMCGroupOfFrames&     gof,
                               VMCFrameInfo&               frameInfo,
                               VMCFrame&                   frame,
                               VMCStats&                   stats,
                               const VMCDecoderParameters& params) {
  if (decodeFrameHeader(bitstream, frameInfo) != 0) { return -1; }
  const auto frameIndex = frameInfo.frameIndex + _gofInfo.startFrameIndex_;
  auto&      base       = frame.base;
  auto&      qpositions = frame.qpositions;
  if (frameInfo.type == FrameType::INTRA) {
    printf("Inter index = %d n", frameInfo.frameIndex);
    auto     bitstreamByteCount0 = _byteCounter;
    uint32_t byteCountBaseMesh   = 0;
    bitstream.read(byteCountBaseMesh, _byteCounter);

    // Get geometry bitstream
    std::vector<uint8_t> geometryBitstream(
      bitstream.buffer.begin() + _byteCounter,
      bitstream.buffer.begin() + _byteCounter + byteCountBaseMesh);
    _byteCounter += byteCountBaseMesh;
    stats.baseMeshByteCount += _byteCounter - bitstreamByteCount0;

    // Decode base mesh
    auto decoder =
      VirtualGeometryDecoder<double>::create(GeometryCodecId::DRACO);
    decoder->decode(geometryBitstream, base);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      auto prefix = params.intermediateFilesPathPrefix + "GOF_"
                    + std::to_string(_gofInfo.index_) + "_fr_"
                    + std::to_string(frameIndex) + "_base";
      base.save(prefix + "_dec.obj");
      save(prefix + ".drc", geometryBitstream);
    }

    qpositions.resize(base.pointCount());
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      auto&       qpos  = qpositions[v];
      const auto& point = base.point(v);
      for (int32_t k = 0; k < 3; ++k) {
        qpos[k] = int32_t(std::round(point[k]));
      }
    }
    const auto scaleTexCoord  = std::pow(2.0, _sps.qpTexCoord) - 1.0;
    const auto iscaleTexCoord = 1.0 / scaleTexCoord;
    for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
      base.setTexCoord(tc, base.texCoord(tc) * iscaleTexCoord);
    }
  } else {
    printf("Inter index = %d ref = %d \n",
           frameInfo.frameIndex,
           frameInfo.referenceFrameIndex);
    const auto& refFrame     = gof.frame(frameInfo.referenceFrameIndex);
    base                     = refFrame.base;
    auto bitstreamByteCount0 = _byteCounter;
    decompressMotion(
      bitstream, base.triangles(), refFrame.qpositions, qpositions, params);
    stats.motionByteCount += _byteCounter - bitstreamByteCount0;
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      const auto& qpos  = qpositions[v];
      auto&       point = base.point(v);
      for (int32_t k = 0; k < 3; ++k) { point[k] = qpos[k]; }
    }
  }

  const auto scalePosition =
    ((1 << _sps.qpPosition) - 1.0) / ((1 << _sps.bitDepthPosition) - 1.0);
  const auto iscalePosition = 1.0 / scalePosition;
  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.setPoint(v, base.point(v) * iscalePosition);
  }

  subdivideBaseMesh(
    frame, _sps.subdivisionMethod, _sps.subdivisionIterationCount);

  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decompressDisplacementsVideo(const Bitstream&            bitstream,
                                         const VMCDecoderParameters& params) {
  // Get video bitstream
  uint32_t byteCountDispVideo = 0;
  bitstream.read(byteCountDispVideo, _byteCounter);
  std::vector<uint8_t> videoBitstream(bitstream.buffer.begin() + _byteCounter,
                                      bitstream.buffer.begin() + _byteCounter
                                        + byteCountDispVideo);
  _byteCounter += byteCountDispVideo;

  // Decode video
  auto decoder = VirtualVideoDecoder<uint16_t>::create(VideoCodecId::HM);
  decoder->decode(videoBitstream, _dispVideo, 10);

  // Save intermediate files
  if (params.keepIntermediateFiles) {
    auto dispPath =
      _dispVideo.createName(params.intermediateFilesPathPrefix + "GOF_"
                              + std::to_string(_gofInfo.index_) + "_disp_dec",
                            10);
    save(removeExtension(dispPath) + ".bin", videoBitstream);
    _dispVideo.save(dispPath);
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decompressTextureVideo(const Bitstream&            bitstream,
                                   VMCGroupOfFrames&           gof,
                                   const VMCDecoderParameters& params) {
  // get video bitstream
  uint32_t byteCountTexVideo = 0;
  bitstream.read(byteCountTexVideo, _byteCounter);
  std::vector<uint8_t> videoBitstream(bitstream.buffer.begin() + _byteCounter,
                                      bitstream.buffer.begin() + _byteCounter
                                        + byteCountTexVideo);
  _byteCounter += byteCountTexVideo;

  // Decode video
  FrameSequence<uint16_t> yuv;
  FrameSequence<uint16_t> brg;
  auto decoder = VirtualVideoDecoder<uint16_t>::create(VideoCodecId::HM);
  decoder->decode(videoBitstream, yuv, 10);

  // Convert video
  auto convert = VirtualColourConverter<uint16_t>::create(1);
  convert->convert(params.textureVideoHDRToolDecConfig, yuv, brg);

  // Store result in 8 bits frames
  for (int32_t f = 0; f < brg.frameCount(); ++f) {
    gof.frame(f).outputTexture = brg[f];
  }

  // Save intermediate files
  if (params.keepIntermediateFiles) {
    FrameSequence<uint8_t> brg8(brg);
    auto                   videoPath =
      brg8.createName(params.intermediateFilesPathPrefix + "GOF_"
                        + std::to_string(_gofInfo.index_) + "_texture_dec",
                      8);
    save(removeExtension(videoPath) + ".bin", videoBitstream);
    brg8.save(videoPath);
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decompress(const Bitstream&            bitstream,
                       VMCGroupOfFramesInfo&       gofInfo,
                       VMCGroupOfFrames&           gof,
                       size_t&                     byteCounter,
                       const VMCDecoderParameters& params) {
  _byteCounter              = byteCounter;
  _gofInfo.index_           = gofInfo.index_;
  _gofInfo.startFrameIndex_ = gofInfo.startFrameIndex_;
  auto& stats               = gof.stats;
  stats.reset();
  stats.totalByteCount = _byteCounter;
  if (decodeSequenceHeader(bitstream) != 0) { return -1; }
  _dispVideo.resize(_sps.widthDispVideo,
                    _sps.heightDispVideo,
                    ColourSpace::YUV444p,
                    _sps.frameCount);
  _gofInfo.frameCount_ = _sps.frameCount;
  _gofInfo.framesInfo_.resize(_sps.frameCount);
  gof.resize(_sps.frameCount);
  stats.frameCount = _sps.frameCount;
  for (int32_t frameIndex = 0; frameIndex < _sps.frameCount; ++frameIndex) {
    auto& frameInfo      = _gofInfo.framesInfo_[frameIndex];
    auto& frame          = gof.frame(frameIndex);
    frameInfo.frameIndex = frameIndex;
    decompressBaseMesh(bitstream, gof, frameInfo, frame, stats, params);
    stats.baseMeshVertexCount += frame.base.pointCount();
    stats.vertexCount += frame.rec.pointCount();
    stats.faceCount += frame.rec.triangleCount();
  }
  stats.displacementsByteCount = _byteCounter;
  if (_sps.encodeDisplacementsVideo
      && (decompressDisplacementsVideo(bitstream, params) != 0)) {
    return -1;
  }
  stats.displacementsByteCount = _byteCounter - stats.displacementsByteCount;

  if (_sps.encodeDisplacementsVideo) {
    for (int32_t frameIndex = 0; frameIndex < _sps.frameCount; ++frameIndex) {
      auto& frame = gof.frame(frameIndex);
      reconstructDisplacementFromVideoFrame(_dispVideo.frame(frameIndex),
                                            frame,
                                            _sps.geometryVideoBlockSize,
                                            _sps.geometryVideoBitDepth);
      inverseQuantizeDisplacements(frame,
                                   _sps.bitDepthPosition,
                                   _sps.liftingLevelOfDetailInverseScale,
                                   _sps.liftingQuantizationParameters);
      computeInverseLinearLifting(frame.disp,
                                  frame.subdivInfoLevelOfDetails,
                                  frame.subdivEdges,
                                  _sps.liftingPredictionWeight,
                                  _sps.liftingUpdateWeight,
                                  _sps.liftingSkipUpdate);
      applyDisplacements(frame, _sps.displacementCoordinateSystem);
    }
  }
  stats.textureByteCount = _byteCounter;
  // decompress texture
  if (_sps.encodeTextureVideo
      && (decompressTextureVideo(bitstream, gof, params) != 0)) {
    return -1;
  }
  stats.textureByteCount = _byteCounter - stats.textureByteCount;
  stats.totalByteCount   = _byteCounter - stats.totalByteCount;
  byteCounter            = _byteCounter;
  gofInfo                = _gofInfo;
  if (!params.normalizeUV) {
    const auto scale = (1 << _sps.bitDepthTexCoord) - 1.0;
    for (int32_t frameIndex = 0; frameIndex < _sps.frameCount; ++frameIndex) {
      auto&      rec           = gof.frame(frameIndex).rec;
      const auto texCoordCount = rec.texCoordCount();
      for (int32_t i = 0; i < texCoordCount; ++i) {
        rec.setTexCoord(i, rec.texCoord(i) * scale);
      }
    }
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decodeSequenceHeader(const Bitstream& bitstream) {
  uint16_t frameCount             = 0;
  uint8_t  bitField               = 0;
  uint16_t widthDispVideo         = 0;
  uint16_t heightDispVideo        = 0;
  uint16_t widthTexVideo          = 0;
  uint16_t heightTexVideo         = 0;
  uint8_t  geometryVideoBlockSize = 0;
  uint8_t  bitDepth               = 0;
  uint8_t  qpBaseMesh             = 0;
  uint8_t  subdivInfo             = 0;
  uint8_t  liftingQPs[3]          = {};
  uint8_t  meshCodecId = uint8_t(GeometryCodecId::UNKNOWN_GEOMETRY_CODEC);
  uint8_t  geometryVideoCodecId = uint8_t(VideoCodecId::UNKNOWN_VIDEO_CODEC);
  uint8_t  textureVideoCodecId  = uint8_t(VideoCodecId::UNKNOWN_VIDEO_CODEC);

  bitstream.read(frameCount, _byteCounter);
  bitstream.read(bitField, _byteCounter);
  bitstream.read(bitDepth, _byteCounter);
  bitstream.read(subdivInfo, _byteCounter);
  bitstream.read(meshCodecId, _byteCounter);
  bitstream.read(qpBaseMesh, _byteCounter);
  _sps.frameCount               = frameCount;
  _sps.encodeDisplacementsVideo = ((bitField & 1) != 0);
  _sps.encodeTextureVideo       = (((bitField >> 1) & 1) != 0);
  if (_sps.encodeDisplacementsVideo) {
    bitstream.read(geometryVideoCodecId, _byteCounter);
    bitstream.read(widthDispVideo, _byteCounter);
    bitstream.read(heightDispVideo, _byteCounter);
    bitstream.read(geometryVideoBlockSize, _byteCounter);
    bitstream.read(liftingQPs[0], _byteCounter);
    bitstream.read(liftingQPs[1], _byteCounter);
    bitstream.read(liftingQPs[2], _byteCounter);
  }
  if (_sps.encodeTextureVideo) {
    bitstream.read(textureVideoCodecId, _byteCounter);
    bitstream.read(widthTexVideo, _byteCounter);
    bitstream.read(heightTexVideo, _byteCounter);
  }

  // Update SPS
  _sps.widthDispVideo                   = widthDispVideo;
  _sps.heightDispVideo                  = heightDispVideo;
  _sps.widthTexVideo                    = widthTexVideo;
  _sps.heightTexVideo                   = heightTexVideo;
  _sps.geometryVideoBlockSize           = geometryVideoBlockSize;
  _sps.bitDepthPosition                 = 1 + (bitDepth & 15);
  _sps.bitDepthTexCoord                 = 1 + ((bitDepth >> 4) & 15);
  _sps.subdivisionMethod                = SubdivisionMethod(subdivInfo & 15);
  _sps.subdivisionIterationCount        = (subdivInfo >> 4) & 15;
  _sps.qpPosition                       = 1 + (qpBaseMesh & 15);
  _sps.qpTexCoord                       = 1 + ((qpBaseMesh >> 4) & 15);
  _sps.liftingQuantizationParameters[0] = liftingQPs[0];
  _sps.liftingQuantizationParameters[1] = liftingQPs[1];
  _sps.liftingQuantizationParameters[2] = liftingQPs[2];
  _sps.meshCodecId                      = GeometryCodecId(meshCodecId);
  _sps.geometryVideoCodecId             = VideoCodecId(geometryVideoCodecId);
  _sps.textureVideoCodecId              = VideoCodecId(textureVideoCodecId);
  return 0;
}

int32_t
VMCDecoder::decodeFrameHeader(const Bitstream& bitstream,
                              VMCFrameInfo&    frameInfo) {
  uint8_t frameType = 0;
  bitstream.read(frameType, _byteCounter);
  frameInfo.type             = FrameType(frameType);
  uint8_t patchCountMinusOne = 0;
  bitstream.read(patchCountMinusOne, _byteCounter);
  frameInfo.patchCount = patchCountMinusOne + 1;
  if (frameInfo.type != FrameType::INTRA) {
    uint8_t referenceFrameIndex = 0;
    bitstream.read(referenceFrameIndex, _byteCounter);
    frameInfo.referenceFrameIndex =
      frameInfo.frameIndex - int32_t(referenceFrameIndex) - 1;
  }
  return 0;
}

//============================================================================

}  // namespace vmesh
