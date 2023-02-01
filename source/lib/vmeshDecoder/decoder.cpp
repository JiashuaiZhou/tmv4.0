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

#include "geometryDecoder.hpp"
#include "videoDecoder.hpp"
#include "colourConverter.hpp"

namespace vmesh {

//============================================================================

//============================================================================

bool
VMCDecoder::decompressMotion(const Bitstream&                  bitstream,
                             const std::vector<Vec3<int32_t>>& trianglesReference,
                             const std::vector<Vec3<int32_t>>& referenceReference,
                             const std::vector<Vec2<int32_t>>& baseIntegrateIndicesReference,
                             const std::vector<Vec3<int32_t>>& trianglesBase,
                             const std::vector<Vec3<int32_t>>& referenceBase,
                             std::vector<Vec3<int32_t>>&       current,
                             const VMCDecoderParameters& /*params*/) {
  printf("decompressMotion \n");
  fflush(stdout);
  // here we should decode all_skip_mode from bitstream before starting to decode the MVs
  bool all_skip_mode = true;
  int no_skip_num = 0;
  std::vector<uint8_t> no_skip_vindices;
  uint8_t skip_mode_bits;  // 1 bit for all_skip_mode, 7 bits for no_skip_num
  bitstream.read(skip_mode_bits, _byteCounter);
  int skip_mode_size = sizeof(skip_mode_bits);
  std::vector<Vec3<int32_t>> triangles;
  std::vector<Vec3<int32_t>> reference;
  std::vector<Vec2<int32_t>> baseIntegrateIndices;
  if (skip_mode_bits == 255) {
    triangles = trianglesBase;
    reference = referenceBase;
  } else if (skip_mode_bits == 128) {
    triangles            = trianglesReference;
    reference            = referenceReference;
    baseIntegrateIndices = baseIntegrateIndicesReference;
  } else {
    all_skip_mode = false;
    no_skip_num   = static_cast<decltype(no_skip_num)>(skip_mode_bits);
    triangles            = trianglesReference;
    reference            = referenceReference;
    baseIntegrateIndices = baseIntegrateIndicesReference;
  }
  if (!all_skip_mode){
    no_skip_vindices.resize(no_skip_num);
    const auto byteCount = no_skip_num * sizeof(decltype(no_skip_vindices)::value_type);
    std::copy(bitstream.buffer.begin() + _byteCounter,
              bitstream.buffer.begin() + _byteCounter + byteCount,
              reinterpret_cast<uint8_t*>(no_skip_vindices.data()));
    _byteCounter += byteCount;
    skip_mode_size += byteCount;
    for (int i = 0; i < no_skip_num; ++i) {
      std::cout << "[DEBUG][stat] non-skippable MV " << static_cast<int32_t>(no_skip_vindices[i])
                << " , index: " << i
                << ". Total non-skippable MVs: " << no_skip_num
                << "\n";
    }
  }
  printf("skip_mode_bits = %u \n",skip_mode_bits);
  std::cout << "[DEBUG][stat]skip mode bits: " << skip_mode_bits << std::endl;
  std::cout << "[DEBUG][stat]skip mode bytes: " << skip_mode_size << std::endl;

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
  const auto refPointCount = static_cast<int32_t>(reference.size());
  const auto motionCount   = refPointCount + no_skip_num;
  const auto pointCount = refPointCount + static_cast<int32_t>(baseIntegrateIndices.size());
  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(triangles, motionCount, vertexToTriangle);
  std::vector<int8_t>        available(motionCount, 0);
  std::vector<int8_t>        vtags(motionCount);
  std::vector<int32_t>       vadj;
  std::vector<int32_t>       tadj;
  std::vector<Vec3<int32_t>> motion(motionCount);
  current.resize(pointCount);
  std::cout << "[DEBUG][stat] total vertex num: " << pointCount
            << " duplicated vertex num: " << baseIntegrateIndices.size()
            << " non-skippable MV num: " << no_skip_num << std::endl;
  int32_t remainP = motionCount;
  int32_t vindexS = 0, vindexE = 0, vCount = 0;
  while (remainP) {
    vindexS = vindexE;
    const auto    predIndex = arithmeticDecoder.decode(ctx.ctxPred);
    vCount = 0;
    while ((vCount < _sps.motionGroupSize) && (remainP)) {
        int vindex0 = vindexE;
        ++vindexE;
        --remainP;
        ++vCount;
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
	      int vindex = vindex0;
	      if (vindex0 >= refPointCount) {
	        auto idx = no_skip_vindices[vindex0 - refPointCount];
	        auto integrate_to = baseIntegrateIndices[idx][1];
	        auto it = std::lower_bound(
	          baseIntegrateIndices.begin(),
	          baseIntegrateIndices.end(),
	          Vec2<int32_t>(integrate_to, 0),
	          [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) { return a[0] < b[0]; });
	        auto shift = std::distance(baseIntegrateIndices.begin(), it);
	        vindex = static_cast<int32_t>(integrate_to - shift);
	      }
	      ComputeAdjacentVertices(
	        vindex, triangles, vertexToTriangle, vtags, vadj);
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
  }
  for (int vindex = 0; vindex < pointCount; ++vindex) {
    auto index = vindex;
    auto it =
      std::lower_bound(
        baseIntegrateIndices.begin(),
        baseIntegrateIndices.end(),
        Vec2<int32_t>(index, 0),
        [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) { return a[0] < b[0]; });
    if (it != baseIntegrateIndices.end() && (*it)[0] == index) {
      // set integrated index
      index = (*it)[1];
      it =
        std::lower_bound(
          baseIntegrateIndices.begin(),
          baseIntegrateIndices.end(),
          Vec2<int32_t>(index, 0),
          [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) { return a[0] < b[0]; });
    }
    // move up index
    auto shift       = std::distance(baseIntegrateIndices.begin(), it);
    auto refindex    = index - shift;
    auto motionindex = refindex;
    auto it_no_skip =
      std::find_if(no_skip_vindices.begin(),
                   no_skip_vindices.end(),
                   [&](const decltype(no_skip_vindices)::value_type& a) {
                     return baseIntegrateIndices[a][0] == vindex;
                   });
    if (it_no_skip != no_skip_vindices.end()) {
      auto no_skip_index = std::distance(no_skip_vindices.begin(), it_no_skip);
      motionindex        = refPointCount + no_skip_index;
    }
    current[vindex] = reference[refindex] + motion[motionindex];
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressBaseMesh(const Bitstream&            bitstream,
                               const VMCGroupOfFrames&     gof,
                               VMCFrameInfo&               frameInfo,
                               VMCFrame&                   frame,
                               TriangleMesh<MeshType>&     rec,
                               const VMCDecoderParameters& params) {
  printf("Decompress base mesh: Frame = %d \n", frameInfo.frameIndex);
  fflush(stdout);
  if (!decodeFrameHeader(bitstream, frameInfo)) return false;
  auto& base       = frame.base;
  auto& qpositions = frame.qpositions;
  if (frameInfo.type == FrameType::INTRA) {
    printf("Intra index = %d \n", frameInfo.frameIndex);
    auto     bitstreamByteCount0 = _byteCounter;
    uint32_t byteCountBaseMesh   = 0;
    bitstream.read(byteCountBaseMesh, _byteCounter);

    // Get geometry bitstream
    std::vector<uint8_t> geometryBitstream(
      bitstream.buffer.begin() + _byteCounter,
      bitstream.buffer.begin() + _byteCounter + byteCountBaseMesh);
    _byteCounter += byteCountBaseMesh;
    _stats.baseMeshByteCount += _byteCounter - bitstreamByteCount0;

    // Decode base mesh
    printf("Decode base mesh \n");
    auto decoder = GeometryDecoder<MeshType>::create(_sps.meshCodecId);
    GeometryDecoderParameters decoderParams;
    decoderParams.dracoUsePosition_ = _sps.dracoUsePosition;
    decoderParams.dracoUseUV_       = _sps.dracoUseUV;
    printf("BaseMeshDeco: use_position = %d use_uv = %d \n",
           decoderParams.dracoUsePosition_,
           decoderParams.dracoUseUV_);
    fflush(stdout);
    decoder->decode(geometryBitstream, decoderParams, base);
    printf("BaseMeshDeco: done \n");
    fflush(stdout);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      auto prefix = _keepFilesPathPrefix + "fr_"
                    + std::to_string(frameInfo.frameIndex) + "_base";
      base.save(prefix + "_dec.ply");
      save(prefix + ".drc", geometryBitstream);
    }

    printf("Round qposition \n");
    fflush(stdout);
    qpositions.resize(base.pointCount());
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      qpositions[v] = base.point(v).round();
    }
    printf("Scale TexCoord \n");
    fflush(stdout);
    const auto scaleTexCoord  = std::pow(2.0, _sps.qpTexCoord) - 1.0;
    const auto iscaleTexCoord = 1.0 / scaleTexCoord;
    for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
      base.texCoord(tc) *= iscaleTexCoord;
    }
    // Duplicated Vertex Reduction
    removeDuplicatedVertices(frame);
    if (params.keepIntermediateFiles) {
      auto& baseClean = frame.baseClean;
      baseClean.save(_keepFilesPathPrefix + "fr_"
                     + std::to_string(frameInfo.frameIndex)
                     + "_baseClean.obj");
    }
  } else {
    printf("Inter index = %d ref = %d \n",
           frameInfo.frameIndex,
           frameInfo.referenceFrameIndex);
    const auto& refFrame     = gof.frame(frameInfo.referenceFrameIndex);
    base                     = refFrame.base;
    auto bitstreamByteCount0 = _byteCounter;
    decompressMotion(bitstream,
                     refFrame.baseClean.triangles(),
                     refFrame.baseClean.points(),
                     refFrame.baseIntegrateIndices,
                     base.triangles(),
                     refFrame.qpositions,
                     qpositions,
                     params);
    // decompressMotion(
    //     bitstream, base.triangles(), refFrame.qpositions, {}, qpositions, params);

    _stats.motionByteCount += _byteCounter - bitstreamByteCount0;
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      base.point(v) = qpositions[v];
    }
    // Duplicated Vertex Reduction
    removeDuplicatedVertices(frame);
  }

  printf("Scale position: Frame = %d \n", frameInfo.frameIndex);
  fflush(stdout);
  const auto scalePosition =
    ((1 << _sps.qpPosition) - 1.0) / ((1 << _sps.bitDepthPosition) - 1.0);
  const auto iscalePosition = 1.0 / scalePosition;
  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.point(v) *= iscalePosition;
  }

  subdivideBaseMesh(
    frame, rec, _sps.subdivisionMethod, _sps.subdivisionIterationCount);

  _stats.baseMeshVertexCount += frame.base.pointCount();
  _stats.vertexCount += rec.pointCount();
  _stats.faceCount += rec.triangleCount();
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressDisplacementsVideo(const Bitstream&            bitstream,
                                         FrameSequence<uint16_t>&    dispVideo,
                                         const VMCDecoderParameters& params) {
  printf("Decompress displacements video \n");
  fflush(stdout);
  // Get video bitstream
  uint32_t byteCountDispVideo = 0;
  bitstream.read(byteCountDispVideo, _byteCounter);
  std::vector<uint8_t> videoBitstream(bitstream.buffer.begin() + _byteCounter,
                                      bitstream.buffer.begin() + _byteCounter
                                        + byteCountDispVideo);
  _byteCounter += byteCountDispVideo;

  // Decode video
  auto decoder =
    VideoDecoder<uint16_t>::create(VideoCodecId(_sps.geometryVideoCodecId));
  decoder->decode(videoBitstream, dispVideo, 10);

  // Save intermediate files
  if (params.keepIntermediateFiles) {
    auto path = dispVideo.createName(_keepFilesPathPrefix + "disp_dec", 10);
    save(removeExtension(path) + ".bin", videoBitstream);
    dispVideo.save(path);
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressTextureVideo(const Bitstream&            bitstream,
                                   Sequence&                   reconsctruct,
                                   const VMCDecoderParameters& params) {
  printf("Decompress texture video\n");
  fflush(stdout);
  if (_sps.encodeTextureVideo == 0) {
    for (auto& texture : reconsctruct.textures()) {
      texture.clear();
      texture.resize(1, 1, ColourSpace::BGR444p);
      texture.zero();
    }
  } else {
    // get video bitstream
    uint32_t byteCountTexVideo = 0;
    bitstream.read(byteCountTexVideo, _byteCounter);
    printf("byteCountTexVideo = %u \n", byteCountTexVideo);
    fflush(stdout);
    std::vector<uint8_t> videoBitstream(
      bitstream.buffer.begin() + _byteCounter,
      bitstream.buffer.begin() + _byteCounter + byteCountTexVideo);
    _byteCounter += byteCountTexVideo;

    // Decode video
    printf("Decode video \n");
    fflush(stdout);
    FrameSequence<uint16_t> dec;
    auto                    decoder =
      VideoDecoder<uint16_t>::create(VideoCodecId(_sps.textureVideoCodecId));
    decoder->decode(videoBitstream, dec, 10);

    // Convert video
    printf("Convert video \n");
    fflush(stdout);
#if USE_HDRTOOLS
    auto convert = ColourConverter<uint16_t>::create(1);
    convert->initialize(params.textureVideoHDRToolDecConfig);
#else
    auto convert = ColourConverter<uint16_t>::create(0);
    auto mode    = "YUV420p_BGR444p_10_8_"
                + std::to_string(params.textureVideoUpsampleFilter) + "_"
                + std::to_string(params.textureVideoFullRange);
    convert->initialize(mode);
#endif
    convert->convert(dec);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      FrameSequence<uint8_t> dec8(dec);
      auto path = dec8.createName(_keepFilesPathPrefix + "tex_dec", 8);
      dec8.save(path);
      save(removeExtension(path) + ".bin", videoBitstream);
    }

    // Store result in 8 bits frames
    printf("Store result in 8 bits frames \n");
    fflush(stdout);
    reconsctruct.textures() = dec;
    dec.clear();
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompress(const Bitstream&            bitstream,
                       VMCGroupOfFramesInfo&       gofInfoSrc,
                       Sequence&                   reconstruct,
                       size_t&                     byteCounter,
                       const VMCDecoderParameters& params) {
  VMCGroupOfFrames     gof;
  VMCGroupOfFramesInfo gofInfo;
  _byteCounter             = byteCounter;
  gofInfo.index_           = gofInfoSrc.index_;
  gofInfo.startFrameIndex_ = gofInfoSrc.startFrameIndex_;
  _stats.reset();
  _stats.totalByteCount = _byteCounter;
  printf("Decompress gop: \n");
  fflush(stdout);
  if (!decodeSequenceHeader(bitstream)) return false;
  const auto colourSpaceDispVideo = _sps.applyOneDimensionalDisplacement
                                      ? ColourSpace::YUV400p
                                      : ColourSpace::YUV444p;
  FrameSequence<uint16_t> dispVideo;
  dispVideo.resize(_sps.widthDispVideo,
                   _sps.heightDispVideo,
                   colourSpaceDispVideo,
                   _sps.frameCount);
  gofInfo.frameCount_ = _sps.frameCount;
  gofInfo.framesInfo_.resize(_sps.frameCount);
  gof.resize(_sps.frameCount);
  reconstruct.resize(_sps.frameCount);
  _stats.frameCount = _sps.frameCount;
  for (int32_t frameIndex = 0; frameIndex < _sps.frameCount; ++frameIndex) {
    auto& frameInfo      = gofInfo.framesInfo_[frameIndex];
    auto& frame          = gof.frame(frameIndex);
    auto& rec            = reconstruct.mesh(frameIndex);
    frameInfo.frameIndex = frameIndex;
    decompressBaseMesh(bitstream, gof, frameInfo, frame, rec, params);
  }
  _stats.displacementsByteCount = _byteCounter;
  if (_sps.encodeDisplacementsVideo
      && !decompressDisplacementsVideo(bitstream, dispVideo, params)) {
    return false;
  }
  _stats.displacementsByteCount = _byteCounter - _stats.displacementsByteCount;

  if (_sps.encodeDisplacementsVideo) {
    for (int32_t frameIndex = 0; frameIndex < _sps.frameCount; ++frameIndex) {
      auto& frame = gof.frame(frameIndex);
      auto& rec   = reconstruct.mesh(frameIndex);
      reconstructDisplacementFromVideoFrame(dispVideo.frame(frameIndex),
                                            frame,
                                            rec,
                                            _sps.geometryVideoBlockSize,
                                            _sps.geometryVideoBitDepth);
      inverseQuantizeDisplacements(frame,
                                   _sps.bitDepthPosition,
                                   _sps.liftingLevelOfDetailInverseScale,
                                   _sps.liftingQP);
      computeInverseLinearLifting(frame.disp,
                                  frame.subdivInfoLevelOfDetails,
                                  frame.subdivEdges,
                                  _sps.liftingPredictionWeight,
                                  _sps.liftingUpdateWeight,
                                  _sps.liftingSkipUpdate);
      applyDisplacements(frame, rec, _sps.displacementCoordinateSystem);
    }
  }
  _stats.textureByteCount = _byteCounter;
  // decompress texture
  if (!decompressTextureVideo(bitstream, reconstruct, params)) {
    return false;
  }
  printf("Done \n");
  fflush(stdout);
  _stats.textureByteCount = _byteCounter - _stats.textureByteCount;
  _stats.totalByteCount   = _byteCounter - _stats.totalByteCount;
  byteCounter             = _byteCounter;
  gofInfoSrc              = gofInfo;

  // Quantize UV coordinate
  if (!params.dequantizeUV) {
    const auto scale = (1 << _sps.bitDepthTexCoord) - 1.0;
    for (auto& rec : reconstruct.meshes()) {
      const auto texCoordCount = rec.texCoordCount();
      for (int32_t i = 0; i < texCoordCount; ++i) { rec.texCoord(i) *= scale; }
    }
  }
  return true;
}

//----------------------------------------------------------------------------

bool
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
  uint8_t  motionGroupSize        = 0;
  uint8_t  liftingQPs[3]          = {};
  uint8_t  meshCodecId = uint8_t(GeometryCodecId::UNKNOWN_GEOMETRY_CODEC);
  uint8_t  geometryVideoCodecId = uint8_t(VideoCodecId::UNKNOWN_VIDEO_CODEC);
  uint8_t  textureVideoCodecId  = uint8_t(VideoCodecId::UNKNOWN_VIDEO_CODEC);
  uint8_t  dracoUsePosition     = 0;
  uint8_t  dracoUseUV           = 0;

  bitstream.read(frameCount, _byteCounter);
  bitstream.read(bitField, _byteCounter);
  bitstream.read(bitDepth, _byteCounter);
  bitstream.read(subdivInfo, _byteCounter);
  bitstream.read(motionGroupSize, _byteCounter);
#if defined(CODE_CODEC_ID)
  bitstream.read(meshCodecId, _byteCounter);
#else
  meshCodecId = uint8_t(GeometryCodecId::DRACO);
#endif
  bitstream.read(dracoUsePosition, _byteCounter);
  bitstream.read(dracoUseUV, _byteCounter);
  bitstream.read(qpBaseMesh, _byteCounter);
  _sps.frameCount                      = frameCount;
  _sps.encodeDisplacementsVideo        = ((bitField & 1) != 0);
  _sps.encodeTextureVideo              = (((bitField >> 1) & 1) != 0);
  _sps.applyOneDimensionalDisplacement = (((bitField >> 2) & 1) != 0);
  if (_sps.encodeDisplacementsVideo) {
#if defined(CODE_CODEC_ID)
    bitstream.read(geometryVideoCodecId, _byteCounter);
#else
    geometryVideoCodecId = uint8_t(VideoCodecId::HM);
#endif
    bitstream.read(widthDispVideo, _byteCounter);
    bitstream.read(heightDispVideo, _byteCounter);
    bitstream.read(geometryVideoBlockSize, _byteCounter);
    bitstream.read(liftingQPs[0], _byteCounter);
    bitstream.read(liftingQPs[1], _byteCounter);
    bitstream.read(liftingQPs[2], _byteCounter);
  }
  if (_sps.encodeTextureVideo) {
#if defined(CODE_CODEC_ID)
    bitstream.read(textureVideoCodecId, _byteCounter);
#else
    textureVideoCodecId = uint8_t(VideoCodecId::HM);
#endif
    bitstream.read(widthTexVideo, _byteCounter);
    bitstream.read(heightTexVideo, _byteCounter);
  }

  // Update SPS
  _sps.widthDispVideo            = widthDispVideo;
  _sps.heightDispVideo           = heightDispVideo;
  _sps.widthTexVideo             = widthTexVideo;
  _sps.heightTexVideo            = heightTexVideo;
  _sps.geometryVideoBlockSize    = geometryVideoBlockSize;
  _sps.bitDepthPosition          = 1 + (bitDepth & 15);
  _sps.bitDepthTexCoord          = 1 + ((bitDepth >> 4) & 15);
  _sps.subdivisionMethod         = SubdivisionMethod(subdivInfo & 15);
  _sps.subdivisionIterationCount = (subdivInfo >> 4) & 15;
  _sps.qpPosition                = 1 + (qpBaseMesh & 15);
  _sps.qpTexCoord                = 1 + ((qpBaseMesh >> 4) & 15);
  _sps.liftingQP[0]              = liftingQPs[0];
  _sps.liftingQP[1]              = liftingQPs[1];
  _sps.liftingQP[2]              = liftingQPs[2];
  _sps.meshCodecId               = GeometryCodecId(meshCodecId);
  _sps.geometryVideoCodecId      = VideoCodecId(geometryVideoCodecId);
  _sps.textureVideoCodecId       = VideoCodecId(textureVideoCodecId);
  _sps.dracoUsePosition          = dracoUsePosition;
  _sps.dracoUseUV                = dracoUseUV;
  _sps.motionGroupSize           = motionGroupSize;
  return true;
}

bool
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
  return true;
}

//============================================================================

}  // namespace vmesh
