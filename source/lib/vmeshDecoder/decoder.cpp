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

#include "motionContexts.hpp"
#include "entropy.hpp"

#include "geometryDecoder.hpp"
#include "videoDecoder.hpp"
#include "colourConverter.hpp"

namespace vmesh {

//============================================================================

bool
VMCDecoder::decompressMotion(
  const V3cBitstream&               syntax,
  const BaseMeshTileLayer&          bmtl,
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
  bool                 all_skip_mode = true;
  int                  no_skip_num   = 0;
  std::vector<uint8_t> no_skip_vindices;
  uint8_t skip_mode_bits;  // 1 bit for all_skip_mode, 7 bits for no_skip_num

  int                        skip_mode_size = sizeof(skip_mode_bits);
  std::vector<Vec3<int32_t>> triangles;
  std::vector<Vec3<int32_t>> reference;
  std::vector<Vec2<int32_t>> baseIntegrateIndices;
  auto&                      bmth  = bmtl.getHeader();
  auto&                      bmtdu = bmtl.getDataUnit();
  if (!bmtdu.getMotionSkipFlag()) {  // skip_mode_bits == 255
    skip_mode_bits = 255;
    triangles      = trianglesBase;
    reference      = referenceBase;
  } else if (bmtdu.getMotionSkipAll()) {  // skip_mode_bits == 128
    skip_mode_bits       = 128;
    triangles            = trianglesReference;
    reference            = referenceReference;
    baseIntegrateIndices = baseIntegrateIndicesReference;
  } else {
    all_skip_mode        = false;
    no_skip_num          = bmtdu.getMotionSkipCount();
    skip_mode_bits       = bmtdu.getMotionSkipCount();
    triangles            = trianglesReference;
    reference            = referenceReference;
    baseIntegrateIndices = baseIntegrateIndicesReference;
  }
  if (!all_skip_mode) {
    no_skip_vindices.resize(no_skip_num);
    // const auto byteCount =
    //   no_skip_num * sizeof(decltype(no_skip_vindices)::value_type);
    // std::copy(bitstream.buffer.begin() + _byteCounter,
    //           bitstream.buffer.begin() + _byteCounter + byteCount,
    //           reinterpret_cast<uint8_t*>(no_skip_vindices.data()));
    // _byteCounter += byteCount;
    // skip_mode_size += byteCount;
    no_skip_vindices = bmtdu.getMotionSkipVextexIndices();
    for (int i = 0; i < no_skip_num; ++i) {
      std::cout << "[DEBUG][stat] non-skippable MV "
                << static_cast<int32_t>(no_skip_vindices[i])
                << " , index: " << i
                << ". Total non-skippable MVs: " << no_skip_num << "\n";
    }
  }
  printf("MotionSkipFlag  = %d  \n", bmtdu.getMotionSkipFlag());
  printf("MotionSkipAll   = %d  \n", bmtdu.getMotionSkipAll());
  printf("MotionSkipCount = %d  \n", bmtdu.getMotionSkipCount());
  printf("skip_mode_bits  = %u \n", skip_mode_bits);
  printf("skip_mode_size  = %d \n", skip_mode_size);

  VMCMotionACContext ctx;
  EntropyDecoder     arithmeticDecoder;
  // const auto* const  bufferPtr =
  //   reinterpret_cast<const char*>(bitstream.buffer.data() + _byteCounter);
  // _byteCounter += byteCount;
  // arithmeticDecoder.setBuffer(byteCount, bufferPtr);
  const auto& atlas    = syntax.getAtlas();
  const auto& asps     = atlas.getAtlasSequenceParameterSet(0);
  const auto& ext      = asps.getAspsVdmcExtension();
  const auto& baseMesh = syntax.getBaseMesh();
  const auto  bmfpsId  = bmth.getBaseMeshFrameParameterSetId();
  const auto& bmfps    = baseMesh.getBaseMeshFrameParameterSet(bmfpsId);
  const auto& buffer   = bmtdu.getData().vector();

  uint32_t byteCount = buffer.size();
  // bitstream.read(byteCount, _byteCounter);
  std::cout << "Motion byte count = " << byteCount << '\n';
  arithmeticDecoder.setBuffer(buffer.size(),
                              reinterpret_cast<const char*>(buffer.data()));
  arithmeticDecoder.start();
  const auto refPointCount = static_cast<int32_t>(reference.size());
  const auto motionCount   = refPointCount + no_skip_num;
  const auto pointCount =
    refPointCount + static_cast<int32_t>(baseIntegrateIndices.size());
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
    vindexS              = vindexE;
    const auto predIndex = arithmeticDecoder.decode(ctx.ctxPred);
    vCount               = 0;
    while ((vCount < bmfps.getMotionGroupSize()) && (remainP)) {
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
                       0, ctx.ctxCoeffRemPrefix);
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
          auto idx          = no_skip_vindices[vindex0 - refPointCount];
          auto integrate_to = baseIntegrateIndices[idx][1];
          auto it           = std::lower_bound(
            baseIntegrateIndices.begin(),
            baseIntegrateIndices.end(),
            Vec2<int32_t>(integrate_to, 0),
            [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) {
              return a[0] < b[0];
            });
          auto shift = std::distance(baseIntegrateIndices.begin(), it);
          vindex     = static_cast<int32_t>(integrate_to - shift);
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
      std::lower_bound(baseIntegrateIndices.begin(),
                       baseIntegrateIndices.end(),
                       Vec2<int32_t>(index, 0),
                       [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) {
                         return a[0] < b[0];
                       });
    if (it != baseIntegrateIndices.end() && (*it)[0] == index) {
      // set integrated index
      index = (*it)[1];
      it =
        std::lower_bound(baseIntegrateIndices.begin(),
                         baseIntegrateIndices.end(),
                         Vec2<int32_t>(index, 0),
                         [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) {
                           return a[0] < b[0];
                         });
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
VMCDecoder::decompressBaseMesh(const V3cBitstream&         syntax,
                               const BaseMeshTileLayer&    bmtl,
                               const VMCGroupOfFrames&     gof,
                               VMCFrameInfo&               frameInfo,
                               VMCFrame&                   frame,
                               TriangleMesh<MeshType>&     rec,
                               const VMCDecoderParameters& params) {
  printf("Decompress base mesh: Frame = %d \n", frameInfo.frameIndex);
  fflush(stdout);
  // if (!decodeFrameHeader(bitstream, frameInfo)) return false;
  auto& bmth       = bmtl.getHeader();
  auto& bmtdu      = bmtl.getDataUnit();
  auto& baseMesh   = syntax.getBaseMesh();
  auto  bmspsId    = bmth.getBaseMeshFrameParameterSetId();
  auto& bmsps      = baseMesh.getBaseMeshSequenceParameterSet(bmspsId);
  auto& base       = frame.base;
  auto& qpositions = frame.qpositions;
  frameInfo.type   = bmth.getBaseMeshType();
  if (frameInfo.type == I_BASEMESH) {
    printf("Intra index = %d \n", frameInfo.frameIndex);

    // Decode base mesh
    printf("Decode base mesh \n");
    auto decoder = GeometryDecoder<MeshType>::create(
      GeometryCodecId(bmsps.getBaseMeshCodecId()));
    GeometryDecoderParameters decoderParams;
    decoder->decode(bmtdu.getData().vector(), decoderParams, base);
    printf("BaseMeshDeco: done \n");
    fflush(stdout);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      auto prefix = _keepFilesPathPrefix + "fr_"
                    + std::to_string(frameInfo.frameIndex) + "_base";
      base.save(prefix + "_dec.ply");
      save(prefix + ".drc", bmtdu.getData().vector());
    }
    qpositions.resize(base.pointCount());
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      qpositions[v] = base.point(v).round();
    }

    const auto scaleTexCoord =
      std::pow(2.0, bmsps.getQpTexCoordMinus1() + 1) - 1.0;
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
  } else if (frameInfo.type == P_BASEMESH) {
    // frameInfo.referenceFrameIndex =
    //   frameInfo.frameIndex - 1 - int32_t(bmth.getReferenceFrameIndex());
    const auto& refFrame = gof.frame(frameInfo.referenceFrameIndex);
    base                 = refFrame.base;
    decompressMotion(syntax,
                     bmtl,
                     refFrame.baseClean.triangles(),
                     refFrame.baseClean.points(),
                     refFrame.baseIntegrateIndices,
                     base.triangles(),
                     refFrame.qpositions,
                     qpositions,
                     params);
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      base.point(v) = qpositions[v];
    }
    // Duplicated Vertex Reduction
    removeDuplicatedVertices(frame);
  } else {
    const auto& refFrame = gof.frame(frameInfo.referenceFrameIndex);
    base                 = refFrame.base;
    qpositions           = refFrame.qpositions;
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      base.point(v) = qpositions[v];
    }
    // Duplicated Vertex Reduction
    frame.baseClean            = refFrame.baseClean;
    frame.baseIntegrateIndices = refFrame.baseIntegrateIndices;
  }


  auto& atlas = syntax.getAtlas();
  auto& asps  = atlas.getAtlasSequenceParameterSet(0);
  auto& ext   = asps.getAspsVdmcExtension();
  printf("Scale position: Frame = %d \n", frameInfo.frameIndex);
  fflush(stdout);
  const auto bitDepthPosition = asps.getGeometry3dBitdepthMinus1() + 1;
  printf("BitDepthPosition = %u \n", bitDepthPosition);
  const auto scalePosition = ((1 << (bmsps.getQpPositionMinus1() + 1)) - 1.0)
                             / ((1 << (bitDepthPosition)) - 1.0);
  const auto iscalePosition = 1.0 / scalePosition;
  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.point(v) *= iscalePosition;
  }

  // Note JR: not referred to base mesh but to subdivision, should be moved outside of this function
  subdivideBaseMesh(frame,
                    rec,
                    ext.getSubdivisionMethod(),
                    ext.getSubdivisionIterationCount(),
                    ext.getInterpolateDisplacementNormals(),
                    ext.getAddReconstructedNormals());

  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressDisplacementsVideo(const V3cBitstream&         syntax,
                                         FrameSequence<uint16_t>&    dispVideo,
                                         const VMCDecoderParameters& params) {
  printf("Decompress displacements video \n");
  fflush(stdout);
  const auto& atlas        = syntax.getAtlas();
  auto&       displacement = atlas.getDisplacement();

  // Decode video
  auto& vps = syntax.getVps();
  auto& gi  = vps.getGeometryInformation(0);
  auto  decoder =
    VideoDecoder<uint16_t>::create(VideoCodecId(gi.getGeometryCodecId()));
  decoder->decode(displacement, dispVideo, 10);

  // Save intermediate files
  if (params.keepIntermediateFiles) {
    auto path = dispVideo.createName(_keepFilesPathPrefix + "disp_dec", 10);
    save(removeExtension(path) + ".bin", displacement);
    dispVideo.save(path);
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressTextureVideo(const V3cBitstream&         syntax,
                                   Sequence&                   reconsctruct,
                                   const VMCDecoderParameters& params) {
  printf("Decompress texture video\n");
  fflush(stdout);
  auto& vps = syntax.getVps();
  if (!vps.getAttributeVideoPresentFlag(0)) {
    for (auto& texture : reconsctruct.textures()) {
      texture.clear();
      texture.resize(1, 1, ColourSpace::BGR444p);
      texture.zero();
    }
  } else {
    auto& altas     = syntax.getAtlas();
    auto& attribute = altas.getAttribute();

    // Decode video
    printf("Decode video \n");
    fflush(stdout);
    FrameSequence<uint16_t> dec;
    auto&                   ai = vps.getAttributeInformation(0);
    auto                    decoder =
      VideoDecoder<uint16_t>::create(VideoCodecId(ai.getAttributeCodecId(0)));
    decoder->decode(attribute, dec, 10);

    if (dec.colourSpace() != ColourSpace::BGR444p
        && dec.colourSpace() != ColourSpace::GBR444p
        && dec.colourSpace() != ColourSpace::RGB444p) {
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
    }

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      FrameSequence<uint8_t> dec8(dec);
      auto path = dec8.createName(_keepFilesPathPrefix + "tex_dec", 8);
      dec8.save(path);
      save(removeExtension(path) + ".bin", attribute);
    }

    // Store result in 8 bits frames
    reconsctruct.textures() = dec;
    dec.clear();
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompress(const V3cBitstream&         syntax,
                       VMCGroupOfFramesInfo&       gofInfoSrc,
                       Sequence&                   reconstruct,
                       const VMCDecoderParameters& params) {
  VMCGroupOfFrames     gof;
  VMCGroupOfFramesInfo gofInfo;
  gofInfo.index_           = gofInfoSrc.index_;
  gofInfo.startFrameIndex_ = gofInfoSrc.startFrameIndex_;
  printf("Decompress gop: \n");
  fflush(stdout);
  auto&                   vps        = syntax.getVps();
  const auto&             atlas      = syntax.getAtlas();
  const auto&             baseMesh   = syntax.getBaseMesh();
  const auto&             asps       = atlas.getAtlasSequenceParameterSet(0);
  const auto&             afps       = atlas.getAtlasFrameParameterSet(0);
  auto&                   ext        = asps.getAspsVdmcExtension();
  auto&                   gi         = vps.getGeometryInformation(0);
  const auto              frameCount = atlas.getFrameCount(asps, afps);
  FrameSequence<uint16_t> dispVideo;
  gofInfo.frameCount_ = frameCount;
  gofInfo.framesInfo_.resize(frameCount);
  gof.resize(frameCount);
  reconstruct.resize(frameCount);
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    auto& bmtl           = baseMesh.getBaseMeshTileLayer(frameIndex);
    auto& bmth           = bmtl.getHeader();
    auto& bmtdu          = bmtl.getDataUnit();
    auto& refList        = bmth.getRefListStruct();
    auto& frameInfo      = gofInfo.framesInfo_[frameIndex];
    auto& frame          = gof.frame(frameIndex);
    auto& rec            = reconstruct.mesh(frameIndex);
    frameInfo.frameIndex = frameIndex;
    frameInfo.type       = bmth.getBaseMeshType();
    if (bmth.getBaseMeshType() == P_BASEMESH) {
      if (refList.getStrafEntrySignFlag(0))
        frameInfo.referenceFrameIndex =
          frameInfo.frameIndex + refList.getAbsDeltaAfocSt(0);
      else
        frameInfo.referenceFrameIndex =
          frameInfo.frameIndex - refList.getAbsDeltaAfocSt(0);
    } else if (bmth.getBaseMeshType() == SKIP_BASEMESH) {
      frameInfo.referenceFrameIndex = frameInfo.frameIndex - 1; 
    }
    printf("Frame %4d: type = %s ReferenceFrame = %4d \n",
           frameInfo.frameIndex,
           toString(frameInfo.type).c_str(),
           frameInfo.referenceFrameIndex);
    decompressBaseMesh(syntax, bmtl, gof, frameInfo, frame, rec, params);
  }
  if (vps.getGeometryVideoPresentFlag(0)
      && !decompressDisplacementsVideo(syntax, dispVideo, params)) {
    return false;
  }
  if (vps.getGeometryVideoPresentFlag(0)) {
    for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
      auto& frame = gof.frame(frameIndex);
      auto& rec   = reconstruct.mesh(frameIndex);
      reconstructDisplacementFromVideoFrame(
        dispVideo.frame(frameIndex),
        frame,
        rec,
        1 << asps.getLog2PatchPackingBlockSize(),
        gi.getGeometry2dBitdepthMinus1() + 1,
        ext.getDisplacement1D(),
        ext.getDisplacementReversePacking());

      const auto bitDepthPosition = asps.getGeometry3dBitdepthMinus1() + 1;
      if (ext.getLodDisplacementQuantizationFlag()) {
        inverseQuantizeDisplacements(
          frame,
          bitDepthPosition,
          ext.getLiftingQuantizationParametersPerLevelOfDetails());
      } else {
        inverseQuantizeDisplacements(frame,
                                     bitDepthPosition,
                                     ext.getLiftingLevelOfDetailInverseScale(),
                                     ext.getLiftingQPs());
      }
      computeInverseLinearLifting(frame.disp,
                                  frame.subdivInfoLevelOfDetails,
                                  frame.subdivEdges,
                                  ext.getLiftingPredictionWeight(),
                                  ext.getLiftingUpdateWeight(),
                                  ext.getLiftingSkipUpdate());
      applyDisplacements(frame, bitDepthPosition, rec, ext.getDisplacementCoordinateSystem());
    }
  }

  // decompress texture
  if (!decompressTextureVideo(syntax, reconstruct, params)) { return false; }
  printf("Done \n");
  fflush(stdout);
  gofInfoSrc = gofInfo;

  // Quantize UV coordinate
  if (!params.dequantizeUV) {
    const auto bitDepthTexCoord = asps.getGeometry2dBitdepthMinus1() + 1;
    const auto scale            = (1 << bitDepthTexCoord) - 1.0;
    for (auto& rec : reconstruct.meshes()) {
      const auto texCoordCount = rec.texCoordCount();
      for (int32_t i = 0; i < texCoordCount; ++i) { rec.texCoord(i) *= scale; }
    }
  }
  // Reconstruct normals
  if (!params.reconstructNormals) {
    for (auto& rec : reconstruct.meshes()) { rec.resizeNormals(0); }
  }
  return true;
}

//============================================================================

}  // namespace vmesh
