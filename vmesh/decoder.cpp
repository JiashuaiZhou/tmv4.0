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

#include "vmesh/decoder.hpp"

#include <cstdio>
#include <sstream>

#include "vmesh/bitstream.hpp"
#include "vmesh/contexts.hpp"
#include "vmesh/entropy.hpp"

namespace vmesh {

//============================================================================

int32_t
applyDisplacements(
  VMCFrame& frame,
  const DisplacementCoordinateSystem& displacementCoordinateSystem)
{
  const auto& disp = frame.disp;
  auto& rec = frame.rec;
  for (int32_t v = 0, vcount = rec.pointCount(); v < vcount; ++v) {
    const auto& d = disp[v];
    if (displacementCoordinateSystem == DisplacementCoordinateSystem::LOCAL) {
      const auto n = rec.normal(v);
      Vec3<double> t, b;
      computeLocalCoordinatesSystem(n, t, b);
      rec.point(v) += d[0] * n + d[1] * t + d[2] * b;
    } else {
      rec.point(v) += d;
    }
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
subdivideBaseMesh(
  VMCFrame& frame,
  const SubdivisionMethod subdivisionMethod,
  const int32_t subdivisionIterationCount)
{
  auto& infoLevelOfDetails = frame.subdivInfoLevelOfDetails;
  auto& subdivEdges = frame.subdivEdges;
  auto& rec = frame.rec;
  rec = frame.base;
  rec.computeNormals();
  if (subdivisionMethod == SubdivisionMethod::MID_POINT) {
    rec.subdivideMidPoint(
      subdivisionIterationCount, &infoLevelOfDetails, &subdivEdges);
  } else {
    return -1;
  }
  rec.resizeNormals(rec.pointCount());
  interpolateSubdivision(
    rec.normals(), infoLevelOfDetails, subdivEdges, 0.5, 0.5, true);
  return 0;
}

//----------------------------------------------------------------------------

int32_t
reconstructDisplacementFromVideoFrame(
  const Frame<uint16_t, ColourSpace::YUV444p>& dispVideoFrame,
  VMCFrame& frame,
  const int32_t geometryVideoBlockSize,
  const int32_t geometryVideoBitDepth)
{
  const auto geometryVideoWidthInBlocks =
    dispVideoFrame.width() / geometryVideoBlockSize;
  const auto pixelsPerBlock = geometryVideoBlockSize * geometryVideoBlockSize;
  const auto shift = uint16_t((1 << geometryVideoBitDepth) >> 1);
  const auto& Y = dispVideoFrame.plane(0);
  const auto& U = dispVideoFrame.plane(1);
  const auto& V = dispVideoFrame.plane(2);
  const auto pointCount = frame.rec.pointCount();
  auto& disp = frame.disp;
  disp.resize(pointCount);
  for (int32_t v = 0, vcounter = 0; v < pointCount; ++v) {
    // to do: optimize power of 2
    const auto blockIndex = vcounter / pixelsPerBlock;
    const auto indexWithinBlock = vcounter % pixelsPerBlock;
    const auto x0 =
      (blockIndex % geometryVideoWidthInBlocks) * geometryVideoBlockSize;
    const auto y0 =
      (blockIndex / geometryVideoWidthInBlocks) * geometryVideoBlockSize;
    int32_t x, y;
    computeMorton2D(indexWithinBlock, x, y);
    assert(x < geometryVideoBlockSize);
    assert(y < geometryVideoBlockSize);
    const auto x1 = x0 + x;
    const auto y1 = y0 + y;
    auto& d = disp[v];
    d[0] = double(Y.get(y1, x1)) - shift;
    d[1] = double(U.get(y1, x1)) - shift;
    d[2] = double(V.get(y1, x1)) - shift;
    ++vcounter;
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
inverseQuantizeDisplacements(
  VMCFrame& frame,
  const int32_t bitDepthPosition,
  const double (&liftingLevelOfDetailInverseScale)[3],
  const int32_t (&liftingQuantizationParameters)[3])
{
  const auto& infoLevelOfDetails = frame.subdivInfoLevelOfDetails;
  const auto lodCount = int32_t(infoLevelOfDetails.size());
  assert(lodCount > 0);
  double iscale[3], ilodScale[3];
  for (int32_t k = 0; k < 3; ++k) {
    const auto qp = liftingQuantizationParameters[k];
    iscale[k] =
      qp >= 0 ? pow(0.5, 16 - bitDepthPosition + (4 - qp) / 6.0) : 0.0;
    ilodScale[k] = liftingLevelOfDetailInverseScale[k];
  }
  auto& disp = frame.disp;
  for (int32_t it = 0, vcount0 = 0; it < lodCount; ++it) {
    const auto vcount1 = infoLevelOfDetails[it].pointCount;
    for (int32_t v = vcount0; v < vcount1; ++v) {
      auto& d = disp[v];
      for (int32_t k = 0; k < 3; ++k) {
        d[k] *= iscale[k];
      }
    }
    vcount0 = vcount1;
    for (int32_t k = 0; k < 3; ++k) {
      iscale[k] *= ilodScale[k];
    }
  }
  return 0;
}

//============================================================================

int32_t
VMCDecoder::decompressMotion(
  const Bitstream& bitstream,
  const std::vector<Vec3<int32_t>>& triangles,
  const std::vector<Vec3<int32_t>>& reference,
  std::vector<Vec3<int32_t>>& current,
  const VMCDecoderParameters& /*params*/)
{
  uint32_t byteCount = 0;
  bitstream.read(byteCount, _byteCounter);
  std::cout << "Motion byte count = " << byteCount << '\n';
  VMCMotionACContext ctx;
  EntropyDecoder arithmeticDecoder;
  const auto bufferPtr =
    reinterpret_cast<const char*>(bitstream.buffer.data() + _byteCounter);
  _byteCounter += byteCount;
  arithmeticDecoder.setBuffer(byteCount, bufferPtr);
  arithmeticDecoder.start();
  const auto pointCount = int32_t(reference.size());
  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(triangles, pointCount, vertexToTriangle);
  std::vector<int8_t> available(pointCount, 0);
  std::vector<int8_t> vtags(pointCount);
  std::vector<int32_t> vadj, tadj;
  std::vector<Vec3<int32_t>> motion(pointCount);
  current.resize(pointCount);
  for (int vindex0 = 0; vindex0 < pointCount; ++vindex0) {
    const auto predIndex = arithmeticDecoder.decode(ctx.ctxPred);
    Vec3<int32_t> res;
    for (int32_t k = 0; k < 3; ++k) {
      int32_t value = 0;
      if (arithmeticDecoder.decode(ctx.ctxCoeffGtN[0][k])) {
        const auto sign = arithmeticDecoder.decode(ctx.ctxSign[k]);
        ++value;
        if (arithmeticDecoder.decode(ctx.ctxCoeffGtN[1][k])) {
          value += 1
            + arithmeticDecoder.decodeExpGolomb(
              0, ctx.ctxCoeffRemPrefix[k], ctx.ctxCoeffRemSuffix[k]);
        }
        if (sign) {
          value = -value;
        }
      }
      res[k] = value;
    }
    if (predIndex == 0) {
      motion[vindex0] = res;
    } else {
      ComputeAdjacentVertices(
        vindex0, triangles, vertexToTriangle, vtags, vadj);
      Vec3<int32_t> pred(0);
      int32_t predCount = 0;
      for (int i = 0, count = int(vadj.size()); i < count; ++i) {
        const auto vindex1 = vadj[i];
        if (available[vindex1]) {
          const auto& mv1 = motion[vindex1];
          for (int32_t k = 0; k < 3; ++k) {
            pred[k] += mv1[k];
          }
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
VMCDecoder::decompressBaseMesh(
  const Bitstream& bitstream,
  const VMCGroupOfFrames& gof,
  VMCFrameInfo& frameInfo,
  VMCFrame& frame,
  VMCStats& stats,
  const VMCDecoderParameters& params)
{
  if (decodeFrameHeader(bitstream, frameInfo)) {
    return -1;
  }
  const auto frameIndex = frameInfo.frameIndex + _gofInfo.startFrameIndex;
  auto& base = frame.base;
  auto& qpositions = frame.qpositions;
  if (frameInfo.type == FrameType::INTRA) {
    std::stringstream cbaseFileName, dbaseFileName;
    cbaseFileName << params.intermediateFilesPathPrefix << "gof_"
                  << _gofInfo.index << "_fr_" << frameIndex
                  << "_cbase_dec.drc";
    dbaseFileName << params.intermediateFilesPathPrefix << "gof_"
                  << _gofInfo.index << "_fr_" << frameIndex << "_dbase.obj";
    auto bitstreamByteCount0 = _byteCounter;
    uint32_t byteCountBaseMesh = 0;
    bitstream.read(byteCountBaseMesh, _byteCounter);
    std::ofstream file(cbaseFileName.str(), std::ios::binary);
    if (!file.is_open()) {
      return -1;
    }
    file.write(
      reinterpret_cast<const char*>(bitstream.buffer.data() + _byteCounter),
      byteCountBaseMesh);
    file.close();
    _byteCounter += byteCountBaseMesh;
    stats.baseMeshByteCount += _byteCounter - bitstreamByteCount0;

    std::stringstream cmdDec;
    cmdDec << params.geometryMeshDecoderPath << " -i " << cbaseFileName.str()
           << " -o " << dbaseFileName.str();
    std::cout << cmdDec.str() << std::endl;
    system(cmdDec.str().c_str());
    base.loadFromOBJ(dbaseFileName.str());

    if (!params.keepIntermediateFiles) {
      std::remove(cbaseFileName.str().c_str());
      std::remove(dbaseFileName.str().c_str());
    }

    qpositions.resize(base.pointCount());
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      auto& qpos = qpositions[v];
      const auto& point = base.point(v);
      for (int32_t k = 0; k < 3; ++k) {
        qpos[k] = int32_t(std::round(point[k]));
      }
    }
    const auto scaleTexCoord = std::pow(2.0, _sps.qpTexCoord) - 1.0;
    const auto iscaleTexCoord = 1.0 / scaleTexCoord;
    for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
      base.setTexCoord(tc, base.texCoord(tc) * iscaleTexCoord);
    }
  } else {
    const auto& refFrame = gof.frame(frameInfo.referenceFrameIndex);
    base = refFrame.base;
    auto bitstreamByteCount0 = _byteCounter;
    decompressMotion(
      bitstream, base.triangles(), refFrame.qpositions, qpositions, params);
    stats.motionByteCount += _byteCounter - bitstreamByteCount0;
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      const auto& qpos = qpositions[v];
      auto& point = base.point(v);
      for (int32_t k = 0; k < 3; ++k) {
        point[k] = qpos[k];
      }
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
VMCDecoder::decompressDisplacementsVideo(
  const Bitstream& bitstream, const VMCDecoderParameters& params)
{
  std::stringstream fnameCompressDisp, fnameDispDec;
  const auto width = _dispVideo.width();
  const auto height = _dispVideo.height();
  fnameCompressDisp << params.intermediateFilesPathPrefix << "GOF_"
                    << _gofInfo.index << "_disp_dec.h265";
  fnameDispDec << params.intermediateFilesPathPrefix << "GOF_"
               << _gofInfo.index << "_disp_" << width << "x" << height
               << "_dec.yuv";
  uint32_t byteCountDispVideo = 0;
  bitstream.read(byteCountDispVideo, _byteCounter);
  std::ofstream file(fnameCompressDisp.str(), std::ios::binary);
  if (!file.is_open()) {
    return -1;
  }
  file.write(
    reinterpret_cast<const char*>(bitstream.buffer.data() + _byteCounter),
    byteCountDispVideo);
  file.close();
  _byteCounter += byteCountDispVideo;
  std::stringstream cmd;
  cmd << params.geometryVideoDecoderPath << ' ';
  cmd << "--BitstreamFile=" << fnameCompressDisp.str() << ' ';
  cmd << "--ReconFile=" << fnameDispDec.str() << ' ';
  std::cout << cmd.str() << std::endl;
  system(cmd.str().c_str());
  _dispVideo.load(fnameDispDec.str());

  if (!params.keepIntermediateFiles) {
    std::remove(fnameCompressDisp.str().c_str());
    std::remove(fnameDispDec.str().c_str());
  }

  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decompressTextureVideo(
  const Bitstream& bitstream,
  VMCGroupOfFrames& gof,
  const VMCDecoderParameters& params)
{
  const auto width = _sps.widthTexVideo;
  const auto height = _sps.heightTexVideo;
  const auto frameCount = _sps.frameCount;
  std::stringstream fnameCompressTexture;
  fnameCompressTexture << params.intermediateFilesPathPrefix << "GOF_"
                       << _gofInfo.index << "_texture_dec.h265";
  std::stringstream fnameTextureYUV420Dec;
  fnameTextureYUV420Dec << params.intermediateFilesPathPrefix << "GOF_"
                        << _gofInfo.index << "_tex_" << width << "x" << height
                        << "_420_" << _sps.textureVideoBitDepth
                        << "bit_dec.yuv";
  uint32_t byteCountTexVideo = 0;
  bitstream.read(byteCountTexVideo, _byteCounter);
  std::ofstream file(fnameCompressTexture.str(), std::ios::binary);
  if (!file.is_open()) {
    return -1;
  }
  file.write(
    reinterpret_cast<const char*>(bitstream.buffer.data() + _byteCounter),
    byteCountTexVideo);
  file.close();
  _byteCounter += byteCountTexVideo;
  std::stringstream cmd;
  cmd << params.textureVideoDecoderPath << ' ';
  cmd << "--BitstreamFile=" << fnameCompressTexture.str() << ' ';
  cmd << "--ReconFile=" << fnameTextureYUV420Dec.str() << ' ';
  std::cout << cmd.str() << std::endl;
  system(cmd.str().c_str());

  std::stringstream fnameTextureBGR444Dec;
  fnameTextureBGR444Dec << params.intermediateFilesPathPrefix << "GOF_"
                        << _gofInfo.index << "_tex_" << width << "x" << height
                        << "_444_rec.bgrp";
  cmd.str("");
  cmd << params.textureVideoHDRToolPath
      << " -f " << params.textureVideoHDRToolDecConfig
      << " -p SourceFile=\"" << fnameTextureYUV420Dec.str() << '"'
      << " -p SourceWidth=" << width
      << " -p SourceHeight=" << height
      << " -p NumberOfFrames=" << frameCount
      << " -p OutputFile=\"" << fnameTextureBGR444Dec.str() << '"'
      << " -p SourceBitDepthCmp0=" << _sps.textureVideoBitDepth
      << " -p SourceBitDepthCmp1=" << _sps.textureVideoBitDepth
      << " -p SourceBitDepthCmp2=" << _sps.textureVideoBitDepth << '\n';
  std::cout << cmd.str() << std::endl;
  system(cmd.str().c_str());

  std::ifstream fileTextureVideoDec(fnameTextureBGR444Dec.str());
  if (!fileTextureVideoDec.is_open()) {
    return -1;
  }
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    auto& outTexture = gof.frame(frameIndex).outputTexture;
    outTexture.resize(width, height);
    outTexture.load(fileTextureVideoDec);
  }
  fileTextureVideoDec.close();

  if (!params.keepIntermediateFiles) {
    std::remove(fnameCompressTexture.str().c_str());
    std::remove(fnameTextureYUV420Dec.str().c_str());
    std::remove(fnameTextureBGR444Dec.str().c_str());
  }

  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decompress(
  const Bitstream& bitstream,
  VMCGroupOfFramesInfo& gofInfo,
  VMCGroupOfFrames& gof,
  size_t& byteCounter,
  const VMCDecoderParameters& params)
{
  _byteCounter = byteCounter;
  _gofInfo.index = gofInfo.index;
  _gofInfo.startFrameIndex = gofInfo.startFrameIndex;
  auto& stats = gof.stats;
  stats.reset();
  stats.totalByteCount = _byteCounter;
  if (decodeSequenceHeader(bitstream)) {
    return -1;
  }
  _dispVideo.resize(
    _sps.widthDispVideo, _sps.heightDispVideo, _sps.frameCount);
  _gofInfo.frameCount = _sps.frameCount;
  _gofInfo.framesInfo.resize(_sps.frameCount);
  gof.resize(_sps.frameCount);
  stats.frameCount = _sps.frameCount;
  for (int32_t frameIndex = 0; frameIndex < _sps.frameCount; ++frameIndex) {
    auto& frameInfo = _gofInfo.framesInfo[frameIndex];
    auto& frame = gof.frame(frameIndex);
    frameInfo.frameIndex = frameIndex;
    decompressBaseMesh(bitstream, gof, frameInfo, frame, stats, params);
    stats.baseMeshVertexCount += frame.base.pointCount();
    stats.vertexCount += frame.rec.pointCount();
    stats.faceCount += frame.rec.triangleCount();
  }
  stats.displacementsByteCount = _byteCounter;
  if (
    _sps.encodeDisplacementsVideo
    && decompressDisplacementsVideo(bitstream, params)) {
    return -1;
  }
  stats.displacementsByteCount = _byteCounter - stats.displacementsByteCount;

  if (_sps.encodeDisplacementsVideo) {
    for (int32_t frameIndex = 0; frameIndex < _sps.frameCount; ++frameIndex) {
      auto& frame = gof.frame(frameIndex);
      reconstructDisplacementFromVideoFrame(
        _dispVideo.frame(frameIndex), frame, _sps.geometryVideoBlockSize,
        _sps.geometryVideoBitDepth);
      inverseQuantizeDisplacements(
        frame, _sps.bitDepthPosition, _sps.liftingLevelOfDetailInverseScale,
        _sps.liftingQuantizationParameters);
      computeInverseLinearLifting(
        frame.disp, frame.subdivInfoLevelOfDetails, frame.subdivEdges,
        _sps.liftingPredictionWeight, _sps.liftingUpdateWeight,
        _sps.liftingSkipUpdate);
      applyDisplacements(frame, _sps.displacementCoordinateSystem);
    }
  }
  stats.textureByteCount = _byteCounter;
  // decompress texture
  if (
    _sps.encodeTextureVideo
    && decompressTextureVideo(bitstream, gof, params)) {
    return -1;
  }
  stats.textureByteCount = _byteCounter - stats.textureByteCount;
  stats.totalByteCount = _byteCounter - stats.totalByteCount;
  byteCounter = _byteCounter;
  gofInfo = _gofInfo;
  if (!params.normalizeUV) {
    const auto scale = (1 << _sps.bitDepthTexCoord) - 1.0;
    for (int32_t frameIndex = 0; frameIndex < _sps.frameCount; ++frameIndex) {
      auto& rec = gof.frame(frameIndex).rec;
      const auto texCoordCount = rec.texCoordCount();
      for (int32_t i = 0; i < texCoordCount; ++i) {
        rec.setTexCoord(i, rec.texCoord(i) * scale);
      }
    }
  }
  return 0;
}

//============================================================================

}  // namespace vmesh