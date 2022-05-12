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

#include "vmesh/encoder.hpp"

#include <array>
#include <cstdio>
#include <sstream>

#include "vmesh/bitstream.hpp"
#include "vmesh/contexts.hpp"
#include "vmesh/entropy.hpp"
#include "vmesh/util/kdtree.hpp"
#include "vmesh/vmc.hpp"

namespace vmesh {

//============================================================================

int32_t inverseQuantizeDisplacements(
  VMCFrame& frame,
  const int32_t bitDepthPosition,
  const double (&liftingLevelOfDetailInverseScale)[3],
  const int32_t (&liftingQuantizationParameters)[3]);

//============================================================================

static Vec3<double>
computeNearestPointColour(
  const Vec3<double>& point0,
  const Frame<uint8_t, ColourSpace::BGR444p>& targetTexture,
  const TriangleMesh<double>& targetMesh,
  const KdTree& kdtree,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangleTarget,
  double& minDist2)
{
  const auto neighboursTarget = vertexToTriangleTarget.neighbours();
  const auto nnCount = 1;
  int32_t index;
  double sqrDist;
  nanoflann::KNNResultSet<double, int32_t> resultSet(nnCount);
  resultSet.init(&index, &sqrDist);
  kdtree.query(point0.data(), resultSet);
  auto ruv = targetMesh.texCoord(index);
  minDist2 = std::numeric_limits<double>::max();
  const auto start = vertexToTriangleTarget.neighboursStartIndex(index);
  const auto end = vertexToTriangleTarget.neighboursEndIndex(index);
  for (int j = start; j < end; ++j) {
    const auto tindex = neighboursTarget[j];
    assert(tindex < targetMesh.triangleCount());
    const auto& tri = targetMesh.triangle(tindex);
    const auto& pt0 = targetMesh.point(tri[0]);
    const auto& pt1 = targetMesh.point(tri[1]);
    const auto& pt2 = targetMesh.point(tri[2]);
    Vec3<double> bcoord;
    const auto cpoint = ClosestPointInTriangle(point0, pt0, pt1, pt2, &bcoord);
    //    assert(bcoord[0] >= 0.0 && bcoord[1] >= 0.0 && bcoord[2] >= 0.0);
    assert(fabs(1.0 - bcoord[0] - bcoord[1] - bcoord[2]) < 0.000001);
    const auto d2 = (cpoint - point0).norm2();
    if (d2 < minDist2) {
      minDist2 = d2;
      const auto& triUV = targetMesh.texCoordTriangle(tindex);
      const auto& uv0 = targetMesh.texCoord(triUV[0]);
      const auto& uv1 = targetMesh.texCoord(triUV[1]);
      const auto& uv2 = targetMesh.texCoord(triUV[2]);
      ruv = bcoord[0] * uv0 + bcoord[1] * uv1 + bcoord[2] * uv2;
    }
  }
  return targetTexture.bilinear(ruv[1], ruv[0]);
}

//============================================================================

int32_t
VMCEncoder::compressDisplacementsVideo(
  Bitstream& bitstream, const VMCEncoderParameters& params)
{
  std::stringstream fnameDisp, fnameDispRec;
  const auto width = _dispVideo.width();
  const auto height = _dispVideo.height();
  const auto frameCount = _dispVideo.frameCount();
  fnameDisp << "GOF_" << _gofInfo.index << "_disp_" << width << 'x' << height
            << ".yuv";
  fnameDispRec << "GOF_" << _gofInfo.index << "_disp_" << width << 'x'
               << height << "_rec.yuv";
  _dispVideo.save(fnameDisp.str());
  std::stringstream fnameCompressDisp;
  fnameCompressDisp << "GOF_" << _gofInfo.index << "_disp.h265";
  std::stringstream cmd;
  cmd << params.geometryVideoEncoderPath << ' ';
  cmd << "-c " << params.geometryVideoEncoderConfig << ' ';
  cmd << "--InputFile=" << fnameDisp.str() << ' ';
  cmd << "--InputBitDepth=" << params.geometryVideoBitDepth << ' ';
  cmd << "--InputChromaFormat=444 ";
  cmd << "--OutputBitDepth=" << params.geometryVideoBitDepth << ' ';
  cmd << "--OutputBitDepthC=" << params.geometryVideoBitDepth << ' ';
  cmd << "--FrameRate=" << 30 << ' ';
  cmd << "--FrameSkip=" << 0 << ' ';
  cmd << "--SourceWidth=" << width << ' ';
  cmd << "--SourceHeight=" << height << ' ';
  cmd << "--FramesToBeEncoded=" << frameCount << ' ';
  cmd << "--BitstreamFile=" << fnameCompressDisp.str() << ' ';
  cmd << "--ReconFile=" << fnameDispRec.str() << ' ';
  cmd << "--InternalBitDepth=" << params.geometryVideoBitDepth << ' ';
  cmd << "--InternalBitDepthC=" << params.geometryVideoBitDepth << ' ';
  std::cout << cmd.str() << std::endl;
  system(cmd.str().c_str());
  bitstream.append(fnameCompressDisp.str(), true);
  _dispVideo.load(fnameDispRec.str());

  if (params.keepIntermediateFiles) {
    std::remove(fnameDisp.str().c_str());
    std::remove(fnameCompressDisp.str().c_str());
    std::remove(fnameDispRec.str().c_str());
  }

  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCEncoder::compressTextureVideo(
  VMCGroupOfFrames& gof,
  Bitstream& bitstream,
  const VMCEncoderParameters& params)
{
  const auto width = params.textureWidth;
  const auto height = params.textureHeight;
  const auto frameCount = gof.frameCount();
  std::stringstream fnameTextureBGR444, fnameTextureBGR444Rec;
  fnameTextureBGR444 << "GOF_" << _gofInfo.index << "_tex_" << width << 'x'
                     << height << "_444.bgrp";
  fnameTextureBGR444Rec << "GOF_" << _gofInfo.index << "_tex_" << width << 'x'
                        << height << "_444_rec.bgrp";

  std::ofstream fileTextureVideo(fnameTextureBGR444.str());
  if (!fileTextureVideo.is_open()) {
    return -1;
  }
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    const auto& outTexture = gof.frame(frameIndex).outputTexture;
    assert(outTexture.width() == width);
    assert(outTexture.height() == height);
    outTexture.save(fileTextureVideo);
  }
  fileTextureVideo.close();

  std::stringstream fnameTextureYUV420, fnameTextureYUV420Rec;
  fnameTextureYUV420 << "GOF_" << _gofInfo.index << "_tex_" << width << 'x'
                     << height << "_420_" << params.textureVideoBitDepth
                     << "bit.yuv";
  fnameTextureYUV420Rec << "GOF_" << _gofInfo.index << "_tex_" << width << 'x'
                        << height << "_420_" << params.textureVideoBitDepth
                        << "bit_rec.yuv";

  std::stringstream cmd;
  cmd << params.textureVideoHDRToolPath
      << " -f " << params.textureVideoHDRToolEncConfig
      << " -p SourceFile=\"" << fnameTextureBGR444.str() << '"'
      << " -p SourceWidth=" << width
      << " -p SourceHeight=" << height
      << " -p NumberOfFrames=" << frameCount
      << " -p OutputFile=\"" << fnameTextureYUV420.str() << '"'
      << " -p OutputBitDepthCmp0=" << params.textureVideoBitDepth
      << " -p OutputBitDepthCmp1=" << params.textureVideoBitDepth
      << " -p OutputBitDepthCmp2=" << params.textureVideoBitDepth
      << " -p LogFile=\"\"" << '\n';
  std::cout << cmd.str() << std::endl;
  system(cmd.str().c_str());

  std::stringstream fnameCompressTexture;
  fnameCompressTexture << "GOF_" << _gofInfo.index << "_texture.h265";

  cmd.str("");
  cmd << params.textureVideoEncoderPath << ' ';
  cmd << "-c " << params.textureVideoEncoderConfig << ' ';
  cmd << "--InputFile=" << fnameTextureYUV420.str() << ' ';
  cmd << "--InputBitDepth=" << params.textureVideoBitDepth << ' ';
  cmd << "--InputChromaFormat=420 ";
  cmd << "--OutputBitDepth=" << params.textureVideoBitDepth << ' ';
  cmd << "--OutputBitDepthC=" << params.textureVideoBitDepth << ' ';
  cmd << "--FrameRate=" << 30 << ' ';
  cmd << "--FrameSkip=" << 0 << ' ';
  cmd << "--SourceWidth=" << width << ' ';
  cmd << "--SourceHeight=" << height << ' ';
  cmd << "--FramesToBeEncoded=" << frameCount << ' ';
  cmd << "--BitstreamFile=" << fnameCompressTexture.str() << ' ';
  cmd << "--ReconFile=" << fnameTextureYUV420Rec.str() << ' ';
  cmd << "--InternalBitDepth=10 ";
  cmd << "--InternalBitDepthC=10 ";
  cmd << "--QP=" << params.textureVideoQP;

  std::cout << cmd.str() << std::endl;
  system(cmd.str().c_str());
  bitstream.append(fnameCompressTexture.str(), true);

  cmd.str("");
  cmd << params.textureVideoHDRToolPath
      << " -f " << params.textureVideoHDRToolDecConfig
      << " -p SourceFile=\"" << fnameTextureYUV420Rec.str() << '"'
      << " -p SourceWidth=" << width
      << " -p SourceHeight=" << height
      << " -p NumberOfFrames=" << frameCount
      << " -p OutputFile=\"" << fnameTextureBGR444Rec.str() << '"'
      << " -p SourceBitDepthCmp0=" << params.textureVideoBitDepth
      << " -p SourceBitDepthCmp1=" << params.textureVideoBitDepth
      << " -p SourceBitDepthCmp2=" << params.textureVideoBitDepth
      << " -p LogFile=\"\"" << '\n';
  std::cout << cmd.str() << std::endl;
  system(cmd.str().c_str());

  std::ifstream fileTextureVideoRec(fnameTextureBGR444Rec.str());
  if (!fileTextureVideoRec.is_open()) {
    return -1;
  }
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    auto& outTexture = gof.frame(frameIndex).outputTexture;
    assert(outTexture.width() == width);
    assert(outTexture.height() == height);
    outTexture.load(fileTextureVideoRec);
  }
  fileTextureVideoRec.close();

  if (params.keepIntermediateFiles) {
    std::remove(fnameTextureBGR444.str().c_str());
    std::remove(fnameTextureBGR444Rec.str().c_str());
    std::remove(fnameTextureYUV420.str().c_str());
    std::remove(fnameTextureYUV420Rec.str().c_str());
    std::remove(fnameCompressTexture.str().c_str());
  }

  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCEncoder::computeDracoMapping(
  TriangleMesh<double> base,
  std::vector<int32_t>& mapping,
  const int32_t frameIndex,
  const VMCEncoderParameters& params) const
{
  const auto scalePosition = 1 << (18 - params.bitDepthPosition);
  const auto scaleTexCoord = 1 << 18;
  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.setPoint(v, Round(base.point(v) * scalePosition));
  }
  for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
    base.setTexCoord(tc, Round(base.texCoord(tc) * scaleTexCoord));
  }
  std::stringstream mappingFileName, cmappingFileName, rmappingFileName;
  mappingFileName << "gof_" << _gofInfo.index << "_fr_" << frameIndex
                  << "_mapping.obj";
  cmappingFileName << "gof_" << _gofInfo.index << "_fr_" << frameIndex
                   << "_cmapping.drc";
  rmappingFileName << "gof_" << _gofInfo.index << "_fr_" << frameIndex
                   << "_rmapping.obj";

  base.saveToOBJ<int64_t>(mappingFileName.str());
  auto fsubdiv0 = base;
  fsubdiv0.subdivideMidPoint(params.subdivisionIterationCount);
  std::stringstream cmdEnc;
  cmdEnc << params.geometryMeshEncoderPath << " -cl 10"
         << " -i " << mappingFileName.str() << " -o "
         << cmappingFileName.str();
  std::cout.flush();
  system(cmdEnc.str().c_str());

  std::stringstream cmdDec;
  cmdDec << params.geometryMeshDecoderPath << " -i " << cmappingFileName.str()
         << " -o " << rmappingFileName.str();
  std::cout.flush();
  system(cmdDec.str().c_str());
  base.loadFromOBJ(rmappingFileName.str());

  if (params.keepIntermediateFiles) {
    std::remove(mappingFileName.str().c_str());
    std::remove(cmappingFileName.str().c_str());
    std::remove(rmappingFileName.str().c_str());
  }

  std::vector<int32_t> texCoordToPoint0;
  fsubdiv0.computeTexCoordToPointMapping(texCoordToPoint0);
  struct ArrayHasher {
    std::size_t operator()(const std::array<double, 5>& a) const
    {
      std::size_t h = 0;
      for (auto e : a) {
        h ^= std::hash<double>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
      }
      return h;
    }
  };
  std::unordered_map<std::array<double, 5>, int32_t, ArrayHasher> map0;
  const auto texCoordCount0 = fsubdiv0.texCoordCount();
  for (int32_t v = 0; v < texCoordCount0; ++v) {
    const auto& point0 = fsubdiv0.point(std::max(0, texCoordToPoint0[v]));
    const auto& texCoord0 = fsubdiv0.texCoord(v);
    const std::array<double, 5> vertex0{
      point0[0], point0[1], point0[2], texCoord0[0], texCoord0[1]};
    map0[vertex0] = texCoordToPoint0[v];
  }

  auto fsubdiv1 = base;
  fsubdiv1.subdivideMidPoint(params.subdivisionIterationCount);
  const auto pointCount1 = fsubdiv1.pointCount();
  mapping.resize(pointCount1, -1);
  std::vector<bool> tags(pointCount1, false);
  for (int32_t t = 0, tcount = fsubdiv1.triangleCount(); t < tcount; ++t) {
    const auto& tri = fsubdiv1.triangle(t);
    const auto& triUV = fsubdiv1.texCoordTriangle(t);
    for (int32_t k = 0; k < 3; ++k) {
      const auto indexPos = tri[k];
      if (tags[indexPos]) {
        continue;
      }
      tags[indexPos] = true;
      const auto indexTexCoord = triUV[k];
      const auto& point1 = fsubdiv1.point(indexPos);
      const auto& texCoord1 = fsubdiv1.texCoord(indexTexCoord);
      const std::array<double, 5> vertex1 = {
        point1[0], point1[1], point1[2], texCoord1[0], texCoord1[1]};
      const auto it = map0.find(vertex1);
      if (it != map0.end()) {
        mapping[indexPos] = map0[vertex1];
      } else {
        assert(0);
      }
    }
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCEncoder::compressMotion(
  const std::vector<Vec3<int32_t>>& triangles,
  const std::vector<Vec3<int32_t>>& current,
  const std::vector<Vec3<int32_t>>& reference,
  Bitstream& bitstream,
  const VMCEncoderParameters& /*params*/) const
{
  const auto pointCount = int32_t(current.size());
  const auto maxAcBufLen = pointCount * 3 * 4 + 1024;
  VMCMotionACContext ctx;
  EntropyEncoder arithmeticEncoder;
  arithmeticEncoder.setBuffer(maxAcBufLen, nullptr);
  arithmeticEncoder.start();
  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(triangles, pointCount, vertexToTriangle);
  std::vector<int8_t> available(pointCount, 0);
  std::vector<int8_t> vtags(pointCount);
  std::vector<int32_t> vadj, tadj;
  std::vector<Vec3<int32_t>> motion(pointCount);
  for (int vindex = 0; vindex < pointCount; ++vindex) {
    motion[vindex] = current[vindex] - reference[vindex];
  }
  for (int vindex0 = 0; vindex0 < pointCount; ++vindex0) {
    ComputeAdjacentVertices(vindex0, triangles, vertexToTriangle, vtags, vadj);
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
    available[vindex0] = 1;
    const auto res0 = motion[vindex0];
    const auto res1 = motion[vindex0] - pred;
    const auto bits0 = ctx.estimateBits(res0, 0);
    const auto bits1 = ctx.estimateBits(res1, 1);
    Vec3<int32_t> res;
    if (bits0 <= bits1) {
      res = res0;
      arithmeticEncoder.encode(0, ctx.ctxPred);
    } else {
      res = res1;
      arithmeticEncoder.encode(1, ctx.ctxPred);
    }
    for (int32_t k = 0; k < 3; ++k) {
      auto value = res[k];
      arithmeticEncoder.encode(value != 0, ctx.ctxCoeffGtN[0][k]);
      if (!value) {
        continue;
      }
      arithmeticEncoder.encode(value < 0, ctx.ctxSign[k]);
      value = std::abs(value) - 1;
      arithmeticEncoder.encode(value != 0, ctx.ctxCoeffGtN[1][k]);
      if (!value) {
        continue;
      }
      assert(value > 0);
      arithmeticEncoder.encodeExpGolomb(
        --value, 0, ctx.ctxCoeffRemPrefix[k], ctx.ctxCoeffRemSuffix[k]);
    }
  }

  const auto length = arithmeticEncoder.stop();
  std::cout << "Motion byte count = " << length << '\n';
  assert(length <= std::numeric_limits<uint32_t>::max());
  const auto byteCount = uint32_t(length);
  bitstream.write(byteCount);
  const auto offset = bitstream.size();
  bitstream.resize(offset + byteCount);
  std::copy(
    arithmeticEncoder.buffer(), arithmeticEncoder.buffer() + byteCount,
    bitstream.buffer.begin() + offset);
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCEncoder::compressBaseMesh(
  const VMCGroupOfFrames& gof,
  const VMCFrameInfo& frameInfo,
  VMCFrame& frame,
  Bitstream& bitstream,
  VMCStats& stats,
  const VMCEncoderParameters& params) const
{
  if (encodeFrameHeader(frameInfo, bitstream)) {
    return -1;
  }
  const int32_t frameIndex = frameInfo.frameIndex + _gofInfo.startFrameIndex;
  const auto scalePosition =
    ((1 << params.qpPosition) - 1.0) / ((1 << params.bitDepthPosition) - 1.0);
  const auto scaleTexCoord = std::pow(2.0, params.qpTexCoord) - 1.0;
  const auto iscalePosition = 1.0 / scalePosition;
  const auto iscaleTexCoord = 1.0 / scaleTexCoord;

  auto& base = frame.base;
  auto& subdiv = frame.subdiv;
  auto& mapping = frame.mapping;
  auto& qpositions = frame.qpositions;
  if (frameInfo.type == FrameType::INTRA) {
    const auto texCoordBBox = base.texCoordBoundingBox();
    const auto delta = texCoordBBox.max - texCoordBBox.min;
    const auto d = std::max(delta[0], delta[1]);
    if (d > 2.0) {  // make sure texCoords are between 0.0 and 1.0
      const auto scale = 1.0 / (std::pow(2.0, params.bitDepthTexCoord) - 1.0);
      for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount;
           ++tc) {
        base.setTexCoord(tc, base.texCoord(tc) * scale);
      }
    }
    computeDracoMapping(base, mapping, frameIndex, params);
    // quantize base mesh

    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      base.setPoint(v, Round(base.point(v) * scalePosition));
    }
    for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
      base.setTexCoord(tc, Round(base.texCoord(tc) * scaleTexCoord));
    }

    std::stringstream qbaseFileName, cbaseFileName, rbaseFileName;
    qbaseFileName << "gof_" << _gofInfo.index << "_fr_" << frameIndex
                  << "_qbase.obj";
    cbaseFileName << "gof_" << _gofInfo.index << "_fr_" << frameIndex
                  << "_cbase.drc";
    rbaseFileName << "gof_" << _gofInfo.index << "_fr_" << frameIndex
                  << "_rbase.obj";
    base.saveToOBJ<int64_t>(qbaseFileName.str());

    std::stringstream cmdEnc;
    cmdEnc << params.geometryMeshEncoderPath << " -i " << qbaseFileName.str()
           << " -o " << cbaseFileName.str() << " -qp " << params.qpPosition
           << " -qt " << params.qpTexCoord << " -qn -1 -qg -1 -cl 10 ";
    std::cout << cmdEnc.str() << std::endl;
    system(cmdEnc.str().c_str());

    auto bitstreamByteCount0 = bitstream.size();
    bitstream.append(cbaseFileName.str(), true);
    stats.baseMeshByteCount += bitstream.size() - bitstreamByteCount0;

    std::stringstream cmdDec;
    cmdDec << params.geometryMeshDecoderPath << " -i " << cbaseFileName.str()
           << " -o " << rbaseFileName.str();
    std::cout << cmdDec.str() << std::endl;
    system(cmdDec.str().c_str());
    base.loadFromOBJ(rbaseFileName.str());

    if (params.keepIntermediateFiles) {
      std::remove(qbaseFileName.str().c_str());
      std::remove(cbaseFileName.str().c_str());
      std::remove(rbaseFileName.str().c_str());
    }

    qpositions.resize(base.pointCount());
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      auto& qpos = qpositions[v];
      const auto& point = base.point(v);
      for (int32_t k = 0; k < 3; ++k) {
        qpos[k] = int32_t(std::round(point[k]));
      }
    }
    for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
      base.setTexCoord(tc, base.texCoord(tc) * iscaleTexCoord);
    }
  } else {
    // quantize base mesh
    const auto& refFrame = gof.frame(frameInfo.referenceFrameIndex);
    mapping = refFrame.mapping;
    const auto pointCountRecBaseMesh = refFrame.base.pointCount();
    qpositions.resize(pointCountRecBaseMesh);
    for (int32_t v = 0; v < pointCountRecBaseMesh; ++v) {
      assert(mapping[v] >= 0 && mapping[v] < base.pointCount());
      const auto point = Round(base.point(mapping[v]) * scalePosition);
      auto& qpos = qpositions[v];
      for (int32_t k = 0; k < 3; ++k) {
        qpos[k] = int32_t(std::round(point[k]));
      }
    }
    base = refFrame.base;
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      const auto& qpos = qpositions[v];
      auto& point = base.point(v);
      for (int32_t k = 0; k < 3; ++k) {
        point[k] = qpos[k];
      }
    }
    auto bitstreamByteCount0 = bitstream.size();
    compressMotion(
      base.triangles(), qpositions, refFrame.qpositions, bitstream, params);
    stats.motionByteCount += bitstream.size() - bitstreamByteCount0;
  }
  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.setPoint(v, base.point(v) * iscalePosition);
  }

  subdivideBaseMesh(
    frame, params.subdivisionMethod, params.subdivisionIterationCount);
  const auto& rec = frame.rec;

  auto rsubdiv = rec;
  std::swap(rsubdiv, subdiv);
  for (int32_t v = 0, vcount = subdiv.pointCount(); v < vcount; ++v) {
    const auto vindex = mapping[v];
    assert(vindex >= 0 && vindex < vcount);
    subdiv.setPoint(v, rsubdiv.point(vindex));
  }

  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCEncoder::computeDisplacements(
  VMCFrame& frame, const VMCEncoderParameters& params) const
{
  const auto& subdiv = frame.subdiv;
  const auto& rec = frame.rec;
  auto& disp = frame.disp;
  disp.resize(rec.pointCount());
  for (int32_t v = 0, vcount = rec.pointCount(); v < vcount; ++v) {
    if (
      params.displacementCoordinateSystem
      == DisplacementCoordinateSystem::LOCAL) {
      const auto n = rec.normal(v);
      Vec3<double> t, b;
      computeLocalCoordinatesSystem(n, t, b);
      const auto& pt0 = rec.point(v);
      const auto& pt1 = subdiv.point(v);
      const auto delta = pt1 - pt0;
      disp[v] = Vec3<double>(delta * n, delta * t, delta * b);
    } else {
      disp[v] = subdiv.point(v) - rec.point(v);
    }
  }
  return 0;
}

int32_t
VMCEncoder::quantizeDisplacements(
  VMCFrame& frame, const VMCEncoderParameters& params) const
{
  const auto& infoLevelOfDetails = frame.subdivInfoLevelOfDetails;
  const auto lodCount = int32_t(infoLevelOfDetails.size());
  assert(lodCount > 0);
  double scale[3], lodScale[3];
  for (int32_t k = 0; k < 3; ++k) {
    const auto qp = params.liftingQuantizationParameters[k];
    scale[k] =
      qp >= 0 ? pow(2.0, 16 - params.bitDepthPosition + (4 - qp) / 6.0) : 0.0;
    lodScale[k] = 1.0 / params.liftingLevelOfDetailInverseScale[k];
  }
  auto& disp = frame.disp;
  for (int32_t it = 0, vcount0 = 0; it < lodCount; ++it) {
    const auto vcount1 = infoLevelOfDetails[it].pointCount;
    for (int32_t v = vcount0; v < vcount1; ++v) {
      auto& d = disp[v];
      for (int32_t k = 0; k < 3; ++k) {
        d[k] = d[k] >= 0.0
          ? std::floor(d[k] * scale[k] + params.liftingQuantizationBias[k])
          : -std::floor(-d[k] * scale[k] + params.liftingQuantizationBias[k]);
      }
    }
    vcount0 = vcount1;
    for (int32_t k = 0; k < 3; ++k) {
      scale[k] *= lodScale[k];
    }
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCEncoder::transferTexture(
  VMCFrame& frame, const VMCEncoderParameters& params) const
{
  auto targetMesh = frame.input;
  // normalize texcoords so they are between 0.0 and 1.0
  const auto tcCount = targetMesh.texCoordCount();
  const auto uvScale = 1.0 / ((1 << params.bitDepthTexCoord) - 1);
  for (int32_t uvIndex = 0; uvIndex < tcCount; ++uvIndex) {
    targetMesh.setTexCoord(uvIndex, targetMesh.texCoord(uvIndex) * uvScale);
  }
  targetMesh.subdivideMidPoint(
    params.textureTransferSamplingSubdivisionIterationCount);
  frame.outputTexture.resize(params.textureWidth, params.textureHeight);
  frame.outputTextureOccupancy.resize(
    params.textureWidth, params.textureHeight);
  if (params.invertOrientation) {
    frame.rec.invertOrientation();
  }
  const auto ret = transferTexture(
    targetMesh, frame.rec, frame.inputTexture, frame.outputTexture,
    frame.outputTextureOccupancy, params);
  if (params.invertOrientation) {
    frame.rec.invertOrientation();
  }
  return ret;
}
int32_t
VMCEncoder::transferTexture(
  const TriangleMesh<double>& targetMesh,
  const TriangleMesh<double>& sourceMesh,
  const Frame<uint8_t, ColourSpace::BGR444p>& targetTexture,
  Frame<uint8_t, ColourSpace::BGR444p>& outputTexture,
  Plane<uint8_t>& occupancy,
  const VMCEncoderParameters& params)
{
  if (
    !targetMesh.pointCount() || !sourceMesh.pointCount()
    || targetMesh.triangleCount() != targetMesh.texCoordTriangleCount()
    || sourceMesh.triangleCount() != sourceMesh.texCoordTriangleCount()
    || outputTexture.width() <= 0 || outputTexture.height() <= 0
    || outputTexture.width() != occupancy.width()
    || outputTexture.height() != occupancy.height()) {
    return -1;
  }
  occupancy.fill(uint8_t(0));

  StaticAdjacencyInformation<int32_t> vertexToTriangleTarget;
  ComputeVertexToTriangle(
    targetMesh.triangles(), targetMesh.pointCount(), vertexToTriangleTarget);

  KdTree kdtree(3, targetMesh.points(), 10);  // dim, cloud, max leaf

  const auto oWidth = outputTexture.width();
  const auto oHeight = outputTexture.height();
  const auto oWidthMinus1 = oWidth - 1;
  const auto oHeightMinus1 = oHeight - 1;
  auto& oB = outputTexture.plane(0);
  auto& oG = outputTexture.plane(1);
  auto& oR = outputTexture.plane(2);
  const auto sTriangleCount = sourceMesh.triangleCount();
  Plane<int32_t> triangleMap;
  triangleMap.resize(oWidth, oHeight);

  for (int32_t t = 0; t < sTriangleCount; ++t) {
    const auto& triUV = sourceMesh.texCoordTriangle(t);
    const Vec2<double> uv[3] = {
      sourceMesh.texCoord(triUV[0]), sourceMesh.texCoord(triUV[1]),
      sourceMesh.texCoord(triUV[2])};
    const auto& triPos = sourceMesh.triangle(t);
    const Vec3<double> pos[3] = {
      sourceMesh.point(triPos[0]), sourceMesh.point(triPos[1]),
      sourceMesh.point(triPos[2])};
    const auto area = (uv[1] - uv[0]) ^ (uv[2] - uv[0]);
    if (area <= 0.0) {
      continue;
    }
    assert(area > 0.0);
    const auto iarea = area > 0.0 ? 1.0 / area : 1.0;
    auto i0 = oHeightMinus1;
    auto i1 = 0;
    auto j0 = oWidthMinus1;
    auto j1 = 0;
    for (int32_t k = 0; k < 3; ++k) {
      const auto i = int32_t(std::floor(uv[k][1] * oHeightMinus1));
      const auto j = int32_t(std::floor(uv[k][0] * oWidthMinus1));
      i0 = std::min(i0, i);
      j0 = std::min(j0, j);
      i1 = std::max(i1, i);
      j1 = std::max(j1, j);
    }
    i0 = std::max(i0, 0);
    i1 = std::min(i1, oHeightMinus1);
    j0 = std::max(j0, 0);
    j1 = std::min(j1, oWidthMinus1);
    for (int32_t i = i0; i <= i1; ++i) {
      const auto y = double(i) / oHeightMinus1;
      for (int32_t j = j0; j <= j1; ++j) {
        const auto x = double(j) / oWidthMinus1;
        const Vec2<double> uvP(x, y);
        auto w0 = ((uv[2] - uv[1]) ^ (uvP - uv[1])) * iarea;
        auto w1 = ((uv[0] - uv[2]) ^ (uvP - uv[2])) * iarea;
        auto w2 = ((uv[1] - uv[0]) ^ (uvP - uv[0])) * iarea;
        if (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) {
          const auto point0 = w0 * pos[0] + w1 * pos[1] + w2 * pos[2];
          double minDist2;
          const auto bgr = computeNearestPointColour(
            point0, targetTexture, targetMesh, kdtree, vertexToTriangleTarget,
            minDist2);
          const auto ii = oHeightMinus1 - i;
          oB.set(ii, j, uint8_t(std::round(bgr[0])));
          oG.set(ii, j, uint8_t(std::round(bgr[1])));
          oR.set(ii, j, uint8_t(std::round(bgr[2])));
          occupancy.set(ii, j, uint8_t(255));
          triangleMap.set(ii, j, t);
        }
      }
    }
  }
  const int32_t shift[4][2] = {{-1, -0}, {0, -1}, {1, 0}, {0, 1}};
  for (int32_t it = 0;
       it < params.textureTransferPaddingBoundaryIterationCount; ++it) {
    const uint8_t checkValue = uint8_t(255 - it);
    for (int32_t i = 0; i < oHeight; ++i) {
      for (int32_t j = 0; j < oWidth; ++j) {
        if (occupancy(i, j)) {
          continue;
        }
        double minTriangleDist2 = std::numeric_limits<double>::max();
        Vec3<double> bgr(0.0);
        int32_t count = 0;
        for (int32_t k = 0; k < 4; ++k) {
          const auto i1 = i + shift[k][0];
          const auto j1 = j + shift[k][1];
          if (
            i1 < 0 || j1 < 0 || i1 >= oHeight || j1 >= oWidth
            || occupancy(i1, j1) != checkValue) {
            continue;
          }

          const auto y = double(oHeightMinus1 - i) / oHeightMinus1;
          const auto x = double(j) / oWidthMinus1;

          const Vec2<double> uvP(x, y);

          const auto t = triangleMap(i1, j1);
          const auto& triUV = sourceMesh.texCoordTriangle(t);
          const Vec2<double> uv[3] = {
            sourceMesh.texCoord(triUV[0]), sourceMesh.texCoord(triUV[1]),
            sourceMesh.texCoord(triUV[2])};

          const auto& triPos = sourceMesh.triangle(t);
          const Vec3<double> pos[3] = {
            sourceMesh.point(triPos[0]), sourceMesh.point(triPos[1]),
            sourceMesh.point(triPos[2])};
          const auto area = (uv[1] - uv[0]) ^ (uv[2] - uv[0]);
          assert(area > 0.0);
          const auto iarea = area > 0.0 ? 1.0 / area : 1.0;
          const auto w0 = ((uv[2] - uv[1]) ^ (uvP - uv[1])) * iarea;
          const auto w1 = ((uv[0] - uv[2]) ^ (uvP - uv[2])) * iarea;
          const auto w2 = ((uv[1] - uv[0]) ^ (uvP - uv[0])) * iarea;
          const auto point0 = w0 * pos[0] + w1 * pos[1] + w2 * pos[2];
          double minDist2;
          bgr += computeNearestPointColour(
            point0, targetTexture, targetMesh, kdtree, vertexToTriangleTarget,
            minDist2);
          ++count;
          if (minDist2 < minTriangleDist2) {
            minTriangleDist2 = minDist2;
            triangleMap.set(i, j, t);
          }
        }
        if (count) {
          bgr /= count;
          oB.set(i, j, uint8_t(std::round(bgr[0])));
          oG.set(i, j, uint8_t(std::round(bgr[1])));
          oR.set(i, j, uint8_t(std::round(bgr[2])));
          occupancy.set(i, j, uint8_t(checkValue - 1));
        }
      }
    }
  }
  if (params.textureTransferPaddingDilateIterationCount) {
    Frame<uint8_t, ColourSpace::BGR444p> tmpTexture;
    Plane<uint8_t> tmpOccupancy;
    for (int32_t it = 0;
         it < params.textureTransferPaddingDilateIterationCount; ++it) {
      DilatePadding(outputTexture, occupancy, tmpTexture, tmpOccupancy);
      DilatePadding(tmpTexture, tmpOccupancy, outputTexture, occupancy);
    }
  }
  if (params.textureTransferPaddingMethod == PaddingMethod::PUSH_PULL) {
    PullPushPadding(outputTexture, occupancy);
  } else if (
    params.textureTransferPaddingMethod == PaddingMethod::SPARSE_LINEAR) {
    PullPushPadding(outputTexture, occupancy);
    SparseLinearPadding(
      outputTexture, occupancy,
      params.textureTransferPaddingSparseLinearThreshold);
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCEncoder::compress(
  const VMCGroupOfFramesInfo& gofInfo,
  VMCGroupOfFrames& gof,
  Bitstream& bitstream,
  const VMCEncoderParameters& params)
{
  _gofInfo = gofInfo;
  const int32_t frameCount = _gofInfo.frameCount;
  const int32_t pixelsPerBlock =
    params.geometryVideoBlockSize * params.geometryVideoBlockSize;
  const auto widthDispVideo =
    params.geometryVideoWidthInBlocks * params.geometryVideoBlockSize;
  auto heightDispVideo = 0;
  _dispVideo.resize(widthDispVideo, heightDispVideo, frameCount);
  auto& stats = gof.stats;
  stats.reset();
  stats.totalByteCount = bitstream.size();

  Bitstream bitstreamCompressedMeshes;
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    const auto& frameInfo = _gofInfo.frameInfo(frameIndex);
    auto& frame = gof.frame(frameIndex);
    auto& dispVideoFrame = _dispVideo.frame(frameIndex);
    compressBaseMesh(
      gof, frameInfo, frame, bitstreamCompressedMeshes, stats, params);
    const auto vertexCount = frame.subdiv.pointCount();
    stats.baseMeshVertexCount += frame.base.pointCount();
    stats.vertexCount += frame.rec.pointCount();
    stats.faceCount += frame.rec.triangleCount();
    if (params.encodeDisplacementsVideo) {
      const auto blockCount =
        (vertexCount + pixelsPerBlock - 1) / pixelsPerBlock;
      const auto geometryVideoHeightInBlocks =
        (blockCount + params.geometryVideoWidthInBlocks - 1)
        / params.geometryVideoWidthInBlocks;
      heightDispVideo = std::max(
        heightDispVideo,
        geometryVideoHeightInBlocks * params.geometryVideoBlockSize);
      dispVideoFrame.resize(widthDispVideo, heightDispVideo);
      computeDisplacements(frame, params);
      computeForwardLinearLifting(
        frame.disp, frame.subdivInfoLevelOfDetails, frame.subdivEdges,
        params.liftingPredictionWeight, params.liftingUpdateWeight,
        params.liftingSkipUpdate);
      quantizeDisplacements(frame, params);
      computeDisplacementVideoFrame(frame, dispVideoFrame, params);
    }
  }
  if (params.encodeDisplacementsVideo) {
    // resize all the frame to the same resolution
    _dispVideo.resize(widthDispVideo, heightDispVideo, frameCount);
  } else {
    _dispVideo.resize(0, 0, 0);
  }

  if (encodeSequenceHeader(gof, bitstream, params)) {
    return -1;
  }
  bitstream.append(bitstreamCompressedMeshes.buffer);

  stats.frameCount = frameCount;
  stats.displacementsByteCount = bitstream.size();
  if (
    params.encodeDisplacementsVideo
    && compressDisplacementsVideo(bitstream, params)) {
    return -1;
  }
  stats.displacementsByteCount =
    bitstream.size() - stats.displacementsByteCount;
  if (params.encodeTextureVideo) {
    std::cout << "Generating texture maps ";
    for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
      auto& frame = gof.frame(frameIndex);
      if (params.encodeDisplacementsVideo) {
        reconstructDisplacementFromVideoFrame(
          _dispVideo.frame(frameIndex), frame, params.geometryVideoBlockSize,
          params.geometryVideoBitDepth);
        inverseQuantizeDisplacements(
          frame, params.bitDepthPosition,
          params.liftingLevelOfDetailInverseScale,
          params.liftingQuantizationParameters);
        computeInverseLinearLifting(
          frame.disp, frame.subdivInfoLevelOfDetails, frame.subdivEdges,
          params.liftingPredictionWeight, params.liftingUpdateWeight,
          params.liftingSkipUpdate);
        applyDisplacements(frame, params.displacementCoordinateSystem);
      }
      if (transferTexture(frame, params)) {
        return -1;
      }
      std::cout << '.';
    }
    std::cout << '\n';
    // compress texture
    stats.textureByteCount = bitstream.size();
    if (compressTextureVideo(gof, bitstream, params)) {
      return -1;
    }
  }
  stats.textureByteCount = bitstream.size() - stats.textureByteCount;
  stats.totalByteCount = bitstream.size() - stats.totalByteCount;
  if (!params.normalizeUV) {
    const auto scale = (1 << params.bitDepthTexCoord) - 1.0;
    for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
      auto& rec = gof.frame(frameIndex).rec;
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
VMCEncoder::computeDisplacementVideoFrame(
  const VMCFrame& frame,
  Frame<uint16_t, ColourSpace::YUV444p>& dispVideoFrame,
  const VMCEncoderParameters& params) const
{
  const auto pixelsPerBlock =
    params.geometryVideoBlockSize * params.geometryVideoBlockSize;
  const auto shift = uint16_t((1 << params.geometryVideoBitDepth) >> 1);
  const auto& disp = frame.disp;
  auto& Y = dispVideoFrame.plane(0);
  auto& U = dispVideoFrame.plane(1);
  auto& V = dispVideoFrame.plane(2);
  Y.fill(shift);
  U.fill(shift);
  V.fill(shift);
  for (int32_t v = 0, vcounter = 0, vcount = int32_t(disp.size()); v < vcount;
       ++v) {
    const auto& d = disp[v];
    const auto blockIndex =
      vcounter / pixelsPerBlock;  // to do: optimize power of 2
    const auto indexWithinBlock =
      vcounter % pixelsPerBlock;  // to do: optimize power of 2
    const auto x0 = (blockIndex % params.geometryVideoWidthInBlocks)
      * params.geometryVideoBlockSize;  // to do: optimize power of 2
    const auto y0 = (blockIndex / params.geometryVideoWidthInBlocks)
      * params.geometryVideoBlockSize;  // to do: optimize power of 2
    int32_t x, y;
    computeMorton2D(indexWithinBlock, x, y);
    assert(x < params.geometryVideoBlockSize);
    assert(y < params.geometryVideoBlockSize);

    const auto x1 = x0 + x;
    const auto y1 = y0 + y;
    const auto d0 = int32_t(shift + d[0]);
    const auto d1 = int32_t(shift + d[1]);
    const auto d2 = int32_t(shift + d[2]);

    assert(d0 >= 0 && d0 < (1 << params.geometryVideoBitDepth));
    assert(d1 >= 0 && d0 < (1 << params.geometryVideoBitDepth));
    assert(d2 >= 0 && d0 < (1 << params.geometryVideoBitDepth));
    Y.set(y1, x1, uint16_t(d0));
    U.set(y1, x1, uint16_t(d1));
    V.set(y1, x1, uint16_t(d2));
    ++vcounter;
  }
  return 0;
}

//============================================================================

}  // namespace vmesh
