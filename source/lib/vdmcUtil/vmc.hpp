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

#include "image.hpp"
#include "mesh.hpp"
#include "vmcstats.hpp"

namespace vmesh {

//============================================================================

enum class FrameType
{
  INTER = 0,
  INTRA = 1,
  SKIP = 2
};

enum class DisplacementCoordinateSystem
{
  CANNONICAL = 0,
  LOCAL = 1,
};

enum class PaddingMethod
{
  NONE = 0,
  PUSH_PULL = 1,
  SPARSE_LINEAR = 2
};

//============================================================================

struct VMCSequenceParameterSet {
  int32_t widthDispVideo = 0;
  int32_t heightDispVideo = 0;
  int32_t widthTexVideo = 0;
  int32_t heightTexVideo = 0;
  int32_t frameCount = 0;
  int32_t geometryVideoBlockSize = 0;
  int32_t geometryVideoBitDepth = 10;
  int32_t textureVideoBitDepth = 10;
  int32_t bitDepthPosition = 0;
  int32_t bitDepthTexCoord = 0;
  int32_t qpPosition = 0;
  int32_t qpTexCoord = 0;
  int32_t subdivisionIterationCount = 0;
  int32_t liftingQuantizationParameters[3] = {0, 0, 0};
  double liftingLevelOfDetailInverseScale[3] = {2.0, 2.0, 2.0};
  double liftingUpdateWeight = 0.125;
  double liftingPredictionWeight = 0.5;
  bool liftingSkipUpdate = false;
  bool encodeDisplacementsVideo = true;
  bool encodeTextureVideo = true;
  vmesh::SubdivisionMethod subdivisionMethod =
    vmesh::SubdivisionMethod::MID_POINT;
  DisplacementCoordinateSystem displacementCoordinateSystem =
    DisplacementCoordinateSystem::LOCAL;
};

//============================================================================

struct VMCFrame {
  std::vector<int32_t> mapping;
  std::vector<vmesh::Vec3<int32_t>> qpositions;
  std::vector<vmesh::Vec3<double>> disp;
  std::vector<int64_t> subdivEdges;
  std::vector<vmesh::SubdivisionLevelInfo> subdivInfoLevelOfDetails;
  vmesh::TriangleMesh<double> rec;
  vmesh::TriangleMesh<double> base;
  vmesh::TriangleMesh<double> subdiv;
  vmesh::TriangleMesh<double> input;
  vmesh::Frame<uint8_t> inputTexture;  // ColourSpace::BGR444p
  vmesh::Frame<uint8_t> outputTexture; // ColourSpace::BGR444p
  vmesh::Plane<uint8_t> outputTextureOccupancy;
};

//============================================================================

struct VMCFrameInfo {
  int32_t frameIndex = -1;
  int32_t referenceFrameIndex = -1;
  int32_t patchCount = 1;
  FrameType type = FrameType::INTRA;
};


//============================================================================


struct VMCGroupOfFramesInfo {
  void resize(int32_t fCount) { framesInfo.resize(fCount); }

  const VMCFrameInfo& frameInfo(int32_t frameIndex) const
  {
    assert(frameIndex >= 0 && frameIndex < frameCount);
    return framesInfo[frameIndex];
  }

  VMCFrameInfo& frameInfo(int32_t frameIndex)
  {
    assert(frameIndex >= 0 && frameIndex < frameCount);
    return framesInfo[frameIndex];
  }

  int32_t startFrameIndex = -1;
  int32_t frameCount = -1;
  int32_t index = 0;
  std::vector<vmesh::VMCFrameInfo> framesInfo;
};

//============================================================================

struct VMCGroupOfFrames {
  void resize(int32_t frameCount) { frames.resize(frameCount); }

  const VMCFrame& frame(int32_t frameIndex) const
  {
    assert(frameIndex >= 0 && frameIndex < frameCount());
    return frames[frameIndex];
  }

  VMCFrame& frame(int32_t frameIndex)
  {
    assert(frameIndex >= 0 && frameIndex < frameCount());
    return frames[frameIndex];
  }

  int32_t frameCount() const { return int32_t(frames.size()); }

  VMCStats stats;
  std::vector<VMCFrame> frames;
};

//============================================================================

static int32_t
reconstructDisplacementFromVideoFrame(
  const Frame<uint16_t>& dispVideoFrame,  // ColourSpace::YUV444p
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

static int32_t
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

static int32_t
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

static int32_t
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

//============================================================================

}  // namespace vmesh
