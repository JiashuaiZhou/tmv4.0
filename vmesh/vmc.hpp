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

#include "vmesh/util/image.hpp"
#include "vmesh/util/mesh.hpp"
#include "vmesh/vmcstats.hpp"

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
  vmesh::Frame<uint8_t, vmesh::ColourSpace::BGR444p> inputTexture;
  vmesh::Frame<uint8_t, vmesh::ColourSpace::BGR444p> outputTexture;
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
  std::vector<VMCFrameInfo> framesInfo;
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

int32_t reconstructDisplacementFromVideoFrame(
  const vmesh::Frame<uint16_t, vmesh::ColourSpace::YUV444p>& dispVideoFrame,
  VMCFrame& frame,
  const int32_t geometryVideoBlockSize,
  const int32_t geometryVideoBitDepth);

int32_t subdivideBaseMesh(
  VMCFrame& frame,
  const vmesh::SubdivisionMethod subdivisionMethod,
  const int32_t subdivisionIterationCount);

int32_t applyDisplacements(
  VMCFrame& frame,
  const DisplacementCoordinateSystem& displacementCoordinateSystem);

//============================================================================

}  // namespace vmesh
