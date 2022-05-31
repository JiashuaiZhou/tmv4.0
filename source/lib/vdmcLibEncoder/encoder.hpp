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

#include "mesh.hpp"
#include "vmc.hpp"
#include "bitstream.hpp"

//============================================================================

namespace vmeshenc {
  
//============================================================================

struct VMCEncoderParameters {
  // mesh
  int32_t qpPosition;
  int32_t qpTexCoord;
  int32_t bitDepthPosition;
  int32_t bitDepthTexCoord;
  std::string geometryMeshEncoderPath;
  std::string geometryMeshDecoderPath;

  // geometry video
  int32_t geometryVideoBlockSize = 16;
  int32_t geometryVideoWidthInBlocks = 16;
  int32_t geometryVideoBitDepth = 10;
  std::string geometryVideoEncoderPath;
  std::string geometryVideoEncoderConfig;

  // texture video
  int32_t textureVideoBitDepth = 10;
  int32_t textureVideoQP;
  std::string textureVideoEncoderPath;
  std::string textureVideoEncoderConfig;
  std::string textureVideoHDRToolPath;
  std::string textureVideoHDRToolEncConfig;
  std::string textureVideoHDRToolDecConfig;

  // subdivision
  vmesh::SubdivisionMethod subdivisionMethod =
    vmesh::SubdivisionMethod::MID_POINT;
  int32_t subdivisionIterationCount;

  // displacements
  vmesh::DisplacementCoordinateSystem displacementCoordinateSystem =
    vmesh::DisplacementCoordinateSystem::LOCAL;
  bool encodeDisplacementsVideo;
  bool encodeTextureVideo;

  // lifting
  double liftingUpdateWeight = 0.125;
  double liftingPredictionWeight = 0.5;
  bool liftingSkipUpdate = false;
  double liftingQuantizationBias[3];
  double liftingLevelOfDetailInverseScale[3] = {2.0, 2.0, 2.0};

  int32_t liftingQuantizationParameters[3];

  // texture transfer
  int32_t textureWidth;
  int32_t textureHeight;
  int32_t textureTransferSamplingSubdivisionIterationCount = 3;
  int32_t textureTransferPaddingBoundaryIterationCount = 2;
  int32_t textureTransferPaddingDilateIterationCount = 2;
  vmesh::PaddingMethod textureTransferPaddingMethod = vmesh::PaddingMethod::SPARSE_LINEAR;
  double textureTransferPaddingSparseLinearThreshold = 0.05;  //0.005

  // input
  bool unifyVertices;
  bool invertOrientation;
  bool normalizeUV;

  // output
  std::string intermediateFilesPathPrefix = {};
  bool keepIntermediateFiles;
};

//============================================================================

class VMCEncoder {
public:
  VMCEncoder() = default;
  VMCEncoder(const VMCEncoder& rhs) = delete;
  VMCEncoder& operator=(const VMCEncoder& rhs) = delete;
  ~VMCEncoder() = default;

  int32_t compress(
    const vmesh::VMCGroupOfFramesInfo& gofInfo,
    vmesh::VMCGroupOfFrames& gof,
    vmesh::Bitstream& bitstream,
    const VMCEncoderParameters& params);

private:
  int32_t computeDracoMapping(
    vmesh::TriangleMesh<double> base,
    std::vector<int32_t>& mapping,
    const int32_t frameIndex,
    const VMCEncoderParameters& params) const;
  int32_t encodeSequenceHeader(
    const vmesh::VMCGroupOfFrames& gof,
    vmesh::Bitstream& bitstream,
    const VMCEncoderParameters& params) const;
  int32_t
  encodeFrameHeader(const vmesh::VMCFrameInfo& frameInfo, vmesh::Bitstream& bitstream) const;
  int32_t computeDisplacements(
    vmesh::VMCFrame& frame, const VMCEncoderParameters& params) const;
  int32_t quantizeDisplacements(
    vmesh::VMCFrame& frame, const VMCEncoderParameters& params) const;

  int32_t init(
    const vmesh::VMCGroupOfFrames& gof,
    vmesh::Bitstream& bitstream,
    const VMCEncoderParameters& params);

  int32_t compressBaseMesh(
    const vmesh::VMCGroupOfFrames& gof,
    const vmesh::VMCFrameInfo& frameInfo,
    vmesh::VMCFrame& frame,
    vmesh::Bitstream& bitstream,
    vmesh::VMCStats& stats,
    const VMCEncoderParameters& params) const;
  int32_t compressMotion(
    const std::vector<vmesh::Vec3<int32_t>>& triangles,
    const std::vector<vmesh::Vec3<int32_t>>& current,
    const std::vector<vmesh::Vec3<int32_t>>& reference,
    vmesh::Bitstream& bitstream,
    const VMCEncoderParameters& params) const;
  int32_t computeDisplacementVideoFrame(
    const vmesh::VMCFrame& frame,
    vmesh::Frame<uint16_t, vmesh::ColourSpace::YUV444p>& dispVideoFrame,
    const VMCEncoderParameters& params) const;
  int32_t compressDisplacementsVideo(
    vmesh::Bitstream& bitstream, const VMCEncoderParameters& params);
  int32_t compressTextureVideo(
    vmesh::VMCGroupOfFrames& gof,
    vmesh::Bitstream& bitstream,
    const VMCEncoderParameters& params);

  int32_t
  transferTexture(vmesh::VMCFrame& frame, const VMCEncoderParameters& params) const;

  static int32_t transferTexture(
    const vmesh::TriangleMesh<double>& targetMesh,
    const vmesh::TriangleMesh<double>& sourceMesh,
    const vmesh::Frame<uint8_t, vmesh::ColourSpace::BGR444p>& targetTexture,
    vmesh::Frame<uint8_t, vmesh::ColourSpace::BGR444p>& outputTexture,
    vmesh::Plane<uint8_t>& occupancy,
    const VMCEncoderParameters& params);

private:
  vmesh::VMCGroupOfFramesInfo _gofInfo;
  vmesh::FrameSequence<uint16_t, vmesh::ColourSpace::YUV444p> _dispVideo;
};

//============================================================================

}  // namespace vmesh
