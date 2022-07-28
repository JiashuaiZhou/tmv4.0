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

#include "util/mesh.hpp"
#include "vmc.hpp"
#include "util/bitstream.hpp"

#include <UVAtlas.h>

//============================================================================
namespace DirectX {
  static std::istream& operator>>(std::istream& in, UVATLAS& val)
  {
    std::string str;
    in >> str;
    if (str == "DEFAULT") {
      val = UVATLAS_DEFAULT;
    } else if (str == "FAST") {
      val = UVATLAS_GEODESIC_FAST;
    } else if (str == "QUALITY") {
      val = UVATLAS_GEODESIC_QUALITY;
    } else {
      in.setstate(std::ios::failbit);
    }
    return in;
  }

  //----------------------------------------------------------------------------

  static std::ostream& operator<<(std::ostream& out, UVATLAS val)
  {
    switch (val) {
    case UVATLAS_DEFAULT: out << "DEFAULT"; break;
    case UVATLAS_GEODESIC_FAST: out << "FAST"; break;
    case UVATLAS_GEODESIC_QUALITY: out << "QUALITY"; break;
    default: out << int(val) << " (unknown)";
    }
    return out;
  }
}  // namespace DirectX

//============================================================================

namespace vmesh {
  
//============================================================================

static std::istream&
operator>>(std::istream& in, SmoothingMethod& val)
{
    unsigned int tmp = 0;
    in >> tmp;
    val = SmoothingMethod(tmp);
    return in;
}

//----------------------------------------------------------------------------

static std::ostream&
operator<<(std::ostream& out, SmoothingMethod val)
{
  switch (val) {
  case SmoothingMethod::NONE: out << "0"; break;
  case SmoothingMethod::VERTEX_CONSTRAINT: out << "1"; break;
  }
  return out;
}

//============================================================================

struct GeometryParametrizationParameters{
  int geometrySamplingSubdivisionIterationCount = 3;
  bool applyVertexUnification = true;
  int32_t geometryFittingIterationCount = 16;
  double geometrySmoothingCoeffcient = 0.25;
  double geometrySmoothingCoeffcientDecayRatio = 0.75;
  double geometryMissedVerticesSmoothingCoeffcient = 0.1;
  int32_t geometryMissedVerticesSmoothingIterationCount = 10;
  SubdivisionMethod subdivisionMethod = SubdivisionMethod::MID_POINT;
  int32_t geometryParametrizationSubdivisionIterationCount = 3;
  bool fitSubdivisionSurface = true;
  double initialDeformNormalDeviationThreshold = 0.1;
  int32_t initialDeformNNCount = 1;
  bool initialDeformForceNormalDisplacement = false;
  bool applySmoothingDeform = true;
  SmoothingMethod smoothDeformSmoothingMethod =
    SmoothingMethod::VERTEX_CONSTRAINT;
  bool smoothDeformUpdateNormals = true;
  double smoothDeformTriangleNormalFlipThreshold = -0.5;
  bool smoothingDeformUseInitialGeometry = true;
  bool smoothingDeformSmoothMotion = true;  
};

struct VMCEncoderParameters {
  // mesh
  int32_t qpPosition = 10;
  int32_t qpTexCoord = 8;
  int32_t bitDepthPosition = 12;
  int32_t bitDepthTexCoord = 12;
  double minPosition[3] = {0, 0, 0};
  double maxPosition[3] = {0, 0, 0};

  // Gof analysis
  int32_t groupOfFramesMaxSize = 32;
  bool    analyzeGof = false;

  // geometry video
  int32_t geometryVideoBlockSize = 16;
  int32_t geometryVideoWidthInBlocks = 16;
  int32_t geometryVideoBitDepth = 10;
  std::string geometryVideoEncoderConfig ={};

  // texture video
  int32_t textureVideoBitDepth = 10;
  int32_t textureVideoQP = 8;
  std::string textureVideoEncoderConfig = {};
  std::string textureVideoHDRToolEncConfig = {};
  std::string textureVideoHDRToolDecConfig = {};

  // displacements
  DisplacementCoordinateSystem displacementCoordinateSystem =
    DisplacementCoordinateSystem::LOCAL;
  bool encodeDisplacementsVideo =true;
  bool encodeTextureVideo =true;

  // lifting
  double liftingUpdateWeight = 0.125;
  double liftingPredictionWeight = 0.5;
  bool liftingSkipUpdate = false;
  double liftingQuantizationBias[3] = {1./3., 1./3., 1./3};
  double liftingLevelOfDetailInverseScale[3] = {2.0, 2.0, 2.0};
  int32_t liftingQuantizationParameters[3] = {16, 28, 28};

  // texture transfer
  int32_t textureWidth = 2048;
  int32_t textureHeight = 2048;
  int32_t liftingSubdivisionIterationCount = 2; 
  int32_t textureTransferSamplingSubdivisionIterationCount = 3;
  int32_t textureTransferPaddingBoundaryIterationCount = 2;
  int32_t textureTransferPaddingDilateIterationCount = 2;
  PaddingMethod textureTransferPaddingMethod = PaddingMethod::SPARSE_LINEAR;
  double textureTransferPaddingSparseLinearThreshold = 0.05;  //0.005

  // input
  bool unifyVertices = false;
  bool invertOrientation = false;
  bool normalizeUV = false;

  // output
  std::string intermediateFilesPathPrefix = {};
  bool keepIntermediateFiles = false;

  // GeometryDecimate  
  int32_t texCoordQuantizationBits = 0;
  int32_t minCCTriangleCount = 0;
  double targetTriangleRatio = 0.125;
  double triangleFlipThreshold = 0.3;
  double trackedTriangleFlipThreshold = 0.1;
  double trackedPointNormalFlipThreshold = 0.5;

  // TextureParametrization  
  size_t maxCharts = size_t();
  float maxStretch = 0.16667F;
  float gutter = 2.F;
  size_t width = 512;
  size_t height = 512;
  DirectX::UVATLAS uvOptions = DirectX::UVATLAS_DEFAULT;

  // GeometryParametrization
  bool baseIsSrc    = false;
  bool subdivIsBase = false;
  bool subdivInter  = false;
  bool subdivInterWithMapping = false;
  float maxAllowedD2PSNRLoss = 1.F;
  GeometryParametrizationParameters intraGeoParams;
  GeometryParametrizationParameters interGeoParams;

  // Bug fix 
  bool forceCoordTruncation = true;
};

//============================================================================

class VMCEncoder {
public:
  VMCEncoder() = default;
  VMCEncoder(const VMCEncoder& rhs) = delete;
  VMCEncoder& operator=(const VMCEncoder& rhs) = delete;
  ~VMCEncoder() = default;

  int32_t compressOnly(
    const VMCGroupOfFramesInfo& gofInfo,
    VMCGroupOfFrames& gof,
    Bitstream& bitstream,
    const VMCEncoderParameters& params);

  int32_t compress(
    const VMCGroupOfFramesInfo& gofInfo,
    VMCGroupOfFrames& gof,
    Bitstream& bitstream,
    const VMCEncoderParameters& params);

private:
  static void unifyVertices(
    const VMCGroupOfFramesInfo& gofInfo,
    VMCGroupOfFrames& gof,
    const VMCEncoderParameters& params);
  int32_t computeDracoMapping(
    TriangleMesh<double> base,
    std::vector<int32_t>& mapping,
    int32_t frameIndex,
    const VMCEncoderParameters& params) const;
  int32_t encodeSequenceHeader(
    const VMCGroupOfFrames& gof,
    Bitstream& bitstream,
    const VMCEncoderParameters& params) const;
  static int32_t
  encodeFrameHeader(const VMCFrameInfo& frameInfo, Bitstream& bitstream);
  static int32_t
  computeDisplacements(VMCFrame& frame, const VMCEncoderParameters& params);
  static int32_t
  quantizeDisplacements(VMCFrame& frame, const VMCEncoderParameters& params);

  int32_t init(
    const VMCGroupOfFrames& gof,
    Bitstream& bitstream,
    const VMCEncoderParameters& params);

  int32_t compressBaseMesh(
    const VMCGroupOfFrames& gof,
    const VMCFrameInfo& frameInfo,
    VMCFrame& frame,
    Bitstream& bitstream,
    VMCStats& stats,
    const VMCEncoderParameters& params) const;
  static int32_t compressMotion(
    const std::vector<Vec3<int32_t>>& triangles,
    const std::vector<Vec3<int32_t>>& current,
    const std::vector<Vec3<int32_t>>& reference,
    Bitstream& bitstream,
    const VMCEncoderParameters& params);
  static int32_t computeDisplacementVideoFrame(
    const VMCFrame& frame,
    Frame<uint16_t>& dispVideoFrame,  // ColourSpace::YUV444p
    const VMCEncoderParameters& params);
  int32_t compressDisplacementsVideo(
    Bitstream& bitstream, const VMCEncoderParameters& params);
  int32_t compressTextureVideo(
    VMCGroupOfFrames& gof,
    Bitstream& bitstream,
    const VMCEncoderParameters& params) const;

  static int32_t
  transferTexture(VMCFrame& frame, const VMCEncoderParameters& params);

  static int32_t transferTexture(
    const TriangleMesh<double>& targetMesh,
    const TriangleMesh<double>& sourceMesh,
    const Frame<uint8_t>& targetTexture,  // ColourSpace::BGR444p
    Frame<uint8_t>& outputTexture,        // ColourSpace::BGR444p
    const VMCEncoderParameters& params);

  VMCGroupOfFramesInfo _gofInfo;
  FrameSequence<uint16_t> _dispVideo; // ColourSpace::YUV444p
};

//============================================================================

}  // namespace vmesh
