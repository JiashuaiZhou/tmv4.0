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
#include "v3cBitstream.hpp"

#include <UVAtlas.h>

//============================================================================
namespace DirectX {
static std::istream&
operator>>(std::istream& in, UVATLAS& val) {
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

static std::ostream&
operator<<(std::ostream& out, UVATLAS val) {
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
operator>>(std::istream& in, CachingPoint& val) {
  std::string str;
  in >> str;
  if (str == "simplify" || str == "1") {
    val = CachingPoint::SIMPLIFY;
  } else if (str == "uvaltas" || str == "2") {
    val = CachingPoint::UVATLAS;
  } else if (str == "subdiv" || str == "3") {
    val = CachingPoint::SUBDIV;
  } else {
    val = CachingPoint::NONE;
  }
  return in;
}

//----------------------------------------------------------------------------

static std::ostream&
operator<<(std::ostream& out, CachingPoint val) {
  switch (val) {
  case CachingPoint::NONE: out << "none"; break;
  case CachingPoint::SIMPLIFY: out << "simplify"; break;
  case CachingPoint::UVATLAS: out << "uvaltas"; break;
  case CachingPoint::SUBDIV: out << "subdiv"; break;
  default: out << int(val) << " (unknown)";
  }
  return out;
}

//============================================================================

static std::istream&
operator>>(std::istream& in, SmoothingMethod& val) {
  unsigned int tmp = 0;
  in >> tmp;
  val = SmoothingMethod(tmp);
  return in;
}

//----------------------------------------------------------------------------

static std::ostream&
operator<<(std::ostream& out, SmoothingMethod val) {
  switch (val) {
  case SmoothingMethod::NONE: out << "0"; break;
  case SmoothingMethod::VERTEX_CONSTRAINT: out << "1"; break;
  }
  return out;
}

//============================================================================

static std::istream&
operator>>(std::istream& in, PaddingMethod& val) {
  std::string str;
  in >> str;
  if (str == "push_pull" || str == "1") {
    val = PaddingMethod::PUSH_PULL;
  } else if (str == "sparse_linear" || str == "2") {
    val = PaddingMethod::SPARSE_LINEAR;
  } else {
    val = PaddingMethod::NONE;
  }
  return in;
}

//----------------------------------------------------------------------------

static std::ostream&
operator<<(std::ostream& out, PaddingMethod val) {
  switch (val) {
  case PaddingMethod::NONE: out << "none"; break;
  case PaddingMethod::PUSH_PULL: out << "push_pull"; break;
  case PaddingMethod::SPARSE_LINEAR: out << "sparse_linear"; break;
  }
  return out;
}

//============================================================================

struct GeometryParametrizationParameters {
  int               geometrySamplingSubdivisionIterationCount     = 3;
  bool              applyVertexUnification                        = true;
  int32_t           geometryFittingIterationCount                 = 16;
  double            geometrySmoothingCoeffcient                   = 0.25;
  double            geometrySmoothingCoeffcientDecayRatio         = 0.75;
  double            geometryMissedVerticesSmoothingCoeffcient     = 0.1;
  int32_t           geometryMissedVerticesSmoothingIterationCount = 10;
  SubdivisionMethod subdivisionMethod = SubdivisionMethod::MID_POINT;
  int32_t           geometryParametrizationSubdivisionIterationCount = 3;
  bool              fitSubdivisionSurface                            = true;
  double            initialDeformNormalDeviationThreshold            = 0.1;
  int32_t           initialDeformNNCount                             = 1;
  bool              initialDeformForceNormalDisplacement             = false;
  bool              applySmoothingDeform                             = true;
  SmoothingMethod   smoothDeformSmoothingMethod =
    SmoothingMethod::VERTEX_CONSTRAINT;
  bool   smoothDeformUpdateNormals               = true;
  double smoothDeformTriangleNormalFlipThreshold = -0.5;
  bool   smoothingDeformUseInitialGeometry       = true;
  bool   smoothingDeformSmoothMotion             = true;
};

struct VMCEncoderParameters {
  // Mesh
  int32_t qpPosition       = 10;
  int32_t qpTexCoord       = 8;
  int32_t bitDepthPosition = 12;
  int32_t bitDepthTexCoord = 12;
  double  minPosition[3]   = {0, 0, 0};
  double  maxPosition[3]   = {0, 0, 0};

  // Gof analysis
  int32_t groupOfFramesMaxSize = 32;
  bool    analyzeGof           = false;

  // Geometry video
  int32_t      displacementVideoBlockSize = 16;
  int32_t      geometryVideoWidthInBlocks = 16;
  int32_t      geometryVideoBitDepth      = 10;
  std::string  geometryVideoEncoderConfig = {};
  VideoCodecId geometryVideoCodecId       = VideoCodecId::HM;

  // Texture video
  int32_t      textureVideoBitDepth         = 10;
  int32_t      textureVideoQP               = 8;
  std::string  textureVideoEncoderConfig    = {};
  std::string  textureVideoHDRToolEncConfig = {};
  std::string  textureVideoHDRToolDecConfig = {};
  int32_t      textureVideoDownsampleFilter = 4;
  int32_t      textureVideoUpsampleFilter   = 0;
  bool         textureVideoFullRange        = false;
  VideoCodecId textureVideoCodecId          = VideoCodecId::HM;
  bool         textureBGR444                = false;

  // Base mesh
  GeometryCodecId meshCodecId                     = GeometryCodecId::DRACO;
  bool            dracoUsePosition                = false;
  bool            dracoUseUV                      = false;
  bool            dracoMeshLossless               = false;
  int32_t         motionGroupSize                 = 16;
  bool            motionWithoutDuplicatedVertices = true;

  // Displacements
  DisplacementCoordinateSystem displacementCoordinateSystem =
    DisplacementCoordinateSystem::LOCAL;
  bool encodeDisplacementsVideo        = true;
  bool encodeTextureVideo              = true;
  bool applyOneDimensionalDisplacement = true;
  bool interpolateDisplacementNormals  = false;
  bool addReconstructedNormals         = true;
  bool displacementReversePacking      = true;
  bool displacementUse420              = true;

  // Lifting
  int32_t liftingSubdivisionIterationCount    = 2;
  double  liftingUpdateWeight                 = 0.125;
  double  liftingPredictionWeight             = 0.5;
  bool    liftingSkipUpdate                   = false;
  double  liftingBias[3]                      = {1. / 3., 1. / 3., 1. / 3};
  double  liftingLevelOfDetailInverseScale[3] = {2.0, 2.0, 2.0};
  int32_t liftingQP[3]                        = {16, 28, 28};

  // Texture transfer
  bool          textureTransferEnable                            = true;
  int32_t       textureWidth                                     = 2048;
  int32_t       textureHeight                                    = 2048;
  int32_t       textureTransferSamplingSubdivisionIterationCount = 3;
  int32_t       textureTransferPaddingBoundaryIterationCount     = 2;
  int32_t       textureTransferPaddingDilateIterationCount       = 2;
  PaddingMethod textureTransferPaddingMethod = PaddingMethod::SPARSE_LINEAR;
  double        textureTransferPaddingSparseLinearThreshold = 0.05;  //0.005
  double        textureTransferBasedPointcloud              = true;
  double        textureTransferPreferUV                     = false;
  bool          textureTransferWithMap                      = false;
  bool          textureTransferWithMapSource                = false;
  int32_t       textureTransferMapSamplingParam             = 1;
  int32_t       textureTransferGridSize                     = 0;
  int32_t       textureTransferMapProjDim                   = 0;
  int32_t       textureTransferMapNumPoints                 = 8;
  int32_t       textureTransferMethod                       = 0;
  float         textureTransferSigma                        = 0.2f;

  // Input
  bool unifyVertices     = false;
  bool invertOrientation = false;

  // Output
  bool dequantizeUV          = true;
  bool keepIntermediateFiles = false;
  bool reconstructNormals    = true;

  // GeometryDecimate
  int32_t texCoordQuantizationBits        = 0;
  int32_t minCCTriangleCount              = 0;
  double  targetTriangleRatio             = 0.125;
  double  triangleFlipThreshold           = 0.3;
  double  trackedTriangleFlipThreshold    = 0.1;
  double  trackedPointNormalFlipThreshold = 0.5;

  // TextureParametrization
  size_t           maxCharts  = size_t();
  float            maxStretch = 0.16667F;
  float            gutter     = 2.F;
  size_t           width      = 512;
  size_t           height     = 512;
  DirectX::UVATLAS uvOptions  = DirectX::UVATLAS_DEFAULT;

  // GeometryParametrization
  bool                              baseIsSrc              = false;
  bool                              subdivIsBase           = false;
  bool                              subdivInter            = false;
  bool                              subdivInterWithMapping = false;
  float                             maxAllowedD2PSNRLoss   = 1.F;
  GeometryParametrizationParameters intraGeoParams;
  GeometryParametrizationParameters interGeoParams;

  // Caching
  CachingPoint cachingPoint     = CachingPoint::NONE;
  std::string  cachingDirectory = {};

  // Bitsteam
  uint32_t forceSsvhUnitSizePrecision = 0;

  // New
  int32_t          maxNumRefAtlasList  = 1;
  int32_t          maxNumRefAtlasFrame = 4;
  std::vector<int> refFrameDiff        = {1, 2, 3, 4};
  int32_t          numSubmesh          = 1;
  int32_t          patchSegmentMethod =
    2;  //0. facegroup 1. connected component 2.single patch 3.one triangle one patch"
  std::string compressedFilename   = {};
  int32_t     texturePaddingMethod = (int32_t)PaddingMethod::SPARSE_LINEAR;

  // Metrics
  bool normalCalcModificationEnable = false;
};

//============================================================================

class VMCEncoder {
public:
  VMCEncoder()                                 = default;
  VMCEncoder(const VMCEncoder& rhs)            = delete;
  VMCEncoder& operator=(const VMCEncoder& rhs) = delete;
  ~VMCEncoder()                                = default;

  bool compress(const VMCGroupOfFramesInfo& gofInfo,
                const Sequence&             source,
                V3cBitstream&               syntax,
                Sequence&                   reconstruct,
                const VMCEncoderParameters& params);

  inline void setKeepFilesPathPrefix(const std::string& path) {
    _keepFilesPathPrefix = path;
  }

private:
  void decimateInput(const TriangleMesh<MeshType>& input,
                     VMCFrame&                     frame,
                     TriangleMesh<MeshType>&       decimate,
                     const VMCEncoderParameters&   params);

  void removeDegeneratedTrianglesCrossProduct(TriangleMesh<MeshType>& mesh,
                                              const int32_t& frameIndex);

  void textureParametrization(VMCFrame&                   frame,
                              TriangleMesh<MeshType>&     decimate,
                              const VMCEncoderParameters& params);

  void geometryParametrization(VMCGroupOfFrames&             gof,
                               VMCGroupOfFramesInfo&         gofInfo,
                               VMCFrame&                     frame,
                               const TriangleMesh<MeshType>& input,
                               const VMCEncoderParameters&   params,
                               int32_t& lastIntraFrameIndex);

  void        unifyVertices(const VMCGroupOfFramesInfo& gofInfo,
                            VMCGroupOfFrames&           gof,
                            const VMCEncoderParameters& params);
  bool        computeDracoMapping(TriangleMesh<MeshType>      base,
                                  std::vector<int32_t>&       mapping,
                                  int32_t                     frameIndex,
                                  const VMCEncoderParameters& params) const;
  static BaseMeshType
  chooseSkipOrInter(const std::vector<Vec3<int32_t>>& current,
                              const std::vector<Vec3<int32_t>>& reference);
  static bool computeDisplacements(VMCFrame&                     frame,
                                   const TriangleMesh<MeshType>& rec,
                                   const VMCEncoderParameters&   params);
  static bool quantizeDisplacements(VMCFrame&                   frame,
                                    const VMCEncoderParameters& params);

  bool compressBaseMesh(const VMCGroupOfFrames&     gof,
                        const VMCFrameInfo&         frameInfo,
                        VMCFrame&                   frame,
                        TriangleMesh<MeshType>&     rec,
                        BaseMeshTileLayer&          mfdu,
                        const VMCEncoderParameters& params);
  static bool
  compressMotion(const std::vector<Vec3<int32_t>>& triangles,
                 const std::vector<Vec3<int32_t>>& reference,
                 const std::vector<Vec2<int32_t>>& baseIntegrateIndices,
                 const std::vector<Vec3<int32_t>>& current,
                 BaseMeshTileLayer&                mfdu,
                 const VMCEncoderParameters&       params);
  static bool
       computeDisplacementVideoFrame(const VMCFrame&             frame,
                                     Frame<uint16_t>&            dispVideoFrame,
                                     const VMCEncoderParameters& params);
  bool compressDisplacementsVideo(FrameSequence<uint16_t>&    dispVideo,
                                  V3cBitstream&               syntax,
                                  const VMCEncoderParameters& params);
  bool compressTextureVideo(Sequence&                   reconstruct,
                            V3cBitstream&               syntax,
                            const VMCEncoderParameters& params);

  static bool transferTexture(const TriangleMesh<MeshType>& input,
                              const Frame<uint8_t>&         inputTexture,
                              TriangleMesh<MeshType>&       rec,
                              Frame<uint8_t>&               outputTexture,
                              const VMCEncoderParameters&   params);

  static bool transferTexture(TriangleMesh<MeshType>&     targetMesh,
                              TriangleMesh<MeshType>&     sourceMesh,
                              const Frame<uint8_t>&       targetTexture,
                              Frame<uint8_t>&             outputTexture,
                              const std::vector<int32_t>& srcTri2tgtTri,
                              const VMCEncoderParameters& params);

  std::string _keepFilesPathPrefix = {};
  int32_t     _gofIndex            = 0;
};

//============================================================================

}  // namespace vmesh
