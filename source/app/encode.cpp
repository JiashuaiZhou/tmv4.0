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
#include <program-options-lite/program_options_lite.h>
#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "encoder.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "checksum.hpp"
#include "metrics.hpp"
#include "sequenceInfo.hpp"
#include "v3cParameterSet.hpp"
#include "v3cBitstream.hpp"
#include "bitstream.hpp"
#include "v3cWriter.hpp"
#include "sampleStreamV3CUnit.hpp"

//============================================================================

struct Parameters {
  std::string                 inputMeshPath                = {};
  std::string                 inputTexturePath             = {};
  std::string                 groupOfFramesStructurePath   = {};
  std::string                 compressedStreamPath         = {};
  std::string                 reconstructedMeshPath        = {};
  std::string                 reconstructedTexturePath     = {};
  std::string                 reconstructedMaterialLibPath = {};
  int32_t                     startFrame                   = 0;
  int32_t                     frameCount                   = 1;
  double                      framerate                    = 30.;
  bool                        verbose                      = false;
  bool                        checksum                     = true;
  vmesh::VMCEncoderParameters encParams;
  vmesh::VMCMetricsParameters metParams;
};

//============================================================================

static bool
parseParameters(int argc, char* argv[], Parameters& params) try {
  namespace po                        = df::program_options_lite;
  bool                 print_help     = false;
  auto&                encParams      = params.encParams;
  auto&                metParams      = params.metParams;
  auto&                intraGeoParams = params.encParams.intraGeoParams;
  auto&                interGeoParams = params.encParams.interGeoParams;
  std::vector<int32_t> liftingQP2;
  /* clang-format off */
  po::Options opts;
  opts.addOptions()
  (po::Section("Common"))
    ("help",      print_help,          false, "This help text")
    ("config,c",  po::parseConfigFile, "Configuration file name")
    ("verbose,v", metParams.verbose,   false, "Verbose output")

  (po::Section("Input"))
    ("srcMesh", 
      params.inputMeshPath, 
      params.inputMeshPath, 
      "Input mesh")
    ("srcTex", 
      params.inputTexturePath, 
      params.inputTexturePath, 
      "Input texture")
    ("positionBitDepth", 
      encParams.bitDepthPosition, 
      encParams.bitDepthPosition, 
      "Input positions bit depth")
    ("texCoordBitDepth", 
      encParams.bitDepthTexCoord, 
      encParams.bitDepthTexCoord, 
      "Input texture coordinates bit depth")    
    ("startFrameIndex", 
      params.startFrame, 
      params.startFrame, 
      "First frame number")
    ("frameCount", 
      params.frameCount, 
      params.frameCount, 
      "Number of frames")
    ("framerate", 
      params.framerate, 
      params.framerate, 
      "Frame rate")   

  (po::Section("Output"))
    ("compressed",
      params.compressedStreamPath, 
      params.compressedStreamPath, 
      "Compressed bitstream")
    ("recMesh", 
      params.reconstructedMeshPath, 
      params.reconstructedMeshPath, 
      "Reconstructed mesh")
    ("recTex", 
      params.reconstructedTexturePath, 
      params.reconstructedTexturePath, 
      "Reconstructed texture")
    ("dequantizeUV", 
      encParams.dequantizeUV, 
      encParams.dequantizeUV, 
      "Dequantize texture coordinates of the reconstructed meshes")
    ("recMat", 
      params.reconstructedMaterialLibPath, 
      params.reconstructedMaterialLibPath, 
      "Reconstructed materials")
    ("reconstructNormals", 
      encParams.reconstructNormals,
      encParams.reconstructNormals,
      "0:no Normals 1:local coordinate")

  (po::Section("General"))
    ("keep", 
      encParams.keepIntermediateFiles, 
      encParams.keepIntermediateFiles, 
      "Keep intermediate files")  
    ("checksum", 
      params.checksum, 
      params.checksum, 
      "Compute checksum")

  (po::Section("Group of frames analysis"))
    ("gofMaxSize", 
      encParams.groupOfFramesMaxSize, 
      encParams.groupOfFramesMaxSize, 
      "Maximum group of frames size")
    ("analyzeGof", 
      encParams.analyzeGof, 
      encParams.analyzeGof, 
      "Analyze group of frames")
  
  (po::Section("Geometry decimate"))
    ("target", 
      encParams.targetTriangleRatio, 
      encParams.targetTriangleRatio, 
      "Target triangle count ratio")
    ("minCCTriangleCount", 
      encParams.minCCTriangleCount, 
      encParams.minCCTriangleCount, 
      "minimum triangle count per connected component")
    ("minPosition", 
      encParams.minPosition, 
      { 0.0, 0.0, 0.0 }, 
      "Min position")
    ("maxPosition", 
      encParams.maxPosition, 
      { 0.0, 0.0, 0.0 }, 
      "Max position")

  (po::Section("Texture parametrization"))
    ("textureParametrizationQuality", 
      encParams.uvOptions, 
      encParams.uvOptions,       
      "Quality level of DEFAULT, FAST or QUALITY")
    ("textureParametrizationMaxCharts", 
      encParams.maxCharts, 
      encParams.maxCharts, 
      "Maximum number of charts to generate")
    ("textureParametrizationMaxStretch", 
      encParams.maxStretch, 
      encParams.maxStretch,       
      "Maximum amount of stretch 0 to 1")
    ("textureParametrizationGutter", 
      encParams.gutter, 
      encParams.gutter, 
      "Gutter width betwen charts in texels")
    ("textureParametrizationWidth", 
      encParams.width, 
      encParams.width,       
      "texture width")
    ("textureParametrizationHeight", 
      encParams.height, 
      encParams.height,       
      "texture height")    
    ("textureBGR444",
      encParams.textureBGR444,
      encParams.textureBGR444,
      "Texture video encoded in BGR444")

   
  (po::Section("Geometry parametrization"))
      ("baseIsSrc", 
      encParams.baseIsSrc, 
      encParams.baseIsSrc, 
      "Base models are src models")
    ("subdivIsBase", 
      encParams.subdivIsBase, 
      encParams.subdivIsBase, 
      "Subdiv models are src models")
    ("subdivInter", 
      encParams.subdivInter, 
      encParams.subdivInter, 
      "Subdiv inter")
    ("subdivInterWithMapping", 
      encParams.subdivInterWithMapping, 
      encParams.subdivInterWithMapping, 
      "Subdiv inter with mapping")
    ("maxAllowedD2PSNRLoss", 
      encParams.maxAllowedD2PSNRLoss, 
      encParams.maxAllowedD2PSNRLoss, 
      "Maximum allowed D2 PSNR Loss")
    ("normalCalcModificationEnable",
      encParams.normalCalcModificationEnable,
      encParams.normalCalcModificationEnable,
      "0: Calculate normal of cloudB from cloudA, 1: Use normal of cloudB(default)")

  (po::Section("Intra geometry parametrization"))
    ("ai_sdeform", 
      intraGeoParams.applySmoothingDeform, 
      intraGeoParams.applySmoothingDeform, 
      "Apply deformation refinement stage")
    ("ai_subdivIt", 
      intraGeoParams.geometryParametrizationSubdivisionIterationCount, 
      intraGeoParams.geometryParametrizationSubdivisionIterationCount, 
      "Subdivision iteration count")
    ("ai_forceNormalDisp", 
      intraGeoParams.initialDeformForceNormalDisplacement, 
      intraGeoParams.initialDeformForceNormalDisplacement,       
      "Force displacements to aligned with the surface normals")
    ("ai_unifyVertices", 
      intraGeoParams.applyVertexUnification, 
      intraGeoParams.applyVertexUnification, 
      "Unify duplicated vertices")
    ("ai_deformNNCount", 
      intraGeoParams.initialDeformNNCount, 
      intraGeoParams.initialDeformNNCount, 
      "Number of nearest neighbours used during the initial deformation stage")
    ("ai_deformNormalThres",
      intraGeoParams.initialDeformNormalDeviationThreshold, 
      intraGeoParams.initialDeformNormalDeviationThreshold, 
      "Maximum allowed normal deviation during the initial deformation stage")
    ("ai_sampIt", 
      intraGeoParams.geometrySamplingSubdivisionIterationCount, 
      intraGeoParams.geometrySamplingSubdivisionIterationCount, 
      "Number of subdivision iterations used for geometry sampling")
    ("ai_fitIt", 
      intraGeoParams.geometryFittingIterationCount,
      intraGeoParams.geometryFittingIterationCount, 
      "Number of iterations used during the deformation refinement stage")
    ("ai_smoothCoeff", 
      intraGeoParams.geometrySmoothingCoeffcient, 
      intraGeoParams.geometrySmoothingCoeffcient, 
      "Initial smoothing coefficient used to smooth the deformed mesh "
      "during deformation refinement")
    ("ai_smoothDecay", 
      intraGeoParams.geometrySmoothingCoeffcientDecayRatio, 
      intraGeoParams.geometrySmoothingCoeffcientDecayRatio, 
      "Decay factor applied to intial smoothing coefficient after every "
      "iteration of deformation refinement")
    ("ai_smoothMissedCoeff", 
      intraGeoParams.geometryMissedVerticesSmoothingCoeffcient, 
      intraGeoParams.geometryMissedVerticesSmoothingCoeffcient, 
      "Smoothing coefficient applied to the missed vertices")
    ("ai_smoothMissedIt", 
      intraGeoParams.geometryMissedVerticesSmoothingIterationCount,
      intraGeoParams.geometryMissedVerticesSmoothingIterationCount,
      "Number of iterations when smoothing the positions of the missed vertices")
    ("ai_smoothMethod", 
      intraGeoParams.smoothDeformSmoothingMethod, 
      intraGeoParams.smoothDeformSmoothingMethod, 
      "Smoothing method to be applied when smoothing the deformed mesh during"
      "the deformation refinement stage")
    ("ai_deformUpdateNormals", 
      intraGeoParams.smoothDeformUpdateNormals, 
      intraGeoParams.smoothDeformUpdateNormals, 
      "Recompute normals after each iteration of deformation refinement")
    ("ai_deformFlipThres", 
      intraGeoParams.smoothDeformTriangleNormalFlipThreshold, 
      intraGeoParams.smoothDeformTriangleNormalFlipThreshold, 
      "Threshold to detect triangle normals flip")
    ("ai_useInitialGeom", 
      intraGeoParams.smoothingDeformUseInitialGeometry, 
      intraGeoParams.smoothingDeformUseInitialGeometry, 
      "Use the initial geometry during the the deformation refinement stage")
    ("ai_fitSubdiv", 
      intraGeoParams.fitSubdivisionSurface, 
      intraGeoParams.fitSubdivisionSurface, 
      "Update the positions of the decimated mesh to minimize displacements "
      "between the subdivided mesh and the deformed mesh")
    ("ai_smoothMotion", 
      intraGeoParams.smoothingDeformSmoothMotion, 
      intraGeoParams.smoothingDeformSmoothMotion, 
      "Apply smoothing to motion instead of vertex positions")

  (po::Section("Inter geometry parametrization"))
    ("ld_sdeform", 
      interGeoParams.applySmoothingDeform, 
      interGeoParams.applySmoothingDeform, 
      "Apply deformation refinement stage")
    ("ld_subdivIt", 
      interGeoParams.geometryParametrizationSubdivisionIterationCount, 
      interGeoParams.geometryParametrizationSubdivisionIterationCount, 
      "Subdivision iteration count")
    ("ld_forceNormalDisp", 
      interGeoParams.initialDeformForceNormalDisplacement, 
      interGeoParams.initialDeformForceNormalDisplacement,       
      "Force displacements to aligned with the surface normals")
    ("ld_unifyVertices", 
      interGeoParams.applyVertexUnification, 
      interGeoParams.applyVertexUnification, 
      "Unify duplicated vertices")
    ("ld_deformNNCount", 
      interGeoParams.initialDeformNNCount, 
      interGeoParams.initialDeformNNCount, 
      "Number of nearest neighbours used during the initial deformation stage")
    ("ld_deformNormalThres",
      interGeoParams.initialDeformNormalDeviationThreshold, 
      interGeoParams.initialDeformNormalDeviationThreshold, 
      "Maximum allowed normal deviation during the initial deformation stage")
    ("ld_sampIt", 
      interGeoParams.geometrySamplingSubdivisionIterationCount, 
      interGeoParams.geometrySamplingSubdivisionIterationCount, 
      "Number of subdivision iterations used for geometry sampling")
    ("ld_fitIt", 
      interGeoParams.geometryFittingIterationCount,
      interGeoParams.geometryFittingIterationCount, 
      "Number of iterations used during the deformation refinement stage")
    ("ld_smoothCoeff", 
      interGeoParams.geometrySmoothingCoeffcient, 
      interGeoParams.geometrySmoothingCoeffcient, 
      "Initial smoothing coefficient used to smooth the deformed mesh "
      "during deformation refinement")
    ("ld_smoothDecay", 
      interGeoParams.geometrySmoothingCoeffcientDecayRatio, 
      interGeoParams.geometrySmoothingCoeffcientDecayRatio, 
      "Decay factor applied to intial smoothing coefficient after every "
      "iteration of deformation refinement")
    ("ld_smoothMissedCoeff", 
      interGeoParams.geometryMissedVerticesSmoothingCoeffcient, 
      interGeoParams.geometryMissedVerticesSmoothingCoeffcient, 
      "Smoothing coefficient applied to the missed vertices")
    ("ld_smoothMissedIt", 
      interGeoParams.geometryMissedVerticesSmoothingIterationCount,
      interGeoParams.geometryMissedVerticesSmoothingIterationCount,
      "Number of iterations when smoothing the positions of the missed vertices")
    ("ld_smoothMethod", 
      interGeoParams.smoothDeformSmoothingMethod, 
      interGeoParams.smoothDeformSmoothingMethod, 
      "Smoothing method to be applied when smoothing the deformed mesh during"
      "the deformation refinement stage")
    ("ld_deformUpdateNormals", 
      interGeoParams.smoothDeformUpdateNormals, 
      interGeoParams.smoothDeformUpdateNormals, 
      "Recompute normals after each iteration of deformation refinement")
    ("ld_deformFlipThres", 
      interGeoParams.smoothDeformTriangleNormalFlipThreshold, 
      interGeoParams.smoothDeformTriangleNormalFlipThreshold, 
      "Threshold to detect triangle normals flip")
    ("ld_useInitialGeom", 
      interGeoParams.smoothingDeformUseInitialGeometry, 
      interGeoParams.smoothingDeformUseInitialGeometry, 
      "Use the initial geometry during the the deformation refinement stage")
    ("ld_fitSubdiv", 
      interGeoParams.fitSubdivisionSurface, 
      interGeoParams.fitSubdivisionSurface, 
      "Update the positions of the decimated mesh to minimize displacements "
      "between the subdivided mesh and the deformed mesh")
    ("ld_smoothMotion", 
      interGeoParams.smoothingDeformSmoothMotion, 
      interGeoParams.smoothingDeformSmoothMotion, 
      "Apply smoothing to motion instead of vertex positions")

  (po::Section("Lifting"))
    ("liftingIterationCount",
      encParams.liftingSubdivisionIterationCount,
      encParams.liftingSubdivisionIterationCount,
      "Lifting subdivision iteration count")
    ("liftingLevelOfDetailInverseScale",
      encParams.liftingLevelOfDetailInverseScale,
       {2.0, 2.0, 2.0},
      "Quantization LoD inverse scale for displacements")
    ("liftingQP",
      encParams.liftingQP,
       {16, 28, 28},
      "Quantization parameter for displacements")
    ("liftingBias",
      encParams.liftingBias,
      {1./3., 1./3., 1./3},
      "Quantization bias for displacements")
    ("lodDisplacementQuantizationFlag",
      encParams.lodDisplacementQuantizationFlag,
      encParams.lodDisplacementQuantizationFlag,
      "Use quantization parameter per LoD for displacements")
    ("liftingQP2",
      liftingQP2,
      {16, 28, 28, 22, 34, 34, 28, 40, 40},
      "Quantization parameter for displacements")
  
  (po::Section("Base mesh"))
    ("baseMeshPositionBitDepth", 
      encParams.qpPosition, 
      encParams.qpPosition, 
      "Quantization bits for base mesh positions")
    ("baseMeshTexCoordBitDepth", 
      encParams.qpTexCoord, 
      encParams.qpTexCoord, 
      "Quantization bits for base mesh texture coordinates")  
    ("invertOrientation", 
      encParams.invertOrientation, 
      encParams.invertOrientation, 
      "Invert triangles orientation")
    ("unifyVertices", 
      encParams.unifyVertices, 
      encParams.unifyVertices, 
      "Unify duplicated vertices")
    ("meshCodecId", 
      encParams.meshCodecId, 
      encParams.meshCodecId, 
      "Mesh codec id")
    ("predCoder",
        encParams.predCoder,
        encParams.predCoder,
        "EB pred coder")
    ("topoCoder",
        encParams.topoCoder,
        encParams.topoCoder,
        "EB topo coder")
    ("baseMeshDeduplicatePositions",
        encParams.baseMeshDeduplicatePositions,
        encParams.baseMeshDeduplicatePositions,
        "base mesh deduplicate positions")
    ("dracoUsePosition", 
      encParams.dracoUsePosition, 
      encParams.dracoUsePosition, 
      "Draco use position")
    ("dracoUseUV", 
      encParams.dracoUseUV, 
      encParams.dracoUseUV, 
      "Draco use UV")
    ("dracoMeshLossless", 
      encParams.dracoMeshLossless, 
      encParams.dracoMeshLossless, 
      "draco mesh lossless")
  
  (po::Section("Motion"))
    ("motionGroupSize",
      encParams.motionGroupSize,
      encParams.motionGroupSize,
      "Motion field coding vertices group size")
    ("motionWithoutDuplicatedVertices",
      encParams.motionWithoutDuplicatedVertices,
      encParams.motionWithoutDuplicatedVertices,
      "Motion field coding by integrating duplicated vertices in reference frames")

  (po::Section("Geometry video"))
    ("geometryVideoCodecId",
      encParams.geometryVideoCodecId,
      encParams.geometryVideoCodecId,
      "Geometry video codec id")
    ("geometryVideoEncoderConfig", 
      encParams.geometryVideoEncoderConfig, 
      encParams.geometryVideoEncoderConfig, 
      "Geometry video config file")
        
  (po::Section("Displacements"))
    ("encodeDisplacements", 
      encParams.encodeDisplacements, 
      encParams.encodeDisplacements, 
      "Displacements coding mode:\n"
      "  0: no displacements coding\n"
      "  1: arithmetic coding\n"
      "  2: video coding")
    ("applyOneDimensionalDisplacement",
      encParams.applyOneDimensionalDisplacement,
      encParams.applyOneDimensionalDisplacement,
      "Apply one dimensional displacement")
    ("interpolateDisplacementNormals",
      encParams.interpolateDisplacementNormals,
      encParams.interpolateDisplacementNormals,
      "Interpolate displacement normals")
    ("addReconstructedNormals",
      encParams.addReconstructedNormals,
      encParams.addReconstructedNormals,
      "add reconstructed normals")    
    ("displacementReversePacking",
      encParams.displacementReversePacking,
      encParams.displacementReversePacking,
      "Displacement reverse packing")
    ("displacementUse420",
      encParams.displacementUse420,
      encParams.displacementUse420,
      "Displacement use 4:2:0 encoding")
    ("subBlockSize", 
      encParams.subBlockSize, 
      encParams.subBlockSize, 
      "Subblock size for arithmetic coding")
      
  (po::Section("Transfer texture"))    
    ("textureTransferEnable", 
      encParams.textureTransferEnable, 
      encParams.textureTransferEnable, 
      "Texture transfer enable")
    ("textureTransferSamplingSubdivisionIterationCount", 
      encParams.textureTransferSamplingSubdivisionIterationCount, 
      encParams.textureTransferSamplingSubdivisionIterationCount, 
      "Texture transfer sampling subdivision iteration count")
    ("textureTransferPaddingBoundaryIterationCount", 
      encParams.textureTransferPaddingBoundaryIterationCount, 
      encParams.textureTransferPaddingBoundaryIterationCount, 
      "Texture transfer padding boundary iteration count")
    ("textureTransferPaddingDilateIterationCount", 
      encParams.textureTransferPaddingDilateIterationCount, 
      encParams.textureTransferPaddingDilateIterationCount, 
      "Texture transfer padding dilate iteration count")
    ("textureTransferPaddingMethod", 
      encParams.textureTransferPaddingMethod, 
      encParams.textureTransferPaddingMethod, 
      "Texture transfer padding method: ")
    ("textureTransferPaddingSparseLinearThreshold", 
      encParams.textureTransferPaddingSparseLinearThreshold, 
      encParams.textureTransferPaddingSparseLinearThreshold, 
      "Texture transfer padding sparse linear threshold")
    ("textureTransferBasedPointcloud", 
      encParams.textureTransferBasedPointcloud, 
      encParams.textureTransferBasedPointcloud, 
      "Texture transfer padding sparse linear threshold")
    ("textureTransferPreferUV", 
      encParams.textureTransferPreferUV, 
      encParams.textureTransferPreferUV, 
      "Texture transfer prefer UV")
    ("textureTransferWithMap", 
      encParams.textureTransferWithMap, 
      encParams.textureTransferWithMap, 
      "Texture transfer with map for reconstructe sampling" )
    ("textureTransferWithMapSource", 
      encParams.textureTransferWithMapSource, 
      encParams.textureTransferWithMapSource, 
      "Texture transfer with map for source sampling" )
    ("textureTransferMapSamplingParam", 
      encParams.textureTransferMapSamplingParam, 
      encParams.textureTransferMapSamplingParam, 
      "Texture transfer map sampling param")
    ("textureTransferMethod", 
      encParams.textureTransferMethod, 
      encParams.textureTransferMethod, 
      "Texture transfer method: 0: pcc 1: simple 2: simple new, 3: optimized")
    ("textureTransferGridSize", 
      encParams.textureTransferGridSize, 
      encParams.textureTransferGridSize,       
      "textureTransferGridSize")
    ("textureTransferMapProjDim", 
      encParams.textureTransferMapProjDim, 
      encParams.textureTransferMapProjDim,       
      "textureTransferMapProjDim")
    ("textureTransferSigma", 
      encParams.textureTransferSigma, 
      encParams.textureTransferSigma,
      "textureTransferSigma")
    ("textureTransferMapNumPoints", 
      encParams.textureTransferMapNumPoints, 
      encParams.textureTransferMapNumPoints,
      "textureTransferMapNumPoints")
    ("textureTransferCopyBackground",
      encParams.textureTransferCopyBackground,
      encParams.textureTransferCopyBackground,
      "textureTransferMapNumPoints")
  (po::Section("Motion coding"))
    ("maxNumNeighborsMotion",
      encParams.maxNumNeighborsMotion,
      encParams.maxNumNeighborsMotion,
      "Max number of vertex neighbors in motion coding")
  (po::Section("Texture video"))
    ("encodeTextureVideo", 
      encParams.encodeTextureVideo, 
      encParams.encodeTextureVideo, 
      "Encode texture video")
    ("textureVideoCodecId",
      encParams.textureVideoCodecId,
      encParams.textureVideoCodecId,
      "Texture video codec id")
    ("textureVideoEncoderConfig", 
      encParams.textureVideoEncoderConfig, 
      encParams.textureVideoEncoderConfig, 
      "Texture video encoder configuration file")
    ("textureVideoEncoderConvertConfig", 
      encParams.textureVideoHDRToolEncConfig, 
      encParams.textureVideoHDRToolEncConfig, 
      "HDRTools encode configuration file")
    ("textureVideoDecoderConvertConfig", 
      encParams.textureVideoHDRToolDecConfig, 
      encParams.textureVideoHDRToolDecConfig, 
      "HDRTools decode configuration file")    
    ("textureVideoDownsampleFilter", 
      encParams.textureVideoDownsampleFilter, 
      encParams.textureVideoDownsampleFilter, 
      "Chroma downsample filter in [0;22]")
    ("textureVideoUpsampleFilter", 
      encParams.textureVideoUpsampleFilter, 
      encParams.textureVideoUpsampleFilter, 
      "Chroma upsample filter in [0;7]")
    ("textureVideoFullRange", 
      encParams.textureVideoFullRange, 
      encParams.textureVideoFullRange, 
      "Texture video range: 0: limited, 1: full")
    ("textureVideoQP", 
      encParams.textureVideoQP, 
      encParams.textureVideoQP, 
      "Quantization parameter for texture video")
    ("textureVideoWidth", 
      encParams.textureWidth, 
      encParams.textureWidth, 
      "Output texture width")
    ("textureVideoHeight", 
      encParams.textureHeight, 
      encParams.textureHeight, 
      "Output texture height")

  (po::Section("Bitstreams"))
    ("forceSsvhUnitSizePrecision", 
      encParams.forceSsvhUnitSizePrecision, 
      encParams.forceSsvhUnitSizePrecision, 
      "force SampleStreamV3CUnit size precision bytes")

  (po::Section("Metrics"))
    ("pcc",
      metParams.computePcc,
      metParams.computePcc,
      "Compute pcc metrics")
    ("ibsm",
      metParams.computeIbsm,
      metParams.computeIbsm,
      "Compute ibsm metrics")
    ("pcqm",
      metParams.computePcqm,
      metParams.computePcqm,
      "Compute pcqm metrics")
    ("gridSize",
      metParams.gridSize,
      metParams.gridSize,
      "Grid size")
    ("resolution",
      metParams.resolution,
      metParams.resolution,
      "Resolution")
    ("pcqmRadiusCurvature",
      metParams.pcqmRadiusCurvature,
      metParams.pcqmRadiusCurvature,
      "PCQM radius curvature")
    ("pcqmThresholdKnnSearch",
      metParams.pcqmThresholdKnnSearch,
      metParams.pcqmThresholdKnnSearch,
      "PCQM threshold Knn search")
    ("pcqmRadiusFactor",
      metParams.pcqmRadiusFactor,
      metParams.pcqmRadiusFactor,
      "PCQM radius factor")  

  (po::Section("Caching"))    
    ("cachingDirectory", 
      encParams.cachingDirectory, 
      encParams.cachingDirectory, 
      "Caching directory")
    ("cachingPoint", 
      encParams.cachingPoint, 
      encParams.cachingPoint, 
      "Caching points: \n"
      "  - 0/none    : off \n"
      "  - 1/simplify: symplify\n"
      "  - 2/uvatlas : uvatlas\n"
      "  - 3/subdiv  : subdiv \n"
      "  - 255/create: create caching files")
  ;
  /* clang-format on */

  po::setDefaults(opts);
  po::ErrorReporter             err;
  const std::list<const char*>& argv_unhandled =
    po::scanArgv(opts, argc, (const char**)argv, err);

  for (const auto* const arg : argv_unhandled) {
    err.warn() << "Unhandled argument ignored: " << arg << '\n';
  }

  if (argc == 1 || print_help) {
    std::cout << "usage: " << argv[0] << " [arguments...] \n\n";
    po::doHelp(std::cout, opts, 78);
    return false;
  }

  if (params.compressedStreamPath.empty()) {
    err.error() << "compressed input/output not specified\n";
  }

  if (params.inputMeshPath.empty()) {
    err.error() << "input mesh not specified\n";
  }

  if (params.inputTexturePath.empty()) {
    err.error() << "input texture not specified\n";
  }

  if (encParams.geometryVideoEncoderConfig.empty()
      && encParams.encodeDisplacements == 2) {
    err.error() << "geometry video config not specified\n";
  }

  if (encParams.textureVideoEncoderConfig.empty()) {
    err.error() << "texture video encoder config not specified\n";
  }

  if (encParams.lodDisplacementQuantizationFlag) {
    auto lodCount        = encParams.liftingSubdivisionIterationCount + 1;
    auto liftingQP2Count = static_cast<int32_t>(liftingQP2.size());
    if (liftingQP2.empty()) {
      err.error() << "liftingQP2 not specified\n";
    } else if (liftingQP2Count != lodCount * 3) {
      err.error() << "the length of liftingQP2 not " << lodCount * 3 << "\n";
    } else {
      encParams.liftingQuantizationParametersPerLevelOfDetails.clear();
      for (int32_t i = 0; i < liftingQP2Count; i += 3) {
        encParams.liftingQuantizationParametersPerLevelOfDetails.push_back(
          {liftingQP2[i + 0], liftingQP2[i + 1], liftingQP2[i + 2]});
      }
    }
  }

  if (err.is_errored) { return false; }

  // Dump the complete derived configuration
  po::dumpCfgBySection(std::cout, opts);

  // Copy duplicate parameters
  for (int k = 0; k < 3; k++) {
    metParams.minPosition[k] = encParams.minPosition[k];
    metParams.maxPosition[k] = encParams.maxPosition[k];
  }
  metParams.qp           = encParams.bitDepthPosition;
  metParams.qt           = encParams.bitDepthTexCoord;
  metParams.dequantizeUV = encParams.dequantizeUV;
  metParams.verbose      = params.verbose;
  return true;
} catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

bool
compress(const Parameters& params) {
  const auto basePath = vmesh::removeExtension(params.compressedStreamPath);
  const bool computeMetrics = params.metParams.computePcc
                              || params.metParams.computeIbsm
                              || params.metParams.computePcqm;
  vmesh::SampleStreamV3CUnit ssvu;
  vmesh::BitstreamStat       bitstreamStat;
  vmesh::Checksum            checksum;
  vmesh::VMCMetrics          metrics;
#if defined(BITSTREAM_TRACE)
  vmesh::Logger loggerHls;
  loggerHls.initilalize(basePath + "_hls", true);
#endif

  // Generate gof structure
  vmesh::SequenceInfo sequenceInfo;
  sequenceInfo.generate(params.frameCount,
                        params.startFrame,
                        params.encParams.groupOfFramesMaxSize,
                        params.encParams.analyzeGof,
                        params.inputMeshPath);
  if (params.encParams.keepIntermediateFiles)
    sequenceInfo.save(basePath + "_gof.txt");

  // Compress GOF
  for (const auto& gofInfo : sequenceInfo) {
    printf("GOF = %d / %d \n", gofInfo.index_, sequenceInfo.gofCount());
    vmesh::VMCEncoder   encoder;
    vmesh::Sequence     source;
    vmesh::Sequence     reconstruct;
    vmesh::V3cBitstream syntax;
    syntax.setBitstreamStat(bitstreamStat);
    encoder.setKeepFilesPathPrefix(basePath + "_GOF_"
                                   + std::to_string(gofInfo.index_) + "_");
    
    // Load source sequence
    if (!source.load(params.inputMeshPath,
                     params.inputTexturePath,
                     gofInfo.startFrameIndex_,
                     gofInfo.frameCount_)) {
      std::cerr << "Error: can't load source sequence\n";
      return false;
    }

    // Compress meshes
    vmesh::tic( "compress" );
    if (!encoder.compress(
          gofInfo, source, syntax, reconstruct, params.encParams)) {
      std::cerr << "Error: can't compress group of frames!\n";
      return false;
    }
    auto end                       = std::chrono::steady_clock::now();
    vmesh::toc( "compress" );

    // Create V3C sample stream units
    vmesh::V3CWriter writer;    
#if defined(BITSTREAM_TRACE)
    writer.setLogger(loggerHls);
#endif
    vmesh::tic( "writer" );
    writer.encode(syntax, ssvu);
    vmesh::toc( "writer" );

    // Save reconstructed models
    if (!reconstruct.save(params.reconstructedMeshPath,
                          params.reconstructedTexturePath,
                          params.reconstructedMaterialLibPath,
                          gofInfo.startFrameIndex_)) {
      std::cerr << "Error: can't save reconstruct sequence \n";
      return false;
    }
    if (params.checksum) { checksum.add(reconstruct); }
    if (computeMetrics) {
      metrics.compute(source, reconstruct, params.metParams);
    }
    for (auto& rec : reconstruct.meshes())
      bitstreamStat.getFaceCount() += rec.triangleCount();
  }

  // Write V3C bistream
  vmesh::V3CWriter writer;
  vmesh::Bitstream bitstream;
#if defined(BITSTREAM_TRACE)
  vmesh::Logger loggerV3c;
  loggerV3c.initilalize(basePath + "_v3c", true);
  bitstream.setLogger(loggerV3c);
  bitstream.setTrace(true);
  writer.setLogger(loggerV3c);
#endif
  size_t headerSize =
    writer.write(ssvu, bitstream, params.encParams.forceSsvhUnitSizePrecision);
  bitstreamStat.incrHeader(headerSize);
  if (!bitstream.save(params.compressedStreamPath)) {
    std::cerr << "Error: can't save compressed bitstream!\n";
    return false;
  }

  // Write checksums
  if (params.checksum) {
    checksum.write(basePath + ".checksum");
    checksum.print();
  }

  // Display stat: metrics, duractions, bitstreams, memory and face counts
  if (computeMetrics) {
    printf("\n------- All frames metrics -----------\n");
    metrics.display();
    printf("---------------------------------------\n");    
  }
  bitstreamStat.trace();
  printf("\nAll frames have been encoded. \n");
  return true;
}

//============================================================================

int
main(int argc, char* argv[]) {
  std::cout << "MPEG VMESH version " << ::vmesh::version << '\n';

  // this is mandatory to print floats with full precision
  std::cout.precision(std::numeric_limits<float>::max_digits10);

  Parameters params;
  if (!parseParameters(argc, argv, params)) { return 1; }

  if (params.verbose) { vmesh::vout.rdbuf(std::cout.rdbuf()); }

  if (!compress(params)) {
    std::cerr << "Error: can't compress animation!\n";
    return 1;
  }

  return 0;
}
