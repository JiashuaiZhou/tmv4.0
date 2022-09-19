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

#include <chrono>
#include <program-options-lite/program_options_lite.h>

#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "util/bitstream.hpp"
#include "util/memory.hpp"
#include "encoder.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "checksum.hpp"
#include "metrics.hpp"
#include "vmcStats.hpp"
#include "sequenceInfo.hpp"

//============================================================================

struct Parameters {
  std::string                 inputMeshPath                = {};
  std::string                 inputTexturePath             = {};
  std::string                 baseMeshPath                 = {};
  std::string                 subdivMeshPath               = {};
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
  namespace po         = df::program_options_lite;
  bool  print_help     = false;
  auto& encParams      = params.encParams;
  auto& metParams      = params.metParams;
  auto& intraGeoParams = params.encParams.intraGeoParams;
  auto& interGeoParams = params.encParams.interGeoParams;
  /* clang-format off */
  po::Options opts;
  opts.addOptions()
  (po::Section("Common"))
    ("help", print_help, false, "This help text")
    ("config,c", po::parseConfigFile, "Configuration file name")
    ("verbose,v", params.verbose, true, "Verbose output")

  (po::Section("Input"))
    ("imesh", 
      params.inputMeshPath, 
      params.inputMeshPath, 
      "Input mesh")
    ("itex", 
      params.inputTexturePath, 
      params.inputTexturePath, 
      "Input texture")

  (po::Section("Output"))
    ("compressed",
      params.compressedStreamPath, 
      params.compressedStreamPath, 
      "Compressed bitstream")
    ("recmat", 
      params.reconstructedMaterialLibPath, 
      params.reconstructedMaterialLibPath, 
      "Reconstructed materials")
    ("recmesh", 
      params.reconstructedMeshPath, 
      params.reconstructedMeshPath, 
      "Reconstructed mesh")
    ("rectex", 
      params.reconstructedTexturePath, 
      params.reconstructedTexturePath, 
      "Reconstructed texture")

  (po::Section("General"))
    ("fstart", 
      params.startFrame, 
      params.startFrame, 
      "First frame number")
    ("fcount", 
      params.frameCount, 
      params.frameCount, 
      "Number of frames")
    ("framerate", 
      params.framerate, 
      params.framerate, 
      "Frame rate")  
    ("keep", 
      encParams.keepIntermediateFiles, 
      encParams.keepIntermediateFiles, 
      "Keep intermediate files")  
    ("checksum", 
      params.checksum, 
      params.checksum, 
      "Compute checksum")

  (po::Section("Gof analysis"))
    ("gofmax", 
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
    ("qt", 
      encParams.texCoordQuantizationBits, 
      encParams.texCoordQuantizationBits, 
      "texture coordinate quantization bits")
    ("cctcount", 
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
    ("quality", 
      encParams.uvOptions, 
      encParams.uvOptions,       
      "Quality level of DEFAULT, FAST or QUALITY")
    ("maxCharts", 
      encParams.maxCharts, 
      encParams.maxCharts, 
      "Maximum number of charts to generate")
    ("stretch", 
      encParams.maxStretch, 
      encParams.maxStretch,       
      "Maximum amount of stretch 0 to 1")
    ("gutter", 
      encParams.gutter, 
      encParams.gutter, 
      "Gutter width betwen charts in texels")
    ("width", 
      encParams.width, 
      encParams.width,       
      "texture width")
    ("height", 
      encParams.height, 
      encParams.height,       
      "texture height")
   
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

  (po::Section("Mesh"))
    ("gqp", 
      encParams.qpPosition, 
      encParams.qpPosition, 
      "Quantization bits for base mesh positions")
    ("tqp", 
      encParams.qpTexCoord, 
      encParams.qpTexCoord, 
      "Quantization bits for base mesh texture coordinates")
    ("gdepth", 
      encParams.bitDepthPosition, 
      encParams.bitDepthPosition, 
      "Input positions bit depth")
    ("tdepth", 
      encParams.bitDepthTexCoord, 
      encParams.bitDepthTexCoord, 
      "Input texture coordinates bit depth")    
    ("invorient", 
      encParams.invertOrientation, 
      encParams.invertOrientation, 
      "Invert triangles orientation")
    ("unifvertices", 
      encParams.unifyVertices, 
      encParams.unifyVertices, 
      "Unify duplicated vertices")
    ("normuv", 
      encParams.normalizeUV, 
      encParams.normalizeUV, 
      "Normalize uv texture coordinates")    
    ("meshCodecId", 
      encParams.meshCodecId, 
      encParams.meshCodecId, 
      "Mesh codec id")    

  (po::Section("Geometry video"))
    ("geometryVideoCodecId",
      encParams.geometryVideoCodecId,
      encParams.geometryVideoCodecId,
      "Geometry video codec id")
    ("gvencconfig", 
      encParams.geometryVideoEncoderConfig, 
      encParams.geometryVideoEncoderConfig, 
      "Geometry video cfg")    
        
  (po::Section("Texture video"))
    ("textureVideoCodecId",
      encParams.textureVideoCodecId,
      encParams.textureVideoCodecId,
      "Texture video codec id")
    ("tvencconfig", 
      encParams.textureVideoEncoderConfig, 
      encParams.textureVideoEncoderConfig, 
      "Texture video cfg")
    ("cscencconfig", 
      encParams.textureVideoHDRToolEncConfig, 
      encParams.textureVideoHDRToolEncConfig, 
      "HDRTools encode cfg")
    ("cscdecconfig", 
      encParams.textureVideoHDRToolDecConfig, 
      encParams.textureVideoHDRToolDecConfig, 
      "HDRTools decode cfg")    
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
      "Texture video range")
    ("tvqp", 
      encParams.textureVideoQP, 
      encParams.textureVideoQP, 
      "Quantization parameter for texture video")
  
  (po::Section("Displacements"))
    ("encdisp", 
      encParams.encodeDisplacementsVideo, 
      encParams.encodeDisplacementsVideo, 
      "Encode displacements video")
    ("enctex", 
      encParams.encodeTextureVideo, 
      encParams.encodeTextureVideo, 
      "Encode texture video")
      
  (po::Section("Lifting"))
    ("liftingIt", 
      encParams.liftingSubdivisionIterationCount, 
      encParams.liftingSubdivisionIterationCount,       
      "Lifting subdivision iteration count")
    ("dqp", 
      encParams.liftingQuantizationParameters, 
       {16, 28, 28},
      "Quantization parameter for displacements")
    ("dqb", 
      encParams.liftingQuantizationBias,
      {1./3., 1./3., 1./3},
      "Quantization bias for displacements")

  (po::Section("Texture transfer"))
    ("texwidth", 
      encParams.textureWidth, 
      encParams.textureWidth, 
      "Output texture width")
    ("texheight", 
      encParams.textureHeight, 
      encParams.textureHeight, 
      "Output texture height")
  
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
    ("minPosition",
      metParams.minPosition,
      {0, 0, 0},
      "Min position")
    ("maxPosition",
      metParams.maxPosition,
      {0, 0, 0},
      "Max position")
    ("qp",
      metParams.qp,
      metParams.qp,
      "qp")
    ("qt",
      metParams.qt,
      metParams.qt,
      "qt")
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
    ("fstart",
      metParams.frameStart, 
      metParams.frameStart, 
      "Metric frame start")
    ("fcount",
      metParams.frameCount, 
      metParams.frameCount, 
      "Metric frame count")
    ("srcMesh",
      metParams.srcMeshPath,
      metParams.srcMeshPath,
      "Metric Source mesh path")
    ("srcTex", 
      metParams.srcTexturePath, 
      metParams.srcTexturePath,
      "Source texture path")

  (po::Section("Caching"))    
    ("ignoreTextureEncoding", 
      encParams.ignoreTextureEncoding, 
      encParams.ignoreTextureEncoding, 
      "Ignore texture encoding")
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
      "  - 1/uvatlas : symplify\n"
      "  - 2/subdiv  : subdiv \n"
      "  - 255/create: create caching files")

  (po::Section("Bug fix"))    
    ("forceCoordTruncation", 
      encParams.forceCoordTruncation, 
      encParams.forceCoordTruncation, 
      "Force truncation of the precision of the intermediate mesh files")
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

  if (encParams.geometryVideoEncoderConfig.empty()) {
    err.error() << "geometry video config not specified\n";
  }

  if (encParams.textureVideoEncoderConfig.empty()) {
    err.error() << "texture video encoder config not specified\n";
  }

  if (err.is_errored) { return false; }

  // Dump the complete derived configuration
  std::cout << "+ Configuration parameters\n";
  po::dumpCfg(std::cout, opts, "Common", 4);
  po::dumpCfg(std::cout, opts, "Input", 4);
  po::dumpCfg(std::cout, opts, "Output", 4);
  po::dumpCfg(std::cout, opts, "General", 4);
  po::dumpCfg(std::cout, opts, "Geometry decimate", 4);
  po::dumpCfg(std::cout, opts, "Texture parametrization", 4);
  po::dumpCfg(std::cout, opts, "Geometry parametrization", 4);
  po::dumpCfg(std::cout, opts, "Intra geometry parametrization", 4);
  if (encParams.subdivInter) {
    po::dumpCfg(std::cout, opts, "Inter geometry parametrization", 4);
  }
  po::dumpCfg(std::cout, opts, "Mesh", 4);
  po::dumpCfg(std::cout, opts, "Geometry video", 4);
  po::dumpCfg(std::cout, opts, "Texture video", 4);
  po::dumpCfg(std::cout, opts, "Displacements", 4);
  po::dumpCfg(std::cout, opts, "Lifting", 4);
  po::dumpCfg(std::cout, opts, "Metrics", 4);
  po::dumpCfg(std::cout, opts, "Caching", 4);
  po::dumpCfg(std::cout, opts, "Bug fix", 4);
  std::cout << '\n';
  return true;
} catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}
//----------------------------------------------------------------------------

int32_t
loadGroupOfFrames(const vmesh::VMCGroupOfFramesInfo& gofInfo,
                  vmesh::VMCGroupOfFrames&           gof,
                  const Parameters&                  params) {
  const auto startFrame = gofInfo.startFrameIndex_;
  const auto frameCount = gofInfo.frameCount_;
  const auto lastFrame  = startFrame + frameCount - 1;
  std::cout << "Loading group of frames (" << startFrame << '-' << lastFrame
            << ") ";
  gof.resize(frameCount);
  for (int f = startFrame; f <= lastFrame; ++f) {
    const auto nameInputTexture = vmesh::expandNum(params.inputTexturePath, f);
    const auto findex           = f - startFrame;
    auto&      frame            = gof.frames[findex];
    std::cout << '.' << std::flush;
    if (!frame.input.load(vmesh::expandNum(params.inputMeshPath, f))
        || !LoadImage(nameInputTexture, frame.inputTexture)) {
      printf("Error loading frame %d / %d \n", f, frameCount);
      return -1;
    }
  }
  std::cout << "\n" << std::flush;
  return 0;
}

//============================================================================

int32_t
saveGroupOfFrames(const vmesh::VMCGroupOfFramesInfo& gofInfo,
                  vmesh::VMCGroupOfFrames&           gof,
                  const Parameters&                  params) {
  int ret = 0;
  if (!params.reconstructedMeshPath.empty()
      && !params.reconstructedTexturePath.empty()
      && !params.reconstructedMaterialLibPath.empty()) {
    for (int f = 0; f < gofInfo.frameCount_; ++f) {
      const auto n      = gofInfo.startFrameIndex_ + f;
      auto       strObj = vmesh::expandNum(params.reconstructedMeshPath, n);
      auto       strTex = vmesh::expandNum(params.reconstructedTexturePath, n);
      auto strMat = vmesh::expandNum(params.reconstructedMaterialLibPath, n);
      if (!SaveImage(strTex, gof[f].outputTexture)) { ret = -1; }
      vmesh::Material<double> material;
      material.texture = vmesh::basename(strTex);
      if (!material.save(strMat)) { ret = -1; }
      gof[f].rec.setMaterialLibrary(vmesh::basename(strMat));
      if (!gof[f].rec.save(strObj)) { ret = -1; }
    }
  }
  return ret;
}

//============================================================================

int32_t
compress(const Parameters& params) {
  // Generate gof structure
  vmesh::SequenceInfo sequenceInfo;
  sequenceInfo.generate(params.frameCount,
                        params.startFrame,
                        params.encParams.groupOfFramesMaxSize,
                        params.encParams.analyzeGof,
                        params.inputMeshPath);
  std::string keepFilesPathPrefix = "";
  if (params.encParams.keepIntermediateFiles) {
    keepFilesPathPrefix = vmesh::dirname(params.compressedStreamPath);
    auto gofPath        = keepFilesPathPrefix + "gof.txt";
    sequenceInfo.save(gofPath);
  }

  vmesh::Bitstream  bitstream;
  vmesh::VMCStats   totalStats;
  vmesh::Checksum   checksum;
  vmesh::VMCMetrics metrics;
  auto&             metParams = params.metParams;
  for (int g = 0; g < sequenceInfo.gofCount(); ++g) {
    vmesh::VMCEncoder       encoder;
    vmesh::VMCGroupOfFrames gof;
    const auto&             gofInfo = sequenceInfo[g];
    printf("loadGroupOfFrames GOF = %d / %d \n", g, sequenceInfo.gofCount());
    encoder.setKeepFilesPathPrefix(keepFilesPathPrefix);

    // Load group of frame
    if (loadGroupOfFrames(gofInfo, gof, params) != 0) {
      std::cerr << "Error: can't load group of frames!\n";
      return -1;
    }

    // Compress group of frame
    auto start = std::chrono::steady_clock::now();
    if (encoder.compress(gofInfo, gof, bitstream, params.encParams) != 0) {
      std::cerr << "Error: can't compress group of frames!\n";
      return -1;
    }
    auto end                 = std::chrono::steady_clock::now();
    gof.stats.processingTime = end - start;

    // Save reconsctructed models
    if (saveGroupOfFrames(gofInfo, gof, params) != 0) {
      std::cerr << "Error: can't save rec group of frames!\n";
      return -1;
    }
    if (params.checksum)
      for (auto& frame : gof) checksum.add(frame.rec, frame.outputTexture);

    if (metParams.computePcc || metParams.computeIbsm || metParams.computePcqm)
      metrics.compute(gof, metParams);

    totalStats += gof.stats;
    if (vmesh::vout) {
      vmesh::vout << "\n------- Group of frames " << gofInfo.index_
                  << " -----------\n";
      gof.stats.dump("GOF", params.framerate);
      vmesh::vout << "---------------------------------------\n";
    }
  }

  if (params.checksum) {
    checksum.write(vmesh::removeFileExtension(params.compressedStreamPath)
                   + ".checksum");
    checksum.print();
  }

  if (metParams.computePcc || metParams.computeIbsm || metParams.computePcqm) {
    std::cout << "\n------- All frames metrics -----------\n";
    metrics.display();
    std::cout << "---------------------------------------\n";
  }

  std::cout << "\n------- All frames -----------\n";
  totalStats.dump("Sequence", params.framerate);
  std::cout << "Sequence peak memory " << vmesh::getPeakMemory() << " KB\n";
  std::cout << "---------------------------------------\n";

  // save bistream
  if (bitstream.save(params.compressedStreamPath)) {
    std::cerr << "Error: can't save compressed bitstream!\n";
    return -1;
  }
  std::cout << "\nAll frames have been encoded. \n";
  return 0;
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

  if (compress(params) != 0) {
    std::cerr << "Error: can't compress animation!\n";
    return 1;
  }

  return 0;
}
