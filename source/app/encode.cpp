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

#include "util/bitstream.hpp"
#include "encoder.hpp"
#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "vmcStats.hpp"
#include "sequenceInfo.hpp"

//============================================================================

struct Parameters {
  bool verbose = true;
  std::string inputMeshPath = {};
  std::string inputTexturePath = {};
  std::string baseMeshPath = {};
  std::string subdivMeshPath = {};
  std::string groupOfFramesStructurePath = {};
  std::string compressedStreamPath = {};
  std::string reconstructedMeshPath = {};
  std::string reconstructedTexturePath = {};
  std::string reconstructedMaterialLibPath = {};
  int32_t startFrame = 0;
  int32_t frameCount = 1;
  int32_t groupOfFramesMaxSize = 32;
  double framerate = 30.;
  vmesh::VMCEncoderParameters encParams;
};

//============================================================================

static bool
parseParameters(int argc, char* argv[], Parameters& params)
try {
  namespace po = df::program_options_lite;

  bool print_help = false;

  /* clang-format off */
  po::Options opts;
  opts.addOptions()
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
    ("gofmax", 
      params.groupOfFramesMaxSize, 
      params.groupOfFramesMaxSize, 
      "Maximum group of frames size")
    ("keep", 
      params.encParams.keepIntermediateFiles, 
      params.encParams.keepIntermediateFiles, 
      "Keep intermediate files")
    ("intermediateFilesPathPrefix", 
      params.encParams.intermediateFilesPathPrefix, 
      params.encParams.intermediateFilesPathPrefix, 
      "Intermediate files path prefix")

  (po::Section("Geometry decimate"))
    ("target", 
      params.encParams.targetTriangleRatio, 
      params.encParams.targetTriangleRatio, 
      "Target triangle count ratio")
    ("qt", 
      params.encParams.texCoordQuantizationBits, 
      params.encParams.texCoordQuantizationBits, 
      "texture coordinate quantization bits")
    ("cctcount", 
      params.encParams.minCCTriangleCount, 
      params.encParams.minCCTriangleCount, 
      "minimum triangle count per connected component")

  (po::Section("Texture parametrization"))
    ("quality", 
      params.encParams.uvOptions, 
      params.encParams.uvOptions,       
      "Quality level of DEFAULT, FAST or QUALITY")
    ("maxCharts", 
      params.encParams.maxCharts, 
      params.encParams.maxCharts, 
      "Maximum number of charts to generate")
    ("stretch", 
      params.encParams.maxStretch, 
      params.encParams.maxStretch,       
      "Maximum amount of stretch 0 to 1")
    ("gutter", 
      params.encParams.gutter, 
      params.encParams.gutter, 
      "Gutter width betwen charts in texels")
    ("width", 
      params.encParams.width, 
      params.encParams.width,       
      "texture width")
    ("height", 
      params.encParams.height, 
      params.encParams.height,       
      "texture height")
   
  (po::Section("Geometry parametrization"))
    ("sdeform", 
      params.encParams.applySmoothingDeform, 
      params.encParams.applySmoothingDeform, 
      "Apply deformation refinement stage")
    ("it", 
      params.encParams.subdivisionIterationCount, 
      params.encParams.subdivisionIterationCount, 
      "Subdivision iteration count")
    ("forceNormalDisp", 
      params.encParams.initialDeformForceNormalDisplacement, 
      params.encParams.initialDeformForceNormalDisplacement,       
      "Force displacements to aligned with the surface normals")
    ("unifyVertices", 
      params.encParams.applyVertexUnification, 
      params.encParams.applyVertexUnification, 
      "Unify duplicated vertices")
    ("deformNNCount", 
      params.encParams.initialDeformNNCount, 
      params.encParams.initialDeformNNCount, 
      "Number of nearest neighbours used during the initial deformation stage")
    ("deformNormalThres",
      params.encParams.initialDeformNormalDeviationThreshold, 
      params.encParams.initialDeformNormalDeviationThreshold, 
      "Maximum allowed normal deviation during the initial deformation stage")
    ("sampIt", 
      params.encParams.geometrySamplingSubdivisionIterationCount, 
      params.encParams.geometrySamplingSubdivisionIterationCount, 
      "Number of subdivision iterations used for geometry sampling")
    ("fitIt", 
      params.encParams.geometryFittingIterationCount,
      params.encParams.geometryFittingIterationCount, 
      "Number of iterations used during the deformation refinement stage")
    ("smoothCoeff", 
      params.encParams.geometrySmoothingCoeffcient, 
      params.encParams.geometrySmoothingCoeffcient, 
      "Initial smoothing coefficient used to smooth the deformed mesh "
      "during deformation refinement")
    ("smoothDecay", 
      params.encParams.geometrySmoothingCoeffcientDecayRatio, 
      params.encParams.geometrySmoothingCoeffcientDecayRatio, 
      "Decay factor applied to intial smoothing coefficient after every "
      "iteration of deformation refinement")
    ("smoothMissedCoeff", 
      params.encParams.geometryMissedVerticesSmoothingCoeffcient, 
      params.encParams.geometryMissedVerticesSmoothingCoeffcient, 
      "Smoothing coefficient applied to the missed vertices")
    ("smoothMissedIt", 
      params.encParams.geometryMissedVerticesSmoothingIterationCount,
      params.encParams.geometryMissedVerticesSmoothingIterationCount,
      "Number of iterations when smoothing the positions of the missed vertices")
    ("smoothMethod", 
      params.encParams.smoothDeformSmoothingMethod, 
      params.encParams.smoothDeformSmoothingMethod, 
      "Smoothing method to be applied when smoothing the deformed mesh during"
      "the deformation refinement stage")
    ("deformUpdateNormals", 
      params.encParams.smoothDeformUpdateNormals, 
      params.encParams.smoothDeformUpdateNormals, 
      "Recompute normals after each iteration of deformation refinement")
    ("deformFlipThres", 
      params.encParams.smoothDeformTriangleNormalFlipThreshold, 
      params.encParams.smoothDeformTriangleNormalFlipThreshold, 
      "Threshold to detect triangle normals flip")
    ("useInitialGeom", 
      params.encParams.smoothingDeformUseInitialGeometry, 
      params.encParams.smoothingDeformUseInitialGeometry, 
      "Use the initial geometry during the the deformation refinement stage")
    ("fitSubdiv", 
      params.encParams.fitSubdivisionSurface, 
      params.encParams.fitSubdivisionSurface, 
      "Update the positions of the decimated mesh to minimize displacements "
      "between the subdivided mesh and the deformed mesh")
    ("smoothMotion", 
      params.encParams.smoothingDeformSmoothMotion, 
      params.encParams.smoothingDeformSmoothMotion, 
      "Apply smoothing to motion instead of vertex positions")

  (po::Section("Mesh"))
    ("gqp", 
      params.encParams.qpPosition, 
      params.encParams.qpPosition, 
      "Quantization bits for base mesh positions")
    ("tqp", 
      params.encParams.qpTexCoord, 
      params.encParams.qpTexCoord, 
      "Quantization bits for base mesh texture coordinates")
    ("gdepth", 
      params.encParams.bitDepthPosition, 
      params.encParams.bitDepthPosition, 
      "Input positions bit depth")
    ("tdepth", 
      params.encParams.bitDepthTexCoord, 
      params.encParams.bitDepthTexCoord, 
      "Input texture coordinates bit depth")    
    ("invorient", 
      params.encParams.invertOrientation, 
      params.encParams.invertOrientation, 
      "Invert triangles orientation")
    ("unifvertices", 
      params.encParams.unifyVertices, 
      params.encParams.unifyVertices, 
      "Unify duplicated vertices")
    ("normuv", 
      params.encParams.normalizeUV, 
      params.encParams.normalizeUV, 
      "Normalize uv texture coordinates")    

  (po::Section("Geometry video"))
    ("gvencconfig", 
      params.encParams.geometryVideoEncoderConfig, 
      params.encParams.geometryVideoEncoderConfig, 
      "Geometry video cfg")    
        
  (po::Section("Texture video"))
    ("tvencconfig", 
      params.encParams.textureVideoEncoderConfig, 
      params.encParams.textureVideoEncoderConfig, 
      "Texture video cfg")
    ("cscencconfig", 
      params.encParams.textureVideoHDRToolEncConfig, 
      params.encParams.textureVideoHDRToolEncConfig, 
      "HDRTools encode cfg")
    ("cscdecconfig", 
      params.encParams.textureVideoHDRToolDecConfig, 
      params.encParams.textureVideoHDRToolDecConfig, 
      "HDRTools decode cfg")    
    ("tvqp", 
      params.encParams.textureVideoQP, 
      params.encParams.textureVideoQP, 
      "Quantization parameter for texture video")
  
  (po::Section("Displacements"))
    ("encdisp", 
      params.encParams.encodeDisplacementsVideo, 
      params.encParams.encodeDisplacementsVideo, 
      "Encode displacements video")
    ("enctex", 
      params.encParams.encodeTextureVideo, 
      params.encParams.encodeTextureVideo, 
      "Encode texture video")
      
  (po::Section("Lifting"))
    ("it", 
      params.encParams.subdivisionIterationCount, 
      params.encParams.subdivisionIterationCount,       
      "Subdivision iteration count")
    ("dqp", 
      params.encParams.liftingQuantizationParameters, 
       {16, 28, 28},
      "Quantization parameter for displacements")
    ("dqb", 
      params.encParams.liftingQuantizationBias,
      {1./3., 1./3., 1./3},
      "Quantization bias for displacements")

  (po::Section("Texture transfer"))
    ("texwidth", 
      params.encParams.textureWidth, 
      params.encParams.textureWidth, 
      "Output texture width")
    ("texheight", 
      params.encParams.textureHeight, 
      params.encParams.textureHeight, 
      "Output texture height")
  
  (po::Section("Bug fix"))    
    ("forceWriteReadIntermediateModels", 
      params.encParams.forceWriteReadIntermediateModels, 
      params.encParams.forceWriteReadIntermediateModels, 
      "HDRTools decode cfg")
  ;
  /* clang-format on */

  po::setDefaults(opts);
  po::ErrorReporter err;
  const std::list<const char*>& argv_unhandled =
    po::scanArgv(opts, argc, (const char**)argv, err);

  for (const auto arg : argv_unhandled)
    err.warn() << "Unhandled argument ignored: " << arg << '\n';

  if (argc == 1 || print_help) {
    std::cout << "usage: " << argv[0] << " [arguments...] \n\n";
    po::doHelp(std::cout, opts, 78);
    return false;
  }

  if (params.compressedStreamPath.empty())
    err.error() << "compressed input/output not specified\n";

  if (params.inputMeshPath.empty())
    err.error() << "input mesh not specified\n";

  if (params.inputTexturePath.empty())
    err.error() << "input texture not specified\n";

  if (params.encParams.geometryVideoEncoderConfig.empty())
    err.error() << "geometry video config not specified\n";

  if (params.encParams.textureVideoEncoderConfig.empty())
    err.error() << "texture video encoder config not specified\n";

  if (params.encParams.textureVideoHDRToolEncConfig.empty())
    err.error() << "hdrtools encoder config not specified\n";

  if (params.encParams.textureVideoHDRToolDecConfig.empty())
    err.error() << "hdrtools decoder config not specified\n";

  if (err.is_errored)
    return false;

  // Dump the complete derived configuration
  std::cout << "+ Configuration parameters\n";
  po::dumpCfg(std::cout, opts, "Input", 4);
  po::dumpCfg(std::cout, opts, "Output", 4);
  po::dumpCfg(std::cout, opts, "General", 4);
  po::dumpCfg(std::cout, opts, "Geometry decimate", 4);
  po::dumpCfg(std::cout, opts, "Texture parametrization", 4);
  po::dumpCfg(std::cout, opts, "Geometry parametrization", 4);  
  po::dumpCfg(std::cout, opts, "Mesh", 4);
  po::dumpCfg(std::cout, opts, "Geometry video", 4);
  po::dumpCfg(std::cout, opts, "Texture video", 4);
  po::dumpCfg(std::cout, opts, "Displacements", 4);
  po::dumpCfg(std::cout, opts, "Lifting", 4);
  po::dumpCfg(std::cout, opts, "Bug fix", 4);
  std::cout << '\n';
  return true;
}
catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}
//----------------------------------------------------------------------------

int32_t
loadGroupOfFrames(
  const vmesh::VMCGroupOfFramesInfo& gofInfo,
  vmesh::VMCGroupOfFrames& gof,
  const Parameters& params)
{
  const auto startFrame = gofInfo.startFrameIndex_;
  const auto frameCount = gofInfo.frameCount_;
  const auto lastFrame = startFrame + frameCount - 1;
  std::cout << "Loading group of frames (" << startFrame << '-' << lastFrame
       << ") ";
  gof.resize(frameCount);
  for (int f = startFrame; f <= lastFrame; ++f) {
    const auto nameInputTexture = vmesh::expandNum(params.inputTexturePath, f);
    const auto findex = f - startFrame;
    auto& frame = gof.frames[findex];
    std::cout << '.' << std::flush;
    if (
      !frame.input.loadFromOBJ(params.inputMeshPath, f)
      || !LoadImage(nameInputTexture, frame.inputTexture)) {
      printf("Error loading frame %d / %d \n", f, frameCount);
      return -1;
    }
  }
  return 0;
} 

//============================================================================

int32_t
saveGroupOfFrames(
  const vmesh::VMCGroupOfFramesInfo& gofInfo,
  vmesh::VMCGroupOfFrames& gof,
  const Parameters& params)
{
  if (
    !params.reconstructedMeshPath.empty()
    && !params.reconstructedTexturePath.empty()
    && !params.reconstructedMaterialLibPath.empty()) {
    for (int f = 0; f < gofInfo.frameCount_; ++f) {
      const auto n = gofInfo.startFrameIndex_ + f;
      auto strObj = vmesh::expandNum(params.reconstructedMeshPath, n);
      auto strTex = vmesh::expandNum(params.reconstructedTexturePath, n);
      auto strMat = vmesh::expandNum(params.reconstructedMaterialLibPath, n);
      SaveImage(strTex, gof[f].outputTexture);
      vmesh::Material<double> material;
      material.texture = vmesh::basename(strTex);
      material.save(strMat);
      gof[f].rec.setMaterialLibrary(vmesh::basename(strMat));
      gof[f].rec.saveToOBJ(strObj);
    }
    return 0;
  }
  return -1;
}

//============================================================================

int32_t
compress(const Parameters& params)
{
  // Generate gof structure
  vmesh::SequenceInfo sequenceInfo;
  sequenceInfo.generate(
    params.frameCount, params.startFrame, params.groupOfFramesMaxSize,
    params.inputMeshPath);
  if (params.encParams.keepIntermediateFiles)
    sequenceInfo.save(params.encParams.intermediateFilesPathPrefix + "gof.txt");

  vmesh::VMCEncoder encoder;
  vmesh::Bitstream bitstream;
  vmesh::VMCStats totalStats;
  for (int g = 0; g < sequenceInfo.gofCount(); ++g) {
    vmesh::VMCGroupOfFrames gof;
    const auto& gofInfo = sequenceInfo[g];
    printf("loadGroupOfFrames GOF = %d / %zu \n", g, sequenceInfo.gofCount());
    
    // Load group of frame
    if (loadGroupOfFrames(gofInfo, gof, params)) {
      std::cerr << "Error: can't load group of frames!\n";
      return -1;
    }
    
    // Compress group of frame
    auto start = std::chrono::steady_clock::now();
    if (encoder.compress(gofInfo, gof, bitstream, params.encParams)) {
      std::cerr << "Error: can't compress group of frames!\n";
      return -1;
    }
    auto end = std::chrono::steady_clock::now();
    gof.stats.processingTimeInSeconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Save reconsctructed models
    if (saveGroupOfFrames(gofInfo, gof, params)) {
      std::cerr << "Error: can't save rec group of frames!\n";
      return -1;
    }

    totalStats += gof.stats;
    if (vmesh::vout) {
      vmesh::vout << "\n------- Group of frames " << gofInfo.index_
                  << " -----------\n";
      gof.stats.dump("GOF", params.framerate);
      vmesh::vout << "---------------------------------------\n";
    }
  }
  std::cout << "\n------- All frames -----------\n";
  totalStats.dump("Sequence", params.framerate);
  std::cout << "---------------------------------------\n";

  // save bistream
  if (bitstream.save(params.compressedStreamPath)) {
    std::cerr << "Error: can't save compressed bitstream!\n";
    return -1;
  }

  return 0;
}

//============================================================================

int
main(int argc, char* argv[])
{
  std::cout << "MPEG VMESH version " << ::vmesh::version << '\n';

  Parameters params;
  if (!parseParameters(argc, argv, params))
    return 1;

  if (params.verbose)
    vmesh::vout.rdbuf(std::cout.rdbuf());

  if ( compress(params)) {
    std::cerr << "Error: can't compress animation!\n";
    return 1;
  } 

  return 0;
}
