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
  bool verbose;
  std::string inputMeshPath;
  std::string inputTexturePath;
  std::string baseMeshPath;
  std::string subdivMeshPath;
  std::string groupOfFramesStructurePath;
  std::string compressedStreamPath;
  std::string reconstructedMeshPath;
  std::string reconstructedTexturePath;
  std::string reconstructedMaterialLibPath;

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

  (po::Section("General"))
    ("compressed", params.compressedStreamPath, {}, "Compressed bitstream")

  (po::Section("Input (Encoder)"))
    ("imesh",  params.inputMeshPath,    {}, "Input mesh")
    ("itex",   params.inputTexturePath, {}, "Input texture")
    ("base",   params.baseMeshPath,     {}, "Base mesh")
    ("subdiv", params.subdivMeshPath,   {}, "Subdiv mesh")

  (po::Section("Output (Encoder)"))
    ("recmat",  params.reconstructedMaterialLibPath, {},
    "Reconstructed materials")

    ("recmesh", params.reconstructedMeshPath, {},
    "Reconstructed mesh")

    ("rectex",  params.reconstructedTexturePath, {},
    "Reconstructed texture")

    ("intermediateFilesPathPrefix", params.encParams.intermediateFilesPathPrefix, {},
    "Intermediate files path prefix")

    ("keep",    params.encParams.keepIntermediateFiles, false,
    "Keep intermediate files")

  (po::Section("Common"))
    ("fstart",    params.startFrame, 1, "First frame number")
    ("fcount",    params.frameCount, 1, "Number of frames")
    ("framerate", params.framerate, 30., "Frame rate")

  (po::Section("Encoder"))
    ("gofmax", params.groupOfFramesMaxSize, 32,
    "Maximum group of frames size")

    ("gofstruct", params.groupOfFramesStructurePath, {},
    "Prediction structure file")

    ("it", params.encParams.subdivisionIterationCount, 2,
    "Subdivision iteration count")

    ("gqp", params.encParams.qpPosition, 10,
    "Quantization bits for base mesh positions")

    ("tqp", params.encParams.qpTexCoord, 8,
    "Quantization bits for base mesh texture coordinates")

    ("dqp", params.encParams.liftingQuantizationParameters, {16, 28, 28},
    "Quantization parameter for displacements")

    ("dqb", params.encParams.liftingQuantizationBias, {1./3., 1./3., 1./3},
    "Quantization bias for displacements")

    ("tvqp", params.encParams.textureVideoQP, 28,
    "Quantization parameter for texture video")

    ("gdepth", params.encParams.bitDepthPosition, 12,
    "Input positions bit depth")

    ("tdepth", params.encParams.bitDepthTexCoord, 12,
    "Input texture coordinates bit depth")

    ("texwidth", params.encParams.textureWidth, 2048,
    "Output texture width")

    ("texheight", params.encParams.textureHeight, 2048,
    "Output texture height")

    ("invorient", params.encParams.invertOrientation, false,
    "Invert triangles orientation")

    ("unifvertices", params.encParams.unifyVertices, false,
    "Unify duplicated vertices")

    ("encdisp", params.encParams.encodeDisplacementsVideo, true,
    "Encode displacements video")

    ("enctex", params.encParams.encodeTextureVideo, true,
    "Encode texture video")

    ("normuv", params.encParams.normalizeUV, true,
    "Normalize uv texture coordinates")
    
    ("gvencconfig", params.encParams.geometryVideoEncoderConfig, {},
    "Geometry video cfg")
    
    ("tvencconfig", params.encParams.textureVideoEncoderConfig, {},
    "Texture video cfg")

    ("cscencconfig", params.encParams.textureVideoHDRToolEncConfig, {},
    "HDRTools encode cfg")

    ("cscdecconfig", params.encParams.textureVideoHDRToolDecConfig, {},
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

  if (params.baseMeshPath.empty())
    err.error() << "base mesh not specified\n";

  if (params.subdivMeshPath.empty())
    err.error() << "subdivision mesh not specified\n";

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
  po::dumpCfg(std::cout, opts, "General", 4);
  po::dumpCfg(std::cout, opts, "Input (Encoder)", 4);
  po::dumpCfg(std::cout, opts, "Output (Encoder)", 4);
  po::dumpCfg(std::cout, opts, "Common", 4);
  po::dumpCfg(std::cout, opts, "Encoder", 4);
  po::dumpCfg(std::cout, opts, "External tools (Encoder)", 4);
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
      || !LoadImage(nameInputTexture, frame.inputTexture)
      || !frame.base.loadFromOBJ(params.baseMeshPath, f)
      || !frame.subdiv.loadFromOBJ(params.subdivMeshPath, f)) {
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
  vmesh::SequenceInfo sequenceInfo;
  sequenceInfo.load(
    params.frameCount, params.startFrame, params.groupOfFramesMaxSize,
    params.groupOfFramesStructurePath);

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
    if (encoder.compressOnly(gofInfo, gof, bitstream, params.encParams)) {
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
