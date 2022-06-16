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
#include "decoder.hpp"
#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "vmcStats.hpp"

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
  std::string decodedMeshPath;
  std::string decodedTexturePath;
  std::string decodedMaterialLibPath;

  int32_t startFrame =0; 
  double framerate = 30.;

  vmesh::VMCDecoderParameters decParams;
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

  (po::Section("Output (Decoder)"))
    ("decmat",  params.decodedMaterialLibPath, {},   "Decoded materials")
    ("decmesh", params.decodedMeshPath, {},   "Decoded mesh")
    ("dectex",  params.decodedTexturePath, {},   "Decoded texture")
    ("intermediateFilesPathPrefix", params.decParams.intermediateFilesPathPrefix, {},
    "Intermediate files path prefix")
    ("keep",    params.decParams.keepIntermediateFiles, false,
    "Keep intermediate files")

  (po::Section("Common"))
    ("fstart",    params.startFrame, 1, "First frame number")
    ("framerate", params.framerate, 30., "Frame rate")

  (po::Section("Decoder"))
    ("normuv", params.decParams.normalizeUV, true,
    "Normalize uv texture coordinates")
    ("cscdecconfig", params.decParams.textureVideoHDRToolDecConfig, {},
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

  if (params.decodedMeshPath.empty())
    err.error() << "decoded mesh not specified\n";

  if (params.decodedTexturePath.empty())
    err.error() << "decoded texture not specified\n";

  if (params.decodedMaterialLibPath.empty())
    err.error() << "decoded materials not specified\n";

  if (params.decParams.textureVideoHDRToolDecConfig.empty())
    err.error() << "hdrtools decoder config not specified\n";

  if (err.is_errored)
    return false;

  // Dump the complete derived configuration
  std::cout << "+ Configuration parameters\n";
  po::dumpCfg(std::cout, opts, "General", 4);
  po::dumpCfg(std::cout, opts, "Output (Decoder)", 4);
  po::dumpCfg(std::cout, opts, "Common", 4);
  po::dumpCfg(std::cout, opts, "Decoder", 4);
  std::cout << '\n';
  return true;
}
catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

int32_t
saveGroupOfFrames(
  const vmesh::VMCGroupOfFramesInfo& gofInfo,
  vmesh::VMCGroupOfFrames& gof,
  const Parameters& params)
{
  if (
    !params.decodedMeshPath.empty()
    && !params.decodedTexturePath.empty()
    && !params.decodedMaterialLibPath.empty()) {
    for (int f = 0; f < gofInfo.frameCount_; ++f) {
      const auto n = gofInfo.startFrameIndex_ + f;
      auto strObj = vmesh::expandNum(params.decodedMeshPath, n);
      auto strTex = vmesh::expandNum(params.decodedTexturePath, n);
      auto strMat = vmesh::expandNum(params.decodedMaterialLibPath, n);
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
decompress(const Parameters& params)
{
  vmesh::Bitstream bitstream;
  if (bitstream.load(params.compressedStreamPath)) {
    std::cerr << "Error: can't load compressed bitstream ! ("
         << params.compressedStreamPath << ")\n";
    return -1;
  }

  vmesh::VMCDecoder decoder;
  vmesh::VMCGroupOfFramesInfo gofInfo;
  vmesh::VMCStats totalStats;
  totalStats.reset();
  size_t byteCounter = 0;
  gofInfo.index_ = 0;
  gofInfo.startFrameIndex_ = params.startFrame;
  while (byteCounter != bitstream.size()) {
    vmesh::VMCGroupOfFrames gof;
    
    // Decompress
    auto start = std::chrono::steady_clock::now();
    if (decoder.decompress(
          bitstream, gofInfo, gof, byteCounter, params.decParams)) {
      std::cerr << "Error: can't decompress group of frames!\n";
      return -1;
    }    
    auto end = std::chrono::steady_clock::now();
    gof.stats.processingTimeInSeconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Save reconsctructed models
    if (saveGroupOfFrames(gofInfo, gof, params)) {
      std::cerr << "Error: can't save dec group of frames!\n";
      return -1;
    }

    totalStats += gof.stats;
    gofInfo.startFrameIndex_ += gof.stats.frameCount;
    ++gofInfo.index_;

    if (vmesh::vout) {
      vmesh::vout << "\n------- Group of frames " << gofInfo.index_
           << " -----------\n";
      gof.stats.dump( "GOF", params.framerate);
      vmesh::vout << "---------------------------------------\n";
    }
  }

  std::cout << "\n------- All frames -----------\n";
  totalStats.dump( "Sequence", params.framerate);
  std::cout << "---------------------------------------\n";
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

  if (decompress(params)) {
    std::cerr << "Error: can't decompress animation!\n";
    return 1;
  }

  return 0;
}
