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

#include "bitstream.hpp"
#include "decoder.hpp"
#include "misc.hpp"
#include "verbose.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "vmcstats.hpp"

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

  int32_t startFrame;
  int32_t frameCount;
  int32_t groupOfFramesMaxSize;
  double framerate;

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
  ("decmat",  params.decodedMaterialLibPath, {},
   "Decoded materials")

  ("decmesh", params.decodedMeshPath, {},
   "Decoded mesh")

  ("dectex",  params.decodedTexturePath, {},
   "Decoded texture")

  ("intermediateFilesPathPrefix", params.decParams.intermediateFilesPathPrefix, {},
   "Intermediate files path prefix")

  ("keep",    params.decParams.keepIntermediateFiles, false,
   "Keep intermediate files")

  (po::Section("Common"))
  ("fstart",    params.startFrame, 1, "First frame number")
  ("fcount",    params.frameCount, 1, "Number of frames")
  ("framerate", params.framerate, 30., "Frame rate")

  (po::Section("Decoder"))
  ("normuv", params.decParams.normalizeUV, true,
   "Normalize uv texture coordinates")

  (po::Section("External tools (Decoder)"))
  ("gmdec", params.decParams.geometryMeshDecoderPath, {},
   "Mesh decoder cmd")

  ("gvdec", params.decParams.geometryVideoDecoderPath, {},
   "Geometry video decoder cmd")

  ("tvdec", params.decParams.textureVideoDecoderPath, {},
   "Texture video decoder cmd")

  ("csc", params.decParams.textureVideoHDRToolPath, {},
   "HDRTools cmd")

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

  if (params.decParams.geometryMeshDecoderPath.empty())
    err.error() << "mesh decoder command not specified\n";

  if (params.decParams.geometryVideoDecoderPath.empty())
    err.error() << "geometry video decoder command not specified\n";

  if (params.decParams.textureVideoDecoderPath.empty())
    err.error() << "texture video decoder command not specified\n";

  if (params.decParams.textureVideoHDRToolPath.empty())
    err.error() << "hdrtools command not specified\n";

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
  po::dumpCfg(std::cout, opts, "External tools (Decoder)", 4);
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
  vmesh::VMCGroupOfFrames gof;
  vmesh::VMCStats totalStats;
  totalStats.reset();
  size_t byteCounter = 0;
  int32_t f = 0;
  gofInfo.index = 0;
  gofInfo.startFrameIndex = params.startFrame;
  while (byteCounter != bitstream.size()) {
    auto start = std::chrono::steady_clock::now();
    if (decoder.decompress(
          bitstream, gofInfo, gof, byteCounter, params.decParams)) {
      std::cerr << "Error: can't decompress group of frames!\n";
      return -1;
    }

    auto end = std::chrono::steady_clock::now();
    gof.stats.processingTimeInSeconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    const auto& stats = gof.stats;
    const auto startFrameGOF = f + params.startFrame;
    const auto frameCountGOF = int32_t(stats.frameCount);
    if (
      !params.decodedMeshPath.empty() && !params.decodedTexturePath.empty()
      && !params.decodedMaterialLibPath.empty()) {
      for (int fIndex = 0; fIndex < frameCountGOF; ++fIndex) {
        const auto fNum = startFrameGOF + fIndex;
        auto nameDecMesh = vmesh::expandNum(params.decodedMeshPath, fNum);
        auto nameDecTexture = vmesh::expandNum(params.decodedTexturePath, fNum);
        auto nameDecMaterial = vmesh::expandNum(params.decodedMaterialLibPath, fNum);

        const auto& frame = gof.frame(fIndex);
        vmesh::SaveImage(nameDecTexture, frame.outputTexture);
        vmesh::Material<double> material;
        material.texture = nameDecTexture;
        material.save(nameDecMaterial);
        auto& recmesh = gof.frame(fIndex).rec;
        recmesh.setMaterialLibrary(nameDecMaterial);
        recmesh.saveToOBJ(nameDecMesh);
      }
    }

    totalStats += stats;
    f += stats.frameCount;
    gofInfo.startFrameIndex += stats.frameCount;
    ++gofInfo.index;

    if (vmesh::vout) {
      vmesh::vout << "\n------- Group of frames " << gofInfo.index
           << " -----------\n";
      stats.dump( "GOF", params.framerate);
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
