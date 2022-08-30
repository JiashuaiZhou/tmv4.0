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
#include "decoder.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "checksum.hpp"
#include "metrics.hpp"
#include "vmcStats.hpp"
#include "sequenceInfo.hpp"

//============================================================================

struct Parameters {
  std::string                 compressedStreamPath   = {};
  std::string                 decodedMeshPath        = {};
  std::string                 decodedTexturePath     = {};
  std::string                 decodedMaterialLibPath = {};
  int32_t                     startFrame             = 0;
  double                      framerate              = 30.;
  bool                        verbose                = false;
  bool                        checksum               = true;
  vmesh::VMCDecoderParameters decParams;
  vmesh::VMCMetricsParameters metParams;
};

//============================================================================

static bool
parseParameters(int argc, char* argv[], Parameters& params) try {
  namespace po     = df::program_options_lite;
  bool  print_help = false;
  auto& decParams  = params.decParams;
  auto& metParams  = params.metParams;

  /* clang-format off */
  po::Options opts;
  opts.addOptions()
  (po::Section("Common"))
    ("help",      print_help,                     false,  "This help text")
    ("config,c",  po::parseConfigFile,                    "Configuration file name")
    ("verbose,v", params.verbose,                 true,   "Verbose output")

  (po::Section("Input"))
    ("compressed", params.compressedStreamPath, {},     "Compressed bitstream")

  (po::Section("Output"))
    ("decmat",  params.decodedMaterialLibPath,  {},     "Decoded materials")
    ("decmesh", params.decodedMeshPath,         {},     "Decoded mesh")
    ("dectex",  params.decodedTexturePath,      {},     "Decoded texture")

  (po::Section("General"))
    ("fstart",   
      params.startFrame, 
      params.startFrame,
      "First frame number")
    ("framerate", 
      params.framerate, 
      params.framerate, 
      "Frame rate")
    ("keep",    
      decParams.keepIntermediateFiles, 
      decParams.keepIntermediateFiles, 
      "Keep intermediate files")
    ("interFilesPathPrefix", 
      decParams.intermediateFilesPathPrefix, 
      decParams.intermediateFilesPathPrefix,
      "Intermediate files path prefix")
    ("checksum", 
      params.checksum, 
      params.checksum, 
      "Compute checksum")

  (po::Section("Decoder"))
    ("normuv",
      decParams.normalizeUV, 
      decParams.normalizeUV,
      "Normalize uv texture coordinates")
    ("cscdecconfig", 
      decParams.textureVideoHDRToolDecConfig, 
      decParams.textureVideoHDRToolDecConfig,
      "HDRTools decode cfg")

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

  if (params.decodedMeshPath.empty()) {
    err.error() << "decoded mesh not specified\n";
  }

  if (params.decodedTexturePath.empty()) {
    err.error() << "decoded texture not specified\n";
  }

  if (params.decodedMaterialLibPath.empty()) {
    err.error() << "decoded materials not specified\n";
  }

  if (params.decParams.textureVideoHDRToolDecConfig.empty()) {
    err.error() << "hdrtools decoder config not specified\n";
  }

  if (err.is_errored) { return false; }

  // Dump the complete derived configuration
  std::cout << "+ Configuration parameters\n";
  po::dumpCfg(std::cout, opts, "Common", 4);
  po::dumpCfg(std::cout, opts, "Input", 4);
  po::dumpCfg(std::cout, opts, "Output", 4);
  po::dumpCfg(std::cout, opts, "General", 4);
  po::dumpCfg(std::cout, opts, "Decoder", 4);
  po::dumpCfg(std::cout, opts, "Metrics", 4);
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
                  const vmesh::VMCMetricsParameters& metParams) {
  const auto startFrame = gofInfo.startFrameIndex_;
  const auto frameCount = gofInfo.frameCount_;
  const auto lastFrame  = startFrame + frameCount - 1;
  std::cout << "Loading group of frames (" << startFrame << '-' << lastFrame
            << ") ";
  gof.resize(frameCount);
  for (int f = startFrame; f <= lastFrame; ++f) {
    const auto nameInputTexture =
      vmesh::expandNum(metParams.srcTexturePath, f);
    const auto findex = f - startFrame;
    auto&      frame  = gof.frames[findex];
    std::cout << '.' << std::flush;
    if (!frame.input.loadFromOBJ(metParams.srcMeshPath, f)
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
  if (!params.decodedMeshPath.empty() && !params.decodedTexturePath.empty()
      && !params.decodedMaterialLibPath.empty()) {
    for (int f = 0; f < gofInfo.frameCount_; ++f) {
      const auto n      = gofInfo.startFrameIndex_ + f;
      auto       strObj = vmesh::expandNum(params.decodedMeshPath, n);
      auto       strTex = vmesh::expandNum(params.decodedTexturePath, n);
      auto       strMat = vmesh::expandNum(params.decodedMaterialLibPath, n);
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
decompress(const Parameters& params) {
  vmesh::Bitstream bitstream;
  if (bitstream.load(params.compressedStreamPath)) {
    std::cerr << "Error: can't load compressed bitstream ! ("
              << params.compressedStreamPath << ")\n";
    return -1;
  }

  vmesh::VMCDecoder           decoder;
  vmesh::VMCGroupOfFramesInfo gofInfo;
  vmesh::VMCStats             totalStats;
  vmesh::Checksum             checksum;
  vmesh::VMCMetrics           metrics;
  auto&                       metParams   = params.metParams;
  size_t                      byteCounter = 0;
  gofInfo.index_                          = 0;
  gofInfo.startFrameIndex_                = params.startFrame;
  while (byteCounter != bitstream.size()) {
    // Decompress
    vmesh::VMCGroupOfFrames gof;
    auto                    start = std::chrono::steady_clock::now();
    if (decoder.decompress(
          bitstream, gofInfo, gof, byteCounter, params.decParams)
        != 0) {
      std::cerr << "Error: can't decompress group of frames!\n";
      return -1;
    }
    printf("gof.stats.frameCount = %d \n", gof.stats.frameCount);
    auto end                 = std::chrono::steady_clock::now();
    gof.stats.processingTime = end - start;

    // Save reconsctructed models
    if (saveGroupOfFrames(gofInfo, gof, params) != 0) {
      std::cerr << "Error: can't save dec group of frames!\n";
      return -1;
    }
    if (params.checksum)
      for (auto& frame : gof) checksum.add(frame.rec, frame.outputTexture);

    if (metParams.computePcc || metParams.computeIbsm
        || metParams.computePcqm) {
      loadGroupOfFrames(gofInfo, gof, metParams);
      metrics.compute(gof, metParams);
    }
    totalStats += gof.stats;
    gofInfo.startFrameIndex_ += gof.stats.frameCount;
    ++gofInfo.index_;

    if (vmesh::vout) {
      vmesh::vout << "\n------- Group of frames " << gofInfo.index_
                  << " -----------\n";
      gof.stats.dump("GOF", params.framerate);
      vmesh::vout << "---------------------------------------\n";
    }
  }
  if (params.checksum) {
    vmesh::Checksum checksumEnc;
    checksumEnc.read(vmesh::removeFileExtension(params.compressedStreamPath)
                     + ".checksum");
    if (checksum != checksumEnc)
      totalStats.processingTime =
        std::chrono::steady_clock::duration((int)-1E9);
  }

  std::cout << "\n------- All frames -----------\n";
  totalStats.dump("Sequence", params.framerate);
  std::cout << "Sequence peak memory " << vmesh::getPeakMemory() << " KB\n";
  std::cout << "---------------------------------------\n";

  std::cout << "\nAll frames have been decoded. \n";
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

  if (decompress(params) != 0) {
    std::cerr << "Error: can't decompress animation!\n";
    return 1;
  }

  return 0;
}
