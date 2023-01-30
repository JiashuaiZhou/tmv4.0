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
  std::string                 sourceMeshPath         = {};
  std::string                 sourceTexturePath      = {};
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
    ("help",      print_help,          false, "This help text")
    ("config,c",  po::parseConfigFile, "Configuration file name")
    ("verbose,v", metParams.verbose,   false, "Verbose output")

  (po::Section("Input"))
    ("compressed", 
      params.compressedStreamPath, 
      params.compressedStreamPath, 
      "Compressed bitstream")

  (po::Section("Output"))
    ("decMesh", 
      params.decodedMeshPath,
      params.decodedMeshPath,
      "Decoded mesh")
    ("decTex",  
      params.decodedTexturePath, 
      params.decodedTexturePath,
       "Decoded texture")
    ("decMat",  
      params.decodedMaterialLibPath,  
      params.decodedMaterialLibPath, 
      "Decoded materials")
    ("dequantizeUV",
      decParams.dequantizeUV, 
      decParams.dequantizeUV,
      "Dequantize texture coordinates of the decoded meshes")
    ("startFrameIndex",   
      params.startFrame, 
      params.startFrame,
      "First frame number")
    ("framerate", 
      params.framerate, 
      params.framerate, 
      "Frame rate")

  (po::Section("General"))
    ("keep",    
      decParams.keepIntermediateFiles, 
      decParams.keepIntermediateFiles, 
      "Keep intermediate files")
    ("checksum", 
      params.checksum, 
      params.checksum, 
      "Compute checksum")

  (po::Section("Decoder"))
    ("textureVideoDecoderConvertConfig", 
      decParams.textureVideoHDRToolDecConfig, 
      decParams.textureVideoHDRToolDecConfig,
      "HDRTools decode cfg") 
    ("textureVideoUpsampleFilter", 
      decParams.textureVideoUpsampleFilter, 
      decParams.textureVideoUpsampleFilter, 
      "Chroma upsample filter in [0;7]")
    ("textureVideoFullRange", 
      decParams.textureVideoFullRange, 
      decParams.textureVideoFullRange, 
      "Texture video range")

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
    ("positionBitDepth",
      metParams.qp,
      metParams.qp,
      "Position bit depth")
    ("texCoordBitDepth",
      metParams.qt,
      metParams.qt,
      "Texture coordinate bit depth")
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
    ("srcMesh",
      params.sourceMeshPath,
      params.sourceMeshPath,
      "Metric Source mesh path")
    ("srcTex", 
      params.sourceTexturePath, 
      params.sourceTexturePath,
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
  if ((metParams.computePcc || metParams.computeIbsm || metParams.computePcqm)
      && (params.sourceMeshPath.empty() || params.sourceTexturePath.empty())) {
    err.error() << "Source mesh/texture must be define to compute metrics\n";
  }
  metParams.dequantizeUV = decParams.dequantizeUV;

  if (err.is_errored) { return false; }

  // Dump the complete derived configuration
  po::dumpCfgBySection(std::cout, opts);
  return true;
} catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

bool
decompress(const Parameters& params) {
  vmesh::Bitstream bitstream;
  if (!bitstream.load(params.compressedStreamPath)) {
    std::cerr << "Error: can't load compressed bitstream ! ("
              << params.compressedStreamPath << ")\n";
    return false;
  }
  std::string keepFilesPathPrefix = "";
  if (params.decParams.keepIntermediateFiles) {
    keepFilesPathPrefix = vmesh::dirname(params.compressedStreamPath);
  }

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
    vmesh::VMCDecoder decoder;
    vmesh::Sequence   reconstruct;
    decoder.setKeepFilesPathPrefix(keepFilesPathPrefix + "GOF_"
                                   + std::to_string(gofInfo.index_) + "_");
    auto start = std::chrono::steady_clock::now();
    if (!decoder.decompress(
          bitstream, gofInfo, reconstruct, byteCounter, params.decParams)) {
      std::cerr << "Error: can't decompress group of frames!\n";
      return false;
    }
    auto end                       = std::chrono::steady_clock::now();
    decoder.stats().processingTime = end - start;
    printf(
      "gof decoded: frameCount = %zu in %f sec. \n",
      decoder.stats().frameCount,
      std::chrono::duration<double>(decoder.stats().processingTime).count());
    fflush(stdout);

    // Save reconsctructed models
    if (!reconstruct.save(params.decodedMeshPath,
                          params.decodedTexturePath,
                          params.decodedMaterialLibPath,
                          gofInfo.startFrameIndex_)) {
      std::cerr << "Error: can't save decoded sequence \n";
      return false;
    }
    if (params.checksum) { checksum.add(reconstruct); }

    if (metParams.computePcc || metParams.computeIbsm
        || metParams.computePcqm) {
      vmesh::Sequence source;
      if (!source.load(params.sourceMeshPath,
                       params.sourceTexturePath,
                       gofInfo.startFrameIndex_,
                       gofInfo.frameCount_)) {
        std::cerr << "Error: can't load source sequence\n";
        return false;
      }
      metrics.compute(source, reconstruct, metParams);
    }
    totalStats += decoder.stats();
    gofInfo.startFrameIndex_ += decoder.stats().frameCount;
    ++gofInfo.index_;

    if (vmesh::vout) {
      vmesh::vout << "\n------- Group of frames " << gofInfo.index_
                  << " -----------\n";
      decoder.stats().dump("GOF", params.framerate);
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

  if (metParams.computePcc || metParams.computeIbsm || metParams.computePcqm) {
    std::cout << "\n------- All frames metrics -----------\n";
    metrics.display();
    std::cout << "---------------------------------------\n";
  }

  std::cout << "\n------- All frames -----------\n";
  totalStats.dump("Sequence", params.framerate);
  std::cout << "Sequence peak memory " << vmesh::getPeakMemory() << " KB\n";
  std::cout << "---------------------------------------\n";

  std::cout << "\nAll frames have been decoded. \n";
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

  if (!decompress(params)) {
    std::cerr << "Error: can't decompress animation!\n";
    return 1;
  }

  return 0;
}
