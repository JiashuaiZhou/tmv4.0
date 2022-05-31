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

using namespace std;
using namespace vmesh;
using namespace vmeshdec;

//============================================================================

namespace {
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

  VMCDecoderParameters decParams;
};
}  // namespace

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
  const list<const char*>& argv_unhandled =
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
  cout << "+ Configuration parameters\n";
  po::dumpCfg(cout, opts, "General", 4);
  po::dumpCfg(cout, opts, "Output (Decoder)", 4);
  po::dumpCfg(cout, opts, "Common", 4);
  po::dumpCfg(cout, opts, "Decoder", 4);
  po::dumpCfg(cout, opts, "External tools (Decoder)", 4);

  cout << '\n';

  return true;
}
catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

void
dumpStats(
  const VMCStats& stats, const string& header, const Parameters& params)
{
  const auto byteCountToBitrate =
    (8.0 * params.framerate) / (stats.frameCount * 1000000.0);
  const auto baseMeshBitrate = stats.baseMeshByteCount * byteCountToBitrate;
  const auto motionBitrate = stats.motionByteCount * byteCountToBitrate;
  const auto displacementBitrate =
    stats.displacementsByteCount * byteCountToBitrate;
  const auto textureBitrate = stats.textureByteCount * byteCountToBitrate;
  const auto totalBitrate = stats.totalByteCount * byteCountToBitrate;

  const auto baseMeshBitsPerVertex =
    stats.baseMeshByteCount * 8.0 / stats.baseMeshVertexCount;
  const auto motionBitsPerVertex =
    stats.motionByteCount * 8.0 / stats.baseMeshVertexCount;
  const auto textureBitsPerVertex =
    stats.textureByteCount * 8.0 / stats.vertexCount;
  const auto displacementBitsPerVertex =
    stats.displacementsByteCount * 8.0 / stats.vertexCount;
  const auto totalBitsPerVertex =
    stats.totalByteCount * 8.0 / stats.vertexCount;

  cout << header << " frame count " << stats.frameCount << '\n';
  cout << header << " face count " << stats.faceCount << '\n';
  cout << header << " vertex count " << stats.vertexCount << '\n';
  cout << header << " processing time " << stats.processingTimeInSeconds
       << " s \n";
  cout << header << " meshes bitrate " << baseMeshBitrate << " mbps "
       << stats.baseMeshByteCount << " B "
       << baseMeshBitsPerVertex << " bpv\n";
  cout << header << " motion bitrate " << motionBitrate << " mbps "
       << stats.motionByteCount << " B " << motionBitsPerVertex << " bpv\n";
  cout << header << " displacements bitrate " << displacementBitrate
       << " mbps " << stats.displacementsByteCount << " B "
       << displacementBitsPerVertex << " bpv\n";
  cout << header << " texture bitrate " << textureBitrate << " mbps "
       << stats.textureByteCount << " B " << textureBitsPerVertex << " bpv\n";
  cout << header << " total bitrate " << totalBitrate << " mbps "
       << stats.totalByteCount << " B " << totalBitsPerVertex << " bpv\n";
}

//============================================================================

int32_t
loadGroupOfFramesInfo(
  const Parameters& params, vector<VMCGroupOfFramesInfo>& gofsInfo)
{
  gofsInfo.reserve(10);
  if (params.groupOfFramesStructurePath.empty()) {
    for (int32_t f = 0, gofIndex = 0; f < params.frameCount; ++gofIndex) {
      const auto startFrameGOF = f + params.startFrame;
      const auto frameCountGOF =
        std::min(params.groupOfFramesMaxSize, params.frameCount - f);
      gofsInfo.resize(gofsInfo.size() + 1);
      auto& gofInfo = gofsInfo.back();
      gofInfo.frameCount = frameCountGOF;
      gofInfo.startFrameIndex = startFrameGOF;
      gofInfo.index = gofIndex;
      gofInfo.resize(frameCountGOF);
      for (int32_t frameIndexInGOF = 0; frameIndexInGOF < frameCountGOF;
           ++frameIndexInGOF) {
        vmesh::VMCFrameInfo& frameInfo = gofInfo.frameInfo(frameIndexInGOF);
        frameInfo.frameIndex = frameIndexInGOF;
        frameInfo.referenceFrameIndex = -1;
        frameInfo.type = FrameType::INTRA;
      }
      f += frameCountGOF;
    }
    return 0;
  }

  std::ifstream fin(params.groupOfFramesStructurePath);
  if (fin.is_open()) {
    std::string line;
    std::vector<std::string> tokens;
    int32_t currentGofIndex = -1;
    int32_t frameCounter = 0;

    while (getline(fin, line) && frameCounter++ < params.frameCount) {
      size_t prev = 0, pos;
      tokens.resize(0);
      while ((pos = line.find_first_of(" ,;:/", prev)) != std::string::npos) {
        if (pos > prev) {
          tokens.push_back(line.substr(prev, pos - prev));
        }
        prev = pos + 1;
      }

      if (prev < line.length()) {
        tokens.push_back(line.substr(prev, std::string::npos));
      }

      const auto frameIndex = atoi(tokens[0].c_str());
      const auto referenceFrameIndex = atoi(tokens[1].c_str());
      const auto gofIndex = atoi(tokens[2].c_str());
      assert(referenceFrameIndex <= frameIndex);

      if (gofIndex != currentGofIndex) {
        currentGofIndex = gofIndex;
        gofsInfo.resize(gofsInfo.size() + 1);
        auto& gofInfo = gofsInfo.back();
        gofInfo.frameCount = 1;
        gofInfo.startFrameIndex = frameIndex;
        gofInfo.index = gofIndex;
        VMCFrameInfo frameInfo;
        frameInfo.frameIndex = frameIndex - gofInfo.startFrameIndex;
        frameInfo.referenceFrameIndex = -1;
        frameInfo.type = FrameType::INTRA;
        gofInfo.framesInfo.reserve(params.groupOfFramesMaxSize);
        gofInfo.framesInfo.push_back(frameInfo);
      } else {
        auto& gofInfo = gofsInfo.back();
        ++gofInfo.frameCount;
        VMCFrameInfo frameInfo;

        if (referenceFrameIndex == frameIndex) {
          frameInfo.frameIndex = frameIndex - gofInfo.startFrameIndex;
          frameInfo.referenceFrameIndex = -1;
          frameInfo.type = FrameType::INTRA;
        } else {
          frameInfo.frameIndex = frameIndex - gofInfo.startFrameIndex;
          frameInfo.referenceFrameIndex = frameInfo.frameIndex - 1;
          frameInfo.type = FrameType::SKIP;
        }
        gofInfo.framesInfo.push_back(frameInfo);
      }
    }

    fin.close();
    return 0;
  }

  return -1;
}

//============================================================================

int32_t
decompress(const Parameters& params)
{
  Bitstream bitstream;
  if (bitstream.load(params.compressedStreamPath)) {
    cerr << "Error: can't load compressed bitstream ! ("
         << params.compressedStreamPath << ")\n";
    return -1;
  }

  VMCDecoder decoder;
  VMCGroupOfFramesInfo gofInfo;
  VMCGroupOfFrames gof;
  VMCStats totalStats;
  totalStats.reset();
  size_t byteCounter = 0;
  int32_t f = 0;
  gofInfo.index = 0;
  gofInfo.startFrameIndex = params.startFrame;
  while (byteCounter != bitstream.size()) {
    auto start = chrono::steady_clock::now();
    if (decoder.decompress(
          bitstream, gofInfo, gof, byteCounter, params.decParams)) {
      cerr << "Error: can't decompress group of frames!\n";
      return -1;
    }

    auto end = chrono::steady_clock::now();
    gof.stats.processingTimeInSeconds =
      chrono::duration_cast<chrono::milliseconds>(end - start).count();

    const auto& stats = gof.stats;
    const auto startFrameGOF = f + params.startFrame;
    const auto frameCountGOF = int32_t(stats.frameCount);
    if (
      !params.decodedMeshPath.empty() && !params.decodedTexturePath.empty()
      && !params.decodedMaterialLibPath.empty()) {
      for (int fIndex = 0; fIndex < frameCountGOF; ++fIndex) {
        const auto fNum = startFrameGOF + fIndex;
        auto nameDecMesh = expandNum(params.decodedMeshPath, fNum);
        auto nameDecTexture = expandNum(params.decodedTexturePath, fNum);
        auto nameDecMaterial = expandNum(params.decodedMaterialLibPath, fNum);

        const auto& frame = gof.frame(fIndex);
        SaveImage(nameDecTexture, frame.outputTexture);
        Material<double> material;
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

    if (vout) {
      vout << "\n\n------- Group of frames " << gofInfo.index
           << " -----------\n";
      dumpStats(stats, "GOF", params);
      vout << "---------------------------------------\n\n";
    }
  }

  cout << "\n\n------- All frames -----------\n";
  dumpStats(totalStats, "Sequence", params);
  cout << "---------------------------------------\n\n";
  return 0;
}

//============================================================================

int
main(int argc, char* argv[])
{
  cout << "MPEG VMESH version " << ::vmesh::version << '\n';

  Parameters params;
  if (!parseParameters(argc, argv, params))
    return 1;

  if (params.verbose)
    vmesh::vout.rdbuf(std::cout.rdbuf());

  if (decompress(params)) {
    cerr << "Error: can't decompress animation!\n";
    return 1;
  }

  return 0;
}
