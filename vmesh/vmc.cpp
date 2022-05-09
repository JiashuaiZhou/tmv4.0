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

#include "vmesh/bitstream.hpp"
#include "vmesh/decoder.hpp"
#include "vmesh/encoder.hpp"
#include "vmesh/util/misc.hpp"
#include "vmesh/util/verbose.hpp"
#include "vmesh/version.hpp"
#include "vmesh/vmc.hpp"
#include "vmesh/vmcstats.hpp"

using namespace std;
using namespace vmesh;

//============================================================================

namespace {
struct Parameters {
  bool verbose;
  bool isDecoder;
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

  VMCEncoderParameters encParams;
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
  ("mode", params.isDecoder, true,
    "The encoding/decoding mode:\n"
    "  0: encode\n"
    "  1: decode")

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

  ("keep",    params.encParams.keepIntermediateFiles, false,
   "Keep intermediate files")

  (po::Section("Output (Decoder)"))
  ("decmat",  params.decodedMaterialLibPath, {},
   "Decoded materials")

  ("decmesh", params.decodedMeshPath, {},
   "Decoded mesh")

  ("dectex",  params.decodedTexturePath, {},
   "Decoded texture")

  ("keep",    params.decParams.keepIntermediateFiles, false,
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

  (po::Section("Decoder"))
  ("normuv", params.decParams.normalizeUV, true,
   "Normalize uv texture coordinates")

  (po::Section("External tools (Encoder)"))
  ("gmenc", params.encParams.geometryMeshEncoderPath, {},
   "Mesh encoder cmd")

  ("gmdec", params.encParams.geometryMeshDecoderPath, {},
   "Mesh decoder cmd")

  ("gvenc", params.encParams.geometryVideoEncoderPath, {},
   "Geometry video encoder cmd")

  ("gvencconfig", params.encParams.geometryVideoEncoderConfig, {},
   "Geometry video cfg")

  ("tvenc", params.encParams.textureVideoEncoderPath, {},
   "Texture video encoder cmd")

  ("tvencconfig", params.encParams.textureVideoEncoderConfig, {},
   "Texture video cfg")

  ("csc", params.encParams.textureVideoHDRToolPath, {},
   "HDRTools cmd")

  ("cscencconfig", params.encParams.textureVideoHDRToolEncConfig, {},
   "HDRTools encode cfg")

  ("cscdecconfig", params.encParams.textureVideoHDRToolDecConfig, {},
   "HDRTools decode cfg")

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
    err.warn() << "Unhandled argument ignored: " << arg << "\n";

  if (argc == 1 || print_help) {
    std::cout << "usage: " << argv[0] << " [arguments...] \n\n";
    po::doHelp(std::cout, opts, 78);
    return false;
  }

  if (params.compressedStreamPath.empty())
    err.error() << "compressed input/output not specified\n";

  if (!params.isDecoder) {
    if (params.inputMeshPath.empty())
      err.error() << "input mesh not specified\n";

    if (params.inputTexturePath.empty())
      err.error() << "input texture not specified\n";

    if (params.baseMeshPath.empty())
      err.error() << "base mesh not specified\n";

    if (params.subdivMeshPath.empty())
      err.error() << "subdivision mesh not specified\n";

    if (params.encParams.geometryMeshEncoderPath.empty())
      err.error() << "mesh encoder command not specified\n";

    if (params.encParams.geometryMeshDecoderPath.empty())
      err.error() << "mesh decoder command not specified\n";

    if (params.encParams.geometryVideoEncoderPath.empty())
      err.error() << "geometry video encoder command not specified\n";

    if (params.encParams.geometryVideoEncoderConfig.empty())
      err.error() << "geometry video config not specified\n";

    if (params.encParams.textureVideoEncoderPath.empty())
      err.error() << "texture video encoder command not specified\n";

    if (params.encParams.textureVideoEncoderConfig.empty())
      err.error() << "texture video encoder config not specified\n";

    if (params.encParams.textureVideoHDRToolPath.empty())
      err.error() << "hdrtools command not specified\n";

    if (params.encParams.textureVideoHDRToolEncConfig.empty())
      err.error() << "hdrtools encoder config not specified\n";

    if (params.encParams.textureVideoHDRToolDecConfig.empty())
      err.error() << "hdrtools decoder config not specified\n";
  }

  if (params.isDecoder) {
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
  }

  if (err.is_errored)
    return false;

  // Dump the complete derived configuration
  cout << "+ Configuration parameters\n";
  po::dumpCfg(cout, opts, "General", 4);
  if (!params.isDecoder) {
    po::dumpCfg(cout, opts, "Input (Encoder)", 4);
    po::dumpCfg(cout, opts, "Output (Encoder)", 4);
    po::dumpCfg(cout, opts, "Common", 4);
    po::dumpCfg(cout, opts, "Encoder", 4);
    po::dumpCfg(cout, opts, "External tools (Encoder)", 4);
  }
  if (params.isDecoder) {
    po::dumpCfg(cout, opts, "Output (Decoder)", 4);
    po::dumpCfg(cout, opts, "Common", 4);
    po::dumpCfg(cout, opts, "Decoder", 4);
    po::dumpCfg(cout, opts, "External tools (Decoder)", 4);
  }
  cout << endl;

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

  cout << header << " frame count " << stats.frameCount << endl;
  cout << header << " face count " << stats.faceCount << endl;
  cout << header << " vertex count " << stats.vertexCount << endl;
  cout << header << " processing time " << stats.processingTimeInSeconds
       << " s " << endl;
  cout << header << " meshes bitrate " << baseMeshBitrate << " mbps "
       << stats.baseMeshByteCount << " B " << baseMeshBitsPerVertex << " bpv"
       << endl;
  cout << header << " motion bitrate " << motionBitrate << " mbps "
       << stats.motionByteCount << " B " << motionBitsPerVertex << " bpv"
       << endl;
  cout << header << " displacements bitrate " << displacementBitrate
       << " mbps " << stats.displacementsByteCount << " B "
       << displacementBitsPerVertex << " bpv" << endl;
  cout << header << " texture bitrate " << textureBitrate << " mbps "
       << stats.textureByteCount << " B " << textureBitsPerVertex << " bpv"
       << endl;
  cout << header << " total bitrate " << totalBitrate << " mbps "
       << stats.totalByteCount << " B " << totalBitsPerVertex << " bpv"
       << endl;
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
        VMCFrameInfo& frameInfo = gofInfo.frameInfo(frameIndexInGOF);
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

//----------------------------------------------------------------------------

int32_t
loadGroupOfFrames(
  const VMCGroupOfFramesInfo& gofInfo,
  VMCGroupOfFrames& gof,
  const Parameters& params)
{
  const auto startFrame = gofInfo.startFrameIndex;
  const auto frameCount = gofInfo.frameCount;
  const auto lastFrame = startFrame + frameCount - 1;
  vout << "Loading group of frames (" << startFrame << "-" << lastFrame
       << ") ";

  gof.resize(frameCount);
  vector<vector<int32_t>> umappings(frameCount);
  for (int f = startFrame; f <= lastFrame; ++f) {
    const auto nameInputMesh = expandNum(params.inputMeshPath, f);
    const auto nameInputTexture = expandNum(params.inputTexturePath, f);
    const auto nameBaseMesh = expandNum(params.baseMeshPath, f);
    const auto nameSuvdivMesh = expandNum(params.subdivMeshPath, f);
    const auto findex = f - startFrame;
    auto& frame = gof.frames[findex];
    vout << "." << std::flush;

    if (
      !frame.input.loadFromOBJ(nameInputMesh)
      || !LoadImage(nameInputTexture, frame.inputTexture)
      || !frame.base.loadFromOBJ(nameBaseMesh)
      || !frame.subdiv.loadFromOBJ(nameSuvdivMesh)) {
      return -1;
    }

    frame.base.resizeNormals(0);
    frame.base.resizeNormalTriangles(0);
    frame.subdiv.resizeNormals(0);
    frame.subdiv.resizeNormalTriangles(0);
    if (
      params.encParams.unifyVertices
      && params.encParams.subdivisionIterationCount == 0) {
      auto& base = frame.base;
      auto& subdiv = frame.subdiv;
      assert(base.pointCount() == subdiv.pointCount());
      assert(base.triangleCount() == subdiv.triangleCount());
      assert(base.texCoordCount() == subdiv.texCoordCount());
      assert(base.texCoordTriangleCount() == subdiv.texCoordTriangleCount());
      vector<Vec3<double>> upoints;
      vector<Triangle> utriangles;
      const auto pointCount0 = base.pointCount();
      const auto& frameInfo = gofInfo.frameInfo(findex);
      auto& umapping = umappings[findex];
      if (frameInfo.type == FrameType::INTRA) {
        UnifyVertices(
          base.points(), base.triangles(), upoints, utriangles, umapping);
        std::swap(base.points(), upoints);
        std::swap(base.triangles(), utriangles);
        RemoveDegeneratedTriangles(base);

        const auto pointCount1 = base.pointCount();
        assert(pointCount1 <= pointCount0);
        upoints.resize(pointCount1);
        for (int32_t vindex = 0; vindex < pointCount0; ++vindex) {
          upoints[umapping[vindex]] = subdiv.point(vindex);
        }
        std::swap(subdiv.points(), upoints);
        subdiv.triangles() = base.triangles();
        subdiv.texCoordTriangles() = base.texCoordTriangles();
      } else {
        umapping = umappings[frameInfo.referenceFrameIndex];
        const auto& refFrame = gof.frames[frameInfo.referenceFrameIndex];
        upoints = base.points();
        base = refFrame.base;
        for (int32_t vindex = 0; vindex < pointCount0; ++vindex) {
          base.setPoint(umapping[vindex], upoints[vindex]);
        }
        upoints = subdiv.points();
        subdiv = refFrame.subdiv;
        for (int32_t vindex = 0; vindex < pointCount0; ++vindex) {
          subdiv.setPoint(umapping[vindex], upoints[vindex]);
        }
      }
    }
  }

  cout << "\n";
  return 0;
}

//============================================================================

int32_t
compress(const Parameters& params)
{
  vector<VMCGroupOfFramesInfo> gofsInfo;
  if (loadGroupOfFramesInfo(params, gofsInfo)) {
    cerr << "Error: can't load group of frames structure!" << endl;
    return -1;
  }

  VMCGroupOfFrames gof;
  VMCEncoder encoder;
  Bitstream bitstream;
  VMCStats totalStats;
  for (int g = 0, gofCount = int32_t(gofsInfo.size()); g < gofCount; ++g) {
    const auto& gofInfo = gofsInfo[g];
    if (loadGroupOfFrames(gofInfo, gof, params)) {
      cerr << "Error: can't load group of frames!" << endl;
      return -1;
    }

    auto start = chrono::steady_clock::now();
    if (encoder.compress(gofInfo, gof, bitstream, params.encParams)) {
      cerr << "Error: can't compress group of frames!" << endl;
      return -1;
    }

    auto end = chrono::steady_clock::now();
    gof.stats.processingTimeInSeconds =
      chrono::duration_cast<chrono::milliseconds>(end - start).count();

    const auto& stats = gof.stats;
    totalStats += stats;
    if (
      !params.reconstructedMeshPath.empty()
      && !params.reconstructedTexturePath.empty()
      && !params.reconstructedMaterialLibPath.empty()) {
      for (int fIndex = 0; fIndex < gofInfo.frameCount; ++fIndex) {
        const auto fNum = gofInfo.startFrameIndex + fIndex;
        auto nameRecMesh = expandNum(params.reconstructedMeshPath, fNum);
        auto nameRecTex = expandNum(params.reconstructedTexturePath, fNum);
        auto nameRecMat = expandNum(params.reconstructedMaterialLibPath, fNum);

        const auto& frame = gof.frame(fIndex);
        SaveImage(nameRecTex, frame.outputTexture);
        Material<double> material;
        material.texture = nameRecTex;
        material.save(nameRecMat);
        auto& recmesh = gof.frame(fIndex).rec;
        recmesh.setMaterialLibrary(nameRecMat);
        recmesh.saveToOBJ(nameRecMesh);
      }
    }

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

  if (bitstream.save(params.compressedStreamPath)) {
    cerr << "Error: can't save compressed bitstream!" << endl;
    return -1;
  }

  return 0;
}

//============================================================================

int32_t
decompress(const Parameters& params)
{
  Bitstream bitstream;
  if (bitstream.load(params.compressedStreamPath)) {
    cerr << "Error: can't load compressed bitstream ! ("
         << params.compressedStreamPath << ")" << endl;
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
      cerr << "Error: can't decompress group of frames!" << endl;
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
  cout << "MPEG VMESH version " << ::vmesh::version << endl;

  Parameters params;
  if (!parseParameters(argc, argv, params))
    return 1;

  if (params.verbose)
    vmesh::vout.rdbuf(std::cout.rdbuf());

  if (!params.isDecoder && compress(params)) {
    cerr << "Error: can't compress animation!" << endl;
    return 1;
  } else if (params.isDecoder && decompress(params)) {
    cerr << "Error: can't decompress animation!" << endl;
    return 1;
  }

  return 0;
}