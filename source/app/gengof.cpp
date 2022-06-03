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

#include <cstdint>
#include <fstream>
#include <string>

#include <program-options-lite/program_options_lite.h>

#include "misc.hpp"
#include "mesh.hpp"
#include "vector.hpp"
#include "version.hpp"

using namespace std;
using namespace vmesh;

//============================================================================

namespace {
struct Parameters {
  std::string inputPath;
  std::string outputPath;
  int32_t startFrame;
  int32_t frameCount;
  int32_t maxGOFSize = 32;
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
  ("help", print_help, false, "this help text")
  ("config,c", po::parseConfigFile, "configuration file name")

  (po::Section("Input/Output"))
  ("input,i", params.inputPath, {}, "Input mesh")
  ("output,o", params.outputPath, {}, "Group of frame structure")

  (po::Section("Gof"))
  ("startFrame", params.startFrame, 1, "First frame index")
  ("frameCount", params.frameCount, 300, "Number of frames")
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

  if (params.inputPath.empty())
    err.error() << "input mesh not specified\n";

  if (params.outputPath.empty())
    err.error() << "decimated output mesh not specified\n";

  if (err.is_errored)
    return false;

  // Dump the complete derived configuration
  cout << "+ Configuration parameters\n";
  po::dumpCfg(cout, opts, "Input/Output", 4);
  po::dumpCfg(cout, opts, "Gof", 4);
  cout << '\n';

  return true;
}
catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

int
main(int argc, char* argv[])
{
  cout << "MPEG VMESH version " << ::vmesh::version << '\n';

  Parameters params;
  if (!parseParameters(argc, argv, params))
    return 1;

  if (params.frameCount < 1) {
    cerr << "Error: frameCount < 1\n";
    return 1;
  }

  std::ofstream fout(params.outputPath);
  if (!fout.is_open()) {
    cerr << "Error: can't create output file " << params.outputPath << '\n';
    return 1;
  }

  TriangleMesh<double> mesh0, mesh1;
  auto frameIndex0 = params.startFrame;
  if (1) {
    const auto& name = vmesh::expandNum(params.inputPath, frameIndex0);
    if (!mesh0.loadFromOBJ(name)) {
      cerr << "Error: can't load frame" << frameIndex0 << ' ' << name << '\n';
      return 1;
    }
  }

  vector<Vec3<int32_t>> predStructure(params.frameCount);
  predStructure[0] = Vec3<int32_t>(frameIndex0, frameIndex0, 0);
  int32_t interFrameCount = 0;
  for (int32_t f = 1; f < params.frameCount; ++f) {
    const auto frameIndex1 = params.startFrame + f;
    if (1) {
      const auto& name = vmesh::expandNum(params.inputPath, frameIndex1);
      if (!mesh1.loadFromOBJ(name)) {
        cerr << "Error: can't load frame" << frameIndex1 << ' ' << name << '\n';
        return 1;
      }
    }

    bool same = false;
    if (
      mesh1.pointCount() == mesh0.pointCount()
      && mesh1.triangleCount() == mesh0.triangleCount()
      && mesh1.pointCount() == mesh0.texCoordCount()
      && mesh1.triangleCount() == mesh0.texCoordTriangleCount()) {
      same = true;
      const auto texCoordCount = mesh0.texCoordCount();
      for (int32_t v = 0; same && v < texCoordCount; ++v) {
        const auto& texCoord0 = mesh0.texCoord(v);
        const auto& texCoord1 = mesh1.texCoord(v);
        for (int32_t k = 0; k < 2; ++k) {
          if (texCoord0[k] != texCoord1[k]) {
            same = false;
            break;
          }
        }
      }
      const auto triangleCount = mesh0.triangleCount();
      for (int32_t t = 0; same && t < triangleCount; ++t) {
        const auto& tri0 = mesh0.triangle(t);
        const auto& tri1 = mesh1.triangle(t);
        const auto& triTexCoord0 = mesh0.texCoordTriangle(t);
        const auto& triTexCoord1 = mesh1.texCoordTriangle(t);
        for (int32_t k = 0; k < 3; ++k) {
          if (tri0[k] != tri1[k] || triTexCoord0[k] != triTexCoord1[k]) {
            same = false;
            break;
          }
        }
      }
    }
    if (!same) {
      frameIndex0 = frameIndex1;
      mesh0 = mesh1;
    } else {
      cout << interFrameCount++ << ' ' << frameIndex0 << " -> " << frameIndex1
           << '\n';
    }
    predStructure[f] = Vec3<int32_t>(frameIndex1, frameIndex0, 0);
  }

  int32_t framesInGOF = 0;
  int32_t startFrameIndexGOF = params.startFrame;
  int32_t counterGOF = 0;
  for (int32_t f = 0; f < params.frameCount;) {
    const auto refFrameIndex = predStructure[f][1];
    int32_t coherentFrameCount = 0;
    while (coherentFrameCount <= params.maxGOFSize
           && f + coherentFrameCount < params.frameCount
           && predStructure[f + coherentFrameCount][1] == refFrameIndex) {
      ++coherentFrameCount;
    }

    if (framesInGOF + coherentFrameCount <= params.maxGOFSize) {
      framesInGOF += coherentFrameCount;
    } else {
      const auto start = startFrameIndexGOF;
      const auto end = startFrameIndexGOF + framesInGOF;
      assert(start >= params.startFrame);
      assert(end <= params.startFrame + params.frameCount);
      cout << "GOf[" << counterGOF << "] " << start << " -> " << end << ' '
           << framesInGOF << '\n';
      for (int32_t t = start; t < end; ++t) {
        predStructure[t - params.startFrame][2] = counterGOF;
      }
      startFrameIndexGOF = params.startFrame + f;
      framesInGOF = coherentFrameCount;
      ++counterGOF;
    }
    f += coherentFrameCount;
  }

  if (framesInGOF) {
    const auto start = startFrameIndexGOF;
    const auto end = startFrameIndexGOF + framesInGOF;
    assert(start >= params.startFrame);
    assert(end <= params.startFrame + params.frameCount);
    cout << "GOf[" << counterGOF << "] " << start << " -> " << end << ' '
         << framesInGOF << '\n';
    for (int32_t t = start; t < end; ++t) {
      predStructure[t - params.startFrame][2] = counterGOF;
    }
    ++counterGOF;
  }

  for (int32_t t = 0; t < params.frameCount; ++t) {
    const auto& pred = predStructure[t];
    fout << pred[0] << ' ' << pred[1] << ' ' << pred[2] << '\n';
  }

  return 0;
}
