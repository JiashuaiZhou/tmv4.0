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

#include "metrics.hpp"
#include "misc.hpp"
#include "verbose.hpp"
#include "version.hpp"

//============================================================================

struct Parameters {
  bool verbose;
  std::string srcMeshPath;
  std::string srcTexturePath;
  std::string decMeshPath;
  std::string decTexturePath;
  int32_t startFrame;
  int32_t frameCount;
  vmesh::VMCMetricsParameters metParams;
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

  (po::Section("Input/Output"))
  ("srcMesh",  params.srcMeshPath,    {}, "Source mesh")
  ("srcTex",   params.srcTexturePath, {}, "Source texture")
  ("decMesh",  params.decMeshPath,    {}, "Reconsctructed/decoded mesh")
  ("decTex",   params.decTexturePath, {}, "Reconsctructed/decoded texture")
  ("fstart", params.startFrame,      1, "First frame number")
  ("fcount", params.frameCount,      1, "Number of frames")
  
  (po::Section("Metrics"))
  ("computeMetrics", params.metParams.computeMetrics, true, "Compute metrics")
  ("gridSize", params.metParams.gridSize, 1024, "Grid size")
  ("minPosition", params.metParams.minPosition,  {0,0,0}, "min position")
  ("maxPosition", params.metParams.minPosition,  {0,0,0}, "min position")

  ("pcqmRadiusCurvature", params.metParams.pcqmRadiusCurvature,  0.001, 
    "PCQM radius curvature")
  ("pcqmThresholdKnnSearch", params.metParams.pcqmThresholdKnnSearch, 20, 
    "PCQM threshold Knn search")
  ("pcqmRadiusFactor", params.metParams.pcqmRadiusFactor,   2.0, 
    "PCQM radius factor")  ;
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

  if (params.srcMeshPath.empty())
    err.error() << "Src mesh not specified\n";
  if (params.srcTexturePath.empty())
    err.error() << "Src texture not specified\n";
  if (params.decMeshPath.empty())
    err.error() << "Rec/dec mesh not specified\n";
  if (params.decTexturePath.empty())
    err.error() << "Rec/dec texture not specified\n";
  if (err.is_errored)
    return false;

  // Dump the complete derived configuration
  std::cout << "+ Configuration parameters\n";
  po::dumpCfg(std::cout, opts, "Input/Output", 4);
  po::dumpCfg(std::cout, opts, "Metrics", 4);  
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
metrics(const Parameters& params)
{
  vmesh::VMCMetrics metrics;
  const int lastFrame =  params.startFrame +  params.frameCount;
  for (int f = params.startFrame; f <= lastFrame; ++f) {
    vmesh::VMCGroupOfFrames gof;
    gof.resize(1);
    const auto nameSrcMesh = vmesh::expandNum(params.srcMeshPath, f);
    const auto nameSrcTexture = vmesh::expandNum(params.srcTexturePath, f);
    const auto nameRecMesh = vmesh::expandNum(params.decMeshPath, f);
    const auto nameRecTexture = vmesh::expandNum(params.decTexturePath, f);
    auto& frame = gof.frames[0];
    if (!frame.input.loadFromOBJ(nameSrcMesh)) {
      printf(
        "Error loading src mesh %d / %d: %s \n", f, params.frameCount,
        nameSrcMesh.c_str());
      return -1;
    }
    if (!vmesh::LoadImage(nameSrcTexture, frame.inputTexture)) {
      printf(
        "Error loading src texture %d / %d: %s \n", f, params.frameCount,
        nameSrcTexture.c_str());
      return -1;
    }
    if (!frame.rec.loadFromOBJ(nameRecMesh)) {
      printf(
        "Error loading rec mesh %d / %d: %s \n", f, params.frameCount,
        nameRecMesh.c_str());
      return -1;
    }
    if (!vmesh::LoadImage(nameRecTexture, frame.outputTexture)) {
      printf(
        "Error loading rec texture %d / %d: %s \n", f, params.frameCount,
        nameRecTexture.c_str());
      return -1;
    }
    metrics.compute( gof, params.metParams);
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

  if ( metrics(params)) {
    std::cerr << "Error: can't compute metrics!\n";
    return 1;
  } 

  return 0;
}
