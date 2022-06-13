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
#include <cstdint>
#include <string>
#include <program-options-lite/program_options_lite.h>

#include "mesh.hpp"
#include "misc.hpp"
#include "verbose.hpp"
#include "version.hpp"
#include "simplifyer.hpp"
#include "vmc.hpp"

//============================================================================

namespace {
struct Parameters {
  bool verbose;

  // frame number for expansion in input/output filenames
  int fnum;

  std::string inputMeshPath;
  std::string decimatedMeshPath;
  std::string mappedMeshPath;
  std::string referenceMeshPath;
  vmesh::VMCSimplifyParameters params;
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

  (po::Section("Input/Output"))
  ("fnum",      params.fnum,              {}, "Frame number for %d expansion")
  ("input",     params.inputMeshPath,     {}, "Input mesh")
  ("decimated", params.decimatedMeshPath, {}, "Decimated mesh")
  ("mapped",    params.mappedMeshPath,    {}, "Mapped mesh")
  ("reference", params.referenceMeshPath, {}, "Reference mesh")

  (po::Section("Simplification"))
  ("target", params.params.targetTriangleRatio, 0.125,
   "Target triangle count ratio")

  ("qt", params.params.texCoordQuantizationBits, 0,
   "texture coordinate quantization bits")

  ("cctcount", params.params.minCCTriangleCount, 0,
   "minimum triangle count per connected component")
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

  if (params.inputMeshPath.empty())
    err.error() << "input mesh not specified\n";

  if (params.decimatedMeshPath.empty())
    err.error() << "decimated output mesh not specified\n";

  if (params.mappedMeshPath.empty())
    err.error() << "mapped output mesh not specified\n";

  if (params.referenceMeshPath.empty())
    err.error() << "reference mesh not specified\n";

  if (err.is_errored)
    return false;

  // Dump the complete derived configuration
  std::cout << "+ Configuration parameters\n";
  po::dumpCfg(std::cout, opts, "Input/Output", 4);
  po::dumpCfg(std::cout, opts, "Simplification", 4);
  std::cout << '\n';

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
  std::cout << "MPEG VMESH version " << ::vmesh::version << '\n';

  Parameters params;
  if (!parseParameters(argc, argv, params)) {
    return 1;
  }

  if (params.verbose)
    vmesh::vout.rdbuf(std::cout.rdbuf());

  // Load input mesh   
  vmesh::VMCFrame frame;
  if (!frame.input.loadFromOBJ( vmesh::expandNum(params.inputMeshPath, params.fnum) )) {
    std::cerr << "Error: can't load " << params.inputMeshPath << "!\n";
    return 1;
  }

  // Simplify 
  vmesh::Simplifyer Simplifyer;
  if (Simplifyer.simplify(frame, params.params )) {
    std::cerr << "Error: can't simplify mesh !\n";
    return 1;
  }

  // Save generated meshes
  if (!frame.decimate.saveToOBJ(vmesh::expandNum(params.decimatedMeshPath, params.fnum))) {
    std::cerr << "Error: can't save decimated mesh\n";
    return 1;
  }
  if (!frame.mapped.saveToOBJ(vmesh::expandNum(params.mappedMeshPath, params.fnum))) {
    std::cerr << "Error: can't save mapped mesh\n";
    return 1;
  }
  if (!frame.reference.saveToOBJ(
        vmesh::expandNum(params.referenceMeshPath, params.fnum))) {
    std::cerr << "Error: can't save reference mesh\n";
    return 1;
  }

  return 0;
}
