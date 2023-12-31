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
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

#include <fstream>  // looks like we need this too (edit by π)
#include <program-options-lite/program_options_lite.h>

#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "version.hpp"
#include "bitstream.hpp"

#include "geometryEncoder.hpp"

//============================================================================

struct Parameters {
  bool                   verbose{};
  std::string            inputPath;
  std::string            bitstreamPath;
  std::string            outputPath;
  vmesh::GeometryCodecId codecId;

  vmesh::GeometryEncoderParameters params;
};

//============================================================================

static bool
parseParameters(int argc, char* argv[], Parameters& params) try {
  namespace po = df::program_options_lite;

  bool print_help = false;

  /* clang-format off */
  po::Options opts;
  opts.addOptions()
  ("help", print_help, false, "This help text")
  ("config,c", po::parseConfigFile, "Configuration file name")
  ("verbose,v", params.verbose, true, "Verbose output")

  (po::Section("Input/Output"))
  ("inputPath",     params.inputPath,     {}, "Input mesh path")
  ("bitstreamPath", params.bitstreamPath, {}, "Generated bitstream path")
  ("recPath",       params.outputPath,    {}, "Reconstructed mesh path")
  ("codecId",       params.codecId,       vmesh::GeometryCodecId::UNKNOWN_GEOMETRY_CODEC, 
    "Video codec Id: DRACO")  
    
  (po::Section("Draco"))
  ("qp",  params.params.qp_, 11, "qp")
  ("qt",  params.params.qt_, 10, "qt")
  ("qn",  params.params.qn_, -1, "qn")
  ("qg",  params.params.qg_, -1, "qg")
  ("cl",  params.params.cl_, 10, "cl")
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

  if (params.inputPath.empty()) { err.error() << "Src mesh not specified\n"; }
  if (params.bitstreamPath.empty()) {
    err.error() << "Output bitstream path not specified\n";
  }
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

int
main(int argc, char* argv[]) {
  std::cout << "MPEG VMESH version " << ::vmesh::version << '\n';

  Parameters params;
  if (!parseParameters(argc, argv, params)) { return 1; }

  if (params.verbose) { vmesh::vout.rdbuf(std::cout.rdbuf()); }

  // Load input mesh
  vmesh::TriangleMesh<double> mesh;
  mesh.load(params.inputPath);

  // Encode
  vmesh::TriangleMesh<double> rec;
  vmesh::Bitstream            bitstream;
  auto encoder = vmesh::GeometryEncoder<double>::create(params.codecId);
  encoder->encode(mesh, params.params, bitstream.vector(), rec);

  // Save reconstructed mesh
  rec.save(params.outputPath);

  // Save bitstream
  bitstream.save(params.bitstreamPath);

  return 0;
}
