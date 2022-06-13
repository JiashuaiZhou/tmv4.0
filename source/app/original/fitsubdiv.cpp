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

#include <cassert>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <program-options-lite/program_options_lite.h>

#include "misc.hpp"
#include "verbose.hpp"
#include "version.hpp"
#include "mesh.hpp"
#include "fitsubdiv.hpp"

//============================================================================

struct Parameters {
  bool verbose;

  ///
  // Frame number.
  // Used in the expansion of input/output filenames.
  int fnum;

  ///
  // Reference frame number.
  // Used in the expansion of input filenames.
  int fnumRef;

  std::string targetMeshPath;
  std::string sourceMeshPath;
  std::string targetModifierMeshPath;
  std::string mappedMeshPath;
  std::string subdiv0MeshPath;
  std::string baseMeshPath;
  std::string subdivMeshPath;
  std::string nsubdivMeshPath;
  
  vmesh::VMCFitSubDivParameters params; 
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
  ("fnum",     params.fnum, {},    "Frame number for %d expansion")
  ("rnum",     params.fnumRef, -1, "Reference frame number for %d expansion")

  ("source,i", params.sourceMeshPath,  {}, "Source mesh")
  ("target",   params.targetMeshPath,  {}, "Target mesh")
  ("mtarget",  params.targetModifierMeshPath, {}, "Target modifier mesh")
  ("mapped",   params.mappedMeshPath,  {}, "Mapped mesh")
  ("subdiv0",  params.subdiv0MeshPath, {}, "Initial deform mesh")
  ("base",     params.baseMeshPath,    {}, "Base mesh")
  ("subdiv",   params.subdivMeshPath,  {}, "Subdivision mesh")
  ("nsubdiv",  params.nsubdivMeshPath, {}, "Normal subdivision mesh")

  (po::Section("Subdiv"))
  ("sdeform",
   params.params.applySmoothingDeform, true,
   "Apply deformation refinement stage")

  ("it",
   params.params.subdivisionIterationCount, 3,
   "Subdivision iteration count")

  ("forceNormalDisp",
   params.params.initialDeformForceNormalDisplacement, false,
   "Force displacements to aligned with the surface normals")

  ("unifyVertices",
   params.params.applyVertexUnification, true,
   "Unify duplicated vertices")

  ("deformNNCount",
   params.params.initialDeformNNCount, 1,
   "Number of nearest neighbours used during the initial deformation stage")

  ("deformNormalThres",
   params.params.initialDeformNormalDeviationThreshold, 0.1,
   "Maximum allowed normal deviation during the initial deformation stage")

  ("sampIt",
   params.params.geometrySamplingSubdivisionIterationCount, 3,
   "Number of subdivision iterations used for geometry sampling")

  ("fitIt",
   params.params.geometryFittingIterationCount, 16,
   "Number of iterations used during the deformation refinement stage")

  ("smoothCoeff",
   params.params.geometrySmoothingCoeffcient, 0.25,
   "Initial smoothing coefficient used to smooth the deformed mesh "
   "during deformation refinement")

  ("smoothDecay",
   params.params.geometrySmoothingCoeffcientDecayRatio, 0.75,
   "Decay factor applied to intial smoothing coefficient after every "
   "iteration of deformation refinement")

  ("smoothMissedCoeff",
   params.params.geometryMissedVerticesSmoothingCoeffcient, 0.1,
   "Smoothing coefficient applied to the missed vertices")

  ("smoothMissedIt",
   params.params.geometryMissedVerticesSmoothingIterationCount, 10,
   "Number of iterations when smoothing the positions of the missed vertices")

  ("smoothMethod",
   params.params.smoothDeformSmoothingMethod, vmesh::SmoothingMethod::VERTEX_CONSTRAINT,
   "Smoothing method to be applied when smoothing the deformed mesh during"
   "the deformation refinement stage")

  ("deformUpdateNormals",
   params.params.smoothDeformUpdateNormals, true,
   "Recompute normals after each iteration of deformation refinement")

  ("deformFlipThres",
   params.params.smoothDeformTriangleNormalFlipThreshold, -0.5,
   "Threshold to detect triangle normals flip")

  ("useInitialGeom",
   params.params.smoothingDeformUseInitialGeometry, true,
   "Use the initial geometry during the the deformation refinement stage")

  ("fitSubdiv",
   params.params.fitSubdivisionSurface, true,
   "Update the positions of the decimated mesh to minimize displacements "
   "between the subdivided mesh and the deformed mesh")

  ("smoothMotion",
   params.params.smoothingDeformSmoothMotion, true,
   "Apply smoothing to motion instead of vertex positions")
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

  if (params.sourceMeshPath.empty())
    err.error() << "input mesh not specified\n";

  if (params.targetMeshPath.empty())
    err.error() << "target mesh not specified\n";

  if (params.subdivMeshPath.empty())
    err.error() << "subdivision mesh not specified\n";

  if (err.is_errored)
    return false;

  // If --rnum is not set, default to --fnum
  if (params.fnumRef < 0)
    params.fnumRef = params.fnum;

  // expand path names
  // if mtarget is specified, target is reference frame
  // otherwise,               target is current frame
  params.targetMeshPath = vmesh::expandNum(params.targetMeshPath,
    params.targetModifierMeshPath.empty() ? params.fnum : params.fnumRef);

  params.targetModifierMeshPath =
    vmesh::expandNum(params.targetModifierMeshPath, params.fnum);

  // reference frames use fnumRef
  params.sourceMeshPath = vmesh::expandNum(params.sourceMeshPath, params.fnumRef);
  params.mappedMeshPath = vmesh::expandNum(params.mappedMeshPath, params.fnumRef);
  params.subdiv0MeshPath = vmesh::expandNum(params.subdiv0MeshPath, params.fnumRef);

  // output file names use fnum
  params.baseMeshPath = vmesh::expandNum(params.baseMeshPath, params.fnum);
  params.nsubdivMeshPath = vmesh::expandNum(params.nsubdivMeshPath, params.fnum);
  params.subdivMeshPath = vmesh::expandNum(params.subdivMeshPath, params.fnum);

  // Dump the complete derived configuration
  std::cout << "+ Configuration parameters\n";
  po::dumpCfg(std::cout, opts, "Input/Output", 4);
  po::dumpCfg(std::cout, opts, "Subdiv", 4);
  std::cout << '\n';

  return true;
}
catch (df::program_options_lite::ParseFailure& e) {
  std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
            << e.val << "\".\n";
  return false;
}

//============================================================================

bool
LoadInputMeshes(
  vmesh::TriangleMesh<double>& target,
  vmesh::TriangleMesh<double>& source,
  vmesh::TriangleMesh<double>& mapped,
  vmesh::TriangleMesh<double>& mtarget,
  const Parameters& params)
{
  return true;
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

  
  vmesh::VMCFrame frame;
  vmesh::TriangleMesh<double> mtarget, subdiv0;
  
  // Load reference/Target mesh
  if (!frame.reference.loadFromOBJ(params.targetMeshPath)) {
    std::cerr << "Error: can't load target mesh " << params.targetMeshPath
              << '\n';
    return 1;
  }
  // Load decimate texture/source mesh 
  if (!frame.decimateTexture.loadFromOBJ(params.sourceMeshPath)) {
    std::cerr << "Error: can't load source mesh " << params.sourceMeshPath
              << '\n';
    return 1;
  }
  // Load mapped mesh
  if (!params.mappedMeshPath.empty()) {
    if (!frame.mapped.loadFromOBJ(params.mappedMeshPath)) {
      std::cerr << "Error: can't load mapped mesh " << params.mappedMeshPath
                << '\n';
      return 1;
    }
  }
  // Load target modifier mesh/mtarget
  if (!params.targetModifierMeshPath.empty()) {
    if (!mtarget.loadFromOBJ(params.targetModifierMeshPath)) {
      std::cerr << "Error: can't load target modifier mesh "
                << params.targetModifierMeshPath << '\n';
      return 1;
    }
  }
  // Load target modifier mesh/mtarget
  if (!params.subdiv0MeshPath.empty()) {
    if (!subdiv0.loadFromOBJ(params.subdiv0MeshPath)) {
      std::cerr << "Error: can't load target modifier mesh "
                << params.subdiv0MeshPath << '\n';
      return 1;
    }
  }

  vmesh::FitSubDiv fitsubdiv;
  fitsubdiv.generate( frame, params.params, mtarget, subdiv0 );

  // Save
  if (!params.baseMeshPath.empty()) {
    const auto& name = vmesh::expandNum(params.baseMeshPath, params.fnum);
    if (!frame.base.saveToOBJ(name)) {
      std::cerr << "Error: can't save base mesh!\n";
      return 1;
    }
  }

  if (!params.subdivMeshPath.empty()) {
    const auto& name = vmesh::expandNum(params.subdivMeshPath, params.fnum);
    if (!frame.subdiv.saveToOBJ(name)) {
      std::cerr << "Error: can't save subdivision mesh!\n";
      return 1;
    }
  }

  if (!params.nsubdivMeshPath.empty()) {
    const auto& name = vmesh::expandNum(params.nsubdivMeshPath, params.fnum);
    if (!frame.nsubdiv.saveToOBJ(name)) {
      std::cerr << "Error: can't save normal subdivision mesh!\n";
      return 1;
    }
  }

  return 0;
}
