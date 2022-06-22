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
#include <iostream>

#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "encoder.hpp"
#include "geometryParametrization.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

void
intra(
  const std::string referencePath,
  const std::string decimateTexturePath,
  const std::string mappedPath)
{
  disableSubProcessLog.disable();
  // Parameters 
  vmesh::VMCEncoderParameters params;
  params.geometryParametrizationSubdivisionIterationCount = 3;
  params.applySmoothingDeform = true;
  params.initialDeformForceNormalDisplacement = false;
  params.applyVertexUnification = true;
  params.initialDeformNormalDeviationThreshold = -2.0;
  params.initialDeformNNCount = 1;
  params.smoothDeformTriangleNormalFlipThreshold = -2.0;
  params.smoothingDeformUseInitialGeometry = true;
  params.smoothingDeformSmoothMotion = false;
  params.smoothDeformSmoothingMethod = vmesh::SmoothingMethod::VERTEX_CONSTRAINT;

  std::string baseNewPath = "baseMeshNew.obj";
  std::string baseNswPath = "baseMeshNsw.obj";
  std::string baseOswPath = "baseMeshOsw.obj";  
  std::string subdNewPath = "subdMeshNew.obj";
  std::string subdNswPath = "subdMeshNsw.obj";
  std::string subdOswPath = "subdMeshOsw.obj";
  std::string nsubNewPath = "nsubMeshNew.obj";
  std::string nsubNswPath = "nsubMeshNsw.obj";
  std::string nsubOswPath = "nsubMeshOsw.obj";

  // GeometryParametrization old application
  std::stringstream cmd;
  
  cmd << g_fitsubdivOldPath 
      << "  --target=" << referencePath
      << "  --source=" << decimateTexturePath 
      << "  --mapped=" << mappedPath
      << "  --base=" << baseOswPath 
      << "  --subdiv=" << subdOswPath
      << "  --nsubdiv=" << nsubOswPath
      << "  --it=" << params.geometryParametrizationSubdivisionIterationCount
      << "  --sdeform=" << params.applySmoothingDeform
      << "  --forceNormalDisp=" << params.initialDeformForceNormalDisplacement
      << "  --unifyVertices=" << params.applyVertexUnification
      << "  --deformNormalThres="
      << params.initialDeformNormalDeviationThreshold
      << "  --deformNNCount=" << params.initialDeformNNCount
      << "  --deformFlipThres="
      << params.smoothDeformTriangleNormalFlipThreshold
      << "  --useInitialGeom=" << params.smoothingDeformUseInitialGeometry
      << "  --smoothMotion=" << params.smoothingDeformSmoothMotion
      << "  --smoothMethod=" << (int)params.smoothDeformSmoothingMethod;
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // GeometryParametrization new application
  cmd.str("");
  cmd << g_fitsubdivNewPath 
      << "  --target=" << referencePath 
      << "  --source=" << decimateTexturePath 
      << "  --mapped=" << mappedPath 
      << "  --base=" << baseNswPath 
      << "  --subdiv=" << subdNswPath 
      << "  --nsubdiv=" << nsubNswPath
      << "  --it=" << params.geometryParametrizationSubdivisionIterationCount 
      << "  --sdeform=" << params.applySmoothingDeform 
      << "  --forceNormalDisp=" << params.initialDeformForceNormalDisplacement
      << "  --unifyVertices=" << params.applyVertexUnification
      << "  --deformNormalThres="
      << params.initialDeformNormalDeviationThreshold 
      << "  --deformNNCount=" << params.initialDeformNNCount 
      << "  --deformFlipThres=" << params.smoothDeformTriangleNormalFlipThreshold
      << "  --useInitialGeom=" << params.smoothingDeformUseInitialGeometry 
      << "  --smoothMotion=" << params.smoothingDeformSmoothMotion 
      << "  --smoothMethod=" << (int)params.smoothDeformSmoothingMethod;
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());  

  // New 
  vmesh::VMCFrame frame;
  vmesh::TriangleMesh<double> mtarget, subdiv0;

  // Load reference/Target mesh
  if (!frame.reference.loadFromOBJ(referencePath)) {
    std::cerr << "Error: can't load target mesh " << referencePath
              << '\n';
    return ;
  }
  // Load decimate texture/source mesh 
  if (!frame.decimateTexture.loadFromOBJ(decimateTexturePath)) {
    std::cerr << "Error: can't load source mesh " << decimateTexturePath
              << '\n';
    return ;
  }
  // Load mapped mesh
  if (!frame.mapped.loadFromOBJ(mappedPath)) {
    std::cerr << "Error: can't load mapped mesh " << mappedPath
              << '\n';
    return ;
  }

  vmesh::GeometryParametrization fitsubdiv;
  fitsubdiv.generate( frame, params, mtarget, subdiv0 );

  // Save
  if (!frame.base.saveToOBJ(baseNewPath)) {
    std::cerr << "Error: can't save base mesh!\n";
    return ;
  }
  if (!frame.subdiv.saveToOBJ(subdNewPath)) {
    std::cerr << "Error: can't save subdivision mesh!\n";
    return ;
  }
  if (!frame.nsubdiv.saveToOBJ(nsubNewPath)) {
    std::cerr << "Error: can't save normal subdivision mesh!\n";
    return ;
  }

  // Compute hashes
  auto hashBaseNew = hash(baseNewPath);
  auto hashBaseNsw = hash(baseNswPath);
  auto hashBaseOsw = hash(baseOswPath);
  auto hashSubdNew = hash(subdNewPath);
  auto hashSubdNsw = hash(subdNswPath);
  auto hashSubdOsw = hash(subdOswPath);
  auto hashNsubNew = hash(nsubNewPath);
  auto hashNsubNsw = hash(nsubNswPath);
  auto hashNsubOsw = hash(nsubOswPath);
  std::cout << "hashBaseNew = " << std::hex << hashBaseNew << "\n";
  std::cout << "hashBaseNsw = " << std::hex << hashBaseNsw << "\n";
  std::cout << "hashBaseOsw = " << std::hex << hashBaseOsw << "\n";
  std::cout << "hashSubdNew = " << std::hex << hashSubdNew << "\n";
  std::cout << "hashSubdNsw = " << std::hex << hashSubdNsw << "\n";
  std::cout << "hashSubdOsw = " << std::hex << hashSubdOsw << "\n";
  std::cout << "hashNsubNew = " << std::hex << hashNsubNew << "\n";
  std::cout << "hashNsubNsw = " << std::hex << hashNsubNsw << "\n";
  std::cout << "hashNsubOsw = " << std::hex << hashNsubOsw << "\n";

  disableSubProcessLog.enable();

  // Compare hashes
  ASSERT_EQ(hashBaseOsw, hashBaseNsw) << "Old sw and new sw base are differentes";
  ASSERT_EQ(hashBaseOsw, hashBaseNew) << "Old sw and new lib base are differentes";
  ASSERT_EQ(hashSubdOsw, hashSubdNsw) << "Old sw and new sw subdiv are differentes";
  ASSERT_EQ(hashSubdOsw, hashSubdNew) << "Old sw and new lib subdiv are differentes";
  ASSERT_EQ(hashNsubOsw, hashNsubNsw) << "Old sw and new sw nsubdiv are differentes";
  ASSERT_EQ(hashNsubOsw, hashNsubNew) << "Old sw and new lib nsubdiv are differentes";

  // Remove tmp files
  remove(baseNewPath.c_str());
  remove(baseNswPath.c_str());
  remove(baseOswPath.c_str());
  remove(subdNewPath.c_str());
  remove(subdNswPath.c_str());
  remove(subdOswPath.c_str());
  remove(nsubNewPath.c_str());
  remove(nsubNswPath.c_str());
  remove(nsubOswPath.c_str());
}

TEST(GeometryParametrization, Intra)
{
  intra(
    "data/levi_fr0000_reference.obj",
    "data/levi_fr0000_decimated_tex.obj",
    "data/levi_fr0000_mapped.obj");
}
