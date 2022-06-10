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

#include "misc.hpp"
#include "verbose.hpp"
#include "version.hpp"
#include "vmc.hpp"
 #include "reparametrization.hpp"
#include "simplifyMesh.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

void reparametrize( const std::string inputPath)
{
  // Parameters
  vmeshenc::VMCReparametrizationParameters params;
  params.width = 4096;
  params.height = 4096;
  params.uvOptions = DirectX::UVATLAS_GEODESIC_QUALITY;
  params.gutter = 32;

  std::string outputNewPath = "decimateTextureNew.obj";
  std::string outputNswPath = "decimateTextureNsw.obj";
  std::string outputOswPath = "decimateTextureOsw.obj";

  std::string decimatePath = "decimate.obj";

  // Generate reference mesh
  vmeshenc::VMCSimplifyParameters paramsSimplify;
  paramsSimplify.targetTriangleRatio = 0.03;
  paramsSimplify.minCCTriangleCount = 8;
  paramsSimplify.texCoordQuantizationBits = 13;
  vmesh::VMCFrame frame0;
  if (!frame0.input.loadFromOBJ( inputPath ) ) {
    std::cerr << "Error: can't load " << inputPath << "!\n";
    return ;
  }
  vmeshenc::SimplifyMesh simplifyMesh;
  if (simplifyMesh.simplify(frame0, paramsSimplify )) {
    std::cerr << "Error: can't simplify mesh !\n";
    return ;
  }
  if (!frame0.decimate.saveToOBJ(decimatePath)) {
    std::cerr << "Error: can't save decimated mesh\n";
    return ;
  }

  // Reparametrization old application
  std::stringstream cmd;
  cmd << g_uvatlasOldPath << " "
      << "  --input=" << decimatePath   << " "
      << "  --output=" << outputOswPath << " "
      << "  --width=" << params.width << " "  
      << "  --height=" << params.height << " "  
      << "  --gutter=" << params.gutter << " "  
      << "  --quality=QUALITY " ;
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());  
  
  // Reparametrization new application
  cmd.str("");
  cmd << g_uvatlasNewPath << " "
      << "  --input=" << decimatePath   << " "
      << "  --output=" << outputNswPath << " "
      << "  --width=" << params.width << " "  
      << "  --height=" << params.height << " "  
      << "  --gutter=" << params.gutter << " "  
      << "  --quality=QUALITY " ;
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Load input mesh   
  // vmesh::VMCFrame frame;
  printf("START LIB REPARAMETRIZATION \n");
  vmesh::VMCFrame frame;
  
  if (!frame.decimate.loadFromOBJ( decimatePath ) ) {
    std::cerr << "Error: can't load " << inputPath << "!\n";
    return ;
  }

  // Reparametrization
  vmeshenc::Reparametrization reparametrization;
  reparametrization.generate(frame, params);

  // Save generated meshes
  if (!frame.decimateTexture.saveToOBJ(outputNewPath)) {
    std::cerr << "Error: can't save decimated mesh\n";
    return;
  }

  // Compute hashes
  auto hashNew = hash(outputNewPath);
  auto hashOsw = hash(outputOswPath);
  auto hashNsw = hash(outputNswPath);
  std::cout << "hashOsw = " << std::hex << hashOsw << "\n";
  std::cout << "hashNsw = " << std::hex << hashNsw << "\n";
  std::cout << "hashNew = " << std::hex << hashNew << "\n";

  disableSubProcessLog.enable();

  // Compare hashes
  ASSERT_EQ(hashNew, hashNsw) << "New and new sw reparametrizations are differentes";
  ASSERT_EQ(hashNew, hashOsw) << "New and old sw reparametrizations are differentes";
 
  // Remove tmp files
  remove( outputOswPath.c_str() );
  remove( outputNswPath.c_str() );
  remove( outputNewPath.c_str() );
  remove( decimatePath.c_str() );
}

TEST(Reparametrization, generate)
{
  reparametrize("data/levi_fr0000_qp12_qt13.obj");
}
