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
#include "simplifyer.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

void create( const std::string inputPath)
{
  disableSubProcessLog.disable();
  // Parameters
  vmesh::VMCSimplifyParameters params;
  params.targetTriangleRatio = 0.03;
  params.minCCTriangleCount = 8;
  params.texCoordQuantizationBits = 13;

  std::string mappedNewPath = "mappedNew.obj";
  std::string refereNewPath = "refereNew.obj";
  std::string decimaNewPath = "decimaNew.obj";
  std::string mappedNswPath = "mappedNsw.obj";
  std::string refereNswPath = "refereNsw.obj";
  std::string decimaNswPath = "decimaNsw.obj";
  std::string mappedOswPath = "mappedOsw.obj";
  std::string refereOswPath = "refereOsw.obj";
  std::string decimaOswPath = "decimaOsw.obj";

  //  Simplify old application
  std::stringstream cmd;
  cmd << g_simplifyOldPath << " "
      << "  --input=" << inputPath << " "
      << "  --decimated=" << decimaOswPath << " "
      << "  --mapped=" << mappedOswPath << " "
      << "  --reference=" << refereOswPath << " "
      << "  --target=" << params.targetTriangleRatio << " "
      << "  --qt=" << params.texCoordQuantizationBits << " "
      << "  --cctcount=" << params.minCCTriangleCount << " ";
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  //  Simplify new application
  cmd.str("");
  cmd << g_simplifyNewPath << " "
      << "  --input=" << inputPath << " "
      << "  --decimated=" << decimaNswPath << " "
      << "  --mapped=" << mappedNswPath << " "
      << "  --reference=" << refereNswPath << " "
      << "  --target=" << params.targetTriangleRatio << " "
      << "  --qt=" << params.texCoordQuantizationBits << " "
      << "  --cctcount=" << params.minCCTriangleCount << " ";
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Load input mesh   
  vmesh::VMCFrame frame;
  if (!frame.input.loadFromOBJ( inputPath ) ) {
    std::cerr << "Error: can't load " << inputPath << "!\n";
    return ;
  }

  // Simplify 
  vmesh::Simplifyer Simplifyer;
  if (Simplifyer.simplify(frame, params )) {
    std::cerr << "Error: can't simplify mesh !\n";
    return ;
  }

  // Save generated meshes
  if (!frame.decimate.saveToOBJ(decimaNewPath)) {
    std::cerr << "Error: can't save decimated mesh\n";
    return ;
  }
  if (!frame.mapped.saveToOBJ(mappedNewPath)) {
    std::cerr << "Error: can't save mapped mesh\n";
    return ;
  }
  if (!frame.reference.saveToOBJ(refereNewPath)) {
    std::cerr << "Error: can't save reference mesh\n";
    return ;
  }

  // Compute hashes
  auto hashMappedNew = hash(mappedNewPath);
  auto hashRefereNew = hash(refereNewPath);
  auto hashDecimaNew = hash(decimaNewPath);
  auto hashMappedNsw = hash(mappedNswPath);
  auto hashRefereNsw = hash(refereNswPath);
  auto hashDecimaNsw = hash(decimaNswPath);
  auto hashMappedOsw = hash(mappedOswPath);
  auto hashRefereOsw = hash(refereOswPath);
  auto hashDecimaOsw = hash(decimaOswPath);

  std::cout << "hashMappedNew = " << std::hex << hashMappedNew << "\n";
  std::cout << "hashMappedNsw = " << std::hex << hashMappedNsw << "\n";
  std::cout << "hashMappedOsw = " << std::hex << hashMappedOsw << "\n";
  std::cout << "hashDecimaNew = " << std::hex << hashDecimaNew << "\n";
  std::cout << "hashDecimaNsw = " << std::hex << hashDecimaNsw << "\n";
  std::cout << "hashDecimaOsw = " << std::hex << hashDecimaOsw << "\n";
  std::cout << "hashRefereNsw = " << std::hex << hashRefereNsw << "\n";
  std::cout << "hashRefereNew = " << std::hex << hashRefereNew << "\n";
  std::cout << "hashRefereOsw = " << std::hex << hashRefereOsw << "\n";

  disableSubProcessLog.enable();

  // Compare hashes
  ASSERT_EQ(hashRefereNew, hashRefereNsw) << "New and new sw reference are differentes";
  ASSERT_EQ(hashRefereNew, hashRefereOsw) << "New and old sw reference are differentes";
 
  ASSERT_EQ(hashDecimaNew, hashDecimaNsw) << "New and new sw decimate are differentes";
  ASSERT_EQ(hashDecimaNew, hashDecimaOsw) << "New and old sw decimate are differentes";
 
  ASSERT_EQ(hashMappedNew, hashMappedNsw) << "New and new sw mapped are differentes";
  ASSERT_EQ(hashMappedNew, hashMappedOsw) << "New and old sw mapped are differentes";
  
  // Remove tmp files
  remove( mappedNewPath.c_str() );
  remove( refereNewPath.c_str() );
  remove( decimaNewPath.c_str() );
  remove( mappedNswPath.c_str() );
  remove( refereNswPath.c_str() );
  remove( decimaNswPath.c_str() );
  remove( mappedOswPath.c_str() );
  remove( refereOswPath.c_str() );
  remove( decimaOswPath.c_str() );
}

TEST(Simplify, create)
{
  create("data/levi_fr0000_qp12_qt13.obj");
}
