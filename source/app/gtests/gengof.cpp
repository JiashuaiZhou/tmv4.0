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
#include "sequenceInfo.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

void create( 
    const int frameCount,
    const int startFrame,
    const int maxGOFSize,
    const std::string inputPath)
{
  disableSubProcessLog.disable();
  std::string gofNewPath = "gof_new.gof";
  std::string gofSavPath = "gof_sav.gof";
  std::string gofNswPath = "gof_nsw.gof";
  std::string gofOswPath = "gof_osw.gof";

  // Generate gof structure new 
  vmesh::SequenceInfo sequenceInfo;
  sequenceInfo.generate( frameCount, startFrame, maxGOFSize, inputPath );
  sequenceInfo.save( gofNewPath );

  // Reload sequence info
  vmesh::SequenceInfo readSequenceInfo;
  readSequenceInfo.load( frameCount, startFrame, maxGOFSize, gofNewPath );
  readSequenceInfo.save( gofSavPath );

  //  Generate gof structure old application
  std::stringstream cmd;
  cmd << g_gengofOldPath << " "
      << "  --input=" << inputPath   << " "
      << "  --output=" << gofOswPath << " "
      << "  --startFrame=" << startFrame << " "  
      << "  --frameCount=" << frameCount;
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());  
  
  //  Generate gof structure old application
  cmd.str("");
  cmd << g_gengofNewPath << " "
      << "  --input=" << inputPath   << " "
      << "  --output=" << gofNswPath << " "
      << "  --startFrame=" << startFrame << " "  
      << "  --frameCount=" << frameCount;
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Compute hashes
  auto hashNew = hash(gofNewPath);
  auto hashOsw = hash(gofOswPath);
  auto hashNsw = hash(gofNswPath);
  auto hashSav = hash(gofSavPath);
  std::cout << "hashNew = " << std::hex << hashNew << "\n";
  std::cout << "hashOsw = " << std::hex << hashOsw << "\n";
  std::cout << "hashNsw = " << std::hex << hashNsw << "\n";
  std::cout << "hashSav = " << std::hex << hashSav << "\n";

  disableSubProcessLog.enable();

  // Compare hashes
  ASSERT_EQ(hashNew, hashOsw)<< "New and old sw gof structure are differentes";
  ASSERT_EQ(hashNsw, hashOsw)<< "New sw and old sw gof structure are differentes";
  ASSERT_EQ(hashNew, hashSav)<< "New and save gof structure are differentes";    

  // Remove tmp files
  remove( gofNewPath.c_str() );
  remove( gofOswPath.c_str() );
  remove( gofNswPath.c_str() );
  remove( gofSavPath.c_str() );
}


TEST(Gengof, create)
{
  create(
    150, 0, 32,
    "/home/library24/PCC/contents/mpeg_vmesh_cfp_v02/contents/voxelized/"
    "levi_voxelized/levi_fr%04d_qp12_qt13.obj");
}
