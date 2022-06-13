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
#include "encoder.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

void
encode(
  const std::string srcPath,
  const std::string texPath,
  const std::string basePath,
  const std::string subdivPath,
  const size_t framecount )
{
  disableSubProcessLog.disable();
  // Parameters 
  vmesh::VMCEncoderParameters params;
  params.textureVideoHDRToolDecConfig = "cfg/hdrconvert/yuv420tobgr444.cfg"; 
  params.normalizeUV = 0;
  params.keepIntermediateFiles = 1;

  
  std::string binNewPath = "bistream_new.vdmc";
  std::string binNswPath = "bistream_nsw.vdmc";
  std::string binOswPath = "bistream_osw.vdmc";
  std::string objNewPath = "rec_fr%04d_new.obj";
  std::string objNswPath = "rec_fr%04d_nsw.obj";
  std::string objOswPath = "rec_fr%04d_osw.obj";  
  std::string texNewPath = "rec_fr%04d_new.png";
  std::string texNswPath = "rec_fr%04d_nsw.png";
  std::string texOswPath = "rec_fr%04d_osw.png";
  std::string mtlPath = "rec_fr%04d.mtl";

  // Encode old application
  std::stringstream cmd;
  /* clang-format off */
  cmd << g_encoderOldPath << "  "
      << "  --mode="         << 0
      << "  --imesh="        << srcPath
      << "  --itex="         << texPath
      << "  --base="         << basePath
      << "  --subdiv="       << subdivPath
      << "  --compressed="   << binOswPath
      << "  --recmesh="      << objOswPath
      << "  --rectex="       << texOswPath
      << "  --recmat="       << mtlPath
      << "  --fstart="       << 0
      << "  --fcount="       << framecount
      << "  --framerate="    << 30
      << "  --keep="         << params.keepIntermediateFiles
      << "  --it="           << 3
      << "  --gqp="          << 11
      << "  --tqp="          << 10
      << "  --dqp="          << "39,51,51"
      << "  --dqb="          << "0.3333,0.333333,0.333333"
      << "  --tvqp="         << 38
      << "  --gdepth="       << 12
      << "  --tdepth="       << 13
      << "  --texwidth="     << 512
      << "  --texheight="    << 512
      << "  --invorient="    << 0
      << "  --unifvertices=" << 0
      << "  --encdisp="      << 1
      << "  --enctex="       << 1
      << "  --normuv="       << 0
      << "  --gmenc="        << g_dracoEncoderPath
      << "  --gmdec="        << g_dracoDecoderPath
      << "  --gvenc="        << g_hmEncoderPath
      << "  --gvencconfig="  << "cfg//hm/ctc-hm-displacements-map-ai-main10.cfg"
      << "  --tvenc="        << g_hmEncoderPath
      << "  --tvencconfig="  << "cfg//hm/ctc-hm-texture-ai.cfg"
      << "  --csc="          << g_hdrConvertPath
      << "  --cscencconfig=" << "cfg//hdrconvert/bgr444toyuv420.cfg"
      << "  --cscdecconfig=" << "cfg//hdrconvert/yuv420tobgr444.cfg";
  /* clang-format on */
   if (disableSubProcessLog.disableLog())
     cmd << " 2>&1 > /dev/null";
   printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // encoder new application
  cmd.str("");
  
  /* clang-format off */
  cmd << g_encoderNewPath << "  "
      << "  --imesh="        << srcPath
      << "  --itex="         << texPath
      << "  --base="         << basePath
      << "  --subdiv="       << subdivPath
      << "  --compressed="   << binNswPath
      << "  --recmesh="      << objNswPath
      << "  --rectex="       << texNswPath
      << "  --recmat="       << mtlPath
      << "  --fstart="       << 0
      << "  --fcount="       << framecount
      << "  --framerate="    << 30
      << "  --keep="         << params.keepIntermediateFiles
      << "  --it="           << 3
      << "  --gqp="          << 11
      << "  --tqp="          << 10
      << "  --dqp="          << "39,51,51"
      << "  --dqb="          << "0.3333,0.333333,0.333333"
      << "  --tvqp="         << 38
      << "  --gdepth="       << 12
      << "  --tdepth="       << 13
      << "  --texwidth="     << 512
      << "  --texheight="    << 512
      << "  --invorient="    << 0
      << "  --unifvertices=" << 0
      << "  --encdisp="      << 1
      << "  --enctex="       << 1
      << "  --normuv="       << 0
      << "  --gmenc="        << g_dracoEncoderPath
      << "  --gmdec="        << g_dracoDecoderPath
      << "  --gvenc="        << g_hmEncoderPath
      << "  --gvencconfig="  << "cfg//hm/ctc-hm-displacements-map-ai-main10.cfg"
      << "  --tvenc="        << g_hmEncoderPath
      << "  --tvencconfig="  << "cfg//hm/ctc-hm-texture-ai.cfg"
      << "  --csc="          << g_hdrConvertPath
      << "  --cscencconfig=" << "cfg//hdrconvert/bgr444toyuv420.cfg"
      << "  --cscdecconfig=" << "cfg//hdrconvert/yuv420tobgr444.cfg";
  /* clang-format on */
   if (disableSubProcessLog.disableLog())
     cmd << " 2>&1 > /dev/null";
   printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());


  disableSubProcessLog.enable();

  // Compute hashes
  //auto hashBinNew = hash(binNewPath);
  auto hashBinNsw = hash(binNswPath);
  auto hashBinOsw = hash(binOswPath);
  //std::cout << "hashBinNew " << f << " = " << std::hex << hashBinNew << "\n";
  std::cout << "hashBinNsw = " << std::hex << hashBinNsw << "\n";
  std::cout << "hashBinOsw = " << std::hex << hashBinOsw << "\n";

  // Compare hashes
  ASSERT_EQ(hashBinOsw, hashBinNsw) << "Old sw and new sw bin are differentes";
  // ASSERT_EQ(hashBinOsw, hashBinNew)      << "Old sw and new lib bin are differentes";

  for (int f = 0; f < framecount; f++) {
    //auto hashObjNew = hash(vmesh::expandNum(objNewPath, f));
    auto hashObjNsw = hash(vmesh::expandNum(objNswPath, f));
    auto hashObjOsw = hash(vmesh::expandNum(objOswPath, f));
    // auto hashTexNew = hash(vmesh::expandNum(texNewPath, f));
    auto hashTexNsw = hash(vmesh::expandNum(texNswPath, f));
    auto hashTexOsw = hash(vmesh::expandNum(texOswPath, f));
    if( !disableSubProcessLog.disableLog()  ){
      //std::cout << "hashObjNew " << f << " = " << std::hex << hashObjNew << "\n";
      std::cout << "hashObjNsw " << f << " = " << std::hex << hashObjNsw << "\n";
      std::cout << "hashObjOsw " << f << " = " << std::hex << hashObjOsw << "\n";
      // std::cout << "hashTexNew " << f << " = " << std::hex << hashTexNew << "\n";
      std::cout << "hashTexNsw " << f << " = " << std::hex << hashTexNsw << "\n";
      std::cout << "hashTexOsw " << f << " = " << std::hex << hashTexOsw << "\n";
    }

    // Compare hashes
    ASSERT_EQ(hashObjOsw, hashObjNsw)
      << "Old sw and new sw obj are differentes";
    // ASSERT_EQ(hashObjOsw, hashObjNew)      << "Old sw and new lib obj are differentes";
    ASSERT_EQ(hashTexOsw, hashTexNsw)
      << "Old sw and new sw tex are differentes";
    // ASSERT_EQ(hashTexOsw, hashTexNew)  << "Old sw and new lib tex are differentes";
  }

  // // Remove tmp files
  // for (int f = 0; f < gofInfo.startFrameIndex; f++) {
  //   remove(vmesh::expandNum(objNewPath, f).c_str());
  //   remove(vmesh::expandNum(objNswPath, f).c_str());
  //   remove(vmesh::expandNum(objOswPath, f).c_str());
  //   remove(vmesh::expandNum(texNewPath, f).c_str());
  //   remove(vmesh::expandNum(texNswPath, f).c_str());
  //   remove(vmesh::expandNum(texOswPath, f).c_str());
  //   remove(vmesh::expandNum(mtlPath, f).c_str());
  // }
}

TEST(Encode, OneFrame)
{
  encode(
    "data/levi_fr%04d_qp12_qt13.obj", "data/levi_fr%04d.png",
    "data/levi_fr%04d_base.obj", "data/levi_fr%04d_subdiv.obj", 1);
}
