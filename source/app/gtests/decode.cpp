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
#include "decoder.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

void
decode(const std::string bitstreamPath)
{
  disableSubProcessLog.disable();
  // Parameters 
  vmesh::VMCDecoderParameters params;
  params.geometryMeshDecoderPath = g_dracoDecoderPath; 
  params.geometryVideoDecoderPath = g_hmDecoderPath; 
  params.textureVideoDecoderPath = g_hmDecoderPath; 
  params.textureVideoHDRToolPath = g_hdrConvertPath; 
  params.textureVideoHDRToolDecConfig = "cfg/hdrconvert/yuv420tobgr444.cfg"; 
  params.normalizeUV = 0;
  params.keepIntermediateFiles = 0;

  std::string objNewPath = "decoded_obj_fr%04d_new.obj";
  std::string objNswPath = "decoded_obj_fr%04d_nsw.obj";
  std::string objOswPath = "decoded_obj_fr%04d_osw.obj";  
  std::string texNewPath = "decoded_obj_fr%04d_new.png";
  std::string texNswPath = "decoded_obj_fr%04d_nsw.png";
  std::string texOswPath = "decoded_obj_fr%04d_osw.png";
  std::string mtlPath = "decoded_obj_fr%04d.mtl";

  // Decode old application
  std::stringstream cmd;
  cmd << g_decoderOldPath << "  "
      << "  --mode=" << 1
      << "  --compressed=" << bitstreamPath
      << "  --fstart=" << 0
      << "  --cscdecconfig=" << params.textureVideoHDRToolDecConfig
      << "  --normuv=" << params.normalizeUV
      << "  --csc=" << g_hdrConvertPath
      << "  --gmdec=" << g_dracoDecoderPath
      << "  --gvdec=" << g_hmDecoderPath
      << "  --tvdec=" << g_hmDecoderPath
      << "  --decmesh=" << objOswPath
      << "  --dectex=" << texOswPath
      << "  --decmat=" << mtlPath;

  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // FitSubDiv new application
  cmd.str("");
  cmd << g_decoderNewPath << "  "
      << "  --compressed=" << bitstreamPath
      << "  --fstart=" << 0
      << "  --cscdecconfig=" << "cfg/hdrconvert/yuv420tobgr444.cfg"
      << "  --normuv=" << 0
      << "  --csc=" << g_hdrConvertPath
      << "  --gmdec=" << g_dracoDecoderPath
      << "  --gvdec=" << g_hmDecoderPath
      << "  --tvdec=" << g_hmDecoderPath
      << "  --decmesh=" << objNswPath
      << "  --dectex=" << texNswPath
      << "  --decmat=" << mtlPath;

  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());  

  // New   
  vmesh::Bitstream bitstream;
  if (bitstream.load(bitstreamPath)) {
    std::cerr << "Error: can't load compressed bitstream ! ("
         << bitstreamPath << ")\n";
    return ;
  }

  vmesh::VMCDecoder decoder;
  vmesh::VMCGroupOfFramesInfo gofInfo;
  vmesh::VMCGroupOfFrames gof;
  vmesh::VMCStats totalStats;
  totalStats.reset();
  size_t byteCounter = 0;
  gofInfo.index = 0;
  gofInfo.startFrameIndex = 0;
  while (byteCounter != bitstream.size()) {
    if (decoder.decompress(
          bitstream, gofInfo, gof, byteCounter, params)) {
      std::cerr << "Error: can't decompress group of frames!\n";
      return ;
    }
    for (int fIndex = 0; fIndex < int32_t(gof.stats.frameCount); ++fIndex) {
      const auto fNum = gofInfo.startFrameIndex + fIndex;
      auto nameDecObj = vmesh::expandNum(objNewPath, fNum);
      auto nameDecTex = vmesh::expandNum(texNewPath, fNum);
      auto nameDecMtl = vmesh::expandNum(mtlPath, fNum);
      const auto& frame = gof.frame(fIndex);
      vmesh::SaveImage(nameDecTex, frame.outputTexture);
      vmesh::Material<double> material;
      material.texture = nameDecTex;
      material.save(nameDecMtl);
      auto& recmesh = gof.frame(fIndex).rec;
      recmesh.setMaterialLibrary(nameDecMtl);
      recmesh.saveToOBJ(nameDecObj);
    }
    gofInfo.startFrameIndex += gof.stats.frameCount;
    ++gofInfo.index;
  }

  disableSubProcessLog.enable();

  // Compute hashes
  for (int f = 0; f < gofInfo.startFrameIndex; f++) {
    auto hashObjNew = hash(vmesh::expandNum(objNewPath, f));
    auto hashObjNsw = hash(vmesh::expandNum(objNswPath, f));
    auto hashObjOsw = hash(vmesh::expandNum(objOswPath, f));
    auto hashTexNew = hash(vmesh::expandNum(texNewPath, f));
    auto hashTexNsw = hash(vmesh::expandNum(texNswPath, f));
    auto hashTexOsw = hash(vmesh::expandNum(texOswPath, f));
    if( !disableSubProcessLog.disableLog()  ){
      std::cout << "hashObjNew " << f << " = " << std::hex << hashObjNew << "\n";
      std::cout << "hashObjNsw " << f << " = " << std::hex << hashObjNsw << "\n";
      std::cout << "hashObjOsw " << f << " = " << std::hex << hashObjOsw << "\n";
      std::cout << "hashTexNew " << f << " = " << std::hex << hashTexNew << "\n";
      std::cout << "hashTexNsw " << f << " = " << std::hex << hashTexNsw << "\n";
      std::cout << "hashTexOsw " << f << " = " << std::hex << hashTexOsw << "\n";
    }

    // Compare hashes
    ASSERT_EQ(hashObjOsw, hashObjNsw)
      << "Old sw and new sw obj are differentes";
    ASSERT_EQ(hashObjOsw, hashObjNew)
      << "Old sw and new lib obj are differentes";
    ASSERT_EQ(hashTexOsw, hashTexNsw)
      << "Old sw and new sw tex are differentes";
    ASSERT_EQ(hashTexOsw, hashTexNew)
      << "Old sw and new lib tex are differentes";
  }

  // Remove tmp files
  for (int f = 0; f < gofInfo.startFrameIndex; f++) {
    remove(vmesh::expandNum(objNewPath, f).c_str());
    remove(vmesh::expandNum(objNswPath, f).c_str());
    remove(vmesh::expandNum(objOswPath, f).c_str());
    remove(vmesh::expandNum(texNewPath, f).c_str());
    remove(vmesh::expandNum(texNswPath, f).c_str());
    remove(vmesh::expandNum(texOswPath, f).c_str());
    remove(mtlPath.c_str());
  }
}

TEST(Decode, OneFrame)
{
  decode("data/levi_voxelized.vmesh");
}
