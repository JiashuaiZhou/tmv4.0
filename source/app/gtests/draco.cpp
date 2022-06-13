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
#include "bitstream.hpp"

#include "virtualGeometryEncoder.hpp"
#include "virtualGeometryDecoder.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

TEST(Draco, Encode)
{
  disableSubProcessLog.disable();
  // Set parameters
  vmesh::GeometryCodecId codecId = vmesh::GeometryCodecId::DRACO;
  vmesh::GeometryEncoderParameters params;
  std::string inputMesh = "data/gof_0_fr_0_qbase.obj";
  std::string recOrg = "rec_org.obj";
  std::string binOrg = "bin_org.drc";
  std::string recNew = "rec_new.obj";
  std::string binNew = "bin_new.drc";
  params.qp_ = 11;
  params.qt_ = 10;
  params.qn_ = -1;
  params.qg_ = -1;
  params.cl_ = 10;
  
  if (!checkSoftwarePath())
    return;
  if (!exists(inputMesh)) {
    printf("Input path not exists (%s) \n", inputMesh.c_str());
    return;
  }

  // Encode with VirtualGeometryEncoder
  vmesh::TriangleMesh<double> src;
  vmesh::TriangleMesh<double> rec;
  vmesh::TriangleMesh<double> dec;
  vmesh::Bitstream bitstream;
  src.loadFromOBJ(inputMesh);
  auto encoder = vmesh::VirtualGeometryEncoder<double>::create(codecId);
  encoder->encode(src, params, bitstream.vector(), rec);
  // rec.saveToOBJ(recNew);
  bitstream.save(binNew);

  // Decode with VirtualGeometryDecoder
  auto decoder = vmesh::VirtualGeometryDecoder<double>::create(codecId);
  decoder->decode(bitstream.vector(), dec);

  // Encode with original draco application
  std::stringstream cmd;
  cmd << g_dracoEncoderPath << "  "
      << " -i " << inputMesh    
      << " -o " << binOrg       
      << " -qp " << params.qp_  
      << " -qt " << params.qt_  
      << " -qn " << params.qn_  
      << " -qg " << params.qg_  
      << " -cl " << params.cl_ ;
  if (disableSubProcessLog.disableLog())
    cmd << " > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Decode with original draco application
  cmd.str("");
  cmd << g_dracoDecoderPath << "  "
      << " -i " << binOrg  
      << " -o " << recOrg; 
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Compare bitstreams
  auto hashBinLibs =  hash(binOrg);
  auto hashBinSoft =  hash(binNew);
  std::cout << "hashBinLibs = " << std::hex << hashBinLibs << "\n";
  std::cout << "hashBinSoft = " << std::hex << hashBinSoft << "\n";
  disableSubProcessLog.enable();

  ASSERT_EQ(hashBinLibs, hashBinSoft);
 
  // Remove tmp files
  remove( binOrg.c_str() );
  remove( binNew.c_str() );
  remove( recOrg.c_str() );
  remove( recNew.c_str() );
}
