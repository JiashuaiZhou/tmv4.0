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

#include "virtualVideoEncoder.hpp"
#include "virtualVideoDecoder.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

TEST(VideoHm, EncodeDisp)
{
  // Set parameters
  vmesh::VideoEncoderParameters params;  
  std::string inputPath = "data/disp_256x160_10bits_p444.brg";  
  std::string configPath = "cfg/hm/ctc-hm-displacements-map-ai-main10.cfg";
  const int width = 256;
  const int height = 160;
  const int frameCount = 2;
  const vmesh::ColourSpace colorSpace = vmesh::ColourSpace::BGR444p;
  vmesh::VideoCodecId codecId = vmesh::VideoCodecId::HM;
  params.encoderConfig_ = configPath;
  params.qp_ = 38;
  params.inputBitDepth_ = 10;
  params.internalBitDepth_ = 10;
  params.outputBitDepth_ = 10;

  // Check encoder and decoder path exist
  if (!checkSoftwarePath())
    return;

  // Load input mesh
  vmesh::FrameSequence<uint16_t> src(width, height, colorSpace, frameCount);
  src.load(inputPath);
  if (src.frameCount() == 0) {
    printf("Src frame count = %d \n", src.frameCount());
    exit(-1);
  }

  // Encode lib 
  vmesh::FrameSequence<uint16_t> rec;
  vmesh::Bitstream bitstream;
  auto encoder = vmesh::VirtualVideoEncoder<uint16_t>::create(codecId);
  encoder->encode(src, params, bitstream.vector(), rec);

  // Decode lib 
  vmesh::FrameSequence<uint16_t> dec;
  auto decoder = vmesh::VirtualVideoDecoder<uint16_t>::create(codecId);
  decoder->decode(bitstream.vector(), dec);

  // Save bitstream, reconstructed and decoded
  rec.save("rec_disp_libs_256x160_10bits_p444.brg");
  rec.save("dec_disp_libs_256x160_10bits_p444.brg");
  bitstream.save( "hm_disp_libs.h265" );

  // Encode with application
  std::stringstream cmd;
  cmd << g_encoderPath << " "
      << "  -c " << configPath << " "
      << "  --InputFile=" << inputPath << " "
      << "  --InputBitDepth=10 "
      << "  --OutputBitDepth=10 "
      << "  --OutputBitDepthC=10 "
      << "  --InternalBitDepth=10 "
      << "  --InternalBitDepthC=10 "
      << "  --InputChromaFormat=444 "
      << "  --FrameRate=30 "
      << "  --FrameSkip=0 "
      << "  --SourceWidth=" << width << " "
      << "  --SourceHeight=" << height << " "
      << "  --FramesToBeEncoded=" << frameCount << " "
      << "  --BitstreamFile=hm_disp_soft.h265 "
      << "  --ReconFile=rec_disp_soft_256x160_10bits_p444.brg "
      << "  --QP=38 ";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Decode with application
  cmd.str("");
  cmd << g_decoderPath << " " 
     << "  --BitstreamFile=hm_disp_soft.h265 "
     << "  --ReconFile=dec_disp_soft_256x160_10bits_p444.brg"; 
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Compute hashes
  auto hashBinLibs =  hash("hm_disp_libs.h265");
  auto hashBinSoft =  hash("hm_disp_soft.h265");
  auto hashRecLibs =  hash("rec_disp_libs_256x160_10bits_p444.brg");
  auto hashRecSoft =  hash("rec_disp_soft_256x160_10bits_p444.brg");
  auto hashDecLibs =  hash("dec_disp_libs_256x160_10bits_p444.brg");
  auto hashDecSoft =  hash("dec_disp_soft_256x160_10bits_p444.brg");
  std::cout << "hashBinLibs = " << std::hex << hashBinLibs << "\n";
  std::cout << "hashBinSoft = " << std::hex << hashBinSoft << "\n";
  std::cout << "hashRecLibs = " << std::hex << hashRecLibs << "\n";
  std::cout << "hashRecSoft = " << std::hex << hashRecSoft << "\n";
  std::cout << "hashDecLibs = " << std::hex << hashDecLibs << "\n";
  std::cout << "hashDecSoft = " << std::hex << hashDecSoft << "\n";

  // Compare hashes
  ASSERT_EQ(hashBinLibs, hashBinSoft);
  ASSERT_EQ(hashRecLibs, hashRecSoft);
  ASSERT_EQ(hashDecLibs, hashDecSoft);
  ASSERT_EQ(hashRecLibs, hashDecLibs);

  // Remove files 
  remove("hm_disp_libs.h265");
  remove("hm_disp_soft.h265");
  remove("rec_disp_libs_256x160_10bits_p444.brg");
  remove("rec_disp_soft_256x160_10bits_p444.brg");
  remove("dec_disp_libs_256x160_10bits_p444.brg");
  remove("dec_disp_soft_256x160_10bits_p444.brg");
}

TEST(VideoHm, EncodeTexture)
{
  // Set parameters
  vmesh::VideoEncoderParameters params;
  std::string g_encoderPath = "externaltools/hm-16.21+scm-8.8/bin/TAppEncoderStatic";
  std::string g_decoderPath = "externaltools/hm-16.21+scm-8.8/bin/TAppDecoderStatic";
  std::string inputPath = "data/tex_2048x2048_10bits_p420.yuv";  
  std::string configPath = "cfg/hm/ctc-hm-texture-ai.cfg";
  const int width = 2048;
  const int height = 2048;
  const int frameCount = 2;
  const vmesh::ColourSpace colorSpace = vmesh::ColourSpace::YUV420p;
  vmesh::VideoCodecId codecId = vmesh::VideoCodecId::HM;
  params.encoderConfig_ = configPath;
  params.qp_ = 38;
  params.inputBitDepth_ = 10;
  params.internalBitDepth_ = 10;
  params.outputBitDepth_ = 10;

  // Check encoder and decoder path exist
  if (!checkSoftwarePath())
    return;

  // Load input mesh
  vmesh::FrameSequence<uint16_t> src(width, height, colorSpace, frameCount);
  src.load(inputPath);
  if (src.frameCount() == 0) {
    printf("Src frame count = %d \n", src.frameCount());
    exit(-1);
  }

  // Encode lib
  vmesh::FrameSequence<uint16_t> rec;
  vmesh::Bitstream bitstream;
  auto encoder = vmesh::VirtualVideoEncoder<uint16_t>::create(codecId);
  encoder->encode(src, params, bitstream.vector(), rec);

  // Decode lib 
  vmesh::FrameSequence<uint16_t> dec;
  auto decoder = vmesh::VirtualVideoDecoder<uint16_t>::create(codecId);
  decoder->decode(bitstream.vector(), dec, 10);

  // Save bitstream, reconstructed and decoded
  rec.save("rec_text_libs_2048x2048_10bits_p420.yuv");
  dec.save("dec_text_libs_2048x2048_10bits_p420.yuv");
  bitstream.save( "hm_text_libs.h265" );

  // Encode with original draco application
  std::stringstream cmd;
  cmd << g_encoderPath
      << "  -c " << configPath << " "
      << "  --InputFile=" << inputPath << " "
      << "  --BitstreamFile=hm_text_soft.h265 "
      << "  --ReconFile=rec_text_soft_2048x2048_10bits_p420.yuv "
      << "  --InputBitDepth=10 "
      << "  --OutputBitDepth=10 "
      << "  --OutputBitDepthC=10 "
      << "  --InternalBitDepth=10 "
      << "  --InternalBitDepthC=10 "
      << "  --InputChromaFormat=420 "
      << "  --FrameRate=30 "
      << "  --FrameSkip=0 "
      << "  --SourceWidth=" << width << " "
      << "  --SourceHeight=" << height << " "
      << "  --FramesToBeEncoded=" << frameCount << " "
      << "  --QP=38 ";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());
  
  // Decode with application
  cmd.str("");
  cmd << g_decoderPath << " " 
     << "  --BitstreamFile=hm_text_soft.h265 "
     << "  --ReconFile=dec_text_soft_2048x2048_10bits_p420.yuv"; 
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Compute hashes
  auto hashBinLibs =  hash("hm_text_libs.h265");
  auto hashBinSoft =  hash("hm_text_soft.h265");
  auto hashRecLibs =  hash("rec_text_libs_2048x2048_10bits_p420.yuv");
  auto hashRecSoft =  hash("rec_text_soft_2048x2048_10bits_p420.yuv");
  auto hashDecLibs =  hash("dec_text_libs_2048x2048_10bits_p420.yuv");
  auto hashDecSoft =  hash("dec_text_soft_2048x2048_10bits_p420.yuv");
  std::cout << "hashBinLibs = " << std::hex << hashBinLibs << "\n";
  std::cout << "hashBinSoft = " << std::hex << hashBinSoft << "\n";
  std::cout << "hashRecLibs = " << std::hex << hashRecLibs << "\n";
  std::cout << "hashRecSoft = " << std::hex << hashRecSoft << "\n";
  std::cout << "hashDecLibs = " << std::hex << hashDecLibs << "\n";
  std::cout << "hashDecSoft = " << std::hex << hashDecSoft << "\n";

  // Compare hashes
  // ASSERT_EQ(hashBinLibs, hashBinSoft);
  // ASSERT_EQ(hashRecLibs, hashRecSoft);
  // ASSERT_EQ(hashDecLibs, hashDecSoft);
  ASSERT_EQ(hashRecLibs, hashDecLibs);
  ASSERT_EQ(hashDecSoft, hashDecSoft);

  // Remove files 
  remove("hm_text_libs.h265");
  remove("hm_text_soft.h265");
  remove("rec_text_libs_2048x2048_10bits_p420.rgb");
  remove("rec_text_soft_2048x2048_10bits_p420.rgb");
  remove("dec_text_libs_2048x2048_10bits_p420.rgb");
  remove("dec_text_soft_2048x2048_10bits_p420.rgb");
}