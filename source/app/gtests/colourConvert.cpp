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

#include "virtualColourConverter.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

std::string configPath = "cfg//hdrconvert/yuv420tobgr444.cfg";

static std::string
videoName(
  const std::string prefix,
  const int width,
  const int height,
  const int bits,
  const vmesh::ColourSpace colorSpace)
{
  std::string str;
  str += prefix + "_" + std::to_string(width) + "x" + std::to_string(height)
    + "_" + std::to_string(bits) + "bits_";

  switch (colorSpace) {
  case vmesh::ColourSpace::YUV400p: str += "p400.yuv"; break;
  case vmesh::ColourSpace::YUV420p: str += "p420.yuv"; break;
  case vmesh::ColourSpace::YUV444p: str += "p444.yuv"; break;
  case vmesh::ColourSpace::RGB444p: str += "p444.rgb"; break;
  case vmesh::ColourSpace::BGR444p: str += "p444.bgr"; break;
  case vmesh::ColourSpace::GBR444p: str += "p444.gbr"; break;
  case vmesh::ColourSpace::UNKNOW: str += "UNKNOW.UNKNOW"; break;
  }
  return str;
}

void
test(
  int mode,
  const std::string inputPath,
  const int width,
  const int height,
  const int frameCount,
  const int inputBitDepth,
  const int outputBitDepth,
  const vmesh::ColourSpace inputColourSpace,
  const vmesh::ColourSpace outputColourSpace,

  std::string configPath0,
  std::string configPath1)
{
  auto recLibsPath = videoName(
    "conv_libs_" + std::to_string(mode), width, height, outputBitDepth,
    outputColourSpace);
  auto recSoftPath = videoName(
    "conv_soft_" + std::to_string(mode), width, height, outputBitDepth,
    outputColourSpace);

  // Check encoder and decoder path exist
  if (!checkSoftwarePath())
    return;
  if (!exists(inputPath)) {
    printf("Input path not exists (%s) \n", inputPath.c_str());
    return;
  }
  
  DISABLE_SUB_PROCESS_LOG();

  // Load input mesh
  vmesh::FrameSequence<uint16_t> src(
    width, height, inputColourSpace, frameCount);
  if (inputBitDepth == 8) {
    vmesh::FrameSequence<uint8_t> src8(
      width, height, inputColourSpace, frameCount);
    src8.load(inputPath);
    for (int f = 0; f < src8.frameCount(); f++) {
      for (int c = 0; c < 3; c++) {
        auto& planeSrc = src8.frame(f).plane(c);
        auto& planeDst = src.frame(f).plane(c);
        for (int i = 0; i < planeSrc.size(); i++) {
          planeDst.data()[i] = planeSrc.data()[i];
        }
      }
    }
  } else {
    src.load(inputPath);
  }

  // Convert with lib
  vmesh::FrameSequence<uint16_t> rec;
  auto convert = vmesh::VirtualColourConverter<uint16_t>::create(mode);
  convert->convert(configPath0, src, rec);
  if (outputBitDepth == 8) {
    vmesh::FrameSequence<uint8_t> rec8;
    rec8.resize(
      rec.width(), rec.height(), rec.colourSpace(), rec.frameCount());
    for (int f = 0; f < rec.frameCount(); f++) {
      for (int c = 0; c < 3; c++) {
        auto& planeSrc = rec.frame(f).plane(c);
        auto& planeDst = rec8.frame(f).plane(c);
        for (int i = 0; i < planeSrc.size(); i++) {
          planeDst.data()[i] = planeSrc.data()[i];
        }
      }
    }
    // Save bitstream, reconstructed and decoded
    rec8.save(recLibsPath);
  } else {
    rec.save(recLibsPath);
  }

  // Convert with application
  std::stringstream cmd;
  cmd << g_hdrConvertPath << " "
      << "  -f " << configPath1 << " "
      << "  -p SourceFile=" << inputPath << " "
      << "  -p SourceWidth=" << width << " "
      << "  -p SourceHeight=" << height << " "
      << "  -p NumberOfFrames=" << frameCount << " "
      << "  -p OutputFile=" << recSoftPath << " ";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Compute hashes
  auto hashLibs = hash(recLibsPath);
  auto hashSoft = hash(recSoftPath);
  std::cout << "hashBinLibs = " << std::hex << hashLibs << "\n";
  std::cout << "hashBinSoft = " << std::hex << hashSoft << "\n";

  ENABLE_SUB_PROCESS_LOG()
  // Compare hashes
  ASSERT_EQ(hashLibs, hashSoft);

  // Remove files
  remove(recLibsPath.c_str());
  remove(recSoftPath.c_str());
}

TEST(ColourConvert, HdrToolsUp)
{
  test(
    1, "data/tex_512x512_10bits_p420.yuv", 512, 512, 2, 10, 8,
    vmesh::ColourSpace::YUV420p, vmesh::ColourSpace::BGR444p,
    "cfg/hdrconvert/yuv420tobgr444.cfg", "cfg/hdrconvert/yuv420tobgr444.cfg");
}

TEST(ColourConvert, HdrToolsDown)
{
  test(
    1, "data/tex_512x512_8bits_p444.bgr", 512, 512, 2, 8, 10,
    vmesh::ColourSpace::BGR444p, vmesh::ColourSpace::YUV420p,
    "cfg/hdrconvert/bgr444toyuv420.cfg", "cfg/hdrconvert/bgr444toyuv420.cfg");
}

// TEST(ColourConvert, UpInternal)
// {
//   test(
//     0, "data/tex_512x512_10bits_p420.yuv", 512, 512, 2,
//     vmesh::ColourSpace::YUV420p, vmesh::ColourSpace::BGR444p,
//     "YUV420ToBGR444_10_8_1", "cfg/hdrconvert/yuv420tobgr444.cfg");
// }