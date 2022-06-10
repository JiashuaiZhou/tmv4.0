
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
#pragma once


#include "image.hpp"

struct DisableSubProcessLog {  
  private :
    bool disableLog = true;
    bool isDisable = false;
    std::stringstream binCoutStream; 
    std::stringstream binCerrStream; 
    std::streambuf* oldCoutRdBuf;
    std::streambuf* oldCerrRdBuf;
    char binStdoutBuffer[4096]; 
    char binStderrBuffer[4096]; 
    FILE* binStdout;
    FILE* binStderr;
    FILE* oldStdout;
    FILE* oldStderr;
  public:
  void switchOnLog() { disableLog = false; }
  
  void disable()
  {
    if (disableLog && !isDisable) {
      // Redirect std::cout and std::cerr
      oldCoutRdBuf = std::cout.rdbuf(binCoutStream.rdbuf());
      oldCerrRdBuf = std::cerr.rdbuf(binCerrStream.rdbuf());
      
      // redirect printf
      binStdout = fmemopen(binStdoutBuffer, 4096, "w");
      binStderr = fmemopen(binStderrBuffer, 4096, "w");
      if (!binStdout) 
        return;
      if (!binStderr) 
        return;
      oldStdout = stdout;
      oldStderr = stderr;
      stdout = binStdout;
      stderr = binStderr;
      isDisable = true;
    }
  }
  void enable()
  {
    if (isDisable) {
      std::cout.rdbuf(oldCoutRdBuf);
      std::cerr.rdbuf(oldCerrRdBuf);
      std::fclose(binStdout);
      std::fclose(binStderr);
      stdout = oldStdout;
      stderr = oldStderr;
      isDisable = false;
    }
  }
};

extern DisableSubProcessLog disableSubProcessLog;


static std::string
grep(std::string filename, std::string keyword)
{
  std::ifstream in(filename.c_str());
  if (in.is_open()) {
    std::string line;
    while (getline(in, line)) {
      std::size_t pos = line.find(keyword);
      if (pos != std::string::npos) {
        in.close();
        return line.substr(pos + keyword.size());        
      }
    }
  }
  in.close();
  return std::string();
}

static inline bool
exists(const std::string& name)
{
  std::ifstream file(name.c_str());
  return file.good();
}

static inline size_t hash( const std::string& name){    
  std::ifstream file(name);
  if( file.is_open() ){
    std::string str(
      (std::istreambuf_iterator<char>(file)),
      std::istreambuf_iterator<char>());
    file.close();
    return std::hash<std::string>{}(str);
  }
  return 0;
}

const std::string g_encoderPath =
  "externaltools/hm-16.21+scm-8.8/bin/TAppEncoderHighBitDepthStatic";
const std::string g_decoderPath =
  "externaltools/hm-16.21+scm-8.8/bin/TAppDecoderHighBitDepthStatic";
const std::string g_hdrConvertPath =
  "externaltools/hdrtools/build/bin/HDRConvert";

static bool checkSoftwarePath(){
  bool ret = true;
  if ( !exists( g_encoderPath) ){
    printf("Software path not exists: %s \n",g_encoderPath.c_str());
    ret = false;
  }
  if ( !exists( g_decoderPath) ){
    printf("Software path not exists: %s \n",g_decoderPath.c_str());
    ret = false;
  }
  if ( !exists( g_hdrConvertPath) ){
    printf("Software path not exists: %s \n",g_hdrConvertPath.c_str());
    ret = false;
  }
  if ( !ret ){
    printf("External softwares not installed: ");
    printf("  Please run ./get_dependencies.jh scripts \n");
  }
  return ret;
}


static std::string
name(
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
