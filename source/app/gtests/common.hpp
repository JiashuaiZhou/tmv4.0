
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

#define DISABLE_LOG 0

#if DISABLE_LOG == 1
#  define DISABLE_SUB_PROCESS_LOG() \
    std::stringstream binStream; \
    auto oldRdBuf = std::cout.rdbuf(binStream.rdbuf()); \
    char binBuffer[1024]; \
    auto binFile = fmemopen(binBuffer, 1024, "w"); \
    if (!binFile) { \
      std::printf("error"); \
      return; \
    } \
    auto oldStdout = stdout; \
    stdout = binFile;

#  define ENABLE_SUB_PROCESS_LOG() \
    std::cout.rdbuf(oldRdBuf); \
    std::fclose(binFile); \
    stdout = oldStdout;
#else
#  define DISABLE_SUB_PROCESS_LOG() ;
#  define ENABLE_SUB_PROCESS_LOG() ;
#endif


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
  "externaltools/hm-16.21+scm-8.8/bin/TAppEncoderStatic";
const std::string g_decoderPath =
  "externaltools/hm-16.21+scm-8.8/bin/TAppDecoderStatic";
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
