/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifdef USE_VTM_VIDEO_CODEC
#  include "program_options_lite.h"
#  include "util/image.hpp"
#  include "vtmLibVideoEncoder.hpp"
#  include "vtmLibVideoEncoderImpl.hpp"
#  include "EncoderLib/EncLibCommon.h"

namespace vmesh {

template<typename T>
vtmLibVideoEncoder<T>::vtmLibVideoEncoder() {}
template<typename T>
vtmLibVideoEncoder<T>::~vtmLibVideoEncoder() {}

template<typename T>
void
vtmLibVideoEncoder<T>::encode(FrameSequence<T>&       videoSrc,
                              VideoEncoderParameters& params,
                              std::vector<uint8_t>&   bitstream,
                              FrameSequence<T>&       videoRec) {
  const size_t      width      = videoSrc.width();
  const size_t      height     = videoSrc.height();
  const size_t      frameCount = videoSrc.frameCount();
  std::stringstream cmd;
  cmd << "VTMEncoder";
  cmd << " -c " << params.encoderConfig_;
  // cmd << " --InputFile=" << params.srcYuvFileName_;
  cmd << " --InputBitDepth=" << params.inputBitDepth_;
  cmd << " --OutputBitDepth=" << params.outputBitDepth_;
  cmd << " --OutputBitDepthC=" << params.outputBitDepth_;
  cmd << " --FrameRate=30";
  cmd << " --FrameSkip=0";
  cmd << " --SourceWidth=" << width;
  cmd << " --SourceHeight=" << height;
  cmd << " --ConformanceWindowMode=1 ";
  cmd << " --FramesToBeEncoded=" << frameCount;
  // cmd << " --BitstreamFile=" << params.binFileName_;
  // cmd << " --ReconFile=" << params.recYuvFileName_;
  cmd << " --QP=" << params.qp_;
  if (params.internalBitDepth_ != 0) {
    cmd << " --InternalBitDepth=" << params.internalBitDepth_;
  }
  if (params.usePccMotionEstimation_) {
    cmd << " --UsePccMotionEstimation=1"
        << " --BlockToPatchFile=" << params.blockToPatchFile_
        << " --OccupancyMapFile=" << params.occupancyMapFile_
        << " --PatchInfoFile=" << params.patchInfoFile_;
  }
  if (videoSrc.colourSpace() == ColourSpace::YUV444p
      || videoSrc.colourSpace() == ColourSpace::RGB444p
      || videoSrc.colourSpace() == ColourSpace::BGR444p
      || videoSrc.colourSpace() == ColourSpace::GBR444p) {
    cmd << " --InputChromaFormat=444";
  } else if (videoSrc.colourSpace() == ColourSpace::YUV400p) {
    cmd << " --InputChromaFormat=400";
  } else {
    cmd << " --InputChromaFormat=420";
  }
  std::cout << cmd.str() << std::endl;

  std::string arguments = cmd.str();

  fprintf(stdout, "\n");
  fprintf(stdout, "VVCSoftware: VTM Encoder Version %s ", VTM_VERSION);
  fprintf(stdout, NVM_ONOS);
  fprintf(stdout, NVM_COMPILEDBY);
  fprintf(stdout, NVM_BITS);
  fprintf(stdout, "\n");

  std::ostringstream oss(ostringstream::binary | ostringstream::out);
  std::ostream&      bitstreamFile = oss;
  EncLibCommon       encLibCommon;

  initROM();
  TComHash::initBlockSizeToIndex();

  vtmLibVideoEncoderImpl<T> encoder(bitstreamFile, &encLibCommon);

  std::istringstream iss(arguments);
  std::string        token;
  std::vector<char*> args;
  while (iss >> token) {
    char* arg = new char[token.size() + 1];
    copy(token.begin(), token.end(), arg);
    arg[token.size()] = '\0';
    args.push_back(arg);
  }
  encoder.create();
  // parse configuration
  try {
    if (!encoder.parseCfg(args.size(), &args[0])) {
      encoder.destroy();
#  if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      EnvVar::printEnvVar();
#  endif
      return;
    }
  } catch (df::program_options_lite::ParseFailure& e) {
    std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
              << e.val << "\"." << std::endl;
    return;
  }
  for (size_t i = 0; i < args.size(); i++) { delete[] args[i]; }

  encoder.createLib(0);

  bool eos = false;

  while (!eos) {
    // read GOP
    bool keepLoop = true;
    while (keepLoop) {
#  ifndef _DEBUG
      try {
#  endif
        keepLoop = encoder.encodePrep(eos, videoSrc, arguments, videoRec);
#  ifndef _DEBUG
      } catch (Exception& e) {
        std::cerr << e.what() << std::endl;
        return;
      } catch (const std::bad_alloc& e) {
        std::cout << "Memory allocation failed: " << e.what() << std::endl;
        return;
      }
#  endif
    }

    // encode GOP
    keepLoop = true;
    while (keepLoop) {
#  ifndef _DEBUG
      try {
#  endif
        keepLoop = encoder.encode(videoSrc, arguments, bitstream, videoRec);
#  ifndef _DEBUG
      } catch (Exception& e) {
        std::cerr << e.what() << std::endl;
        return;
      } catch (const std::bad_alloc& e) {
        std::cout << "Memory allocation failed: " << e.what() << std::endl;
        return;
      }
#  endif
    }
  }

  auto buffer = oss.str();
  bitstream.resize(buffer.size());
  std::copy(buffer.data(), buffer.data() + buffer.size(), bitstream.data());

  encoder.destroyLib();
  encoder.destroy();
  destroyROM();
}

template class vtmLibVideoEncoder<uint8_t>;
template class vtmLibVideoEncoder<uint16_t>;

}  // namespace vmesh

#endif
