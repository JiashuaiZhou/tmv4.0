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
#if defined( USE_HM_VIDEO_CODEC )
#include "image.hpp"
#include "hmLibVideoEncoder.hpp"
#include "hmLibVideoEncoderImpl.hpp"

namespace vmesh {

template <typename T>
hmLibVideoEncoder<T>::hmLibVideoEncoder() {}
template <typename T>
hmLibVideoEncoder<T>::~hmLibVideoEncoder() {}

template<typename T>
void
hmLibVideoEncoder<T>::encode(
  FrameSequence<T>& videoSrc,
  VideoEncoderParameters& params,
  std::vector<uint8_t>& bitstream,
  FrameSequence<T>& videoRec)
{
  const size_t width = videoSrc.width();
  const size_t height = videoSrc.height();
  const size_t frameCount = videoSrc.frameCount();
  std::stringstream cmd;
  cmd << "HMEncoder";
  cmd << " -c " << params.encoderConfig_;
  cmd << " --FrameRate=30";
  cmd << " --FrameSkip=0";
  cmd << " --SourceWidth=" << width;
  cmd << " --SourceHeight=" << height;
  // cmd << " --ConformanceWindowMode=1 ";
  cmd << " --FramesToBeEncoded=" << frameCount;
  // cmd << " --InputFile=" << params.srcYuvFileName_;
  // cmd << " --BitstreamFile=" << params.binFileName_;
  // cmd << " --ReconFile=" << params.recYuvFileName_;
  if (
    videoSrc.colourSpace() == ColourSpace::YUV444p
    || videoSrc.colourSpace() == ColourSpace::RGB444p
    || videoSrc.colourSpace() == ColourSpace::BGR444p
    || videoSrc.colourSpace() == ColourSpace::GBR444p)
    cmd << " --InputChromaFormat=444";
  else
    cmd << " --InputChromaFormat=420";

  if( params.qp_ != -8 )
    cmd << " --QP=" << params.qp_;
  cmd << " --InputBitDepth=" << params.inputBitDepth_;
  cmd << " --OutputBitDepth=" << params.outputBitDepth_;
  cmd << " --OutputBitDepthC=" << params.outputBitDepth_;
  if ( params.internalBitDepth_ != 0 ) {
    cmd << " --InternalBitDepth=" << params.internalBitDepth_;
    cmd << " --InternalBitDepthC=" << params.internalBitDepth_;
  }
#if defined( PCC_ME_EXT ) && PCC_ME_EXT
  if ( params.usePccMotionEstimation_ ) {
    cmd << " --UsePccMotionEstimation=1";
    cmd << " --BlockToPatchFile=" << params.blockToPatchFile_;
    cmd << " --OccupancyMapFile=" << params.occupancyMapFile_;
    cmd << " --PatchInfoFile=" << params.patchInfoFile_;
  }
#endif
#if defined( PCC_RDO_EXT ) && PCC_RDO_EXT
  if ( params.usePccRDO_ && !params.inputColourSpaceConvert_ ) {
    cmd << " --UsePccRDO=1";
    cmd << " --OccupancyMapFile=" << params.occupancyMapFile_;
  }
#endif
  std::cout << cmd.str() << std::endl;
  hmLibVideoEncoderImpl<T> encoder;
  encoder.encode( videoSrc, cmd.str(), bitstream, videoRec );
}

template class vmesh::hmLibVideoEncoder<uint8_t>;
template class vmesh::hmLibVideoEncoder<uint16_t>;

} // namespace vmesh 

#endif
