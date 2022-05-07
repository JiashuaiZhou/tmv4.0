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

#include "vmesh/bitstream.hpp"
#include "vmesh/decoder.hpp"
#include "vmesh/encoder.hpp"

namespace vmesh {

//============================================================================

int32_t
VMCEncoder::encodeSequenceHeader(
  const VMCGroupOfFrames& gof,
  Bitstream& bitstream,
  const VMCEncoderParameters& params) const
{
  if (
    _dispVideo.width() < 0 || _dispVideo.width() > 16384
    || _dispVideo.height() < 0 || _dispVideo.height() > 16384
    || (_dispVideo.frameCount() != 0 && gof.frameCount() != gof.frameCount())
    || gof.frameCount() < 0 || gof.frameCount() > 65535
    || params.textureWidth < 0 || params.textureWidth > 16384
    || params.textureHeight < 0 || params.textureHeight > 16384
    || params.bitDepthPosition < 0 || params.bitDepthPosition > 16
    || params.bitDepthTexCoord < 0 || params.bitDepthTexCoord > 16) {
    return -1;
  }
  const uint16_t frameCount = uint16_t(gof.frameCount());
  const uint16_t widthDispVideo = uint32_t(_dispVideo.width());
  const uint16_t heightDispVideo = uint32_t(_dispVideo.height());
  const uint16_t widthTexVideo = uint32_t(params.textureWidth);
  const uint16_t heightTexVideo = uint32_t(params.textureHeight);
  const uint8_t geometryVideoBlockSize = params.geometryVideoBlockSize;
  const uint8_t bitDepth = uint8_t(
    (params.bitDepthPosition - 1) + ((params.bitDepthTexCoord - 1) << 4));
  const uint8_t subdivInfo = uint8_t(params.subdivisionMethod)
    + ((params.subdivisionIterationCount) << 4);
  const uint8_t qpBaseMesh =
    uint8_t((params.qpPosition - 1) + ((params.qpTexCoord - 1) << 4));
  const uint8_t liftingQPs[3] = {
    uint8_t(params.liftingQuantizationParameters[0]),
    uint8_t(params.liftingQuantizationParameters[1]),
    uint8_t(params.liftingQuantizationParameters[2])};
  const uint8_t bitField =
    params.encodeDisplacementsVideo | (params.encodeTextureVideo << 1);
  bitstream.write(frameCount);
  bitstream.write(bitField);
  bitstream.write(bitDepth);
  bitstream.write(subdivInfo);
  bitstream.write(qpBaseMesh);
  if (params.encodeDisplacementsVideo) {
    bitstream.write(widthDispVideo);
    bitstream.write(heightDispVideo);
    bitstream.write(geometryVideoBlockSize);
    bitstream.write(liftingQPs[0]);
    bitstream.write(liftingQPs[1]);
    bitstream.write(liftingQPs[2]);
  }
  if (params.encodeTextureVideo) {
    bitstream.write(widthTexVideo);
    bitstream.write(heightTexVideo);
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decodeSequenceHeader(const Bitstream& bitstream)
{
  uint16_t frameCount = 0;
  uint8_t bitField = 0;
  uint16_t widthDispVideo = 0;
  uint16_t heightDispVideo = 0;
  uint16_t widthTexVideo = 0;
  uint16_t heightTexVideo = 0;
  uint8_t geometryVideoBlockSize = 0;
  uint8_t bitDepth = 0;
  uint8_t qpBaseMesh = 0;
  uint8_t subdivInfo = 0;
  uint8_t liftingQPs[3] = {};
  bitstream.read(frameCount, _byteCounter);
  bitstream.read(bitField, _byteCounter);
  bitstream.read(bitDepth, _byteCounter);
  bitstream.read(subdivInfo, _byteCounter);
  bitstream.read(qpBaseMesh, _byteCounter);
  _sps.frameCount = frameCount;
  _sps.encodeDisplacementsVideo = bitField & 1;
  _sps.encodeTextureVideo = (bitField >> 1) & 1;
  if (_sps.encodeDisplacementsVideo) {
    bitstream.read(widthDispVideo, _byteCounter);
    bitstream.read(heightDispVideo, _byteCounter);
    bitstream.read(geometryVideoBlockSize, _byteCounter);
    bitstream.read(liftingQPs[0], _byteCounter);
    bitstream.read(liftingQPs[1], _byteCounter);
    bitstream.read(liftingQPs[2], _byteCounter);
  }
  if (_sps.encodeTextureVideo) {
    bitstream.read(widthTexVideo, _byteCounter);
    bitstream.read(heightTexVideo, _byteCounter);
  }
  _sps.widthDispVideo = widthDispVideo;
  _sps.heightDispVideo = heightDispVideo;
  _sps.widthTexVideo = widthTexVideo;
  _sps.heightTexVideo = heightTexVideo;
  _sps.geometryVideoBlockSize = geometryVideoBlockSize;
  _sps.bitDepthPosition = 1 + (bitDepth & 15);
  _sps.bitDepthTexCoord = 1 + ((bitDepth >> 4) & 15);
  _sps.subdivisionMethod = SubdivisionMethod(subdivInfo & 15);
  _sps.subdivisionIterationCount = (subdivInfo >> 4) & 15;
  _sps.qpPosition = 1 + (qpBaseMesh & 15);
  _sps.qpTexCoord = 1 + ((qpBaseMesh >> 4) & 15);
  _sps.liftingQuantizationParameters[0] = liftingQPs[0];
  _sps.liftingQuantizationParameters[1] = liftingQPs[1];
  _sps.liftingQuantizationParameters[2] = liftingQPs[2];
  return 0;
}

//============================================================================

int32_t
VMCEncoder::encodeFrameHeader(
  const VMCFrameInfo& frameInfo, Bitstream& bitstream) const
{
  const auto frameType = uint8_t(frameInfo.type);
  assert(frameInfo.patchCount >= 0 && frameInfo.patchCount <= 256);
  const auto patchCountMinusOne = uint8_t(frameInfo.patchCount - 1);
  bitstream.write(frameType);
  bitstream.write(patchCountMinusOne);
  if (frameInfo.type != FrameType::INTRA) {
    uint8_t referenceFrameIndex =
      frameInfo.frameIndex - frameInfo.referenceFrameIndex - 1;
    assert(frameInfo.frameIndex > frameInfo.referenceFrameIndex);
    assert(frameInfo.frameIndex - frameInfo.referenceFrameIndex - 1 < 256);
    bitstream.write(referenceFrameIndex);
  }
  return 0;
}

//----------------------------------------------------------------------------

int32_t
VMCDecoder::decodeFrameHeader(
  const Bitstream& bitstream, VMCFrameInfo& frameInfo)
{
  uint8_t frameType = 0;
  bitstream.read(frameType, _byteCounter);
  frameInfo.type = FrameType(frameType);
  uint8_t patchCountMinusOne = 0;
  bitstream.read(patchCountMinusOne, _byteCounter);
  frameInfo.patchCount = patchCountMinusOne + 1;
  if (frameInfo.type != FrameType::INTRA) {
    uint8_t referenceFrameIndex = 0;
    bitstream.read(referenceFrameIndex, _byteCounter);
    frameInfo.referenceFrameIndex =
      frameInfo.frameIndex - int32_t(referenceFrameIndex) - 1;
  }
  return 0;
}

//============================================================================

}  // namespace vmesh