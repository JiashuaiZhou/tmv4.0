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

#include <cstdlib>
#include <chrono>
#include <iostream>

namespace vmesh {

//============================================================================

struct VMCStats {
  void reset() { *this = VMCStats(); }

  VMCStats& operator+=(const VMCStats& rhs) {
    baseMeshByteCount += rhs.baseMeshByteCount;
    motionByteCount += rhs.motionByteCount;
    displacementsByteCount += rhs.displacementsByteCount;
    textureByteCount += rhs.textureByteCount;
    totalByteCount += rhs.totalByteCount;
    frameCount += rhs.frameCount;
    vertexCount += rhs.vertexCount;
    faceCount += rhs.faceCount;
    baseMeshVertexCount += rhs.baseMeshVertexCount;
    processingTime += rhs.processingTime;
    colorTransferTime += rhs.colorTransferTime;
    return *this;
  }

  void dump(const std::string& header, const int framerate) const {
    const auto byteCountToBitrate =
      (8.0 * framerate) / ((double)frameCount * 1000000.0);
    const auto baseMeshBitrate =
      (double)baseMeshByteCount * byteCountToBitrate;
    const auto motionBitrate = (double)motionByteCount * byteCountToBitrate;
    const auto displacementBitrate =
      (double)displacementsByteCount * byteCountToBitrate;
    const auto textureBitrate = (double)textureByteCount * byteCountToBitrate;
    const auto totalBitrate   = (double)totalByteCount * byteCountToBitrate;

    const auto baseMeshBitsPerVertex =
      (double)baseMeshByteCount * 8.0 / (double)baseMeshVertexCount;
    const auto motionBitsPerVertex =
      (double)motionByteCount * 8.0 / (double)baseMeshVertexCount;
    const auto textureBitsPerVertex =
      (double)textureByteCount * 8.0 / (double)vertexCount;
    const auto displacementBitsPerVertex =
      (double)displacementsByteCount * 8.0 / (double)vertexCount;
    const auto totalBitsPerVertex =
      (double)totalByteCount * 8.0 / (double)vertexCount;

    std::cout << header << " frame count           " << std::setw(16)
              << std::right << frameCount << '\n';
    std::cout << header << " face count            " << std::setw(16)
              << std::right << faceCount << '\n';
    std::cout << header << " vertex count          " << std::setw(16)
              << std::right << vertexCount << '\n';
    std::cout << header << " processing time       " << std::setw(16)
              << std::right
              << std::chrono::duration<double>(processingTime).count()
              << " s \n";
    std::cout << header << " color transfer time   " << std::setw(16)
              << std::right
              << std::chrono::duration<double>(colorTransferTime).count()
              << " s \n";
    std::cout << header << " meshes bitrate        " << std::setw(16)
              << std::right << baseMeshBitrate << " mbps " << std::setw(16)
              << std::right << baseMeshByteCount << " B " << std::setw(16)
              << std::right << baseMeshBitsPerVertex << " bpv\n";
    std::cout << header << " motion bitrate        " << std::setw(16)
              << std::right << motionBitrate << " mbps " << std::setw(16)
              << std::right << motionByteCount << " B " << std::setw(16)
              << std::right << motionBitsPerVertex << " bpv\n";
    std::cout << header << " displacements bitrate " << std::setw(16)
              << std::right << displacementBitrate << " mbps " << std::setw(16)
              << std::right << displacementsByteCount << " B " << std::setw(16)
              << std::right << displacementBitsPerVertex << " bpv\n";
    std::cout << header << " texture bitrate       " << std::setw(16)
              << std::right << textureBitrate << " mbps " << std::setw(16)
              << std::right << textureByteCount << " B " << std::setw(16)
              << std::right << textureBitsPerVertex << " bpv\n";
    std::cout << header << " total bitrate         " << std::setw(16)
              << std::right << totalBitrate << " mbps " << std::setw(16)
              << std::right << totalByteCount << " B " << std::setw(16)
              << std::right << totalBitsPerVertex << " bpv\n";
  }

  size_t                              baseMeshByteCount      = 0;
  size_t                              displacementsByteCount = 0;
  size_t                              textureByteCount       = 0;
  size_t                              totalByteCount         = 0;
  size_t                              motionByteCount        = 0;
  size_t                              frameCount             = 0;
  size_t                              vertexCount            = 0;
  size_t                              faceCount              = 0;
  size_t                              baseMeshVertexCount    = 0;
  std::chrono::steady_clock::duration processingTime{};
  std::chrono::steady_clock::duration colorTransferTime{};
};

//============================================================================

}  // namespace vmesh
