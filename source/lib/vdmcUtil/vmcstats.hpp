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

namespace vmesh {

//============================================================================

struct VMCStats {
  void reset() { *this = VMCStats(); }

  VMCStats& operator+=(const VMCStats& rhs)
  {
    baseMeshByteCount += rhs.baseMeshByteCount;
    motionByteCount += rhs.motionByteCount;
    displacementsByteCount += rhs.displacementsByteCount;
    textureByteCount += rhs.textureByteCount;
    totalByteCount += rhs.totalByteCount;
    frameCount += rhs.frameCount;
    vertexCount += rhs.vertexCount;
    faceCount += rhs.faceCount;
    baseMeshVertexCount += rhs.baseMeshVertexCount;
    processingTimeInSeconds += rhs.processingTimeInSeconds;
    return *this;
  }

  void dump(const std::string& header, const int framerate) const
  {
    const auto byteCountToBitrate = 
      (8.0 * framerate) / (frameCount * 1000000.0);
    const auto baseMeshBitrate = baseMeshByteCount * byteCountToBitrate;
    const auto motionBitrate = motionByteCount * byteCountToBitrate;
    const auto displacementBitrate =
      displacementsByteCount * byteCountToBitrate;
    const auto textureBitrate = textureByteCount * byteCountToBitrate;
    const auto totalBitrate = totalByteCount * byteCountToBitrate;

    const auto baseMeshBitsPerVertex =
      baseMeshByteCount * 8.0 / baseMeshVertexCount;
    const auto motionBitsPerVertex =
      motionByteCount * 8.0 / baseMeshVertexCount;
    const auto textureBitsPerVertex = textureByteCount * 8.0 / vertexCount;
    const auto displacementBitsPerVertex =
      displacementsByteCount * 8.0 / vertexCount;
    const auto totalBitsPerVertex = totalByteCount * 8.0 / vertexCount;

    std::cout << header << " frame count " << frameCount << '\n';
    std::cout << header << " face count " << faceCount << '\n';
    std::cout << header << " vertex count " << vertexCount << '\n';
    std::cout << header << " processing time " << processingTimeInSeconds
              << " s \n";
    std::cout << header << " meshes bitrate " << baseMeshBitrate << " mbps "
              << baseMeshByteCount << " B " << baseMeshBitsPerVertex
              << " bpv\n";
    std::cout << header << " motion bitrate " << motionBitrate << " mbps "
              << motionByteCount << " B " << motionBitsPerVertex << " bpv\n";
    std::cout << header << " displacements bitrate " << displacementBitrate
              << " mbps " << displacementsByteCount << " B "
              << displacementBitsPerVertex << " bpv\n";
    std::cout << header << " texture bitrate " << textureBitrate << " mbps "
              << textureByteCount << " B " << textureBitsPerVertex << " bpv\n";
    std::cout << header << " total bitrate " << totalBitrate << " mbps "
              << totalByteCount << " B " << totalBitsPerVertex << " bpv\n";
  }

  size_t baseMeshByteCount = 0;
  size_t displacementsByteCount = 0;
  size_t textureByteCount = 0;
  size_t totalByteCount = 0;
  size_t motionByteCount = 0;
  size_t frameCount = 0;
  size_t vertexCount = 0;
  size_t faceCount = 0;
  size_t baseMeshVertexCount = 0;
  double processingTimeInSeconds = 0.0;
};

//============================================================================

}  // namespace vmesh
