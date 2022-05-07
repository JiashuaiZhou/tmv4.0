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

#include <sstream>

#include "vmesh/util/image.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

namespace vmesh {

//============================================================================

bool
LoadImage(
  const std::string& fileName, Frame<uint8_t, ColourSpace::BGR444p>& image)
{
  int32_t width = 0;
  int32_t height = 0;
  int32_t channelCount = 0;
  std::unique_ptr<uint8_t[]> buffer(
    stbi_load(fileName.c_str(), &width, &height, &channelCount, 0));
  if (buffer == nullptr || channelCount != 3) {
    return false;
  }
  image.resize(width, height);
  auto& B = image.plane(0);
  auto& G = image.plane(1);
  auto& R = image.plane(2);
  for (int32_t y = 0, i = 0; y < height; y++) {
    for (int32_t x = 0; x < width; x++) {
      const auto r = buffer[i];
      const auto g = buffer[i + 1];
      const auto b = buffer[i + 2];
      R.set(y, x, r);
      G.set(y, x, g);
      B.set(y, x, b);
      i += 3;
    }
  }
  return true;
}

//============================================================================

bool
SaveImage(
  const std::string& fileName,
  const Frame<uint8_t, ColourSpace::BGR444p>& image,
  const ImageFormat format,
  const int32_t quality)
{
  const auto width = int32_t(image.width());
  const auto height = int32_t(image.height());
  const auto channelCount = int32_t(image.planeCount());
  std::unique_ptr<uint8_t[]> buffer(
    new uint8_t[channelCount * width * height]);
  const auto& B = image.plane(0);
  const auto& G = image.plane(1);
  const auto& R = image.plane(2);
  for (int32_t y = 0, i = 0; y < height; y++) {
    for (int32_t x = 0; x < width; x++) {
      buffer[i] = R.get(y, x);
      buffer[i + 1] = G.get(y, x);
      buffer[i + 2] = B.get(y, x);
      i += 3;
    }
  }
  bool ret;
  switch (format) {
  case ImageFormat::JPG:
    ret = stbi_write_jpg(
      fileName.c_str(), width, height, channelCount, buffer.get(), quality);
    break;
  case ImageFormat::TGA:
    ret = stbi_write_tga(
      fileName.c_str(), width, height, channelCount, buffer.get());
    break;
  case ImageFormat::BMP:
    ret = stbi_write_bmp(
      fileName.c_str(), width, height, channelCount, buffer.get());
    break;
  case ImageFormat::PNG:
  default:
    ret = stbi_write_png(
      fileName.c_str(), width, height, channelCount, buffer.get(),
      width * channelCount);
    break;
  }
  return ret;
}

//============================================================================

}  // namespace vmesh
