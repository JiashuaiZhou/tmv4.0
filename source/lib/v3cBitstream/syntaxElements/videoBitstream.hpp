/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ISO/IEC
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
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
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
#pragma once

#include "v3cCommon.hpp"

namespace vmesh {

class VideoBitstream {
public:
  VideoBitstream(VideoType type) : type_(type) { data_.clear(); }
  ~VideoBitstream() { data_.clear(); }

  VideoBitstream&             operator=(const VideoBitstream&) = default;
  void                        resize(size_t size) { data_.resize(size); }
  std::vector<uint8_t>&       vector() { return data_; }
  const std::vector<uint8_t>& vector() const { return data_; }
  uint8_t*                    buffer() { return data_.data(); }
  size_t                      size() const { return data_.size(); }
  VideoType                   type() const { return type_; }

  void trace() const {
    std::cout << "      " << toString(type_) << " ->" << size() << " B "
              << std::endl;
  }

  std::string getExtension();
  bool        write(const std::string& filename);
  bool        read(const std::string& filename);
#if defined(WIN32)
  bool _write(const std::string& filename);
  bool _read(const std::string& filename);
#endif

  void byteStreamToSampleStream(size_t precision                = 4,
                                bool   emulationPreventionBytes = false);

  void sampleStreamToByteStream(bool   isAvc                    = false,
                                bool   isVvc                    = false,
                                size_t precision                = 4,
                                bool   emulationPreventionBytes = false,
                                bool   changeStartCodeSize      = true);

private:
  size_t               getEndOfNaluPosition(size_t startIndex);
  std::vector<uint8_t> data_;
  VideoType            type_;
};

}  // namespace vmesh
