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

#include "misc.hpp"
#include "mesh.hpp"
#include "verbose.hpp"
#include "vector.hpp"
#include "version.hpp"
#include "vmc.hpp"

#include <UVAtlas.h>

//============================================================================
namespace DirectX {
  static std::istream& operator>>(std::istream& in, UVATLAS& val)
  {
    std::string str;
    in >> str;
    if (str == "DEFAULT")
      val = UVATLAS_DEFAULT;
    else if (str == "FAST")
      val = UVATLAS_GEODESIC_FAST;
    else if (str == "QUALITY")
      val = UVATLAS_GEODESIC_QUALITY;
    else
      in.setstate(std::ios::failbit);
    return in;
  }

  //----------------------------------------------------------------------------

  static std::ostream& operator<<(std::ostream& out, UVATLAS val)
  {
    switch (val) {
    case UVATLAS_DEFAULT: out << "DEFAULT"; break;
    case UVATLAS_GEODESIC_FAST: out << "FAST"; break;
    case UVATLAS_GEODESIC_QUALITY: out << "QUALITY"; break;
    default: out << int(val) << " (unknown)";
    }
    return out;
  }
}  // namespace DirectX

namespace vmeshenc {

//============================================================================

struct VMCReparametrizationParameters {
  size_t maxCharts = size_t();
  float maxStretch = 0.16667f;
  float gutter = 2.f;
  size_t width = 512;
  size_t height = 512;
  DirectX::UVATLAS uvOptions = DirectX::UVATLAS_DEFAULT;
};

//============================================================================

class Reparametrization {
public:
  Reparametrization() = default;
  ~Reparametrization() = default;

  bool generate(
    vmesh::VMCFrame& frame, const VMCReparametrizationParameters& params);
};

}  // namespace vmeshenc