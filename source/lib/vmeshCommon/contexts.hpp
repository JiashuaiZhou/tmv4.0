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

#include "entropy.hpp"
#include "util/misc.hpp"
#include "util/vector.hpp"
#include <schroedinger/schroarith.h>

namespace vmesh {

//============================================================================

struct VMCMotionACContext {
  AdaptiveBitModel ctxPred;
  AdaptiveBitModel ctxSign[3];
  AdaptiveBitModel ctxCoeffGtN[2][3];
  AdaptiveBitModel ctxCoeffRemPrefix[3][7];
  AdaptiveBitModel ctxCoeffRemSuffix[3][7];

  int32_t
  estimateBits(const Vec3<int32_t>& residual, const int32_t predIndex) const
  {
    int32_t bits = ctxPred.estimateBits(predIndex != 0);
    for (int32_t k = 0; k < 3; ++k) {
      auto value = residual[k];
      bits += ctxCoeffGtN[0][k].estimateBits(value != 0);
      if (value == 0) {
        continue;
      }
      bits += ctxSign[k].estimateBits(value < 0);
      value = std::abs(value) - 1;
      bits += ctxCoeffGtN[1][k].estimateBits(value != 0);
      if (value == 0) {
        continue;
      }
      const auto log2Delta = 1 + ilog2(uint32_t(value));
      bits += ((log2Delta << 1) + 1) << 10;
    }
    return bits;
  }
};

//============================================================================

}  // namespace vmesh
