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

#include <algorithm>
#include <cassert>
#include <cmath>

#include "vmesh/util/vector.hpp"

namespace vmesh {

//============================================================================

// Replace any occurence of %d with formatted number.  The %d format
// specifier may use the formatting conventions of snprintf().
std::string expandNum(const std::string& src, int num);

//============================================================================

template<class T>
T
Clamp(const T& v, const T& lo, const T& hi)
{
  assert(!(hi < lo));
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

//============================================================================

inline uint32_t
nextPowerOfTwo(uint32_t x)
{
  return x == 1 ? 1 : 1 << (32 - __builtin_clz(x - 1));
}

//============================================================================

template<class T>
Vec3<T>
orderComponents(Vec3<T> v)
{
  if (v[0] > v[1])
    std::swap(v[0], v[1]);
  if (v[1] > v[2])
    std::swap(v[1], v[2]);
  if (v[0] > v[1])
    std::swap(v[0], v[1]);
  return v;
}

//----------------------------------------------------------------------------

inline uint32_t
extracOddBits(uint32_t x)
{
  x = x & 0x55555555;
  x = (x | (x >> 1)) & 0x33333333;
  x = (x | (x >> 2)) & 0x0F0F0F0F;
  x = (x | (x >> 4)) & 0x00FF00FF;
  x = (x | (x >> 8)) & 0x0000FFFF;
  return x;
}

//----------------------------------------------------------------------------

inline void
computeMorton2D(const uint32_t i, int32_t& x, int32_t& y)
{
  x = int32_t(extracOddBits(i >> 1));
  y = int32_t(extracOddBits(i));
}

//============================================================================

template<class T>
void
computeLocalCoordinatesSystem(const Vec3<T>& n, Vec3<T>& t, Vec3<T>& b)
{
  const auto one = T(1);
  const auto zero = T(0);
  const Vec3<T> e0(one, zero, zero);
  const Vec3<T> e1(zero, one, zero);
  const Vec3<T> e2(zero, zero, one);
  const auto d0 = n * e0;
  const auto d1 = n * e1;
  const auto d2 = n * e2;
  const auto ad0 = std::abs(d0);
  const auto ad1 = std::abs(d1);
  const auto ad2 = std::abs(d2);
  if (ad0 <= ad1 && ad0 <= ad2) {
    t = e0 - d0 * n;
  } else if (ad1 <= ad2) {
    t = e1 - d1 * n;
  } else {
    t = e2 - d2 * n;
  }
  t.normalize();
  assert(std::abs(t * n) < 0.000001);
  b = n ^ t;
}

//============================================================================

}  // namespace vmesh
