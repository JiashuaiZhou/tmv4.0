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
#include <string>
#include <fstream>

#include "util/vector.hpp"

namespace vmesh {

//============================================================================

// Replace any occurence of %d with formatted number.  The %d format
// specifier may use the formatting conventions of snprintf().
std::string expandNum(const std::string& src, int num);

//============================================================================

static inline std::string
removeExtension(const std::string& string) {
  size_t pos = string.find_last_of('.');
  return pos != std::string::npos ? string.substr(0, pos) : string;
}

//============================================================================

#ifdef WIN32
#  include <windows.h>
static void
mkdir(const char* pDirectory, int) {
  CreateDirectory(pDirectory, NULL);
}
#else
#  include <sys/dir.h>
#endif

//============================================================================

#ifndef _WIN32
static char
getSeparator() {
  return '/';
}
#else
static char
getSeparator() {
  return '\\';
}
#endif

//============================================================================

static char
getSeparator(const std::string& eFilename) {
  auto pos1 = eFilename.find_last_of('/');
  auto pos2 = eFilename.find_last_of('\\');
  auto pos  = (std::max)(pos1 != std::string::npos ? pos1 : 0,
                        pos2 != std::string::npos ? pos2 : 0);
  return (pos != 0 ? eFilename[pos] : getSeparator());
}

//============================================================================

static std::string
directoryName(const std::string& string) {
  auto position = string.find_last_of(getSeparator(string));
  if (position != std::string::npos) { return string.substr(0, position); }
  return string;
}

//============================================================================
static int
save(const std::string& filename, std::vector<uint8_t>& buffer) {
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    printf("Can not save: %s \n", filename.c_str());
    return -1;
  }
  file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
  file.close();
  return 0;
}

//============================================================================

static inline std::string
basename(const std::string& string) {
  auto position = string.find_last_of(getSeparator(string));
  if (position != std::string::npos) {
    return string.substr(position + 1, string.length());
  }
  return string;
}

//============================================================================

template<class T>
T
Clamp(const T& v, const T& lo, const T& hi) {
  assert(!(hi < lo));
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

//============================================================================
// Round @x up to next power of two.
//
inline uint32_t
ceilpow2(uint32_t x) {
  x--;
  x = x | (x >> 1);
  x = x | (x >> 2);
  x = x | (x >> 4);
  x = x | (x >> 8);
  x = x | (x >> 16);
  return x + 1;
}

//---------------------------------------------------------------------------
// Population count -- return the number of bits set in @x.
//
inline int
popcnt(uint32_t x) {
  x = x - ((x >> 1) & 0x55555555U);
  x = (x & 0x33333333U) + ((x >> 2) & 0x33333333U);
  return (((x + (x >> 4)) & 0xF0F0F0FU) * 0x1010101U) >> 24;
}

//---------------------------------------------------------------------------
// Compute \left\floor \text{log}_2(x) \right\floor.
// NB: ilog2(0) = -1.
inline int
ilog2(uint32_t x) {
  x = ceilpow2(x + 1) - 1;
  return popcnt(x) - 1;
}

//============================================================================

template<class T>
Vec3<T>
orderComponents(Vec3<T> v) {
  if (v[0] > v[1]) { std::swap(v[0], v[1]); }
  if (v[1] > v[2]) { std::swap(v[1], v[2]); }
  if (v[0] > v[1]) { std::swap(v[0], v[1]); }
  return v;
}

//----------------------------------------------------------------------------

inline uint32_t
extracOddBits(uint32_t x) {
  x = x & 0x55555555;
  x = (x | (x >> 1)) & 0x33333333;
  x = (x | (x >> 2)) & 0x0F0F0F0F;
  x = (x | (x >> 4)) & 0x00FF00FF;
  x = (x | (x >> 8)) & 0x0000FFFF;
  return x;
}

//----------------------------------------------------------------------------

inline void
computeMorton2D(const uint32_t i, int32_t& x, int32_t& y) {
  x = int32_t(extracOddBits(i >> 1));
  y = int32_t(extracOddBits(i));
}

//============================================================================

template<class T>
void
computeLocalCoordinatesSystem(const Vec3<T>& n, Vec3<T>& t, Vec3<T>& b) {
  const auto    one  = T(1);
  const auto    zero = T(0);
  const Vec3<T> e0(one, zero, zero);
  const Vec3<T> e1(zero, one, zero);
  const Vec3<T> e2(zero, zero, one);
  const auto    d0  = n * e0;
  const auto    d1  = n * e1;
  const auto    d2  = n * e2;
  const auto    ad0 = std::abs(d0);
  const auto    ad1 = std::abs(d1);
  const auto    ad2 = std::abs(d2);
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
