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

#include <cassert>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "util/matrix.hpp"
#include "util/misc.hpp"
#include "util/vector.hpp"
#include "util/sparsematrix.hpp"

namespace vmesh {

//============================================================================

enum VideoCodecId
{
#if defined(USE_HM_VIDEO_CODEC)
  HM = 0,
#endif
#if defined(USE_VTM_VIDEO_CODEC)
  VTM = 1,
#endif
#if defined(USE_FFMPEG_VIDEO_CODEC)
  FFMPEG = 2,
#endif
  UNKNOWN_VIDEO_CODEC = 255
};

//============================================================================

static std::istream&
operator>>(std::istream& in, VideoCodecId& val)
{
  std::string str;
  in >> str;
  val = VideoCodecId::UNKNOWN_VIDEO_CODEC;
#if defined(USE_HM_VIDEO_CODEC)
  if (str == "HM") {
    val = VideoCodecId::HM;
  }
#endif
#if defined(USE_VTM_VIDEO_CODEC)
  if (str == "VTM")
    val = VideoCodecId::VTM;
#endif
#if defined(USE_FFMPEG_VIDEO_CODEC)
  if (str == "FFMPEG")
    val = VideoCodecId::FFMPEG;
#endif
  if (val == VideoCodecId::UNKNOWN_VIDEO_CODEC) {
    in.setstate(std::ios::failbit);
  }
  return in;
}

//----------------------------------------------------------------------------

static std::ostream&
operator<<(std::ostream& out, VideoCodecId val)
{
  switch (val) {
#if defined(USE_HM_VIDEO_CODEC)
  case VideoCodecId::HM: out << "HM"; break;
#endif
#if defined(USE_VTM_VIDEO_CODEC)
  case VideoCodecId::VTM: out << "VTM"; break;
#endif
#if defined(USE_FFMPEG_VIDEO_CODEC)
  case VideoCodecId::FFMPEG: out << "FFMPEG"; break;
#endif
  case VideoCodecId::UNKNOWN_VIDEO_CODEC: out << "UNKNOWN"; break;
  }
  return out;
}

//============================================================================

enum class ColourSpace
{
  YUV400p,
  YUV420p,
  YUV444p,
  RGB444p,
  BGR444p,
  GBR444p,
  UNKNOW
};


//============================================================================

static std::istream&
operator>>(std::istream& in, ColourSpace& val)
{
  std::string str;
  in >> str;
  if (str == "YUV400p") {
    val = ColourSpace::YUV400p;
  } else if (str == "YUV420p") {
    val = ColourSpace::YUV420p;
  } else if (str == "YUV444p") {
    val = ColourSpace::YUV444p;
  } else if (str == "RGB444p") {
    val = ColourSpace::RGB444p;
  } else if (str == "BGR444p") {
    val = ColourSpace::BGR444p;
  } else if (str == "GBR444p") {
    val = ColourSpace::GBR444p;
  } else {
    in.setstate(std::ios::failbit);
  }
  return in;
}

//----------------------------------------------------------------------------

static std::ostream&
operator<<(std::ostream& out, ColourSpace val)
{
  switch (val) {
  case ColourSpace::YUV400p: out << "YUV400p"; break;
  case ColourSpace::YUV420p: out << "YUV420p"; break;
  case ColourSpace::YUV444p: out << "YUV444p"; break;
  case ColourSpace::RGB444p: out << "RGB444p"; break;
  case ColourSpace::BGR444p: out << "BGR444p"; break;
  case ColourSpace::GBR444p: out << "GBR444p"; break;
  case ColourSpace::UNKNOW: out << "UNKNOW"; break;
  }
  return out;
}

//============================================================================

static std::string
createVideoName(
  const std::string& prefix,
  const int width,
  const int height,
  const int bits,
  const ColourSpace colorSpace)
{
  std::string str;
  str += prefix + "_" + std::to_string(width) + "x" + std::to_string(height)
    + "_" + std::to_string(bits) + "bits_";

  switch (colorSpace) {
  case ColourSpace::YUV400p: str += "p400.yuv"; break;
  case ColourSpace::YUV420p: str += "p420.yuv"; break;
  case ColourSpace::YUV444p: str += "p444.yuv"; break;
  case ColourSpace::RGB444p: str += "p444.rgb"; break;
  case ColourSpace::BGR444p: str += "p444.bgr"; break;
  case ColourSpace::GBR444p: str += "p444.gbr"; break;
  case ColourSpace::UNKNOW: str += "UNKNOW.UNKNOW"; break;
  }
  return str;
}

//============================================================================

template<typename T>
struct Plane {
  void zero() { fill(T(0)); }
  void fill(T v) { std::fill(_buffer.begin(), _buffer.end(), v); }
  void resize(int w = 0, int h = 0)
  {
    _width = w;
    _height = h;
    _buffer.resize(w * h);
  }
  template<typename D>
  Plane& operator=(const Plane<D>& src)
  {
    resize(src.width(), src.height());
    for (int i = 0; i < _height; i++) {
      for (int j = 0; j < _width; j++) {
        set(i, j , (T)src.get(i, j) );
      }
    }
    return *this;
  }

  int size() const {return (int)_buffer.size(); }
  T get(int i, int j) const
  {
    assert(i >= 0 && i < _height);
    assert(j >= 0 && j < _width);
    return _buffer[i * _width + j];
  }
  T& get(int i, int j)
  {
    assert(i >= 0 && i < _height);
    assert(j >= 0 && j < _width);
    return _buffer[i * _width + j];
  }
  T operator()(int i, int j) const
  {
    assert(i >= 0 && i < _height);
    assert(j >= 0 && j < _width);
    return _buffer[i * _width + j];
  }
  T& operator()(int i, int j)
  {
    assert(i >= 0 && i < _height);
    assert(j >= 0 && j < _width);
    return _buffer[i * _width + j];
  }
  T* data(int i, int j)
  {
    assert(i >= 0 && i < _height);
    assert(j >= 0 && j < _width);
    return _buffer + (i * _width + j);
  }
  const T* data(int i, int j) const
  {
    assert(i >= 0 && i < _height);
    assert(j >= 0 && j < _width);
    return _buffer.data() + (i * _width + j);
  }
  void set(int i, int j, T v)
  {
    assert(i >= 0 && i < _height);
    assert(j >= 0 && j < _width);
    _buffer[i * _width + j] = v;
  }
  T* data() { return _buffer.data(); }
  const T* data() const { return _buffer.data(); }
  std::vector<T>& buffer() { return _buffer; }
  const std::vector<T>& buffer() const { return _buffer; }
  int width() const { return _width; }
  int height() const { return _height; }
  void clear() { _buffer.clear(); }

  void log(const std::string& str) const
  {
    printf("%s: ", str.c_str() );
    fflush(stdout);
    for (int v = 0; v < std::min(_height, 1); v++) {
      for (int u = 0; u < std::min(_width, 16); u++) {
        printf("%8u ", get(v, u));
      }
      printf("\n");
    }
  }
  void logF(const std::string& str) const
  {
    printf("%s: ", str.c_str());
    fflush(stdout);
    for (int v = 0; v < std::min(_height, 1); v++) {
      for (int u = 0; u < std::min(_width, 16); u++) {
        printf("%8.4f  ", get(v, u));
      }
      printf("\n");
    }
  }

private:
  int _width;
  int _height;
  std::vector<T> _buffer;
};

//============================================================================

template<typename T>
class Frame {
public:
  Frame(int w = 0 , int h= 0 , ColourSpace colourSpace = ColourSpace::UNKNOW ) { resize(w, h, colourSpace); }

  Vec3<double> fetch(const double y, const double x) const
  {
    const auto heightMinusOne = _height - 1.0;
    const auto widthMinusOne = _width - 1.0;
    const auto i = _height - 1
      - int32_t(Clamp(std::round(y * heightMinusOne), 0.0, heightMinusOne));
    const auto j =
      int32_t(Clamp(std::round(x * widthMinusOne), 0.0, widthMinusOne));
    assert(planeCount() == 3);
    return Vec3<double>(
      _planes[0].get(i, j), _planes[1].get(i, j), _planes[2].get(i, j));
  }

  Vec3<double> bilinear(const double y, const double x) const
  {
    const auto heightMinusOne = _height - 1.0;
    const auto widthMinusOne = _width - 1.0;
    const auto sy = (1.0 - y) * heightMinusOne;
    const auto sx = x * widthMinusOne;
    const auto x0 = int32_t(Clamp(std::floor(sx), 0.0, widthMinusOne));
    const auto x1 = int32_t(Clamp(std::ceil(sx), 0.0, widthMinusOne));
    const auto y0 = int32_t(Clamp(std::floor(sy), 0.0, heightMinusOne));
    const auto y1 = int32_t(Clamp(std::ceil(sy), 0.0, heightMinusOne));
    const auto fx = sx - x0;
    const auto fy = sy - y0;
    const auto oneMinusFx = 1.0 - fx;
    const auto oneMinusFy = 1.0 - fy;
    const auto w00 = oneMinusFx * oneMinusFy;
    const auto w10 = fx * oneMinusFy;
    const auto w01 = oneMinusFx * fy;
    const auto w11 = fx * fy;
    const auto v00 = Vec3<double>(
      _planes[0].get(y0, x0), _planes[1].get(y0, x0), _planes[2].get(y0, x0));
    const auto v10 = Vec3<double>(
      _planes[0].get(y0, x1), _planes[1].get(y0, x1), _planes[2].get(y0, x1));
    const auto v01 = Vec3<double>(
      _planes[0].get(y1, x0), _planes[1].get(y1, x0), _planes[2].get(y1, x0));
    const auto v11 = Vec3<double>(
      _planes[0].get(y1, x1), _planes[1].get(y1, x1), _planes[2].get(y1, x1));
    return w00 * v00 + w10 * v10 + w01 * v01 + w11 * v11;
  }
  void resize(int w, int h, ColourSpace colourSpace)
  {
    _width = w;
    _height = h;
    _colourSpace = colourSpace;
    switch (colourSpace) {
    case ColourSpace::YUV400p:
      _planes.resize(1);
      _planes[0].resize(w, h);
      break;

    default:
    case ColourSpace::YUV420p:
      _planes.resize(3);
      _planes[0].resize(w, h);
      _planes[1].resize(w / 2, h / 2);
      _planes[2].resize(w / 2, h / 2);
      break;

    case ColourSpace::YUV444p:
    case ColourSpace::RGB444p:
    case ColourSpace::BGR444p:
    case ColourSpace::GBR444p:
      _planes.resize(3);
      _planes[0].resize(w, h);
      _planes[1].resize(w, h);
      _planes[2].resize(w, h);
      break;
    };
  }
  void zero()
  {
    for (auto& p : _planes) {
      p.zero();
    }
  }
  void fill(T v)
  {
    for (auto& p : _planes) {
      p.fill(v);
    }
  }
  void clear()
  {
    for ( auto& plane : _planes ) { plane.clear(); }
  }

  Frame(const Frame&) = default;
  Frame& operator=(const Frame&) = default;
  template<typename D>
  Frame& operator=( const Frame<D>& src)
  {
    resize(src.width(), src.height(), src.colourSpace());
    for (int i = 0; i < src.planeCount(); i++) {
      _planes[i] = src.plane(i);
    }
    return *this;
  }

  ~Frame() = default;

  Plane<T>& operator[]( int planeIndex ) { return _planes[planeIndex]; }
  Plane<T>& plane(int planeIndex) { return _planes[planeIndex]; }
  const Plane<T>& plane(int planeIndex) const { return _planes[planeIndex]; }
  int planeCount() const { return int(_planes.size()); }
  int width() const { return _width; }
  int height() const { return _height; }
  ColourSpace colourSpace() const { return _colourSpace; }

  void load(std::istream& is)
  {
    for (auto& plane : _planes) {
      is.read(
        reinterpret_cast<char*>(plane.data()),
        plane.height() * plane.width() * sizeof(T));
    }
  }

  void save(std::ostream& os) const
  {
    for (const auto& plane : _planes) {
      os.write(
        reinterpret_cast<const char*>(plane.data()),
        plane.height() * plane.width() * sizeof(T));
    }
  }
  bool load(const std::string& fin)
  {
    std::ifstream is(fin, std::ios::binary);
    if (!is.is_open()) {
      return false;
    }
    load(is);
    return true;
  }
  bool save(const std::string& fout) const
  {
    std::ofstream os(fout, std::ios::binary);
    if (!os.is_open()) {
      return false;
    }
    save(os);
    return true;
  }

  void log(const std::string& str) const
  {
    printf("%s: %dx%d \n", str.c_str(), _width, _height);
    fflush(stdout);
    for (int v = 0; v < std::min(_height, 1); v++) {
      for (int c = 0; c < 3; c++) {
        for (int u = 0; u < std::min(_width, 16); u++) {
          printf("%2x ", _planes[c].get(v, u));
        }
        printf("    ");
      }
      printf("\n");
    }
  }

  T clamp(T v, T a, T b) const { return ((v < a) ? a : ((v > b) ? b : v)); }

  template <typename Pel>
  void set( const Pel*     Y,
            const Pel*     U,
            const Pel*     V,
            size_t         widthY,
            size_t         heightY,
            size_t         strideY,
            size_t         widthC,
            size_t         heightC,
            size_t         strideC,
            int16_t        shiftbits,
            ColourSpace    format,
            bool           rgb2bgr ) {
    resize( widthY, heightY, format );
    const Pel*   ptr[2][3] = {{Y, U, V}, {V, Y, U}};
    const size_t width[3]  = {widthY, widthC, widthC};
    const size_t height[3] = {heightY, heightC, heightC};
    const size_t stride[3] = {strideY, strideC, strideC};
    int16_t      rounding  = 1 << ( shiftbits - 1 );
    printf(
        "copy image: Shift=%d Round=%d (%4zux%4zu S=%4zu C:%4zux%4zu => "
        "%4zux%4zu) stride = %4zu %4zu bgr=%d sizeof(Pel) = %zu sizeof(T) = %zu \n",
        shiftbits, rounding, widthY, heightY, strideY, widthC, heightC, _width, _height,
        strideY, strideC, rgb2bgr, sizeof(Pel), sizeof(T) );
    for ( size_t c = 0; c < 3; c++ ) {
      auto* src = ptr[rgb2bgr][c];
      auto* dst = _planes[c].data();
      if ( shiftbits > 0 ) {
        T minval = 0;
        T maxval = ( T )( ( 1 << ( 10 - (int)shiftbits ) ) - 1 );
        for ( size_t v = 0; v < height[c]; ++v, src += stride[c], dst += width[c] ) {
          for ( size_t u = 0; u < width[c]; ++u ) {
            dst[u] = clamp( ( T )( ( src[u] + rounding ) >> shiftbits ), minval, maxval );
          }
        }
      } else {
        for ( size_t v = 0; v < height[c]; ++v, src += stride[c], dst += width[c] ) {
          for ( size_t u = 0; u < width[c]; ++u ) { dst[u] = (T)src[u]; }
        }
      }
    }
  }

  template <typename Pel>
  void get( Pel*    Y,
            Pel*    U,
            Pel*    V,
            size_t  widthY,
            size_t  heightY,
            size_t  strideY,
            size_t  widthC,
            size_t  heightC,
            size_t  strideC,
            int16_t shiftbits,
            bool    rgb2bgr ) {
    size_t chromaSubsample = widthY / widthC;
    if ( ( chromaSubsample == 1 && _colourSpace == ColourSpace::YUV420p ) ||
         ( chromaSubsample == 2 && _colourSpace != ColourSpace::YUV420p ) ) {
      printf( "Error: image get not possible from image of format = %d with  chromaSubsample = %zu \n",
              (int32_t)_colourSpace, chromaSubsample );
      exit( -1 );
    }
    size_t       widthChroma  = _width / chromaSubsample;
    size_t       heightChroma = _height / chromaSubsample;
    Pel*         ptr[2][3]    = {{Y, U, V}, {V, Y, U}};
    const size_t width[3]     = {(size_t)_width, widthChroma, widthChroma};
    const size_t heightSrc[3] = {(size_t)_height, heightChroma, heightChroma};
    const size_t heightDst[3] = {heightY, heightC, heightC};
    const size_t stride[3]    = {strideY, strideC, strideC};
    printf( "copy image: Shift = %d (%4zux%4zu => %4zux%4zu S=%4zu C: %4zux%4zu rgb2bgr = %d ) \n", shiftbits, _width,
            _height, widthY, heightY, strideY, widthC, heightC, rgb2bgr );
    for ( size_t c = 0; c < 3; c++ ) {
      auto* src = _planes[c].data();
      auto* dst = ptr[rgb2bgr][c];
      if ( shiftbits > 0 ) {
        for ( size_t v = 0; v < heightSrc[c]; ++v, src += width[c], dst += stride[c] ) {
          for ( size_t u = 0; u < width[c]; ++u ) { dst[u] = ( Pel )( src[u] ) << shiftbits; }
        }
      } else {
        for ( size_t v = 0; v < heightSrc[c]; ++v, src += width[c], dst += stride[c] ) {
          for ( size_t u = 0; u < width[c]; ++u ) { dst[u] = (Pel)src[u]; }
        }
      }
      for ( size_t v = heightSrc[c]; v < heightDst[c]; ++v, dst += stride[c] ) {
        for ( size_t u = 0; u < width[c]; ++u ) { dst[u] = 0; }
      }
    }
  }

private:
  int _width{};
  int _height{};
  ColourSpace _colourSpace;
  std::vector<Plane<T>> _planes;
};

//============================================================================

template<typename T>
class FrameSequence {
public:
  FrameSequence(
    int w = 0, int h = 0, ColourSpace colourSpace = ColourSpace::UNKNOW, int f = 0)
  {
    resize(w, h, colourSpace, f);
  }
  FrameSequence(const FrameSequence&) = default;
  template<typename D>
  FrameSequence(const FrameSequence<D>& src)
  {
    *this = src;    
  }
  FrameSequence& operator=(const FrameSequence&) = default;
  template<typename D>
  FrameSequence& operator=(const FrameSequence<D>& src)
  {
    resize(src.width(), src.height(), src.colourSpace(), src.frameCount());
    for (int i = 0; i < src.frameCount(); i++) {
      _frames[i] = src.frame(i);
    }
    return *this;
  }
  ~FrameSequence() = default;

  std::string createName(const std::string& prefix, int bits)
  {
    return createVideoName(prefix, _width, _height, bits, _colourSpace);
  }
  Frame<T>& operator[]( int frameIndex ) { return _frames[frameIndex]; }
  typename std::vector<Frame<T> >::iterator begin() { return _frames.begin(); }
  typename std::vector<Frame<T> >::iterator end() { return _frames.end(); }

  void resize(int w, int h, ColourSpace colourSpace, int f)
  {
    _width = w;
    _height = h;
    _colourSpace = colourSpace;
    _frames.resize(f);
    for (auto& frame : _frames) {
      frame.resize(w, h, colourSpace);
    }
  }
  void clear()
  {
    for (auto& frame : _frames) {
      frame.clear();
    }
    _frames.clear();
  }

  Frame<T>& frame(int frameIndex)
  {
    assert(frameIndex < frameCount());
    return _frames[frameIndex];
  }
  const Frame<T>& frame(int frameIndex) const
  {
    assert(frameIndex < frameCount());
    return _frames[frameIndex];
  }
  int frameCount() const { return int(_frames.size()); }
  int width() const { return _width; }
  int height() const { return _height; }
  ColourSpace colourSpace() const { return _colourSpace; }

  void load(std::istream& is)
  {
    for (auto& frame : _frames) {
      frame.load(is);
    }
  }
  void save(std::ostream& os) const
  {
    for (auto& frame : _frames) {
      frame.save(os);
    }
  }
  bool load(const std::string& fin)
  {
    std::ifstream is(fin, std::ios::binary);
    if (!is.is_open()) {
      return false;
    }
    load(is);
    return true;
  }

  bool save(const std::string& fout) const
  {
    std::ofstream os(fout, std::ios::binary);
    if (!os.is_open()) {
      return false;
    }
    save(os);
    return true;
  }

private:
  int _width{};
  int _height{};
  ColourSpace _colourSpace;
  std::vector<Frame<T>> _frames;
};

//============================================================================

template<typename T1, typename T2>
void
DilatePadding(
  const Frame<T1>& input,
  const Plane<T2>& inputOccupancy,
  Frame<T1>& output,
  Plane<T2>& outputOccupancy)
{
  const auto width = input.width();
  const auto height = input.height();
  const auto heightMinus1 = height - 1;
  const auto widthMinus1 = width - 1;
  const auto planeCount = input.planeCount();

  assert(inputOccupancy.width() == width && inputOccupancy.height() == height);

  outputOccupancy = inputOccupancy;
  output = input;

  std::vector<int32_t> sum(planeCount);

  for (int32_t i = 0; i < height; ++i) {
    for (int32_t j = 0; j < width; ++j) {
      if (inputOccupancy(i, j)) {
        continue;
      }

      int32_t nCount = 0;
      std::fill(sum.begin(), sum.end(), 0);

      if (i > 0 && inputOccupancy(i - 1, j)) {
        ++nCount;
        for (int32_t p = 0; p < planeCount; ++p) {
          sum[p] += input.plane(p).get(i - 1, j);
        }
      }

      if (j > 0 && inputOccupancy(i, j - 1)) {
        ++nCount;
        for (int32_t p = 0; p < planeCount; ++p) {
          sum[p] += input.plane(p).get(i, j - 1);
        }
      }

      if (i < heightMinus1 && inputOccupancy(i + 1, j)) {
        ++nCount;
        for (int32_t p = 0; p < planeCount; ++p) {
          sum[p] += input.plane(p).get(i + 1, j);
        }
      }
      if (j < widthMinus1 && inputOccupancy(i, j + 1)) {
        ++nCount;
        for (int32_t p = 0; p < planeCount; ++p) {
          sum[p] += input.plane(p).get(i, j + 1);
        }
      }
      if (nCount) {
        outputOccupancy.set(i, j, T2(255));
        for (int32_t p = 0; p < planeCount; ++p) {
          output.plane(p).set(i, j, (sum[p] + (nCount >> 1)) / nCount);
        }
      }
    }
  }
}

//============================================================================

template<typename T1, typename T2>
void
PullPushPadding(Frame<T1>& input, const Plane<T2>& occupancy)
{
  const auto width = input.width();
  const auto height = input.height();
  const auto colourSpace = input.colourSpace();
  const auto planeCount = input.planeCount();

  assert(occupancy.width() == width && occupancy.height() == height);

  struct LevelOfDetail {
    Frame<float> values;
    Plane<float> weights;
  };

  std::vector<std::shared_ptr<LevelOfDetail>> mipmaps;
  mipmaps.reserve(16);

  std::shared_ptr<LevelOfDetail> levelOfDetail0(new LevelOfDetail());
  mipmaps.push_back(levelOfDetail0);

  Frame<float>* values0 = &(levelOfDetail0->values);
  Plane<float>* weights0 = &(levelOfDetail0->weights);
  values0->resize(width, height, colourSpace);
  weights0->resize(width, height);
  for (int32_t p = 0; p < planeCount; ++p) {
    const auto& iplane = input.plane(p);
    auto& vplane0 = values0->plane(p);
    for (int32_t i = 0; i < height; ++i) {
      for (int32_t j = 0; j < width; ++j) {
        vplane0.set(i, j, iplane(i, j));
      }
    }
  }
  for (int32_t i = 0; i < height; ++i) {
    for (int32_t j = 0; j < width; ++j) {
      weights0->set(i, j, occupancy(i, j) != T2(0));
    }
  }

  const int32_t K = 3;
  const int32_t K2 = 1;
  const float kernel[K][K] = {
    {0.0625, 0.1250, 0.0625},
    {0.1250, 0.2500, 0.1250},
    {0.0625, 0.1250, 0.0625},
  };

  std::vector<float> v(planeCount);

  // pull
  int32_t levelOfDetailCount = 1;
  auto width0 = width;
  auto height0 = height;
  while (width0 > 1 && height0 > 1) {
    const auto width1 = (width0 + 1) >> 1;
    const auto height1 = (height0 + 1) >> 1;
    std::shared_ptr<LevelOfDetail> levelOfDetail(new LevelOfDetail());
    Frame<float>* values1 = &(levelOfDetail->values);
    Plane<float>* weights1 = &(levelOfDetail->weights);
    values1->resize(width1, height1,colourSpace);
    weights1->resize(width1, height1);
    values1->fill(0.0F);
    for (int32_t i1 = 0; i1 < height1; ++i1) {
      for (int32_t j1 = 0; j1 < width1; ++j1) {
        std::fill(v.begin(), v.end(), 0.0F);
        float w1 = 0.0F;
        for (int32_t k1 = 0; k1 < K; ++k1) {
          const auto i0 = (i1 << 1) + k1 - K2;
          if (i0 < 0 || i0 >= height0) {
            continue;
          }
          for (int32_t k2 = 0; k2 < K; ++k2) {
            const auto j0 = (j1 << 1) + k2 - K2;
            if (j0 < 0 || j0 >= width0) {
              continue;
            }
            const auto w0 =
              kernel[k1][k2] * std::min(weights0->get(i0, j0), 1.0F);
            for (int32_t p = 0; p < planeCount; ++p) {
              v[p] += w0 * values0->plane(p)(i0, j0);
            }
            w1 += w0;
          }
        }
        for (int32_t p = 0; w1 > 0.0F && p < planeCount; ++p) {
          values1->plane(p)(i1, j1) = v[p] / w1;
        }
        weights1->set(i1, j1, w1);
      }
    }

    mipmaps.push_back(levelOfDetail);
    ++levelOfDetailCount;
    values0 = values1;
    weights0 = weights1;
    width0 = width1;
    height0 = height1;
  };

  for (int32_t indexLoD = levelOfDetailCount - 2; indexLoD >= 0; --indexLoD) {
    const auto levelOfDetailCurrent = mipmaps[indexLoD];
    Frame<float>* valuesCurrent = &(levelOfDetailCurrent->values);
    Plane<float>* weightsCurrent = &(levelOfDetailCurrent->weights);
    const auto widthCurrent = weightsCurrent->width();
    const auto heightCurrent = weightsCurrent->height();

    const auto levelOfDetailNext = mipmaps[indexLoD + 1];
    Frame<float>* valuesNext = &(levelOfDetailNext->values);
    Plane<float>* weightsNext = &(levelOfDetailNext->weights);
    const auto widthNext = weightsNext->width();
    const auto heightNext = weightsNext->height();

    std::shared_ptr<LevelOfDetail> tmpLevelOfDetail(new LevelOfDetail());
    Frame<float>* tvalues0 = &(tmpLevelOfDetail->values);
    Plane<float>* tweights0 = &(tmpLevelOfDetail->weights);
    tvalues0->resize(widthCurrent, heightCurrent, colourSpace);
    tweights0->resize(widthCurrent, heightCurrent);

    for (int32_t i0 = 0; i0 < heightCurrent; ++i0) {
      for (int32_t j0 = 0; j0 < widthCurrent; ++j0) {
        std::fill(v.begin(), v.end(), 0.0F);
        float omega0 = 0.0F;
        for (int32_t k1 = 0; k1 < K; ++k1) {
          const auto i1 = (i0 >> 1) + k1 - K2;
          if (i1 < 0 || i1 >= heightNext) {
            continue;
          }
          for (int32_t k2 = 0; k2 < K; ++k2) {
            const auto j1 = (j0 >> 1) + k2 - K2;
            if (j1 < 0 || j1 >= widthNext) {
              continue;
            }
            const auto w1 =
              kernel[k1][k2] * std::min(weightsNext->get(i1, j1), 1.0F);
            for (int32_t p = 0; p < planeCount; ++p) {
              v[p] += w1 * valuesNext->plane(p)(i1, j1);
            }
            omega0 += w1;
          }
        }
        const auto w0 = std::min(1.0F, weightsCurrent->get(i0, j0));
        ;
        const auto w0Updated = std::min(1.0F, omega0) * (1.0F - w0) + w0;
        tweights0->set(i0, j0, w0Updated);
        assert(omega0 > 0.0f);
        for (int32_t p = 0; p < planeCount; ++p) {
          const auto x0 = valuesCurrent->plane(p)(i0, j0);
          const auto y0 = v[p] / omega0;
          tvalues0->plane(p)(i0, j0) = y0 * (1.0F - w0) + w0 * x0;
        }
      }
    }
    mipmaps[indexLoD].swap(tmpLevelOfDetail);
  }

  const auto maxValue = std::numeric_limits<T1>::max();
  for (int32_t p = 0; p < planeCount; ++p) {
    const auto& vplane = mipmaps[0]->values.plane(p);
    auto& plane = input.plane(p);
    for (int32_t i = 0; i < height; ++i) {
      for (int32_t j = 0; j < width; ++j) {
        if (occupancy(i, j)) {
          continue;
        }
        const auto value = std::round(vplane(i, j));
        if (value >= 0.0F && value <= maxValue) {
          plane.set(i, j, T1(value));
        } else if (value > maxValue) {
          plane.set(i, j, maxValue);
        } else {
          plane.set(i, j, T1(0));
        }
      }
    }
  }
}

template<typename T1, typename T2>
void
SparseLinearPadding(
  Frame<T1>& input,
  const Plane<T2>& occupancy,
  const double maxError = 0.001,
  const int32_t maxIterationCount = -1)
{
  const auto width = input.width();
  const auto height = input.height();
  const auto planeCount = input.planeCount();

  assert(occupancy.width() == width && occupancy.height() == height);
  Plane<int32_t> mapping;
  mapping.resize(width, height);
  mapping.fill(-1);
  int32_t toPaddPixelCount = 0;
  for (int32_t i = 0; i < height; ++i) {
    for (int32_t j = 0; j < width; ++j) {
      if (!occupancy(i, j)) {
        mapping.set(i, j, toPaddPixelCount++);
      }
    }
  }

  SparseMatrix<double> A(toPaddPixelCount, toPaddPixelCount);
  Matrix<double> B(planeCount, toPaddPixelCount);
  Matrix<double> X(planeCount, toPaddPixelCount);
  B = 0.0;
  for (int32_t i = 0; i < height; ++i) {
    for (int32_t j = 0; j < width; ++j) {
      const auto index = mapping(i, j);
      if (index >= 0) {
        const int32_t shift[4][2] = {{-1, -0}, {0, -1}, {1, 0}, {0, 1}};
        int32_t count = 0;
        for (const auto* k : shift) {
          const auto i1 = i + k[0];
          const auto j1 = j + k[1];
          if (i1 < 0 || j1 < 0 || i1 >= height || j1 >= width) {
            continue;
          }
          ++count;
          const auto index1 = mapping(i1, j1);
          if (index1 >= 0) {
            A.addElementInOrder(index, index1, -1.0);
          } else {
            for (int32_t dim = 0; dim < planeCount; ++dim) {
              B(dim, index) += input.plane(dim).get(i1, j1);
            }
          }
        }
        A.addElementInOrder(index, index, count);
        for (int32_t dim = 0; dim < planeCount; ++dim) {
          X(dim, index) = input.plane(dim).get(i, j);
        }
      }
    }
  }
  A.updatePointers();
  const auto itCount =
    maxIterationCount == -1 ? toPaddPixelCount * 2 : maxIterationCount;
  VecN<double> r(toPaddPixelCount);
  VecN<double> p(toPaddPixelCount);
  VecN<double> q(toPaddPixelCount);
  for (int32_t dim = 0; dim < planeCount; ++dim) {
    auto* x = X.row(dim);
    p = A * x;
    const auto* b = B.row(dim);
    for (int32_t i = 0; i < toPaddPixelCount; ++i) {
      p[i] = r[i] = b[i] - p[i];
    }
    double rtr = 0.0;
    for (int32_t i = 0; i < toPaddPixelCount; ++i) {
      rtr += r[i] * r[i];
    }
    double error = std::sqrt(rtr / toPaddPixelCount);
    int32_t it = 0;
    //    std::cout << it << " -> " << error << '\n';
    for (; it < itCount && error > maxError; ++it) {
      q = A * p;
      double pAp = 0.0;
      for (int32_t i = 0; i < toPaddPixelCount; ++i) {
        pAp += p[i] * q[i];
      }
      const auto alpha = rtr / pAp;
      for (int32_t i = 0; i < toPaddPixelCount; ++i) {
        x[i] += alpha * p[i];
        r[i] -= alpha * q[i];
      }
      double r1tr1 = 0.0;
      for (int32_t i = 0; i < toPaddPixelCount; ++i) {
        r1tr1 += r[i] * r[i];
      }
      const auto betha = r1tr1 / rtr;
      for (int32_t i = 0; i < toPaddPixelCount; ++i) {
        p[i] = r[i] + betha * p[i];
      }
      rtr = r1tr1;
      error = std::sqrt(r1tr1 / toPaddPixelCount);
      //      std::cout << it << " -> " << error << '\n';
    }
    const auto maxValue = std::numeric_limits<T1>::max();
    auto& plane = input.plane(dim);
    for (int32_t i = 0; i < height; ++i) {
      for (int32_t j = 0; j < width; ++j) {
        const auto index = mapping(i, j);
        if (index >= 0) {
          const auto v = std::round(x[index]);
          if (v >= 0.0 && v <= maxValue) {
            plane.set(i, j, T1(v));
          } else if (v > maxValue) {
            plane.set(i, j, maxValue);
          } else {
            plane.set(i, j, T1(0));
          }
        }
      }
    }
  }
}

//============================================================================

enum class ImageFormat
{
  PNG,
  BMP,
  TGA,
  JPG
};

//============================================================================

bool LoadImage(
  const std::string& fileName, Frame<uint8_t>& image);
bool LoadImage(
  const std::string& fileName, int f, Frame<uint8_t>& image);

bool SaveImage(
  const std::string& fileName,
  const Frame<uint8_t>& image,
  ImageFormat format = ImageFormat::PNG,
  int32_t quality = 100);

//============================================================================

}  // namespace vmesh
