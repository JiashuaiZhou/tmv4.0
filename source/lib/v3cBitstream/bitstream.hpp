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
#include "util/logger.hpp"
#include "videoBitstream.hpp"
#include "util/memory.hpp"
#include "vmc.hpp"

namespace vmesh {

struct BistreamPosition {
  uint64_t bytes_;
  uint8_t  bits_;
};

class VideoBitstream;

class BitstreamGofStat {
public:
  BitstreamGofStat() {
    v3cUnitSize_.resize(NUM_V3C_UNIT_TYPE, 0);
    videoBinSize_.resize(NUM_VIDEO_TYPE, 0);
    baseMeshBinSize_.resize(RESERVED_BASEMESH + 1, 0);
  }
  ~BitstreamGofStat() {}
  void overwriteV3CUnitSize(V3CUnitType type, size_t size) {
    v3cUnitSize_[type] = size;
  }
  void resetBaseMeshBinSize() {
    for (auto& el : baseMeshBinSize_) el = 0;
  }

  auto& getFaceCount() { return faceCount; }
  void setV3CUnitSize(V3CUnitType type, size_t size) {
    v3cUnitSize_[type] += size;
  }
  void setVideo(const VideoBitstream& video) {
    videoBinSize_[video.type()] += video.size();
  }
  void setBaseMesh(BaseMeshType type, size_t size) {
    baseMeshBinSize_[type] += size;
  }
  size_t getV3CUnitSize(V3CUnitType type) { return v3cUnitSize_[type]; }
  size_t getVideoBinSize(VideoType type) { return videoBinSize_[type]; }
  BitstreamGofStat& operator+=(const BitstreamGofStat& other) {
    v3cUnitSize_     = other.v3cUnitSize_;
    videoBinSize_    = other.videoBinSize_;
    baseMeshBinSize_ = other.baseMeshBinSize_;
    faceCount        = other.faceCount;
    return *this;
  }
  size_t getTotal() {
    return v3cUnitSize_[V3C_VPS] + v3cUnitSize_[V3C_AD] + v3cUnitSize_[V3C_BMD]
           + v3cUnitSize_[V3C_OVD] + v3cUnitSize_[V3C_GVD]
           + v3cUnitSize_[V3C_AVD];
  }
  size_t getTotalGeometry() {
    size_t retVal = 0;
    for (int i = VIDEO_GEOMETRY; i <= VIDEO_GEOMETRY_RAW; i++)
      retVal += videoBinSize_[i];
    return retVal;
  }
  size_t getTotalAttribute() {
    size_t retVal = 0;
    for (int i = VIDEO_ATTRIBUTE;
         i < VIDEO_ATTRIBUTE_RAW + MAX_NUM_ATTR_PARTITIONS;
         i++)
      retVal += videoBinSize_[i];
    return retVal;
  }
  size_t getTotalBaseMesh() {
    return baseMeshBinSize_[I_BASEMESH] + baseMeshBinSize_[P_BASEMESH];
  }
  size_t getTotalBaseMeshIntra() { return baseMeshBinSize_[I_BASEMESH]; }
  size_t getTotalBaseMeshInter() { return baseMeshBinSize_[P_BASEMESH]; }
  size_t getTotalMetadata() {
    return v3cUnitSize_[V3C_VPS] + v3cUnitSize_[V3C_AD]
           + (v3cUnitSize_[V3C_BMD] - getTotalBaseMesh())
           + (v3cUnitSize_[V3C_GVD] - getTotalGeometry())
           + (v3cUnitSize_[V3C_AVD] - getTotalAttribute());
  }

  void trace() {
    printf("    V3CUnitSize[ V3C_VPS ]: %9zu B %9zu b\n",
           v3cUnitSize_[V3C_VPS],
           v3cUnitSize_[V3C_VPS] * 8);
    printf("    V3CUnitSize[ V3C_AD  ]: %9zu B %9zu b\n",
           v3cUnitSize_[V3C_AD],
           v3cUnitSize_[V3C_AD] * 8);
    printf("    V3CUnitSize[ V3C_BMD ]: %9zu B %9zu b ( Header = %9zu B +"
           " Intra = %9zu B + Inter = %9zu B )\n",
           v3cUnitSize_[V3C_BMD],
           v3cUnitSize_[V3C_BMD] * 8,
           v3cUnitSize_[V3C_BMD] - getTotalBaseMesh(),
           getTotalBaseMeshIntra(),
           getTotalBaseMeshInter());
    printf("    V3CUnitSize[ V3C_GVD ]: %9zu B %9zu b ( Header = %9zu B + "
           "Video = %9zu B)\n",
           v3cUnitSize_[V3C_GVD],
           v3cUnitSize_[V3C_GVD] * 8,
           v3cUnitSize_[V3C_GVD] - getTotalGeometry(),
           getTotalGeometry());
    printf("    V3CUnitSize[ V3C_AVD ]: %9zu B %9zu b ( Header = %9zu B + "
           "Video = %9zu B)\n",
           v3cUnitSize_[V3C_AVD],
           v3cUnitSize_[V3C_AVD] * 8,
           v3cUnitSize_[V3C_AVD] - getTotalAttribute(),
           getTotalAttribute());
  }

private:
  std::vector<size_t> v3cUnitSize_;
  std::vector<size_t> videoBinSize_;
  std::vector<size_t> baseMeshBinSize_;
  size_t              faceCount = 0;
};

class BitstreamStat {
public:
  BitstreamStat() { header_ = 0; }
  ~BitstreamStat() { bitstreamGofStat_.clear(); }
  void newGOF() {
    BitstreamGofStat element;
    bitstreamGofStat_.push_back(element);
  }
  void setHeader(size_t size) { header_ = size; }
  void incrHeader(size_t size) { header_ += size; }
  void overwriteV3CUnitSize(V3CUnitType type, size_t size) {
    bitstreamGofStat_.back().overwriteV3CUnitSize(type, size);
  }
  void resetBaseMeshBinSize() {
    bitstreamGofStat_.back().resetBaseMeshBinSize();
  }
  void setV3CUnitSize(V3CUnitType type, size_t size) {
    bitstreamGofStat_.back().setV3CUnitSize(type, size);
  }
  void setVideo(const VideoBitstream& video) {
    bitstreamGofStat_.back().setVideo(video);
  }
  void setBaseMesh(BaseMeshType type, size_t size) {
    bitstreamGofStat_.back().setBaseMesh(type, size);
  }
  
  auto& getFaceCount() { return bitstreamGofStat_.back().getFaceCount(); }
  size_t getV3CUnitSize(V3CUnitType type) {
    return bitstreamGofStat_.back().getV3CUnitSize(type);
  }
  size_t getVideoBinSize(VideoType type) {
    return bitstreamGofStat_.back().getVideoBinSize(type);
  }
  size_t getTotalGeometry() {
    return bitstreamGofStat_.back().getTotalGeometry();
  }
  size_t getTotalAttribute() {
    return bitstreamGofStat_.back().getTotalAttribute();
  }
  size_t getTotalMetadata() {
    return bitstreamGofStat_.back().getTotalMetadata();
  }

  void trace(bool byGOF = false) {
    if( !byGOF)
      printf("\n------- All frames -----------\n");
    printf("Bitstream stat: \n");
    printf(
      "  V3CHeader:                %9zu B %9zu b\n", header_, header_ * 8);
    if (byGOF) {
      for (size_t i = 0; i < bitstreamGofStat_.size(); i++) {
        printf("  Gof %2zu: \n", i);
        bitstreamGofStat_[i].trace();
      }
      printf("  All gofs: \n");
    }
    BitstreamGofStat allGof;
    for (auto& element : bitstreamGofStat_) { allGof += element; }
    allGof.trace();
    auto totalMetadata  = allGof.getTotalMetadata() + header_;
    auto totalBaseMesh  = allGof.getTotalBaseMesh();
    auto totalGeometry  = allGof.getTotalGeometry();
    auto totalAttribute = allGof.getTotalAttribute();
    auto total          = allGof.getTotal() + header_;
    printf("  By types \n");
    printf("    Metadata:               %9zu B %9zu b \n",
           totalMetadata,
           totalMetadata * 8);
    printf("    BaseMesh:               %9zu B %9zu b \n",
           totalBaseMesh,
           totalBaseMesh * 8);
    printf("    Displacement:           %9zu B %9zu b \n",
           totalGeometry,
           totalGeometry * 8);
    printf("    Attribute:              %9zu B %9zu b \n",
           totalAttribute,
           totalAttribute * 8);
    printf("    Total:                  %9zu B %9zu b \n", total, total * 8);
    if (!byGOF) {
      traceTime();
      auto duration = (std::max(getTime("compress"), getTime("decompress")));
      printf("Sequence processing time   %9f s \n", duration);
      printf("Sequence peak memory       %9d KB\n", getPeakMemory());
      printf("Sequence face count        %9zu\n", allGof.getFaceCount());
      printf("---------------------------------------\n");
    }
  }

private:
  size_t                        header_;
  std::vector<BitstreamGofStat> bitstreamGofStat_;
};

class Bitstream {
public:
  Bitstream();
  ~Bitstream();

  bool initialize(std::vector<uint8_t>& data);
  bool initialize(const Bitstream& bitstream);
  void initialize(uint64_t capacity) { data_.resize(capacity, 0); }

  bool load(const std::string& compressedStreamPath);
  bool save(const std::string& compressedStreamPath);

  void clear() {
    data_.clear();
    position_.bits_  = 0;
    position_.bytes_ = 0;
  }
  void beginning() {
    position_.bits_  = 0;
    position_.bytes_ = 0;
  }
  uint8_t*                    buffer() { return data_.data(); }
  std::vector<uint8_t>&       vector() { return data_; }
  const std::vector<uint8_t>& vector() const { return data_; }
  uint64_t&                   size() { return position_.bytes_; }
  uint64_t                    capacity() { return data_.size(); }
  BistreamPosition            getPosition() { return position_; }
  void       setPosition(BistreamPosition& val) { position_ = val; }
  Bitstream& operator+=(const uint64_t size) {
    position_.bytes_ += size;
    return *this;
  }
  void copyFrom(Bitstream&   dataBitstream,
                const size_t startByte,
                const size_t bitstreamSize);
  void copyTo(Bitstream& dataBitstream, const size_t size);
  void write(VideoBitstream& videoBitstream);
  void read(VideoBitstream& videoBitstream, size_t videoStreamSize);
  void read(std::vector<uint8_t>& buffer, size_t size);
  void write(const std::vector<uint8_t>& buffer);
  bool byteAligned() { return (position_.bits_ == 0); }
  bool moreData() { return position_.bytes_ < data_.size(); }
  void computeMD5();

  inline std::string readString() {
    while (!byteAligned()) { read(1); }
    std::string str;
    char        element = read(8);
    while (element != 0x00) {
      str.push_back(element);
      element = read(8);
    }
    return str;
  }

  inline void writeString(std::string str) {
    while (!byteAligned()) { write(0, 1); }
    for (auto& element : str) { write(element, 8); }
    write(0, 8);
  }

  inline uint32_t peekByteAt(uint64_t peekPos) { return data_[peekPos]; }
  inline uint32_t read(uint8_t bits) {
    uint32_t code = read(bits, position_);
    return code;
  }
  template<typename T>
  void write(T value, uint8_t bits) {
    write(static_cast<uint32_t>(value), bits, position_);
  }

  inline void writeS(int32_t value, uint8_t bits) {
    assert(bits > 0);
    uint32_t code;
    if (value >= 0) {
      code = (uint32_t)value;
    } else {
      code = (uint32_t)(value & ((1 << (bits - 1)) - 1));
      code |= (1 << (bits - 1));
    }
    write(code, bits);
  }

  inline int32_t readS(uint8_t bits) {
    assert(bits > 0);
    uint32_t code = read(bits);
    int32_t  value;
    uint32_t midPoint = (1 << (bits - 1));
    if (code < midPoint) {
      value = (int32_t)code;
    } else {
      value = (int32_t)code | ~(midPoint - 1);
    }
    return value;
  }

  template<typename T>
  inline void writeUvlc(T value) {
    uint32_t code   = static_cast<uint32_t>(value);
    uint32_t length = 1, temp = ++code;
    while (1 != temp) {
      temp >>= 1;
      length += 2;
    }
    write(0, length >> 1);
    write(code, (length + 1) >> 1);
  }

  inline uint32_t readUvlc() {
    uint32_t value = 0, code = 0, length = 0;
    code = read(1);
    if (0 == code) {
      length = 0;
      while (!(code & 1)) {
        code = read(1);
        length++;
      }
      value = read(length);
      value += (1 << length) - 1;
    }
    return value;
  }

  template<typename T>
  inline void writeSvlc(T value) {
    int32_t code = static_cast<int32_t>(value);
    writeUvlc((uint32_t)(code <= 0 ? -code << 1 : (code << 1) - 1));
  }

  inline int32_t readSvlc() {
    uint32_t bits = readUvlc();
    return (bits & 1) ? (int32_t)(bits >> 1) + 1 : -(int32_t)(bits >> 1);
  }

  inline void writeFloat(float value) {
    uint32_t code = 0;
    memcpy(&code, &value, sizeof(float));
    write(code, 32);
  }

  inline float readFloat() {
    uint32_t code  = read(32);
    float    value = 0;
    memcpy(&value, &code, sizeof(float));
    return value;
  }

#if defined(BITSTREAM_TRACE)
  void traceBitFloat(const std::string& name,
                     float              value,
                     const std::string& type) {
    if (trace_) {
      logger_->traceStream("[%6llu-%1u]", position_.bytes_, position_.bits_);
      logger_->traceStreamIndent();
      auto              str = name.substr(0, 50 - logger_->getIndent());
      static const char dots[] =
        "..................................................";
      auto len = (std::min)(logger_->getIndent() + str.length(), (size_t)50);
      logger_->traceStream(
        "%s %s = %-6f %s \n", str.c_str(), dots + len, value, type.c_str());
    }
  }
  void traceBitStr(const std::string& name,
                   const std::string& value,
                   const std::string& type) {
    if (trace_) {
      logger_->traceStream("[%6llu-%1u]", position_.bytes_, position_.bits_);
      logger_->traceStreamIndent();
      auto              str = name.substr(0, 50 - logger_->getIndent());
      static const char dots[] =
        "..................................................";
      auto len = (std::min)(logger_->getIndent() + str.length(), (size_t)50);
      logger_->traceStream("%s %s = %-6f %s \n",
                           str.c_str(),
                           dots + len,
                           value.c_str(),
                           type.c_str());
    }
  }
  template<typename T>
  void traceBit(const std::string& name, T value, const std::string& type) {
    if (trace_) {
      logger_->traceStream("[%6llu-%1u]", position_.bytes_, position_.bits_);
      logger_->traceStreamIndent();
      auto              str = name.substr(0, 50 - logger_->getIndent());
      static const char dots[] =
        "..................................................";
      auto len = (std::min)(logger_->getIndent() + str.length(), (size_t)50);
      logger_->traceStream("%s %s = %-6d %s \n",
                           str.c_str(),
                           dots + len,
                           (int)value,
                           type.c_str());
    }
  }

  template<typename... Args>
  void trace(const char* pFormat, Args... eArgs) {
    if (trace_) {
      logger_->traceStream("[%6llu-%1u]", position_.bytes_, position_.bits_);
      logger_->traceStreamIndent();
      logger_->traceStream(pFormat, eArgs...);
    }
  }
  template<typename... Args>
  void traceIn(const char* pFormat, Args... eArgs) {
    if (trace_) {
      trace(pFormat, eArgs...);
      logger_->traceStream("%s\n", "(){");
      logger_->indentIn();
    }
  }
  template<typename... Args>
  void traceOut(const char* pFormat, Args... eArgs) {
    if (trace_) {
      logger_->indentOut();
      trace("%s", "} //~");
      logger_->traceStream(pFormat, eArgs...);
      logger_->traceStream("%s\n", "");
    }
  }
#endif
#if defined(BITSTREAM_TRACE) || defined(CONFORMANCE_TRACE)
  void  setTrace(bool trace) { trace_ = trace; }
  bool  getTrace() { return trace_; }
  void  setLogger(Logger& logger) { logger_ = &logger; }
  auto& getLogger() { return *logger_; }
#endif
private:
  inline void realloc(const size_t size = 4096) {
    data_.resize(data_.size() + (((size / 4096) + 1) * 4096));
  }
  inline uint32_t read(uint8_t bits, BistreamPosition& pos) {
    uint32_t value = 0;
    for (size_t i = 0; i < bits; i++) {
      value |= ((data_[pos.bytes_] >> (7 - pos.bits_)) & 1) << (bits - 1 - i);
      if (pos.bits_ == 7) {
        pos.bytes_++;
        pos.bits_ = 0;
      } else {
        pos.bits_++;
      }
    }
    return value;
  }

  inline void write(uint32_t value, uint8_t bits, BistreamPosition& pos) {
    if (pos.bytes_ + bits + 64 >= data_.size()) { realloc(); }
    for (size_t i = 0; i < bits; i++) {
      data_[pos.bytes_] |= ((value >> (bits - 1 - i)) & 1) << (7 - pos.bits_);
      if (pos.bits_ == 7) {
        pos.bytes_++;
        pos.bits_ = 0;
      } else {
        pos.bits_++;
      }
    }
  }

  std::vector<uint8_t> data_;
  BistreamPosition     position_;

#if defined(CONFORMANCE_TRACE) || defined(BITSTREAM_TRACE)
  bool    trace_;
  Logger* logger_ = nullptr;
#endif
};

}  // namespace vmesh
