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
#include "v3cCommon.hpp"
#include "bitstream.hpp"
#include "v3cBitstream.hpp"
#include "atlasAdaptationParameterSetRbsp.hpp"
#include "rbsp.hpp"
#include "v3cWriter.hpp"
#include <list>

using namespace vmesh;

#if defined(BITSTREAM_TRACE)
static std::string
format(const std::string& var) {
  std::string substr = var;
  auto        npos   = substr.find("get");
  if (npos != std::string::npos) substr = substr.substr(npos + 3);
  npos = substr.find("(");
  if (npos != std::string::npos) substr = substr.substr(0, npos);
  return substr;
}

#  define WRITE_CODE(VAR, LEN) \
    { \
      bitstream.write(VAR, LEN); \
      bitstream.traceBit( \
        format(#VAR), VAR, "u(" + std::to_string(LEN) + ")"); \
    }

#  define WRITE_CODES(VAR, LEN) \
    { \
      bitstream.writeS(VAR, LEN); \
      bitstream.traceBit( \
        format(#VAR), VAR, "s(" + std::to_string(LEN) + ")"); \
    }

#  define WRITE_UVLC(VAR) \
    { \
      bitstream.writeUvlc(VAR); \
      bitstream.traceBit(format(#VAR), VAR, "ue(v)"); \
    }

#  define WRITE_SVLC(VAR) \
    { \
      bitstream.writeSvlc(VAR); \
      bitstream.traceBit(format(#VAR), VAR, "se(v)"); \
    }

#  define WRITE_FLOAT(VAR) \
    { \
      bitstream.writeFloat(VAR); \
      bitstream.traceBitFloat(format(#VAR), VAR, "f(32)"); \
    }
#  define WRITE_STRING(VAR) \
    { \
      bitstream.writeString(VAR); \
      bitstream.traceBitStr(format(#VAR), VAR, "f(32)"); \
    }

#  define WRITE_VECTOR(VAR) \
    { \
      bitstream.write(VAR); \
      bitstream.trace( \
        "Data stream: %s size = %zu \n", format(#VAR).c_str(), VAR.size()); \
    }

#  define WRITE_VIDEO(VAR) \
    { \
      bitstream.write(VAR); \
      bitstream.trace("Video stream: type = %s size = %zu \n", \
                      toString(VAR.type()).c_str(), \
                      VAR.size()); \
    }
#else
#  define WRITE_CODE(VAR, LEN) bitstream.write(VAR, LEN);
#  define WRITE_CODES(VAR, LEN) bitstream.writeS(VAR, LEN);
#  define WRITE_UVLC(VAR) bitstream.writeUvlc(VAR);
#  define WRITE_SVLC(VAR) bitstream.writeSvlc(VAR);
#  define WRITE_FLOAT(VAR) bitstream.writeFloat(VAR);
#  define WRITE_STRING(VAR) bitstream.writeString(VAR);
#  define WRITE_VECTOR(VAR) bitstream.write(VAR);
#  define WRITE_VIDEO(VAR) bitstream.write(VAR);
#endif

uint32_t
computePrecision(const std::vector<size_t>& naluSize) {
  auto     maxSize = *std::max_element(naluSize.begin(), naluSize.end());
  uint32_t precision =
    std::min(std::max(int(ceil(double(ceilLog2(maxSize + 1)) / 8.0)), 1), 8)
    - 1;
  return (uint32_t)precision;
}

V3CWriter::V3CWriter()  = default;
V3CWriter::~V3CWriter() = default;

size_t
V3CWriter::write(SampleStreamV3CUnit& ssvu,
                 Bitstream&           bitstream,
                 uint32_t             forcedSsvhUnitSizePrecisionBytes) {
  TRACE_BITSTREAM_IN("%s", "SampleStreamV3CUnits");
  size_t headerSize = 0;
  // Calculating the precision of the unit size
  uint32_t maxUnitSize = 0;
  for (auto& v3cUnit : ssvu.getV3CUnit()) {
    if (maxUnitSize < v3cUnit.getSize()) {
      maxUnitSize = static_cast<uint32_t>(v3cUnit.getSize());
    }
  }
  uint32_t precision = static_cast<uint32_t>(std::min(
    std::max(
      static_cast<int>(ceil(static_cast<double>(ceilLog2(maxUnitSize)) / 8.0)),
      1),
    8));
  precision          = (std::max)(precision, forcedSsvhUnitSizePrecisionBytes);
  ssvu.getSsvhUnitSizePrecisionBytesMinus1() = precision - 1;

  sampleStreamV3CHeader(bitstream, ssvu);
  headerSize += 1;
  size_t unitCount = 0;
  for (auto& v3cUnit : ssvu.getV3CUnit()) {
    sampleStreamV3CUnit(bitstream, ssvu, v3cUnit);
    unitCount++;
    headerSize += ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1;
  }
  TRACE_BITSTREAM_OUT("%s", "SampleStreamV3CUnits");
  return headerSize;
}

void
V3CWriter::addV3CUnit(V3cBitstream&        syntax,
                      SampleStreamV3CUnit& ssvu,
                      V3CUnitType          v3cUnitType) {
  auto& bistreamStat = syntax.getBitstreamStat();
  auto& nalu         = ssvu.addV3CUnit();
  auto& bitstream    = nalu.getBitstream();
  nalu.getType()     = v3cUnitType;
  v3cUnit(syntax, bitstream, v3cUnitType);
  nalu.getSize() = bitstream.size();
  bistreamStat.overwriteV3CUnitSize(v3cUnitType, bitstream.size());
}

int
V3CWriter::encode(V3cBitstream& syntax, SampleStreamV3CUnit& ssvu) {
  auto& bistreamStat            = syntax.getBitstreamStat();
  auto& vps                     = syntax.getVps();
  auto& atlas                   = syntax.getAtlas();
  auto& asps                    = atlas.getAtlasSequenceParameterSet(0);
  auto& vuhAD                   = syntax.getV3CUnitHeader(V3C_AD);
  auto& vuhMD                   = syntax.getV3CUnitHeader(V3C_BMD);
  auto& vuhGVD                  = syntax.getV3CUnitHeader(V3C_GVD);
  auto& vuhAVD                  = syntax.getV3CUnitHeader(V3C_AVD);
  auto  atlasCount              = vps.getAtlasCountMinus1() + 1;
  vuhAD.getV3CParameterSetId()  = vps.getV3CParameterSetId();
  vuhMD.getV3CParameterSetId()  = vps.getV3CParameterSetId();
  vuhGVD.getV3CParameterSetId() = vps.getV3CParameterSetId();
  vuhAVD.getV3CParameterSetId() = vps.getV3CParameterSetId();
  bistreamStat.newGOF();

  // Add V3C parameter set 
  addV3CUnit(syntax, ssvu, V3C_VPS);

  for (uint8_t atlasIdx = 0; atlasIdx < atlasCount; atlasIdx++) {
    vuhAD.getAtlasId()  = atlasIdx;
    vuhMD.getAtlasId()  = atlasIdx;
    vuhGVD.getAtlasId() = atlasIdx;
    vuhAVD.getAtlasId() = atlasIdx;

    // Add Atlas 
    addV3CUnit(syntax, ssvu, V3C_AD);
    
    // Add Base mesh 
    addV3CUnit(syntax, ssvu, V3C_BMD);

    // Add displacement video
    if (vps.getGeometryVideoPresentFlag(0)) {
      vuhGVD.getMapIndex()           = 0;
      vuhGVD.getAuxiliaryVideoFlag() = false;
      addV3CUnit(syntax, ssvu, V3C_GVD);
    }

    // Add attribute video
    if (vps.getAttributeVideoPresentFlag(0)) {
      auto& ai = vps.getAttributeInformation(atlasIdx);
      for (int attIdx = 0; attIdx < ai.getAttributeCount(); attIdx++) {
        vuhAVD.getAttributeIndex() = attIdx;
        auto dimension = ai.getAttributeDimensionPartitionsMinus1(attIdx) + 1;
        for (int attDim = 0; attDim < dimension; attDim++) {
          vuhAVD.getAttributeDimensionIndex() = attDim;
          vuhAVD.getAttributeIndex()          = attIdx;
          vuhAVD.getAttributeDimensionIndex() = attDim;
          vuhAVD.getMapIndex()                = 0;
          vuhAVD.getAuxiliaryVideoFlag()      = false;
          addV3CUnit(syntax, ssvu, V3C_AVD);
        }
      }
    }
  }
  return 0;
}

void
V3CWriter::videoSubStream(V3cBitstream& syntax,
                          Bitstream&    bitstream,
                          V3CUnitType   v3cUnitType) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&           bistreamStat = syntax.getBitstreamStat();
  VideoBitstream& video        = syntax.getVideoBitstream(v3cUnitType);
  WRITE_VIDEO(video);
  bistreamStat.setVideo(video);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.2 V3C unit syntax
// 8.3.2.1 General V3C unit syntax
void
V3CWriter::v3cUnit(V3cBitstream& syntax,
                   Bitstream&    bitstream,
                   V3CUnitType   v3cUnitType) {
#if defined(BITSTREAM_TRACE)
  bitstream.setTrace(true);
  bitstream.setLogger(*logger_);
#endif
  TRACE_BITSTREAM_IN("%s(%s)", __func__, toString(v3cUnitType).c_str());
  v3cUnitHeader(syntax, bitstream, v3cUnitType);
  v3cUnitPayload(syntax, bitstream, v3cUnitType);
  TRACE_BITSTREAM_OUT("%s(%s)", __func__, toString(v3cUnitType).c_str());
}

// 8.3.2.2 V3C unit header syntax
void
V3CWriter::v3cUnitHeader(V3cBitstream& syntax,
                         Bitstream&    bitstream,
                         V3CUnitType   v3cUnitType) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  WRITE_CODE(v3cUnitType, 5);  // u(5)
  if (v3cUnitType == V3C_AVD || v3cUnitType == V3C_GVD
      || v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD
      || v3cUnitType == V3C_BMD) {
    auto& vpcc = syntax.getV3CUnitHeader(v3cUnitType);
    WRITE_CODE(vpcc.getV3CParameterSetId(), 4);  // u(4)
    WRITE_CODE(vpcc.getAtlasId(), 6);            // u(6)
  }
  if (v3cUnitType == V3C_AVD) {
    auto& vpcc = syntax.getV3CUnitHeader(v3cUnitType);
    WRITE_CODE(vpcc.getAttributeIndex(), 7);           // u(7)
    WRITE_CODE(vpcc.getAttributeDimensionIndex(), 5);  // u(5)
    WRITE_CODE(vpcc.getMapIndex(), 4);                 // u(4)
    WRITE_CODE(vpcc.getAuxiliaryVideoFlag(), 1);       // u(1)
  } else if (v3cUnitType == V3C_GVD) {
    auto& vpcc = syntax.getV3CUnitHeader(v3cUnitType);
    WRITE_CODE(vpcc.getMapIndex(), 4);            // u(4)
    WRITE_CODE(vpcc.getAuxiliaryVideoFlag(), 1);  // u(1)
    WRITE_CODE(zero, 12);                         // u(12)
  } else if (v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD
             || v3cUnitType == V3C_BMD) {
    WRITE_CODE(zero, 17);  // u(17)
  } else {
    WRITE_CODE(zero, 27);  // u(27)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.2.3 V3C unit payload syntax
void
V3CWriter::v3cUnitPayload(V3cBitstream& syntax,
                          Bitstream&    bitstream,
                          V3CUnitType   v3cUnitType) {
  TRACE_BITSTREAM_IN("%s(%s)", __func__, toString(v3cUnitType).c_str());
  if (v3cUnitType == V3C_VPS) {
    v3cParameterSet(syntax.getVps(), syntax, bitstream);
  } else if (v3cUnitType == V3C_AD) {
    atlasSubStream(syntax, bitstream);
  } else if (v3cUnitType == V3C_BMD) {
    baseMeshSubStream(syntax, bitstream);
  } else if (v3cUnitType == V3C_OVD || v3cUnitType == V3C_GVD
             || v3cUnitType == V3C_AVD) {
    videoSubStream(syntax, bitstream, v3cUnitType);
  }
  TRACE_BITSTREAM_OUT("%s(%s)", __func__, toString(v3cUnitType).c_str());
}

// Base mesh sub-bitstream syntax
void
V3CWriter::baseMeshSubStream(V3cBitstream& syntax, Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  SampleStreamNalUnit ssnu;
  std::vector<size_t> naluSize;
  auto&               baseMesh      = syntax.getBaseMesh();
  const size_t        nalHeaderSize = 2;
  for (auto& bmsps : baseMesh.getBaseMeshSequenceParameterSetList()) {
    Bitstream tempBitStream;
    baseMeshSequenceParameterSetRbsp(bmsps, syntax, tempBitStream);
    naluSize.push_back(tempBitStream.size() + nalHeaderSize);
  }
  for (auto& bmfps : baseMesh.getBaseMeshFrameParameterSetList()) {
    Bitstream tempBitStream;
    baseMeshFrameParameterSetRbsp(bmfps, syntax, tempBitStream);
    naluSize.push_back(tempBitStream.size() + nalHeaderSize);
  }
  for (auto& bmfl : baseMesh.getBaseMeshTileLayerList()) {
    Bitstream tempBitStream;
    auto&     bmth     = bmfl.getHeader();
    auto      naluType = bmth.getBaseMeshType() == I_BASEMESH
                           ? BASEMESH_NAL_IDR_N_LP
                           : BASEMESH_NAL_TRAIL_N;
    baseMeshTileLayerRbsp(bmfl, naluType, syntax, tempBitStream);
    naluSize.push_back(tempBitStream.size() + nalHeaderSize);
  }

  // Calculation of the max unit size done
  ssnu.getSizePrecisionBytesMinus1() = computePrecision(naluSize);
  sampleStreamNalHeader(bitstream, ssnu);

  // Write sample streams
  size_t index     = 0;
  size_t sizeIndex = 0;
  syntax.getBitstreamStat().resetBaseMeshBinSize();
  
  for (auto& bmsps : baseMesh.getBaseMeshSequenceParameterSetList()) {
    NalUnit nu(BASEMESH_NAL_BMSPS, 0, 1, naluSize[sizeIndex++]);
    sampleStreamNalUnit(syntax, false, bitstream, ssnu, nu, index++);
  }
  index = 0;
  for (auto& bmsps : baseMesh.getBaseMeshFrameParameterSetList()) {
    NalUnit nu(BASEMESH_NAL_BMFPS, 0, 1, naluSize[sizeIndex++]);
    sampleStreamNalUnit(syntax, false, bitstream, ssnu, nu, index++);
  }
  index = 0;
  for (auto& bmfl : baseMesh.getBaseMeshTileLayerList()) {
    auto&   bmth     = bmfl.getHeader();
    auto    naluType = bmth.getBaseMeshType() == I_BASEMESH
                         ? BASEMESH_NAL_IDR_N_LP
                         : BASEMESH_NAL_TRAIL_N;
    NalUnit nu(naluType, 0, 1, naluSize[sizeIndex++]);
    sampleStreamNalUnit(syntax, false, bitstream, ssnu, nu, index++);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh sequence parameter set Rbsp
void
V3CWriter::baseMeshSequenceParameterSetRbsp(
  BaseMeshSequenceParameterSetRbsp& bmsps,
  V3cBitstream&                     syntax,
  Bitstream&                        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_UVLC(bmsps.getBaseMeshSequenceParameterSetId());         // ue(v)
  WRITE_UVLC(bmsps.getBaseMeshCodecId());                        // ue(v)
  WRITE_UVLC(bmsps.getQpPositionMinus1());                       // ue(v)
  WRITE_UVLC(bmsps.getQpTexCoordMinus1());                       // ue(v)
  WRITE_UVLC(bmsps.getLog2MaxBaseMeshFrameOrderCntLsbMinus4());  // ue(v)
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh frame parameter set Rbsp
void
V3CWriter::baseMeshFrameParameterSetRbsp(BaseMeshFrameParameterSetRbsp& bmfps,
                                         V3cBitstream&                  syntax,
                                         Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_UVLC(bmfps.getBaseMeshFrameParameterSetId());     // ue(v)
  WRITE_UVLC(bmfps.getBaseMeshSequenceParameterSetId());  // ue(v)
  auto& baseMesh = syntax.getBaseMesh();
  auto& bmspsId  = bmfps.getBaseMeshSequenceParameterSetId();
  auto& bmsps    = baseMesh.getBaseMeshSequenceParameterSet(bmspsId);
  auto& bmfti    = bmfps.getBaseMeshFrameTileInformation();
  baseMeshFrameTileInformation(bmfti, bmsps, bitstream);
  WRITE_CODE(bmfps.getOutputFlagPresentFlag(), 1);      // u(1)
  WRITE_UVLC(bmfps.getNumRefIdxDefaultActiveMinus1());  // ue(v)
  WRITE_UVLC(bmfps.getAdditionalLtAfocLsbLen());        // ue(v)
  WRITE_CODE(bmfps.getMotionGroupSize(), 8);            // u(8)
  WRITE_CODE(bmfps.getExtensionFlag(), 1);              // u(1)
  if (bmfps.getExtensionFlag()) {
    WRITE_CODE(bmfps.getExtension8Bits(), 8);  // u(8)
  }
  if (bmfps.getExtension8Bits()) {
    while (moreRbspData(bitstream)) { WRITE_CODE(0, 1); }  // u(1)
  }
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh frame tile information syntax
void
V3CWriter::baseMeshFrameTileInformation(
  BaseMeshFrameTileInformation&     bmfti,
  BaseMeshSequenceParameterSetRbsp& bmsps,
  Bitstream&                        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(bmfti.getSingleTileInBaseMeshFrameFlag(), 1);  // u(1)
  if (!bmfti.getSingleTileInBaseMeshFrameFlag()) {
    // TODO
  }
  WRITE_CODE(bmfti.getSignalledTileIdFlag(), 1);  // u(1)
  if (bmfti.getSignalledTileIdFlag()) {
    WRITE_UVLC(bmfti.getSignalledTileIdLengthMinus1());  // ue(v)
    for (size_t i = 0; i <= bmfti.getNumTilesInBaseMeshFrameMinus1(); i++) {
      uint8_t bitCount = bmfti.getSignalledTileIdLengthMinus1() + 1;
      WRITE_CODE(bmfti.getTileId(i), bitCount);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh tile layer
void
V3CWriter::baseMeshTileLayerRbsp(BaseMeshTileLayer&  bmtl,
                                 BaseMeshNalUnitType nalUnitType,
                                 V3cBitstream&       syntax,
                                 Bitstream&          bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& bmth  = bmtl.getHeader();
  auto& bmtdu = bmtl.getDataUnit();
  baseMeshTileHeader(bmth, nalUnitType, syntax, bitstream);
  baseMeshTileDataUnit(bmtdu, bmth, syntax, bitstream);
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh tile group header syntax
void
V3CWriter::baseMeshTileHeader(BaseMeshTileHeader& bmth,
                              BaseMeshNalUnitType nalUnitType,
                              V3cBitstream&       syntax,
                              Bitstream&          bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&  baseMesh = syntax.getBaseMesh();
  size_t bmfpsId  = bmth.getBaseMeshFrameParameterSetId();
  auto&  bmfps    = baseMesh.getBaseMeshFrameParameterSet(bmfpsId);
  size_t bmspsId  = bmfps.getBaseMeshSequenceParameterSetId();
  auto&  bmsps    = baseMesh.getBaseMeshSequenceParameterSet(bmspsId);
  auto&  bmfti    = bmfps.getBaseMeshFrameTileInformation();
  if (nalUnitType >= BASEMESH_NAL_BLA_W_LP
      && nalUnitType <= BASEMESH_NAL_RSV_BMCL_29) {
    WRITE_CODE(bmth.getNoOutputOfPriorBaseMeshFramesFlag(), 1);  // u(1)
  }
  WRITE_UVLC(bmth.getBaseMeshFrameParameterSetId());       // ue(v)
  WRITE_UVLC(bmth.getBaseMeshAdaptationParameterSetId());  // ue(v)
  if (bmfti.getSignalledTileIdFlag()) {
    WRITE_CODE(uint32_t(bmth.getBaseMeshId()),
               bmfti.getSignalledTileIdLengthMinus1() + 1);  // u(v)
  } else {
    if (bmfti.getNumTilesInBaseMeshFrameMinus1() != 0) {
      WRITE_CODE(
        bmth.getBaseMeshId(),
        ceilLog2(bmfti.getNumTilesInBaseMeshFrameMinus1() + 1));  // u(v)
    }
  }
  WRITE_UVLC(bmth.getBaseMeshType());  // ue(v)
  if (bmfps.getOutputFlagPresentFlag()) {
    WRITE_CODE(bmth.getBaseMeshOutputFlag(), 1);
  }
  WRITE_CODE(bmth.getBaseMeshFrmOrderCntLsb(),
             bmsps.getLog2MaxBaseMeshFrameOrderCntLsbMinus4() + 4);  // u(v)
  if (bmsps.getNumRefBaseMeshFrameListsInAsps() > 0) {
    WRITE_CODE(bmth.getRefBaseMeshFrameListSpsFlag(), 1);  // u(1)
  }
  if (static_cast<int>(bmth.getRefBaseMeshFrameListSpsFlag()) == 0) {
    refListStruct(bmth.getRefListStruct(), bmsps, bitstream);
  } else if (bmsps.getNumRefBaseMeshFrameListsInAsps() > 1) {
    size_t bitCount = ceilLog2(bmsps.getNumRefBaseMeshFrameListsInAsps());
    WRITE_CODE(bmth.getRefBaseMeshFrameListIdx(), bitCount);  // u(v)
  }
  uint8_t rlsIdx                = bmth.getRefBaseMeshFrameListIdx();
  auto&   refList               = bmth.getRefBaseMeshFrameListSpsFlag()
                                    ? bmsps.getRefListStruct(rlsIdx)
                                    : bmth.getRefListStruct();
  size_t  numLtrAtlasFrmEntries = 0;
  for (size_t i = 0; i < refList.getNumRefEntries(); i++) {
    if (!refList.getStRefAtlasFrameFlag(i)) { numLtrAtlasFrmEntries++; }
  }
  for (size_t j = 0; j < numLtrAtlasFrmEntries; j++) {
    WRITE_CODE(bmth.getAdditionalAfocLsbPresentFlag(j), 1);  // u(1)
    if (bmth.getAdditionalAfocLsbPresentFlag(j)) {
      uint8_t bitCount = bmfps.getAdditionalLtAfocLsbLen();
      WRITE_CODE(bmth.getAdditionalAfocLsbVal(j), bitCount);  // u(v)
    }
  }
  if (bmth.getBaseMeshType() == P_BASEMESH) {
    if (refList.getNumRefEntries() > 1) {
      WRITE_CODE(bmth.getNumRefIdxActiveOverrideFlag(), 1);  // u(1)
      if (bmth.getNumRefIdxActiveOverrideFlag()) {
        WRITE_UVLC(bmth.getNumRefIdxActiveMinus1());  // ue(v)
      }
    }
  }
  byteAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Reference list structure syntax
void
V3CWriter::refListStruct(RefListStruct&                    rls,
                         BaseMeshSequenceParameterSetRbsp& bmsps,
                         Bitstream&                        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_UVLC(rls.getNumRefEntries());  // ue(v)
  for (size_t i = 0; i < rls.getNumRefEntries(); i++) {
    if (bmsps.getLongTermRefBaseMeshFramesFlag()) {
      WRITE_CODE(rls.getStRefAtlasFrameFlag(i), 1);  // u(1)
    }
    if (rls.getStRefAtlasFrameFlag(i)) {
      WRITE_UVLC(rls.getAbsDeltaAfocSt(i));  // ue(v)
      if (rls.getAbsDeltaAfocSt(i) > 0) {
        WRITE_CODE(rls.getStrafEntrySignFlag(i), 1);  // u(1)
      }
    } else {
      uint8_t bitCount = bmsps.getLog2MaxBaseMeshFrameOrderCntLsbMinus4() + 4;
      WRITE_CODE(rls.getAfocLsbLt(i), bitCount);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// General base mesh tile group data unit syntax =patchTileDataUnit
void
V3CWriter::baseMeshTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                BaseMeshTileHeader&   bmth,
                                V3cBitstream&         syntax,
                                Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  if (bmth.getBaseMeshType() == I_BASEMESH) {
    baseMeshIntraTileDataUnit(bmtdu, bmth, syntax, bitstream);
  } else if (bmth.getBaseMeshType() == P_BASEMESH) {
    baseMeshInterTileDataUnit(bmtdu, bmth, syntax, bitstream);
  } else if (bmth.getBaseMeshType() == SKIP_BASEMESH) {
    baseMeshSkipTileDataUnit(bmtdu, bmth, syntax, bitstream);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// base mesh intra tile data unit
void
V3CWriter::baseMeshIntraTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                     BaseMeshTileHeader&   bmth,
                                     V3cBitstream&         syntax,
                                     Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& bistreamStat = syntax.getBitstreamStat();
  WRITE_VECTOR(bmtdu.getData().vector());
  bistreamStat.setBaseMesh(I_BASEMESH, bmtdu.getData().vector().size());
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// base mesh inter tile data unit
void
V3CWriter::baseMeshInterTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                     BaseMeshTileHeader&   bmth,
                                     V3cBitstream&         syntax,
                                     Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& bistreamStat = syntax.getBitstreamStat();
  WRITE_CODE(bmtdu.getMotionSkipFlag(), 1);  //u1
  if (bmtdu.getMotionSkipFlag()) {
    WRITE_CODE(bmtdu.getMotionSkipAll(), 1);  //u1
    if (!bmtdu.getMotionSkipAll()) {
      WRITE_CODE(bmtdu.getMotionSkipCount(), 7);  //u7
      for (size_t i = 0; i < bmtdu.getMotionSkipCount(); i++)
        WRITE_CODE(bmtdu.getMotionSkipVextexIndices(i), 7);  //u7
    }
  }
  lengthAlignment(bitstream);
  WRITE_VECTOR(bmtdu.getData().vector());
  bistreamStat.setBaseMesh(P_BASEMESH, bmtdu.getData().vector().size());
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// base mesh tile data unit
void
V3CWriter::baseMeshSkipTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                    BaseMeshTileHeader&   bmth,
                                    V3cBitstream&         syntax,
                                    Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// V3C VPS extension
void
V3CWriter::vpsExtension(V3CParameterSet& vps,
                        uint8_t          index,
                        Bitstream&       bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto type = V3CExtensionType(vps.getExtensionType(index));
  if (type == VPS_EXT_PACKED) {
    // Packed
  } else if (type == VPS_EXT_MIV) {
    // Miv
  } else if (type == VPS_EXT_VDMC) {
    vpsVdmcExtension(vps, vps.getVpsVdmcExtension(), bitstream);
  } else {
    for (size_t i = 0; i < vps.getExtensionLength(index); i++)
      WRITE_CODE(vps.getExtensionDataByte(index, i), 8);  // u(8)
  }
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// V3C VPS VDMC extension
void
V3CWriter::vpsVdmcExtension(V3CParameterSet&  vps,
                            VpsVdmcExtension& ext,
                            Bitstream&        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// ASPS VDMC extension syntax
void
V3CWriter::aspsVdmcExtension(Bitstream&                     bitstream,
                             AtlasSequenceParameterSetRbsp& asps,
                             AspsVdmcExtension&             ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(ext.getSubdivisionIterationCount(), 4);       //u4
  WRITE_CODE(ext.getLiftingQPs(0), 8);                     //u8
  WRITE_CODE(ext.getLiftingQPs(1), 8);                     //u8
  WRITE_CODE(ext.getLiftingQPs(2), 8);                     //u8
  WRITE_CODE(ext.getInterpolateDisplacementNormals(), 1);  //u1
  WRITE_CODE(ext.getAddReconstructedNormals(), 1);         //u1
  WRITE_CODE(ext.getDisplacementReversePacking(), 1);      //u1

  // Video streams
  // if (vps.getGeometryVideoPresentFlag(0)) {
  // JR Note: will be move
  WRITE_CODE(ext.getWidthDispVideo(), 16);   //u16
  WRITE_CODE(ext.getHeightDispVideo(), 16);  //u16
  // }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// AFPS V-DMC extension syntax
void
V3CWriter::afpsVdmcExtension(Bitstream& bitstream, AfpsVdmcExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.2.1	AAPS V-PCC extension syntax
void
V3CWriter::aapsVdmcExtension(Bitstream& bitstream, AapsVdmcExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.2.4 Atlas sub-bitstream syntax
void
V3CWriter::atlasSubStream(V3cBitstream& syntax, Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  SampleStreamNalUnit ssnu;
  std::vector<size_t> naluSize;
  auto&               atlas         = syntax.getAtlas();
  size_t              nalHeaderSize = 2;

  // Atlas sequence parameter set
  for (auto& asps : atlas.getAtlasSequenceParameterSetList()) {
    Bitstream tempBitStream;
    atlasSequenceParameterSetRbsp(asps, syntax, tempBitStream);
    naluSize.push_back(tempBitStream.size() + nalHeaderSize);
  }
  // Atlas frame parameter set
  for (auto& afps : atlas.getAtlasFrameParameterSetList()) {
    Bitstream tempBitStream;
    atlasFrameParameterSetRbsp(afps, syntax, tempBitStream);
    naluSize.push_back(tempBitStream.size() + nalHeaderSize);
  }
  for (auto& atgl : atlas.getAtlasTileLayerList()) {
    // Prefix sei message
    for (auto& sei : atgl.getSEI().getSeiPrefix()) {
      Bitstream tempBitStream;
      seiRbsp(syntax,
              tempBitStream,
              ATLAS_NAL_PREFIX_ESEI,
              *sei,
              atgl.getEncFrameIndex());
      naluSize.push_back(tempBitStream.size() + nalHeaderSize);
    }

    // Atlas tile layer data
    Bitstream tempBitStream;
    atlasTileLayerRbsp(atgl, syntax, ATLAS_NAL_SKIP_R, tempBitStream);
    naluSize.push_back(tempBitStream.size() + nalHeaderSize);

    // Suffix sei message
    for (auto& sei : atgl.getSEI().getSeiSuffix()) {
      for (size_t i = 0; i < atgl.getSEI().getSeiSuffix().size(); i++) {
        Bitstream tempBitStream;
        seiRbsp(syntax,
                tempBitStream,
                ATLAS_NAL_SUFFIX_ESEI,
                *sei,
                atgl.getEncFrameIndex());
        naluSize.push_back(tempBitStream.size() + nalHeaderSize);
      }
    }
  }
  ssnu.getSizePrecisionBytesMinus1() = computePrecision(naluSize);
  sampleStreamNalHeader(bitstream, ssnu);

  // Atlas sequence parameter set
  size_t index     = 0;
  size_t sizeIndex = 0;
  for (auto& asps : atlas.getAtlasSequenceParameterSetList()) {
    NalUnit nu(ATLAS_NAL_ASPS, 0, 1, naluSize[sizeIndex++]);
    sampleStreamNalUnit(syntax, true, bitstream, ssnu, nu, index);
    index++;
  }
  // Atlas frame parameter set
  index = 0;
  for (auto& afps : atlas.getAtlasFrameParameterSetList()) {
    NalUnit nu(ATLAS_NAL_AFPS, 0, 1, naluSize[sizeIndex++]);
    sampleStreamNalUnit(syntax, true, bitstream, ssnu, nu, index);
    index++;
  }

  // Atlas tile layers
  index = 0;
  for (auto& atgl : atlas.getAtlasTileLayerList()) {
    // NAL_PREFIX_SEI
    for (size_t i = 0; i < atgl.getSEI().getSeiPrefix().size(); i++) {
      NalUnit nu(ATLAS_NAL_PREFIX_ESEI, 0, 1, naluSize[sizeIndex++]);
      sampleStreamNalUnit(syntax, true, bitstream, ssnu, nu, i, index);
    }

    // Atlas_tile_layer_rbsp
    auto&            atgh     = atgl.getHeader();
    AtlasNalUnitType naluType = ATLAS_NAL_IDR_N_LP;
    if (atgh.getTileNaluTypeInfo() == 1) {
      naluType = ATLAS_NAL_TRAIL_R;
    } else if (atgh.getTileNaluTypeInfo() == 2) {
      naluType = ATLAS_NAL_TRAIL_N;
    }
    NalUnit nu(naluType, 0, 1, naluSize[sizeIndex++]);
    atgl.getDataUnit().getTileOrder() = index;
    sampleStreamNalUnit(syntax, true, bitstream, ssnu, nu, index);

    // NAL_SUFFIX_SEI
    for (size_t i = 0; i < atgl.getSEI().getSeiSuffix().size(); i++) {
      NalUnit nu(ATLAS_NAL_SUFFIX_ESEI, 0, 1, naluSize[sizeIndex++]);
      sampleStreamNalUnit(syntax, true, bitstream, ssnu, nu, i, index);
    }
    index++;
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.3 Byte alignment syntax
void
V3CWriter::byteAlignment(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t one  = 1;
  uint32_t zero = 0;
  WRITE_CODE(one, 1);  // f(1): equal to 1
  while (!bitstream.byteAligned()) {
    WRITE_CODE(zero, 1);  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4 Length alignment syntax
void
V3CWriter::lengthAlignment(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  while (!bitstream.byteAligned()) {
    WRITE_CODE(zero, 1);  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4 V3C parameter set syntax
// 8.3.4.1 General V3C parameter set syntax
void
V3CWriter::v3cParameterSet(V3CParameterSet& vps,
                           V3cBitstream&    syntax,
                           Bitstream&       bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  profileTierLevel(vps.getProfileTierLevel(), bitstream);
  WRITE_CODE(vps.getV3CParameterSetId(), 4);  // u(4)
  WRITE_CODE(zero, 8);                        // u(8)
  WRITE_CODE(vps.getAtlasCountMinus1(), 6);   // u(6)
  for (uint32_t j = 0; j < vps.getAtlasCountMinus1() + 1; j++) {
    WRITE_CODE(vps.getAtlasId(j), 6);         // u(6)
    WRITE_UVLC(vps.getFrameWidth(j));         // ue(v)
    WRITE_UVLC(vps.getFrameHeight(j));        // ue(v)
    WRITE_CODE(vps.getMapCountMinus1(j), 4);  // u(4)
    if (vps.getMapCountMinus1(j) > 0) {
      WRITE_CODE(vps.getMultipleMapStreamsPresentFlag(j), 1);  // u(1)
    }
    for (size_t i = 1; i <= vps.getMapCountMinus1(j); i++) {
      if (vps.getMultipleMapStreamsPresentFlag(j)) {
        WRITE_CODE(vps.getMapAbsoluteCodingEnableFlag(j, i), 1);  // u(1)
      }
      if (static_cast<int>(vps.getMapAbsoluteCodingEnableFlag(j, i)) == 0) {
        WRITE_UVLC(vps.getMapPredictorIndexDiff(j, i));
      }
    }
    WRITE_CODE(vps.getAuxiliaryVideoPresentFlag(j), 1);  // u(1)
    WRITE_CODE(vps.getOccupancyVideoPresentFlag(j), 1);  // u(1)
    WRITE_CODE(vps.getGeometryVideoPresentFlag(j), 1);   // u(1)
    WRITE_CODE(vps.getAttributeVideoPresentFlag(j), 1);  // u(1)
    if (vps.getOccupancyVideoPresentFlag(j)) {
      occupancyInformation(vps.getOccupancyInformation(j), bitstream);
    }
    if (vps.getGeometryVideoPresentFlag(j)) {
      geometryInformation(vps.getGeometryInformation(j), vps, bitstream);
    }
    if (vps.getAttributeVideoPresentFlag(j)) {
      attributeInformation(vps.getAttributeInformation(j), vps, bitstream);
    }
  }
  WRITE_CODE(vps.getExtensionPresentFlag(), 1);  // u(1)
  if (vps.getExtensionPresentFlag()) {
    WRITE_CODE(vps.getExtensionCount(), 8);  // u(8)
  }
  if (vps.getExtensionCount()) {
    WRITE_UVLC(vps.getExtensionLengthMinus1());  // ue(v)
    for (size_t i = 0; i < vps.getExtensionCount(); i++) {
      WRITE_CODE(vps.getExtensionType(i), 8);     // u(8)
      WRITE_CODE(vps.getExtensionLength(i), 16);  // u(16)
      vpsExtension(vps, i, bitstream);
    }
  }
  byteAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.2 Profile, tier, and level syntax
void
V3CWriter::profileTierLevel(ProfileTierLevel& ptl, Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  uint32_t ffff = 0xFFFF;
  WRITE_CODE(ptl.getTierFlag(), 1);                  // u(1)
  WRITE_CODE(ptl.getProfileCodecGroupIdc(), 7);      // u(7)
  WRITE_CODE(ptl.getProfileToolsetIdc(), 8);         // u(8)
  WRITE_CODE(ptl.getProfileReconstructionIdc(), 8);  // u(8)
  WRITE_CODE(zero, 16);                              // u(16)
  WRITE_CODE(ffff, 16);                              // u(16)
  WRITE_CODE(ptl.getLevelIdc(), 8);                  // u(8)
  WRITE_CODE(ptl.getNumSubProfiles(), 6);            // u(6)
  WRITE_CODE(ptl.getExtendedSubProfileFlag(), 1);    // u(1)
  for (size_t i = 0; i < ptl.getNumSubProfiles(); i++) {
    size_t v = ptl.getExtendedSubProfileFlag() == 0 ? 32 : 64;
    WRITE_CODE(ptl.getSubProfileIdc(i), v);  // u(v)
  }
  WRITE_CODE(ptl.getToolConstraintsPresentFlag(), 1);  // u(1)
  if (ptl.getToolConstraintsPresentFlag()) {
    profileToolsetConstraintsInformation(
      ptl.getProfileToolsetConstraintsInformation(), bitstream);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.3 Occupancy parameter set syntax
void
V3CWriter::occupancyInformation(OccupancyInformation& oi,
                                Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(oi.getOccupancyCodecId(), 8);                    // u(8)
  WRITE_CODE(oi.getLossyOccupancyCompressionThreshold(), 8);  // u(8)
  WRITE_CODE(oi.getOccupancy2DBitdepthMinus1(), 5);           // u(5)
  WRITE_CODE(oi.getOccupancyMSBAlignFlag(), 1);               // u(1)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.4 Geometry parameter set syntax
void
V3CWriter::geometryInformation(GeometryInformation& gi,
                               V3CParameterSet&     vps,
                               Bitstream&           bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  size_t atlasIndex = 0;
  WRITE_CODE(gi.getGeometryCodecId(), 8);                      // u(8)
  WRITE_CODE(gi.getGeometry2dBitdepthMinus1(), 5);             // u(5)
  WRITE_CODE(gi.getGeometryMSBAlignFlag(), 1);                 // u(1)
  WRITE_CODE(gi.getGeometry3dCoordinatesBitdepthMinus1(), 5);  // u(5)
  if (vps.getAuxiliaryVideoPresentFlag(atlasIndex)) {
    WRITE_CODE(gi.getAuxiliaryGeometryCodecId(), 8);  // u(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.5 Attribute information
void
V3CWriter::attributeInformation(AttributeInformation& ai,
                                V3CParameterSet&      vps,
                                Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  size_t atlasIndex = 0;
  WRITE_CODE(ai.getAttributeCount(), 7);  // u(7)
  for (uint32_t i = 0; i < ai.getAttributeCount(); i++) {
    WRITE_CODE(ai.getAttributeTypeId(i), 4);   // u(4)
    WRITE_CODE(ai.getAttributeCodecId(i), 8);  // u(8)
    if (vps.getAuxiliaryVideoPresentFlag(atlasIndex)) {
      WRITE_CODE(ai.getAuxiliaryAttributeCodecId(i), 8);  // u(8)
    }
    if (vps.getMapCountMinus1(atlasIndex) > 0) {
      WRITE_CODE(ai.getAttributeMapAbsoluteCodingPersistenceFlag(i),
                 1);  // u(1)
    }
    WRITE_CODE(ai.getAttributeDimensionMinus1(i), 6);  // u(6)
    if (ai.getAttributeDimensionMinus1(i) > 0) {
      WRITE_CODE(ai.getAttributeDimensionPartitionsMinus1(i), 6);  // u(6)
      int32_t remainingDimensions = ai.getAttributeDimensionMinus1(i);
      int32_t k = ai.getAttributeDimensionPartitionsMinus1(i);
      for (int32_t j = 0; j < k; j++) {
        if (k - j != remainingDimensions) {
          WRITE_UVLC(static_cast<uint32_t>(
            ai.getAttributePartitionChannelsMinus1(i, j)));  // ue(v)
        }
        remainingDimensions -=
          ai.getAttributePartitionChannelsMinus1(i, j) + 1;
      }
    }
    WRITE_CODE(ai.getAttribute2dBitdepthMinus1(i), 5);  // u(5)
    WRITE_CODE(ai.getAttributeMSBAlignFlag(i), 1);      // u(1)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.6	Profile toolset constraints information syntax
void
V3CWriter::profileToolsetConstraintsInformation(
  ProfileToolsetConstraintsInformation& ptci,
  Bitstream&                            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(ptci.getOneFrameOnlyFlag(), 1);                       // u(1)
  WRITE_CODE(ptci.getEOMContraintFlag(), 1);                       // u(1)
  WRITE_CODE(ptci.getMaxMapCountMinus1(), 4);                      // u(4)
  WRITE_CODE(ptci.getMaxAtlasCountMinus1(), 4);                    // u(4)
  WRITE_CODE(ptci.getMultipleMapStreamsConstraintFlag(), 1);       // u(1)
  WRITE_CODE(ptci.getPLRConstraintFlag(), 1);                      // u(1)
  WRITE_CODE(ptci.getAttributeMaxDimensionMinus1(), 6);            // u(6)
  WRITE_CODE(ptci.getAttributeMaxDimensionPartitionsMinus1(), 6);  // u(6)
  WRITE_CODE(ptci.getNoEightOrientationsConstraintFlag(), 1);      // u(1)
  WRITE_CODE(ptci.getNo45DegreeProjectionPatchConstraintFlag(),
             1);                                        // u(1)
  WRITE_CODE(0, 6);                                     // u(6)
  WRITE_CODE(ptci.getNumReservedConstraintBytes(), 8);  // u(8)
  for (size_t i = 0; i < ptci.getNumReservedConstraintBytes(); i++) {
    WRITE_CODE(ptci.getReservedConstraintByte(i), 8);  // u(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.5 NAL unit syntax
// 8.3.5.1 General NAL unit syntax
void
V3CWriter::nalUnit(Bitstream& bitstream, NalUnit& nalUnit) {
  TRACE_BITSTREAM_IN("%s", __func__);
  nalUnitHeader(bitstream, nalUnit);
  for (size_t i = 2; i < nalUnit.getSize(); i++) {
    WRITE_CODE(nalUnit.getData(i), 8);  // b(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.5.2 NAL unit header syntax
void
V3CWriter::nalUnitHeader(Bitstream& bitstream, NalUnit& nalUnit) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  WRITE_CODE(zero, 1);                           // f(1)
  WRITE_CODE(nalUnit.getType(), 6);              // u(6)
  WRITE_CODE(nalUnit.getLayerId(), 6);           // u(6)
  WRITE_CODE(nalUnit.getTemporalyIdPlus1(), 3);  // u(3)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.1 Atlas sequence parameter set Rbsp
// 8.3.6.1.1 General Atlas sequence parameter set Rbsp
void
V3CWriter::atlasSequenceParameterSetRbsp(AtlasSequenceParameterSetRbsp& asps,
                                         V3cBitstream&                  syntax,
                                         Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_UVLC(
    static_cast<uint32_t>(asps.getAtlasSequenceParameterSetId()));  // ue(v)
  WRITE_UVLC(asps.getFrameWidth());                                 // ue(v)
  WRITE_UVLC(asps.getFrameHeight());                                // ue(v)
  WRITE_CODE(asps.getGeometry3dBitdepthMinus1(), 5);                // u(5)
  WRITE_CODE(asps.getGeometry2dBitdepthMinus1(), 5);                // u(5)
  WRITE_UVLC(static_cast<uint32_t>(
    asps.getLog2MaxAtlasFrameOrderCntLsbMinus4()));  // ue(v)
  WRITE_UVLC(static_cast<uint32_t>(
    asps.getMaxDecAtlasFrameBufferingMinus1()));        // ue(v)
  WRITE_CODE(asps.getLongTermRefAtlasFramesFlag(), 1);  // u(1)
  WRITE_UVLC(
    static_cast<uint32_t>(asps.getNumRefAtlasFrameListsInAsps()));  // ue(v)
  for (size_t i = 0; i < asps.getNumRefAtlasFrameListsInAsps(); i++) {
    refListStruct(asps.getRefListStruct(i), asps, bitstream);
  }
  WRITE_CODE(asps.getUseEightOrientationsFlag(), 1);       // u(1)
  WRITE_CODE(asps.getExtendedProjectionEnabledFlag(), 1);  // u(1)
  if (asps.getExtendedProjectionEnabledFlag()) {
    WRITE_UVLC(
      static_cast<uint32_t>(asps.getMaxNumberProjectionsMinus1()));  // ue(v)
  }
  WRITE_CODE(asps.getNormalAxisLimitsQuantizationEnabledFlag(),
             1);                                                // u(1)
  WRITE_CODE(asps.getNormalAxisMaxDeltaValueEnabledFlag(), 1);  // u(1)
  WRITE_CODE(asps.getPatchPrecedenceOrderFlag(), 1);            // u(1)
  WRITE_CODE(asps.getLog2PatchPackingBlockSize(), 3);           // u(3)
  WRITE_CODE(asps.getPatchSizeQuantizerPresentFlag(), 1);       // u(1)
  WRITE_CODE(asps.getMapCountMinus1(), 4);                      // u(4)
  WRITE_CODE(asps.getPixelDeinterleavingFlag(), 1);             // u(1)
  if (asps.getPixelDeinterleavingFlag()) {
    for (size_t i = 0; i < asps.getMapCountMinus1() + 1; i++) {
      WRITE_CODE(asps.getPixelDeinterleavingMapFlag(i), 1);  // u(1)
    }
  }
  WRITE_CODE(asps.getRawPatchEnabledFlag(), 1);  // u(1)
  WRITE_CODE(asps.getEomPatchEnabledFlag(), 1);  // u(1)
  if (asps.getEomPatchEnabledFlag() && asps.getMapCountMinus1() == 0) {
    WRITE_CODE(asps.getEomFixBitCountMinus1(), 4);  // u(4)
  }
  if (asps.getRawPatchEnabledFlag() || asps.getEomPatchEnabledFlag()) {
    WRITE_CODE(asps.getAuxiliaryVideoEnabledFlag(), 1);  // u(1)
  }
  WRITE_CODE(asps.getPLREnabledFlag(), 1);  // u(1)
  if (asps.getPLREnabledFlag()) { plrInformation(asps, syntax, bitstream); }
  WRITE_CODE(asps.getVuiParametersPresentFlag(), 1);  // u(1)
  if (asps.getVuiParametersPresentFlag()) {
    vuiParameters(bitstream, asps.getVuiParameters());
  }

  WRITE_CODE(asps.getExtensionFlag(), 1);  // u(1)
  if (asps.getExtensionFlag()) {
    WRITE_CODE(asps.getVpccExtensionFlag(), 1);  // u(1)
    WRITE_CODE(asps.getMivExtensionFlag(), 1);   // u(1)
    WRITE_CODE(asps.getVdmcExtensionFlag(), 1);  // u(1)
    WRITE_CODE(asps.getExtension5Bits(), 5);     // u(5)
    if (asps.getVpccExtensionFlag())
      aspsVpccExtension(bitstream, asps, asps.getAspsVpccExtension());
    if (asps.getVdmcExtensionFlag())
      aspsVdmcExtension(bitstream, asps, asps.getAspsVdmcExtension());
    if (asps.getExtension5Bits())
      while (moreRbspData(bitstream)) WRITE_CODE(0, 1);  // u(1)
  }
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.1.2 Point local reconstruction information syntax
void
V3CWriter::plrInformation(AtlasSequenceParameterSetRbsp& asps,
                          V3cBitstream&                  syntax,
                          Bitstream&                     bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  for (size_t j = 0; j < asps.getMapCountMinus1() + 1; j++) {
    auto& plri = asps.getPLRInformation(j);
    WRITE_CODE(plri.getMapEnabledFlag(), 1);  // u(1)
    if (plri.getMapEnabledFlag()) {
      WRITE_CODE(plri.getNumberOfModesMinus1(), 4);  // u(4)
      for (size_t i = 0; i < plri.getNumberOfModesMinus1(); i++) {
        WRITE_CODE(plri.getInterpolateFlag(i), 1);  // u(1)
        WRITE_CODE(plri.getFillingFlag(i), 1);      // u(1)
        WRITE_CODE(plri.getMinimumDepth(i), 2);     // u(2)
        WRITE_CODE(plri.getNeighbourMinus1(i), 2);  // u(2)
      }
      WRITE_CODE(plri.getBlockThresholdPerPatchMinus1(), 6);  // u(6)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.2 Specification of syntax functions and descriptors
bool
V3CWriter::byteAligned(Bitstream& bitstream) {
  return bitstream.byteAligned();
}
bool
V3CWriter::moreDataInPayload(Bitstream& bitstream) {
  return !bitstream.byteAligned();
}
bool
V3CWriter::moreRbspData(Bitstream& bitstream) {
  return false;
}
bool
V3CWriter::moreRbspTrailingData(Bitstream& bitstream) {
  return false;
}
bool
V3CWriter::moreDataInV3CUnit(Bitstream& bitstream) {
  return false;
}
bool
V3CWriter::payloadExtensionPresent(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
  return false;
}

// 8.3.6.2 Atlas frame parameter set Rbsp syntax
// 8.3.6.2.1 General atlas frame parameter set Rbsp syntax
void
V3CWriter::atlasFrameParameterSetRbsp(AtlasFrameParameterSetRbsp& afps,
                                      V3cBitstream&               syntax,
                                      Bitstream&                  bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_UVLC(afps.getAtlasFrameParameterSetId());     // ue(v)
  WRITE_UVLC(afps.getAtlasSequenceParameterSetId());  // ue(v)
  auto& atlas = syntax.getAtlas();
  auto& asps =
    atlas.getAtlasSequenceParameterSet(afps.getAtlasSequenceParameterSetId());
  atlasFrameTileInformation(
    afps.getAtlasFrameTileInformation(), asps, bitstream);
  WRITE_CODE(afps.getOutputFlagPresentFlag(), 1);                // u(1)
  WRITE_UVLC(afps.getNumRefIdxDefaultActiveMinus1());            // ue(v)
  WRITE_UVLC(afps.getAdditionalLtAfocLsbLen());                  // ue(v)
  WRITE_CODE(afps.getLodModeEnableFlag(), 1);                    // u(1)
  WRITE_CODE(afps.getRaw3dOffsetBitCountExplicitModeFlag(), 1);  // u(1)
  WRITE_CODE(afps.getExtensionFlag(), 1);                        // u(1)
  if (afps.getExtensionFlag()) {
    WRITE_CODE(afps.getExtension8Bits(), 8);  // u(8)
  }
  if (afps.getExtension8Bits()) {
    while (moreRbspData(bitstream)) { WRITE_CODE(0, 1); }  // u(1)
  }
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.2.2 Atlas frame tile information syntax
void
V3CWriter::atlasFrameTileInformation(AtlasFrameTileInformation&     afti,
                                     AtlasSequenceParameterSetRbsp& asps,
                                     Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(afti.getSingleTileInAtlasFrameFlag(), 1);  // u(1)
  if (!afti.getSingleTileInAtlasFrameFlag()) {
    WRITE_CODE(afti.getUniformPartitionSpacingFlag(), 1);  // u(1)
    if (afti.getUniformPartitionSpacingFlag()) {
      WRITE_UVLC(afti.getPartitionColumnWidthMinus1(0));  //  ue(v)
      WRITE_UVLC(afti.getPartitionRowHeightMinus1(0));    //  ue(v)
    } else {
      WRITE_UVLC(afti.getNumPartitionColumnsMinus1());  //  ue(v)
      WRITE_UVLC(afti.getNumPartitionRowsMinus1());     //  ue(v)
      for (size_t i = 0; i < afti.getNumPartitionColumnsMinus1(); i++) {
        WRITE_UVLC(afti.getPartitionColumnWidthMinus1(i));  //  ue(v)
      }
      for (size_t i = 0; i < afti.getNumPartitionRowsMinus1(); i++) {
        WRITE_UVLC(afti.getPartitionRowHeightMinus1(i));  //  ue(v)
      }
    }
    WRITE_CODE(afti.getSinglePartitionPerTileFlag(), 1);  //  u(1)
    if (afti.getSinglePartitionPerTileFlag() == 0u) {
      uint32_t NumPartitionsInAtlasFrame =
        (afti.getNumPartitionColumnsMinus1() + 1)
        * (afti.getNumPartitionRowsMinus1() + 1);
      WRITE_UVLC(afti.getNumTilesInAtlasFrameMinus1());  // ue(v)
      for (size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++) {
        uint8_t bitCount = ceilLog2(NumPartitionsInAtlasFrame);
        WRITE_CODE(afti.getTopLeftPartitionIdx(i), bitCount);     // u(v)
        WRITE_UVLC(afti.getBottomRightPartitionColumnOffset(i));  // ue(v)
        WRITE_UVLC(afti.getBottomRightPartitionRowOffset(i));     // ue(v)
      }
    }
  }
  if (asps.getAuxiliaryVideoEnabledFlag()) {
    WRITE_UVLC(afti.getAuxiliaryVideoTileRowWidthMinus1());  // ue(v)
    for (size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++) {
      WRITE_UVLC(afti.getAuxiliaryVideoTileRowHeight(i));  // ue(v)
    }
  }
  WRITE_CODE(afti.getSignalledTileIdFlag(), 1);  // u(1)
  if (afti.getSignalledTileIdFlag()) {
    WRITE_UVLC(afti.getSignalledTileIdLengthMinus1());  // ue(v)
    for (size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++) {
      uint8_t bitCount = afti.getSignalledTileIdLengthMinus1() + 1;
      WRITE_CODE(afti.getTileId(i), bitCount);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.3	Atlas adaptation parameter set RBSP syntax
// 8.3.6.3.1	General atlas adaptation parameter set RBSP syntax
void
V3CWriter::atlasAdaptationParameterSetRbsp(
  AtlasAdaptationParameterSetRbsp& aaps,
  Bitstream&                       bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_UVLC(aaps.getAtlasAdaptationParameterSetId());  // ue(v)
  WRITE_CODE(aaps.getExtensionFlag(), 1);               // u(1)
  if (aaps.getExtensionFlag()) {
    WRITE_CODE(aaps.getVpccExtensionFlag(), 1);  // u(1)
    WRITE_CODE(aaps.getMivExtensionFlag(), 1);   // u(1)
    WRITE_CODE(aaps.getVdmcExtensionFlag(), 1);  // u(1)
    WRITE_CODE(aaps.getExtension5Bits(), 5);     // u(5)
    if (aaps.getVpccExtensionFlag())
      aapsVpccExtension(bitstream, aaps.getAapsVpccExtension());
    if (aaps.getVdmcExtensionFlag())
      aapsVdmcExtension(bitstream, aaps.getAapsVdmcExtension());
    if (aaps.getExtension5Bits())
      while (moreRbspData(bitstream)) { WRITE_CODE(0, 1); }  // u(1)
  }
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.4  Supplemental enhancement information Rbsp
void
V3CWriter::seiRbsp(V3cBitstream&    syntax,
                   Bitstream&       bitstream,
                   AtlasNalUnitType nalUnitType,
                   SEI&             sei,
                   size_t           atglIndex) {
  TRACE_BITSTREAM_IN("%s", __func__);
  do {
    seiMessage(bitstream, syntax, nalUnitType, sei, atglIndex);
  } while (moreRbspData(bitstream));
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.5 Access unit delimiter RBSP syntax
void
V3CWriter::accessUnitDelimiterRbsp(AccessUnitDelimiterRbsp& aud,
                                   V3cBitstream&            syntax,
                                   Bitstream&               bitstream) {
  WRITE_CODE(aud.getAframeType(), 3);  //	u(3)
  rbspTrailingBits(bitstream);
}
// 8.3.6.6 End of sequence RBSP syntax
void
V3CWriter::endOfSequenceRbsp(EndOfSequenceRbsp& eos,
                             V3cBitstream&      syntax,
                             Bitstream&         bitstream) {}

// 8.3.6.7 End of bitstream RBSP syntax
void
V3CWriter::endOfAtlasSubBitstreamRbsp(EndOfAtlasSubBitstreamRbsp& eoasb,
                                      V3cBitstream&               syntax,
                                      Bitstream&                  bitstream) {}

// 8.3.6.8 Filler data RBSP syntax
void
V3CWriter::fillerDataRbsp(FillerDataRbsp& filler,
                          V3cBitstream&   syntax,
                          Bitstream&      bitstream) {
  rbspTrailingBits(bitstream);
}

// 8.3.6.9 Atlas tile group layer Rbsp syntax = patchTileLayerUnit
void
V3CWriter::atlasTileLayerRbsp(AtlasTileLayerRbsp& atgl,
                              V3cBitstream&       syntax,
                              AtlasNalUnitType    nalUnitType,
                              Bitstream&          bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  atlasTileHeader(atgl.getHeader(), syntax, nalUnitType, bitstream);
  atlasTileDataUnit(atgl.getDataUnit(), atgl.getHeader(), syntax, bitstream);
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.10 RBSP trailing bit syntax
void
V3CWriter::rbspTrailingBits(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t one  = 1;
  uint32_t zero = 0;
  WRITE_CODE(one, 1);  // f(1): equal to 1
  while (!bitstream.byteAligned()) {
    WRITE_CODE(zero, 1);  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.11  Atlas tile group header syntax
void
V3CWriter::atlasTileHeader(AtlasTileHeader& ath,
                           V3cBitstream&    syntax,
                           AtlasNalUnitType nalUnitType,
                           Bitstream&       bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&                       atlas  = syntax.getAtlas();
  size_t                      afpsId = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp& afps   = atlas.getAtlasFrameParameterSet(afpsId);
  size_t                      aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps =
    atlas.getAtlasSequenceParameterSet(aspsId);
  AtlasFrameTileInformation& afti = afps.getAtlasFrameTileInformation();
  if (nalUnitType >= ATLAS_NAL_BLA_W_LP
      && nalUnitType <= ATLAS_NAL_RSV_IRAP_ACL_29) {
    WRITE_CODE(ath.getNoOutputOfPriorAtlasFramesFlag(), 1);  // u(1)
  }
  WRITE_UVLC(ath.getAtlasFrameParameterSetId());       // ue(v)
  WRITE_UVLC(ath.getAtlasAdaptationParameterSetId());  // ue(v)
  if (afti.getSignalledTileIdFlag()) {
    WRITE_CODE(uint32_t(ath.getId()),
               afti.getSignalledTileIdLengthMinus1() + 1);  // u(v)
  } else {
    if (afti.getNumTilesInAtlasFrameMinus1() != 0) {
      WRITE_CODE(uint32_t(ath.getId()),
                 ceilLog2(afti.getNumTilesInAtlasFrameMinus1() + 1));  // u(v)
    }
  }
  WRITE_UVLC(ath.getType());  // ue(v)
  if (afps.getOutputFlagPresentFlag()) {
    WRITE_CODE(ath.getAtlasOutputFlag(), 1);
  }
  WRITE_CODE(ath.getAtlasFrmOrderCntLsb(),
             asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4);  // u(v)
  if (asps.getNumRefAtlasFrameListsInAsps() > 0) {
    WRITE_CODE(ath.getRefAtlasFrameListSpsFlag(), 1);  // u(1)
  }
  if (static_cast<int>(ath.getRefAtlasFrameListSpsFlag()) == 0) {
    refListStruct(ath.getRefListStruct(), asps, bitstream);
  } else if (asps.getNumRefAtlasFrameListsInAsps() > 1) {
    size_t bitCount = ceilLog2(asps.getNumRefAtlasFrameListsInAsps());
    WRITE_CODE(ath.getRefAtlasFrameListIdx(), bitCount);  // u(v)
  }
  uint8_t rlsIdx                = ath.getRefAtlasFrameListIdx();
  auto&   refList               = ath.getRefAtlasFrameListSpsFlag()
                                    ? asps.getRefListStruct(rlsIdx)
                                    : ath.getRefListStruct();
  size_t  numLtrAtlasFrmEntries = 0;
  for (size_t i = 0; i < refList.getNumRefEntries(); i++) {
    if (!refList.getStRefAtlasFrameFlag(i)) { numLtrAtlasFrmEntries++; }
  }
  for (size_t j = 0; j < numLtrAtlasFrmEntries; j++) {
    WRITE_CODE(ath.getAdditionalAfocLsbPresentFlag(j), 1);  // u(1)
    if (ath.getAdditionalAfocLsbPresentFlag(j)) {
      uint8_t bitCount = afps.getAdditionalLtAfocLsbLen();
      WRITE_CODE(ath.getAdditionalAfocLsbVal(j), bitCount);  // u(v)
    }
  }
  if (ath.getType() != SKIP_TILE) {
    if (asps.getNormalAxisLimitsQuantizationEnabledFlag()) {
      WRITE_CODE(ath.getPosMinDQuantizer(), 5);  // u(5)
      if (asps.getNormalAxisMaxDeltaValueEnabledFlag()) {
        WRITE_CODE(ath.getPosDeltaMaxDQuantizer(), 5);  // u(5)
      }
    }
    if (asps.getPatchSizeQuantizerPresentFlag()) {
      WRITE_CODE(ath.getPatchSizeXinfoQuantizer(), 3);  // u(3)
      WRITE_CODE(ath.getPatchSizeYinfoQuantizer(), 3);  // u(3)
    }
    if (afps.getRaw3dOffsetBitCountExplicitModeFlag()) {
      size_t bitCount = floorLog2(asps.getGeometry3dBitdepthMinus1() + 1);
      WRITE_CODE(ath.getRaw3dOffsetAxisBitCountMinus1(),
                 bitCount);  // u(v)
    }
    if (ath.getType() == P_TILE && refList.getNumRefEntries() > 1) {
      WRITE_CODE(ath.getNumRefIdxActiveOverrideFlag(), 1);  // u(1)
      if (ath.getNumRefIdxActiveOverrideFlag()) {
        WRITE_UVLC(ath.getNumRefIdxActiveMinus1());  // ue(v)
      }
    }
  }
  byteAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.12  Reference list structure syntax
void
V3CWriter::refListStruct(RefListStruct&                 rls,
                         AtlasSequenceParameterSetRbsp& asps,
                         Bitstream&                     bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_UVLC(rls.getNumRefEntries());  // ue(v)
  for (size_t i = 0; i < rls.getNumRefEntries(); i++) {
    if (asps.getLongTermRefAtlasFramesFlag()) {
      WRITE_CODE(rls.getStRefAtlasFrameFlag(i), 1);  // u(1)
    }
    if (rls.getStRefAtlasFrameFlag(i)) {
      WRITE_UVLC(rls.getAbsDeltaAfocSt(i));  // ue(v)
      if (rls.getAbsDeltaAfocSt(i) > 0) {
        WRITE_CODE(rls.getStrafEntrySignFlag(i), 1);  // u(1)
      }
    } else {
      uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
      WRITE_CODE(rls.getAfocLsbLt(i), bitCount);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.1  General atlas tile group data unit syntax =patchTileDataUnit
void
V3CWriter::atlasTileDataUnit(AtlasTileDataUnit& atdu,
                             AtlasTileHeader&   ath,
                             V3cBitstream&      syntax,
                             Bitstream&         bitstream) {
  TRACE_BITSTREAM_IN("%s(%s)", __func__, toString(ath.getType()).c_str());
  if (ath.getType() == SKIP_TILE) {
    skipPatchDataUnit(bitstream);
  } else {
    for (size_t puCount = 0; puCount < atdu.getPatchCount(); puCount++) {
      WRITE_UVLC(uint32_t(atdu.getPatchMode(puCount)));  // ue(v)
      auto& pid           = atdu.getPatchInformationData(puCount);
      pid.getTileOrder()  = atdu.getTileOrder();
      pid.getPatchIndex() = puCount;
      patchInformationData(
        pid, atdu.getPatchMode(puCount), ath, syntax, bitstream);
    }
  }
  TRACE_BITSTREAM_OUT("%s(%s)", __func__, toString(ath.getType()).c_str());
}

// 8.3.7.2  Patch information data syntax
void
V3CWriter::patchInformationData(PatchInformationData& pid,
                                size_t                patchMode,
                                AtlasTileHeader&      ath,
                                V3cBitstream&         syntax,
                                Bitstream&            bitstream) {
  if (ath.getType() == SKIP_TILE) {
    // skip mode: currently not supported but added it for convenience. Could
    // easily be removed
  } else if (ath.getType() == P_TILE) {
    if (patchMode == P_SKIP) {
      // skip mode: currently not supported but added it for convenience. Could
      // easily be removed
      skipPatchDataUnit(bitstream);
    } else if (patchMode == P_MERGE) {
      auto& mpdu           = pid.getMergePatchDataUnit();
      mpdu.getTileOrder()  = pid.getTileOrder();
      mpdu.getPatchIndex() = pid.getPatchIndex();
      mergePatchDataUnit(mpdu, ath, syntax, bitstream);
    } else if (patchMode == P_INTRA) {
      auto& pdu           = pid.getPatchDataUnit();
      pdu.getTileOrder()  = pid.getTileOrder();
      pdu.getPatchIndex() = pid.getPatchIndex();
      patchDataUnit(pdu, ath, syntax, bitstream);
    } else if (patchMode == P_INTER) {
      auto& ipdu           = pid.getInterPatchDataUnit();
      ipdu.getTileOrder()  = pid.getTileOrder();
      ipdu.getPatchIndex() = pid.getPatchIndex();
      interPatchDataUnit(ipdu, ath, syntax, bitstream);
    } else if (patchMode == P_RAW) {
      auto& rpdu           = pid.getRawPatchDataUnit();
      rpdu.getTileOrder()  = pid.getTileOrder();
      rpdu.getPatchIndex() = pid.getPatchIndex();
      rawPatchDataUnit(rpdu, ath, syntax, bitstream);
    } else if (patchMode == P_EOM) {
      auto& epdu           = pid.getEomPatchDataUnit();
      epdu.getTileIndex()  = pid.getTileOrder();
      epdu.getPatchIndex() = pid.getPatchIndex();
      eomPatchDataUnit(epdu, ath, syntax, bitstream);
    }
  } else if (ath.getType() == I_TILE) {
    if (patchMode == I_INTRA) {
      auto& pdu           = pid.getPatchDataUnit();
      pdu.getTileOrder()  = pid.getTileOrder();
      pdu.getPatchIndex() = pid.getPatchIndex();
      patchDataUnit(pdu, ath, syntax, bitstream);
    } else if (patchMode == I_RAW) {
      auto& rpdu           = pid.getRawPatchDataUnit();
      rpdu.getTileOrder()  = pid.getTileOrder();
      rpdu.getPatchIndex() = pid.getPatchIndex();
      rawPatchDataUnit(rpdu, ath, syntax, bitstream);
    } else if (patchMode == I_EOM) {
      auto& epdu           = pid.getEomPatchDataUnit();
      epdu.getTileIndex()  = pid.getTileOrder();
      epdu.getPatchIndex() = pid.getPatchIndex();
      eomPatchDataUnit(epdu, ath, syntax, bitstream);
    }
  }
}

// 8.3.7.3  Patch data unit syntax : AtlasTileHeader instead of
// PatchTileHeader
void
V3CWriter::patchDataUnit(PatchDataUnit&   pdu,
                         AtlasTileHeader& ath,
                         V3cBitstream&    syntax,
                         Bitstream&       bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&   atlas      = syntax.getAtlas();
  size_t  afpsId     = ath.getAtlasFrameParameterSetId();
  auto&   afps       = atlas.getAtlasFrameParameterSet(afpsId);
  size_t  aspsId     = afps.getAtlasSequenceParameterSetId();
  auto&   asps       = atlas.getAtlasSequenceParameterSet(aspsId);
  uint8_t bitCountUV = asps.getGeometry3dBitdepthMinus1() + 1;
  uint8_t bitCountD =
    asps.getGeometry3dBitdepthMinus1() - ath.getPosMinDQuantizer() + 1;
  WRITE_UVLC(pdu.get2dPosX());                 // ue(v)
  WRITE_UVLC(pdu.get2dPosY());                 // ue(v)
  WRITE_UVLC(pdu.get2dSizeXMinus1());          // ue(v)
  WRITE_UVLC(pdu.get2dSizeYMinus1());          // ue(v)
  WRITE_CODE(pdu.get3dOffsetU(), bitCountUV);  // u(v)
  WRITE_CODE(pdu.get3dOffsetV(), bitCountUV);  // u(v)
  WRITE_CODE(pdu.get3dOffsetD(), bitCountD);   // u(v)
  if (asps.getNormalAxisMaxDeltaValueEnabledFlag()) {
    uint8_t bitCountForMaxDepth = std::min(asps.getGeometry2dBitdepthMinus1(),
                                           asps.getGeometry3dBitdepthMinus1())
                                  + 1 - ath.getPosDeltaMaxDQuantizer();
    WRITE_CODE(pdu.get3dRangeD(), bitCountForMaxDepth);  // u(v)
  }

  uint32_t numProjection  = ceilLog2(asps.getMaxNumberProjectionsMinus1() + 1);
  uint32_t numOrientation = asps.getUseEightOrientationsFlag() ? 3 : 1;
  WRITE_CODE(pdu.getProjectionId(), numProjection);       // u(5 or 3)
  WRITE_CODE(pdu.getOrientationIndex(), numOrientation);  // u(3 or 1)
  if (afps.getLodModeEnableFlag()) {
    WRITE_CODE(pdu.getLodEnableFlag(), 1);  // u(1)
    if (pdu.getLodEnableFlag()) {
      WRITE_UVLC(pdu.getLodScaleXMinus1());  // ue(v)
      WRITE_UVLC(pdu.getLodScaleYIdc());     // ue(v)
    }
  }
  if (asps.getPLREnabledFlag()) {
    auto& plrd = pdu.getPLRData();
    plrData(plrd, syntax, asps, bitstream);
  }
}

// 8.3.7.4  Skip patch data unit syntax
void
V3CWriter::skipPatchDataUnit(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.5  Merge patch data unit syntax
void
V3CWriter::mergePatchDataUnit(MergePatchDataUnit& mpdu,
                              AtlasTileHeader&    ath,
                              V3cBitstream&       syntax,
                              Bitstream&          bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&                       atlas  = syntax.getAtlas();
  size_t                      afpsId = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp& afps   = atlas.getAtlasFrameParameterSet(afpsId);
  size_t                      aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps =
    atlas.getAtlasSequenceParameterSet(aspsId);
  bool   overridePlrFlag = false;
  size_t numRefIdxActive = atlas.getNumRefIdxActive(ath);
  if (numRefIdxActive > 1) { WRITE_UVLC(mpdu.getRefIndex()); }  // ue(v)
  WRITE_CODE(mpdu.getOverride2dParamsFlag(), 1);                // u(1)
  if (mpdu.getOverride2dParamsFlag()) {
    WRITE_SVLC(mpdu.get2dPosX());        // se(v)
    WRITE_SVLC(mpdu.get2dPosY());        // se(v)
    WRITE_SVLC(mpdu.get2dDeltaSizeX());  // se(v)
    WRITE_SVLC(mpdu.get2dDeltaSizeY());  // se(v)
    if (asps.getPLREnabledFlag()) { overridePlrFlag = true; }
  } else {
    WRITE_CODE(mpdu.getOverride3dParamsFlag(), 1);  // u(1)
    if (mpdu.getOverride3dParamsFlag()) {
      WRITE_SVLC(mpdu.get3dOffsetU());  // se(v)
      WRITE_SVLC(mpdu.get3dOffsetV());  // se(v)
      WRITE_SVLC(mpdu.get3dOffsetD());  // se(v)
      if (asps.getNormalAxisMaxDeltaValueEnabledFlag()) {
        WRITE_SVLC(mpdu.get3dRangeD());  // se(v)
      }
      if (asps.getPLREnabledFlag()) {
        WRITE_CODE(mpdu.getOverridePlrFlag(), 1);  // u(1)
      }
    }
  }
  if (overridePlrFlag && asps.getPLREnabledFlag()) {
    auto& plrd = mpdu.getPLRData();
    plrData(plrd, syntax, asps, bitstream);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.6  Inter patch data unit syntax
void
V3CWriter::interPatchDataUnit(InterPatchDataUnit& ipdu,
                              AtlasTileHeader&    ath,
                              V3cBitstream&       syntax,
                              Bitstream&          bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&  atlas           = syntax.getAtlas();
  size_t afpsId          = ath.getAtlasFrameParameterSetId();
  auto&  afps            = atlas.getAtlasFrameParameterSet(afpsId);
  size_t aspsId          = afps.getAtlasSequenceParameterSetId();
  auto&  asps            = atlas.getAtlasSequenceParameterSet(aspsId);
  size_t numRefIdxActive = atlas.getNumRefIdxActive(ath);
  if (numRefIdxActive > 1) {
    WRITE_UVLC(int32_t(ipdu.getRefIndex()));  // ue(v)
  }
  WRITE_SVLC(int32_t(ipdu.getRefPatchIndex()));  // se(v)
  WRITE_SVLC(int32_t(ipdu.get2dPosX()));         // se(v)
  WRITE_SVLC(int32_t(ipdu.get2dPosY()));         // se(v)
  WRITE_SVLC(int32_t(ipdu.get2dDeltaSizeX()));   // se(v)
  WRITE_SVLC(int32_t(ipdu.get2dDeltaSizeY()));   // se(v)
  WRITE_SVLC(int32_t(ipdu.get3dOffsetU()));      // se(v)
  WRITE_SVLC(int32_t(ipdu.get3dOffsetV()));      // se(v)
  WRITE_SVLC(int32_t(ipdu.get3dOffsetD()));      // se(v)
  if (asps.getNormalAxisMaxDeltaValueEnabledFlag()) {
    WRITE_SVLC(int32_t(ipdu.get3dRangeD()));  // se(v)
  }
  if (asps.getPLREnabledFlag()) {
    auto& plrd = ipdu.getPLRData();
    plrData(plrd, syntax, asps, bitstream);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.7 raw patch data unit syntax
void
V3CWriter::rawPatchDataUnit(RawPatchDataUnit& rpdu,
                            AtlasTileHeader&  ath,
                            V3cBitstream&     syntax,
                            Bitstream&        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  int32_t bitCount = ath.getRaw3dOffsetAxisBitCountMinus1() + 1;
  auto&   atlas    = syntax.getAtlas();
  auto&   afti =
    atlas.getAtlasFrameParameterSet(ath.getAtlasFrameParameterSetId())
      .getAtlasFrameTileInformation();
  auto ath_id        = ath.getId();
  auto tileIdToIndex = afti.getTileId(ath_id);
  if (afti.getAuxiliaryVideoTileRowHeight(tileIdToIndex)) {
    WRITE_CODE(rpdu.getPatchInAuxiliaryVideoFlag(), 1);  // u(1)
  }
  WRITE_UVLC(rpdu.get2dPosX());               // ue(v)
  WRITE_UVLC(rpdu.get2dPosY());               // ue(v)
  WRITE_UVLC(rpdu.get2dSizeXMinus1());        // ue(v)
  WRITE_UVLC(rpdu.get2dSizeYMinus1());        // ue(v)
  WRITE_CODE(rpdu.get3dOffsetU(), bitCount);  // u(v)
  WRITE_CODE(rpdu.get3dOffsetV(), bitCount);  // u(v)
  WRITE_CODE(rpdu.get3dOffsetD(), bitCount);  // u(v)
  WRITE_UVLC(rpdu.getRawPointsMinus1());      // ue(v)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.8 EOM patch data unit syntax
void
V3CWriter::eomPatchDataUnit(EOMPatchDataUnit& epdu,
                            AtlasTileHeader&  ath,
                            V3cBitstream&     syntax,
                            Bitstream&        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& atlas = syntax.getAtlas();
  auto& afti =
    atlas.getAtlasFrameParameterSet(ath.getAtlasFrameParameterSetId())
      .getAtlasFrameTileInformation();
  auto ath_id        = ath.getId();
  auto tileIdToIndex = afti.getTileId(ath_id);
  if (afti.getAuxiliaryVideoTileRowHeight(tileIdToIndex)) {
    WRITE_CODE(epdu.getPatchInAuxiliaryVideoFlag(), 1);  // u(1)
  }
  WRITE_UVLC(epdu.get2dPosX());            // ue(v)
  WRITE_UVLC(epdu.get2dPosY());            // ue(v)
  WRITE_UVLC(epdu.get2dSizeXMinus1());     // ue(v)
  WRITE_UVLC(epdu.get2dSizeYMinus1());     // ue(v)
  WRITE_UVLC(epdu.getPatchCountMinus1());  //  ue(v)
  for (size_t i = 0; i < epdu.getPatchCountMinus1() + 1; i++) {
    WRITE_UVLC(epdu.getAssociatedPatchesIdx(i));  //  ue(v)
    WRITE_UVLC(epdu.getPoints(i));                //  ue(v)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.9 Point local reconstruction data syntax
void
V3CWriter::plrData(PLRData&                       plrd,
                   V3cBitstream&                  syntax,
                   AtlasSequenceParameterSetRbsp& asps,
                   Bitstream&                     bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  for (size_t i = 0; i < asps.getMapCountMinus1() + 1; i++) {
    auto& plri = asps.getPLRInformation(i);
    if (plri.getMapEnabledFlag()) {
      const size_t blockCount =
        plrd.getBlockToPatchMapWidth() * plrd.getBlockToPatchMapHeight();
      const auto bitCountMode =
        uint8_t(ceilLog2(uint32_t(plri.getNumberOfModesMinus1())));
      if (blockCount > plri.getBlockThresholdPerPatchMinus1() + 1) {
        WRITE_CODE(plrd.getLevelFlag(), 1);  // u(1)
      }
      if (!plrd.getLevelFlag()) {
        for (size_t i = 0; i < blockCount; i++) {
          WRITE_CODE(plrd.getBlockPresentFlag(i), 1);  // u(1)
          if (plrd.getBlockPresentFlag(i)) {
            WRITE_CODE(plrd.getBlockModeMinus1(i), bitCountMode);  // u(v)
          }
        }
      } else {
        WRITE_CODE(plrd.getPresentFlag(), 1);  // u(1)
        if (plrd.getPresentFlag()) {
          WRITE_CODE(plrd.getModeMinus1(), bitCountMode);  // u(v)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.8 Supplemental enhancement information message syntax
void
V3CWriter::seiMessage(Bitstream&       bitstream,
                      V3cBitstream&    syntax,
                      AtlasNalUnitType nalUnitType,
                      SEI&             sei,
                      size_t           atglIndex) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto payloadType = static_cast<int32_t>(sei.getPayloadType());
  for (; payloadType >= 0xff; payloadType -= 0xff) {
    WRITE_CODE(0xff, 8);  // u(8)
  }
  WRITE_CODE(payloadType, 8);  // u(8)

  // Note: calculating the size of the sei message before writing it into the bitstream
  Bitstream tempbitstream;
  seiPayload(tempbitstream, syntax, sei, nalUnitType, atglIndex);
  sei.setPayloadSize(tempbitstream.size());

  auto payloadSize = static_cast<int32_t>(sei.getPayloadSize());
  for (; payloadSize >= 0xff; payloadSize -= 0xff) {
    WRITE_CODE(0xff, 8);  // u(8)
  }
  WRITE_CODE(payloadSize, 8);  // u(8)
  seiPayload(bitstream, syntax, sei, nalUnitType, atglIndex);
}

// C.2 Sample stream V3C unit syntax and semantics
// C.2.1 Sample stream V3C header syntax
void
V3CWriter::sampleStreamV3CHeader(Bitstream&           bitstream,
                                 SampleStreamV3CUnit& ssvu) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  WRITE_CODE(ssvu.getSsvhUnitSizePrecisionBytesMinus1(), 3);  // u(3)
  WRITE_CODE(zero, 5);                                        // u(5)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// C.2.2 Sample stream V3C unit syntax
void
V3CWriter::sampleStreamV3CUnit(Bitstream&           bitstream,
                               SampleStreamV3CUnit& ssvu,
                               V3CUnit&             v3cUnit) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(v3cUnit.getSize(),
             8 * (ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1));  // u(v)
  v3cUnit.getBitstream().vector().resize(v3cUnit.getSize());
  WRITE_VECTOR(v3cUnit.getBitstream().vector());
  TRACE_BITSTREAM_OUT(
    "%s type = %s ", __func__, toString(v3cUnit.getType()).c_str());
}

// D.2 Sample stream NAL unit syntax and semantics
// D.2.1 Sample stream NAL header syntax
void
V3CWriter::sampleStreamNalHeader(Bitstream&           bitstream,
                                 SampleStreamNalUnit& ssnu) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  WRITE_CODE(ssnu.getSizePrecisionBytesMinus1(), 3);  // u(3)
  WRITE_CODE(zero, 5);                                // u(5)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// D.2.2 Sample stream NAL unit syntax
void
V3CWriter::sampleStreamNalUnit(V3cBitstream&        syntax,
                               bool                 isAtlas,
                               Bitstream&           bitstream,
                               SampleStreamNalUnit& ssnu,
                               NalUnit&             nalu,
                               size_t               index,
                               size_t               atglIndex) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(nalu.getSize(),
             8 * (ssnu.getSizePrecisionBytesMinus1() + 1));  // u(v)
  Bitstream ssnuBitstream;
#if defined(CONFORMANCE_TRACE) || defined(BITSTREAM_TRACE)
  ssnuBitstream.setTrace(true);
  ssnuBitstream.setLogger(*logger_);
#endif
  nalUnitHeader(ssnuBitstream, nalu);
  if (isAtlas) {
    auto  naluType = AtlasNalUnitType(nalu.getType());
    auto& atlas    = syntax.getAtlas();
    switch (naluType) {
    case ATLAS_NAL_ASPS:
      atlasSequenceParameterSetRbsp(
        atlas.getAtlasSequenceParameterSet(index), syntax, ssnuBitstream);
      break;
    case ATLAS_NAL_AFPS:
      atlasFrameParameterSetRbsp(
        atlas.getAtlasFrameParameterSet(index), syntax, ssnuBitstream);
      break;
    case ATLAS_NAL_TRAIL_N:
    case ATLAS_NAL_TRAIL_R:
    case ATLAS_NAL_TSA_N:
    case ATLAS_NAL_TSA_R:
    case ATLAS_NAL_STSA_N:
    case ATLAS_NAL_STSA_R:
    case ATLAS_NAL_RADL_N:
    case ATLAS_NAL_RADL_R:
    case ATLAS_NAL_RASL_N:
    case ATLAS_NAL_RASL_R:
    case ATLAS_NAL_SKIP_N:
    case ATLAS_NAL_SKIP_R:
    case ATLAS_NAL_IDR_N_LP:
      atlasTileLayerRbsp(
        atlas.getAtlasTileLayer(index), syntax, naluType, ssnuBitstream);
      break;
    case ATLAS_NAL_SUFFIX_ESEI:
    case ATLAS_NAL_SUFFIX_NSEI:
      seiRbsp(syntax,
              ssnuBitstream,
              naluType,
              atlas.getAtlasTileLayer(atglIndex).getSEI().getSeiSuffix(index),
              atglIndex);
      break;
    case ATLAS_NAL_PREFIX_ESEI:
    case ATLAS_NAL_PREFIX_NSEI:
      seiRbsp(syntax,
              ssnuBitstream,
              naluType,
              atlas.getAtlasTileLayer(atglIndex).getSEI().getSeiPrefix(index),
              atglIndex);
      break;
    default:
      fprintf(stderr,
              "sampleStreamNalUnit type = %d not supported\n",
              static_cast<int32_t>(nalu.getType()));
    }
  } else {
    auto  naluType = BaseMeshNalUnitType(nalu.getType());
    auto& baseMesh = syntax.getBaseMesh();
    switch (naluType) {
    case BASEMESH_NAL_BMSPS:
      baseMeshSequenceParameterSetRbsp(
        baseMesh.getBaseMeshSequenceParameterSet(index),
        syntax,
        ssnuBitstream);
      break;
    case BASEMESH_NAL_BMFPS:
      baseMeshFrameParameterSetRbsp(
        baseMesh.getBaseMeshFrameParameterSet(index), syntax, ssnuBitstream);
      break;
    case BASEMESH_NAL_TRAIL_N:
    case BASEMESH_NAL_TRAIL_R:
    case BASEMESH_NAL_TSA_N:
    case BASEMESH_NAL_TSA_R:
    case BASEMESH_NAL_STSA_N:
    case BASEMESH_NAL_STSA_R:
    case BASEMESH_NAL_RADL_N:
    case BASEMESH_NAL_RADL_R:
    case BASEMESH_NAL_RASL_N:
    case BASEMESH_NAL_RASL_R:
    case BASEMESH_NAL_SKIP_N:
    case BASEMESH_NAL_SKIP_R:
    case BASEMESH_NAL_IDR_N_LP:
      baseMeshTileLayerRbsp(
        baseMesh.getBaseMeshTileLayer(index), naluType, syntax, ssnuBitstream);
      break;
    default:
      fprintf(
        stderr, "StreamNalUnit type = %d not supported\n", uint32_t(naluType));
    }
  }
  auto type = isAtlas ? toString(AtlasNalUnitType(nalu.getType()))
                      : toString(BaseMeshNalUnitType(nalu.getType()));
  printf("sampleStreamNalUnit Type = %-25s size = %zu \n",
         type.c_str(),
         ssnuBitstream.size());
  fflush(stdout);
  bitstream.copyFrom(ssnuBitstream, 0, ssnuBitstream.size());
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2  SEI payload syntax
// F.2.1  General SEI message syntax
void
V3CWriter::seiPayload(Bitstream&       bitstream,
                      V3cBitstream&    syntax,
                      SEI&             sei,
                      AtlasNalUnitType nalUnitType,
                      size_t           atglIndex) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto payloadType = sei.getPayloadType();
  if (nalUnitType == ATLAS_NAL_PREFIX_ESEI
      || nalUnitType == ATLAS_NAL_PREFIX_NSEI) {
    if (payloadType == BUFFERING_PERIOD) {  // 0
      bufferingPeriod(bitstream, sei);
    } else if (payloadType == ATLAS_FRAME_TIMING) {  // 1
      assert(syntax.getAtlas().getAtlasTileLayer(atglIndex).getSEI().seiIsPresent(
        ATLAS_NAL_PREFIX_NSEI, BUFFERING_PERIOD));
      auto& atlas = syntax.getAtlas();
      auto& bpsei = *atlas.getAtlasTileLayer(atglIndex).getSEI().getLastSei(
        ATLAS_NAL_PREFIX_NSEI, BUFFERING_PERIOD);
      atlasFrameTiming(bitstream, sei, bpsei, false);
    } else if (payloadType == FILLER_PAYLOAD) {  // 2
      fillerPayload(bitstream, sei);
    } else if (payloadType == USER_DATAREGISTERED_ITUTT35) {  // 3
      userDataRegisteredItuTT35(bitstream, sei);
    } else if (payloadType == USER_DATA_UNREGISTERED) {  // 4
      userDataUnregistered(bitstream, sei);
    } else if (payloadType == RECOVERY_POINT) {  // 5
      recoveryPoint(bitstream, sei);
    } else if (payloadType == NO_RECONSTRUCTION) {  // 6
      noReconstruction(bitstream, sei);
    } else if (payloadType == TIME_CODE) {  // 7
      timeCode(bitstream, sei);
    } else if (payloadType == SEI_MANIFEST) {  // 8
      seiManifest(bitstream, sei);
    } else if (payloadType == SEI_PREFIX_INDICATION) {  // 9
      seiPrefixIndication(bitstream, sei);
    } else if (payloadType == ACTIVE_SUB_BITSTREAMS) {  // 10
      activeSubBitstreams(bitstream, sei);
    } else if (payloadType == COMPONENT_CODEC_MAPPING) {  // 11
      componentCodecMapping(bitstream, sei);
    } else if (payloadType == SCENE_OBJECT_INFORMATION) {  // 12
      sceneObjectInformation(bitstream, sei);
    } else if (payloadType == OBJECT_LABEL_INFORMATION) {  // 13
      objectLabelInformation(bitstream, sei);
    } else if (payloadType == PATCH_INFORMATION) {  // 14
      patchInformation(bitstream, sei);
    } else if (payloadType == VOLUMETRIC_RECTANGLE_INFORMATION) {  // 15
      volumetricRectangleInformation(bitstream, sei);
    } else if (payloadType == ATLAS_OBJECT_INFORMATION) {  // 16
      atlasObjectInformation(bitstream, sei);
    } else if (payloadType == VIEWPORT_CAMERA_PARAMETERS) {  // 17
      viewportCameraParameters(bitstream, sei);
    } else if (payloadType == VIEWPORT_POSITION) {  // 18
      viewportPosition(bitstream, sei);
    } else if (payloadType == ATTRIBUTE_TRANSFORMATION_PARAMS) {  // 64
      attributeTransformationParams(bitstream, sei);
    } else if (payloadType == OCCUPANCY_SYNTHESIS) {  // 65
      occupancySynthesis(bitstream, sei);
    } else if (payloadType == GEOMETRY_SMOOTHING) {  // 66
      geometrySmoothing(bitstream, sei);
    } else if (payloadType == ATTRIBUTE_SMOOTHING) {  // 67
      attributeSmoothing(bitstream, sei);
    } else {
      reservedSeiMessage(bitstream, sei);
    }
  } else { /* nalUnitType  ==  NAL_SUFFIX_SEI  || nalUnitType  ==
              NAL_SUFFIX_NSEI */
    /*SEI& sei = syntax.addSeiSuffix( payloadType, nalUnitType == NAL_SUFFIX_ESEI );*/
    if (payloadType == FILLER_PAYLOAD) {  // 2
      fillerPayload(bitstream, sei);
    } else if (payloadType == USER_DATAREGISTERED_ITUTT35) {  // 3
      userDataRegisteredItuTT35(bitstream, sei);
    } else if (payloadType == USER_DATA_UNREGISTERED) {  // 4
      userDataUnregistered(bitstream, sei);
    } else if (payloadType == DECODED_ATLAS_INFORMATION_HASH) {  // 21
      decodedAtlasInformationHash(bitstream, sei);
    } else {
      reservedSeiMessage(bitstream, sei);
    }
  }
  if (moreDataInPayload(bitstream)) {
    if (payloadExtensionPresent(bitstream)) {
      WRITE_CODE(1, 1);  // u(v)
    }
    byteAlignment(bitstream);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.2  Filler payload SEI message syntax
void
V3CWriter::fillerPayload(Bitstream& bitstream, SEI& sei) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto payloadSize = sei.getPayloadSize();
  for (size_t k = 0; k < payloadSize; k++) {
    WRITE_CODE(0xFF, 8);  // f(8) equal to 0xFF
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
void
V3CWriter::userDataRegisteredItuTT35(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei         = static_cast<SEIUserDataRegisteredItuTT35&>(seiAbstract);
  auto  payloadSize = sei.getPayloadSize();
  WRITE_CODE(sei.getCountryCode(), 8);  // b(8)
  payloadSize--;
  if (sei.getCountryCode() == 0xFF) {
    WRITE_CODE(sei.getCountryCodeExtensionByte(), 8);  // b(8)
    payloadSize--;
  }
  auto& payload = sei.getPayloadByte();
  for (auto& element : payload) {
    WRITE_CODE(element, 8);  // b(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.4  User data unregistered SEI message syntax
void
V3CWriter::userDataUnregistered(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei         = static_cast<SEIUserDataUnregistered&>(seiAbstract);
  auto  payloadSize = sei.getPayloadSize();
  for (size_t i = 0; i < 16; i++) {
    WRITE_CODE(sei.getUuidIsoIec11578(i), 8);  // u(128) <=> 16 * u(8)
  }
  payloadSize -= 16;
  for (size_t i = 0; i < payloadSize; i++) {
    WRITE_CODE(sei.getUserDataPayloadByte(i), 8);  // b(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.5  Recovery point SEI message syntax
void
V3CWriter::recoveryPoint(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIRecoveryPoint&>(seiAbstract);
  WRITE_SVLC(sei.getRecoveryAfocCnt());    // se(v)
  WRITE_CODE(sei.getExactMatchFlag(), 1);  // u(1)
  WRITE_CODE(sei.getBrokenLinkFlag(), 1);  // u(1)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.6  No reconstruction SEI message syntax
void
V3CWriter::noReconstruction(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.7  Reserved SEI message syntax
void
V3CWriter::reservedSeiMessage(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei         = static_cast<SEIReservedSeiMessage&>(seiAbstract);
  auto  payloadSize = sei.getPayloadSize();
  for (size_t i = 0; i < payloadSize; i++) {
    WRITE_CODE(sei.getPayloadByte(i), 8);  // b(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.8  SEI manifest SEI message syntax
void
V3CWriter::seiManifest(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIManifest&>(seiAbstract);
  WRITE_CODE(sei.getNumSeiMsgTypes(), 16);  // u(16)
  for (size_t i = 0; i < sei.getNumSeiMsgTypes(); i++) {
    WRITE_CODE(sei.getSeiPayloadType(i), 16);  // u(16)
    WRITE_CODE(sei.getSeiDescription(i), 8);   // u(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.9  SEI prefix indication SEI message syntax
void
V3CWriter::seiPrefixIndication(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIPrefixIndication&>(seiAbstract);
  WRITE_CODE(sei.getPrefixSeiPayloadType(), 16);          // u(16)
  WRITE_CODE(sei.getNumSeiPrefixIndicationsMinus1(), 8);  // u(8)
  for (size_t i = 0; i <= sei.getNumSeiPrefixIndicationsMinus1(); i++) {
    WRITE_CODE(sei.getNumBitsInPrefixIndicationMinus1(i), 16);  // u(16)
    for (size_t j = 0; j <= sei.getNumBitsInPrefixIndicationMinus1(i); j++) {
      WRITE_CODE(sei.getSeiPrefixDataBit(i, j), 1);  // u(1)
    }
    while (!bitstream.byteAligned()) {
      WRITE_CODE(1, 1);  // f(1): equal to 1
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.10  Active substreams SEI message syntax
void
V3CWriter::activeSubBitstreams(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIActiveSubBitstreams&>(seiAbstract);
  WRITE_CODE(sei.getActiveSubBitstreamsCancelFlag(), 1);  // u(1)
  if (!sei.getActiveSubBitstreamsCancelFlag()) {
    WRITE_CODE(sei.getActiveAttributesChangesFlag(), 1);    // u(1)
    WRITE_CODE(sei.getActiveMapsChangesFlag(), 1);          // u(1)
    WRITE_CODE(sei.getAuxiliarySubstreamsActiveFlag(), 1);  // u(1)
    if (sei.getActiveAttributesChangesFlag()) {
      WRITE_CODE(sei.getAllAttributesActiveFlag(), 1);  // u(1)
      if (!sei.getAllAttributesActiveFlag()) {
        WRITE_CODE(sei.getActiveAttributeCountMinus1(), 7);  // u(7)
        for (size_t i = 0; i <= sei.getActiveAttributeCountMinus1(); i++) {
          WRITE_CODE(sei.getActiveAttributeIdx(i), 7);  // u(7)
        }
      }
    }
    if (sei.getActiveMapsChangesFlag()) {
      WRITE_CODE(sei.getAllMapsActiveFlag(), 1);  // u(1)
      if (!sei.getAllMapsActiveFlag()) {
        WRITE_CODE(sei.getActiveMapCountMinus1(), 4);  // u(4)
        for (size_t i = 0; i <= sei.getActiveMapCountMinus1(); i++) {
          WRITE_CODE(sei.getActiveMapIdx(i), 4);  // u(4)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.11  Component codec mapping SEI message syntax
void
V3CWriter::componentCodecMapping(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIComponentCodecMapping&>(seiAbstract);
  WRITE_CODE(sei.getComponentCodecCancelFlag(), 1);  // u(1)
  if (!sei.getComponentCodecCancelFlag()) {
    WRITE_CODE(sei.getCodecMappingsCountMinus1(), 8);  // u(8)
    sei.allocate();
    for (size_t i = 0; i <= sei.getCodecMappingsCountMinus1(); i++) {
      WRITE_CODE(sei.getCodecId(i), 8);                  // u(8)
      WRITE_STRING(sei.getCodec4cc(sei.getCodecId(i)));  // st(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.12.1	Scene object information SEI message syntax
void
V3CWriter::sceneObjectInformation(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEISceneObjectInformation&>(seiAbstract);
  WRITE_CODE(sei.getPersistenceFlag(), 1);  // u(1)
  WRITE_CODE(sei.getResetFlag(), 1);        // u(1)
  WRITE_UVLC(sei.getNumObjectUpdates());    // ue(v)
  if (sei.getNumObjectUpdates() > 0) {
    WRITE_CODE(sei.getSimpleObjectsFlag(), 1);  // u(1)
    if (static_cast<int>(sei.getSimpleObjectsFlag()) == 0) {
      WRITE_CODE(sei.getObjectLabelPresentFlag(), 1);       // u(1)
      WRITE_CODE(sei.getPriorityPresentFlag(), 1);          // u(1)
      WRITE_CODE(sei.getObjectHiddenPresentFlag(), 1);      // u(1)
      WRITE_CODE(sei.getObjectDependencyPresentFlag(), 1);  // u(1)
      WRITE_CODE(sei.getVisibilityConesPresentFlag(), 1);   // u(1)
      WRITE_CODE(sei.get3dBoundingBoxPresentFlag(), 1);     // u(1)
      WRITE_CODE(sei.getCollisionShapePresentFlag(), 1);    // u(1)
      WRITE_CODE(sei.getPointStylePresentFlag(), 1);        // u(1)
      WRITE_CODE(sei.getMaterialIdPresentFlag(), 1);        // u(1)
      WRITE_CODE(sei.getExtensionPresentFlag(), 1);         // u(1)
    }
    if (sei.get3dBoundingBoxPresentFlag()) {
      WRITE_CODE(sei.get3dBoundingBoxScaleLog2(), 5);        // u(5)
      WRITE_CODE(sei.get3dBoundingBoxPrecisionMinus8(), 5);  // u(5)
    }
    WRITE_CODE(sei.getLog2MaxObjectIdxUpdated(), 5);  // u(5)
    if (sei.getObjectDependencyPresentFlag()) {
      WRITE_CODE(sei.getLog2MaxObjectDependencyIdx(), 5);  // u(5)
    }
    for (size_t i = 0; i <= sei.getNumObjectUpdates(); i++) {
      assert(sei.getObjectIdx(i) >= sei.getNumObjectUpdates());
      WRITE_CODE(sei.getObjectIdx(i),
                 sei.getLog2MaxObjectIdxUpdated());  // u(v)
      size_t k = sei.getObjectIdx(i);
      WRITE_CODE(sei.getObjectCancelFlag(k), 1);  // u(1)
      if (sei.getObjectCancelFlag(k)) {
        if (sei.getObjectLabelPresentFlag()) {
          WRITE_CODE(sei.getObjectLabelUpdateFlag(k), 1);  // u(1)
          if (sei.getObjectLabelUpdateFlag(k)) {
            WRITE_UVLC(sei.getObjectLabelIdx(k));  // ue(v)
          }
        }
        if (sei.getPriorityPresentFlag()) {
          WRITE_CODE(sei.getPriorityUpdateFlag(k), 1);  // u(1)
          if (sei.getPriorityUpdateFlag(k)) {
            WRITE_CODE(sei.getPriorityValue(k), 4);  // u(4)
          }
        }
        if (sei.getObjectHiddenPresentFlag()) {
          WRITE_CODE(sei.getObjectHiddenFlag(k), 1);  // u(1)
        }
        if (sei.getObjectDependencyPresentFlag()) {
          WRITE_CODE(sei.getObjectDependencyUpdateFlag(k), 1);  // u(1)
          if (sei.getObjectDependencyUpdateFlag(k)) {
            WRITE_CODE(sei.getObjectNumDependencies(k), 4);  // u(4)
            size_t bitCount =
              ceil(log2(sei.getObjectNumDependencies(k)) + 0.5);
            for (size_t j = 0; j < sei.getObjectNumDependencies(k); j++) {
              WRITE_CODE(sei.getObjectDependencyIdx(k, j),
                         bitCount);  // u(v)
            }
          }
        }
        if (sei.getVisibilityConesPresentFlag()) {
          WRITE_CODE(sei.getVisibilityConesUpdateFlag(k), 1);  // u(1)
          if (sei.getVisibilityConesUpdateFlag(k)) {
            WRITE_CODE(sei.getDirectionX(k), 16);  // u(16)
            WRITE_CODE(sei.getDirectionY(k), 16);  // u(16)
            WRITE_CODE(sei.getDirectionZ(k), 16);  // u(16)
            WRITE_CODE(sei.getAngle(k), 16);       // u(16)
          }
        }  // cones

        if (sei.get3dBoundingBoxPresentFlag()) {
          WRITE_CODE(sei.get3dBoundingBoxUpdateFlag(k), 1);  // u(1)
          if (sei.get3dBoundingBoxUpdateFlag(k)) {
            WRITE_UVLC(sei.get3dBoundingBoxX(k));       // ue(v)
            WRITE_UVLC(sei.get3dBoundingBoxY(k));       // ue(v)
            WRITE_UVLC(sei.get3dBoundingBoxZ(k));       // ue(v)
            WRITE_UVLC(sei.get3dBoundingBoxDeltaX(k));  // ue(v)
            WRITE_UVLC(sei.get3dBoundingBoxDeltaY(k));  // ue(v)
            WRITE_UVLC(sei.get3dBoundingBoxDeltaZ(k));  // ue(v)
          }
        }  // 3dBB

        if (sei.getCollisionShapePresentFlag()) {
          WRITE_CODE(sei.getCollisionShapeUpdateFlag(k), 1);  // u(1)
          if (sei.getCollisionShapeUpdateFlag(k)) {
            WRITE_CODE(sei.getCollisionShapeId(k), 16);  // u(16)
          }
        }  // collision
        if (sei.getPointStylePresentFlag()) {
          WRITE_CODE(sei.getPointStyleUpdateFlag(k), 1);  // u(1)
          if (sei.getPointStyleUpdateFlag(k)) {
            WRITE_CODE(sei.getPointShapeId(k), 8);  // u(8)
            WRITE_CODE(sei.getPointSize(k), 16);    // u(16)
          }
        }  // pointstyle
        if (sei.getMaterialIdPresentFlag()) {
          WRITE_CODE(sei.getMaterialIdUpdateFlag(k), 1);  // u(1)
          if (sei.getMaterialIdUpdateFlag(k)) {
            WRITE_CODE(sei.getMaterialId(k), 16);  // u(16)
          }
        }  // materialid
      }    // sei.getObjectCancelFlag(k)
    }      // for(size_t i=0; i<=sei.getNumObjectUpdates(); i++)
  }        // if( sei.getNumObjectUpdates() > 0 )
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.12.2 Object label information SEI message syntax
void
V3CWriter::objectLabelInformation(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&    sei  = static_cast<SEIObjectLabelInformation&>(seiAbstract);
  uint32_t zero = 0;
  WRITE_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (!sei.getCancelFlag()) {
    WRITE_CODE(sei.getLabelLanguagePresentFlag(), 1);  // u(1)
    if (sei.getLabelLanguagePresentFlag()) {
      while (!bitstream.byteAligned()) {
        WRITE_CODE(zero, 1);  // u(1)
      }
      WRITE_STRING(sei.getLabelLanguage());  // st(v)
    }
    WRITE_UVLC(sei.getNumLabelUpdates());  // ue(v)
    for (size_t i = 0; i < sei.getNumLabelUpdates(); i++) {
      WRITE_UVLC(sei.getLabelIdx(i));           // ue(v)
      WRITE_CODE(sei.getLabelCancelFlag(), 1);  // u(1)
      if (!sei.getLabelCancelFlag()) {
        while (!bitstream.byteAligned()) {
          WRITE_CODE(zero, 1);  // u(1)
        }
        WRITE_STRING(sei.getLabel(sei.getLabelIdx(i)));  // st(v)
      }
    }
    WRITE_CODE(sei.getPersistenceFlag(), 1);  // u(1)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.12.3 Patch information SEI message syntax
void
V3CWriter::patchInformation(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIPatchInformation&>(seiAbstract);
  WRITE_CODE(sei.getPersistenceFlag(), 1);  // u(1)
  WRITE_CODE(sei.getResetFlag(), 1);        // u(1)
  WRITE_UVLC(sei.getNumTileUpdates());      // ue(v)
  if (sei.getNumTileUpdates() > 0) {
    WRITE_CODE(sei.getLog2MaxObjectIdxTracked(), 5);  // u(5)
    WRITE_CODE(sei.getLog2MaxPatchIdxUpdated(), 4);   // u(4)
  }
  for (size_t i = 0; i < sei.getNumTileUpdates(); i++) {
    WRITE_UVLC(sei.getTileId(i));  // ue(v)
    size_t j = sei.getTileId(i);
    WRITE_CODE(sei.getTileCancelFlag(j), 1);  // u(1)
    WRITE_UVLC(sei.getNumPatchUpdates(j));    // ue(v)
    for (size_t k = 0; k < sei.getNumPatchUpdates(j); k++) {
      WRITE_CODE(sei.getPatchIdx(j, k),
                 sei.getLog2MaxPatchIdxUpdated());  // u(v)
      auto p = sei.getPatchIdx(j, k);
      WRITE_CODE(sei.getPatchCancelFlag(j, p), 1);  // u(1)
      if (!sei.getPatchCancelFlag(j, p)) {
        WRITE_UVLC(sei.getPatchNumberOfObjectsMinus1(j, p));  // ue(v)
        for (size_t n = 0; n < sei.getPatchNumberOfObjectsMinus1(j, p) + 1;
             n++) {
          WRITE_CODE(sei.getPatchObjectIdx(j, p, n),
                     sei.getLog2MaxObjectIdxTracked());  // u(v)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
};

// F.2.12.4 Volumetric rectangle information SEI message syntax
void
V3CWriter::volumetricRectangleInformation(Bitstream& bitstream,
                                          SEI&       seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIVolumetricRectangleInformation&>(seiAbstract);
  WRITE_CODE(sei.getPersistenceFlag(), 1);    // u(1)
  WRITE_CODE(sei.getResetFlag(), 1);          // u(1)
  WRITE_UVLC(sei.getNumRectanglesUpdates());  // ue(v)
  if (sei.getNumRectanglesUpdates() > 0) {
    WRITE_CODE(sei.getLog2MaxObjectIdxTracked(), 5);     // u(5)
    WRITE_CODE(sei.getLog2MaxRectangleIdxUpdated(), 4);  // u(4)
  }
  for (size_t k = 0; k < sei.getNumRectanglesUpdates(); k++) {
    WRITE_CODE(sei.getRectangleIdx(k),
               sei.getLog2MaxRectangleIdxUpdated());  // u(v)
    auto p = sei.getRectangleIdx(k);
    WRITE_CODE(sei.getRectangleCancelFlag(p), 1);  // u(1)
    if (!sei.getRectangleCancelFlag(p)) {
      WRITE_CODE(sei.getBoundingBoxUpdateFlag(p), 1);  // u(1)
      if (sei.getBoundingBoxUpdateFlag(p)) {
        WRITE_UVLC(sei.getBoundingBoxTop(p));     // ue(v)
        WRITE_UVLC(sei.getBoundingBoxLeft(p));    // ue(v)
        WRITE_UVLC(sei.getBoundingBoxWidth(p));   // ue(v)
        WRITE_UVLC(sei.getBoundingBoxHeight(p));  // ue(v)
      }
      WRITE_UVLC(sei.getRectangleNumberOfObjectsMinus1(p));  // ue(v)
      for (size_t n = 0; n < sei.getRectangleNumberOfObjectsMinus1(p) + 1;
           n++) {
        WRITE_CODE(sei.getRectangleObjectIdx(p, n),
                   sei.getLog2MaxObjectIdxTracked());  // u(v)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
};

// F.2.12.5  Atlas object information  SEI message syntax
void
V3CWriter::atlasObjectInformation(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIAtlasInformation&>(seiAbstract);
  WRITE_CODE(sei.getPersistenceFlag(), 1);   //	u(1)
  WRITE_CODE(sei.getResetFlag(), 1);         // 	u(1)
  WRITE_CODE(sei.getNumAtlasesMinus1(), 6);  // 	u(6)
  WRITE_UVLC(sei.getNumUpdates());           // ue(v)
  if (sei.getNumUpdates() > 0) {
    WRITE_CODE(sei.getLog2MaxObjectIdxTracked(), 5);  //	u(5)
    for (size_t i = 0; i < sei.getNumAtlasesMinus1() + 1; i++) {
      WRITE_CODE(sei.getAtlasId(i), 5);  // 	u(6)
    }
    for (size_t i = 0; i < sei.getNumUpdates() + 1; i++) {
      WRITE_CODE(sei.getObjectIdx(i),
                 sei.getLog2MaxObjectIdxTracked());  // u(v)
      for (size_t j = 0; j < sei.getNumAtlasesMinus1() + 1; j++) {
        WRITE_CODE(sei.getObjectInAtlasPresentFlag(i, j), 1);  // u(1)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.13  Buffering period SEI message syntax
void
V3CWriter::bufferingPeriod(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIBufferingPeriod&>(seiAbstract);
  WRITE_CODE(sei.getNalHrdParamsPresentFlag(), 1);             // u(1)
  WRITE_CODE(sei.getAclHrdParamsPresentFlag(), 1);             // u(1)
  WRITE_CODE(sei.getInitialCabRemovalDelayLengthMinus1(), 5);  // u(5)
  WRITE_CODE(sei.getAuCabRemovalDelayLengthMinus1(), 5);       // u(5)
  WRITE_CODE(sei.getDabOutputDelayLengthMinus1(), 5);          // u(5)
  WRITE_CODE(sei.getIrapCabParamsPresentFlag(), 1);            // u(1)
  if (sei.getIrapCabParamsPresentFlag()) {
    WRITE_CODE(sei.getCabDelayOffset(),
               sei.getAuCabRemovalDelayLengthMinus1() + 1);  // u(v)
    WRITE_CODE(sei.getDabDelayOffset(),
               sei.getDabOutputDelayLengthMinus1() + 1);  // u(v)
  }
  WRITE_CODE(sei.getConcatenationFlag(), 1);  // u(1)
  WRITE_CODE(sei.getAtlasCabRemovalDelayDeltaMinus1(),
             sei.getAuCabRemovalDelayLengthMinus1() + 1);  // u(v)
  WRITE_CODE(sei.getMaxSubLayersMinus1(), 3);              // u(3)
  int32_t bitCount = sei.getInitialCabRemovalDelayLengthMinus1() + 1;
  for (size_t i = 0; i <= sei.getMaxSubLayersMinus1(); i++) {
    WRITE_CODE(sei.getHrdCabCntMinus1(i), 3);  // u(3)
    if (sei.getNalHrdParamsPresentFlag()) {
      for (size_t j = 0; j < sei.getHrdCabCntMinus1(i) + 1; j++) {
        WRITE_CODE(sei.getNalInitialCabRemovalDelay(i, j),
                   bitCount);  // u(v)
        WRITE_CODE(sei.getNalInitialCabRemovalOffset(i, j),
                   bitCount);  // u(v)
        if (sei.getIrapCabParamsPresentFlag()) {
          WRITE_CODE(sei.getNalInitialAltCabRemovalDelay(i, j),
                     bitCount);  // u(v)
          WRITE_CODE(sei.getNalInitialAltCabRemovalOffset(i, j),
                     bitCount);  // u(v)
        }
      }
    }
    if (sei.getAclHrdParamsPresentFlag()) {
      for (size_t j = 0; j < sei.getHrdCabCntMinus1(i) + 1; j++) {
        WRITE_CODE(sei.getAclInitialCabRemovalDelay(i, j),
                   bitCount);  // u(v)
        WRITE_CODE(sei.getAclInitialCabRemovalOffset(i, j),
                   bitCount);  // u(v)
        if (sei.getIrapCabParamsPresentFlag()) {
          WRITE_CODE(sei.getAclInitialAltCabRemovalDelay(i, j),
                     bitCount);  // u(v)
          WRITE_CODE(sei.getAclInitialAltCabRemovalOffset(i, j),
                     bitCount);  // u(v)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.14  Atlas frame timing SEI message syntax
void
V3CWriter::atlasFrameTiming(Bitstream& bitstream,
                            SEI&       seiAbstract,
                            SEI&       seiBufferingPeriodAbstract,
                            bool       cabDabDelaysPresentFlag) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei   = static_cast<SEIAtlasFrameTiming&>(seiAbstract);
  auto& bpsei = static_cast<SEIBufferingPeriod&>(seiBufferingPeriodAbstract);
  if (cabDabDelaysPresentFlag) {
    for (uint32_t i = 0; i <= bpsei.getMaxSubLayersMinus1(); i++) {
      WRITE_CODE(sei.getAftCabRemovalDelayMinus1(i),
                 bpsei.getAuCabRemovalDelayLengthMinus1() + 1);  // u(v)
      WRITE_CODE(sei.getAftDabOutputDelay(i),
                 bpsei.getDabOutputDelayLengthMinus1() + 1);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.15.1	Viewport camera parameters SEI messages syntax
void
V3CWriter::viewportCameraParameters(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIViewportCameraParameters&>(seiAbstract);
  WRITE_CODE(sei.getCameraId(), 10);   // u(10)
  WRITE_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (sei.getCameraId() > 0 && !sei.getCancelFlag()) {
    WRITE_CODE(sei.getPersistenceFlag(), 1);              // u(1)
    WRITE_CODE(sei.getCameraType(), 3);                   // u(3)
    if (sei.getCameraType() == 0) {                       // equirectangular
      WRITE_CODE(sei.getErpHorizontalFov(), 32);          // u(32)
      WRITE_CODE(sei.getErpVerticalFov(), 32);            // u(32)
    } else if (sei.getCameraType() == 1) {                // perspective
      WRITE_FLOAT(sei.getPerspectiveAspectRatio());       // fl(32)
      WRITE_CODE(sei.getPerspectiveHorizontalFov(), 32);  // u(32)
    } else if (sei.getCameraType() == 2) {                // orthographic
      WRITE_FLOAT(sei.getOrthoAspectRatio());             // fl(32)
      WRITE_FLOAT(sei.getOrthoHorizontalSize());          // fl(32)
    }
    WRITE_FLOAT(sei.getClippingNearPlane());  // fl(32)
    WRITE_FLOAT(sei.getClippingFarPlane());   // fl(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.15.2	Viewport position SEI messages syntax
void
V3CWriter::viewportPosition(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIViewportPosition&>(seiAbstract);
  WRITE_UVLC(sei.getViewportId());                      // ue(v
  WRITE_CODE(sei.getCameraParametersPresentFlag(), 1);  // u(1)
  if (sei.getCameraParametersPresentFlag()) {
    WRITE_CODE(sei.getViewportId(), 10);  //	u(10)
  }
  WRITE_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (!sei.getCancelFlag()) {
    WRITE_CODE(sei.getPersistenceFlag(), 1);  // u(1)
    for (size_t d = 0; d < 3; d++) {
      WRITE_FLOAT(sei.getPosition(d));  //	fl(32)
    }
    WRITE_CODE(sei.getRotationQX(), 16);     //	i(16)
    WRITE_CODE(sei.getRotationQY(), 16);     //	i(16)
    WRITE_CODE(sei.getRotationQZ(), 16);     //	i(16)
    WRITE_CODE(sei.getCenterViewFlag(), 1);  // 	u(1)
    if (!sei.getCenterViewFlag()) {
      WRITE_CODE(sei.getLeftViewFlag(), 1);  // u(1)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.16 Decoded Atlas Information Hash SEI message syntax
void
V3CWriter::decodedAtlasInformationHash(Bitstream& bitstream,
                                       SEI&       seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIDecodedAtlasInformationHash&>(seiAbstract);
  WRITE_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (!sei.getCancelFlag()) {
    WRITE_CODE(sei.getPersistenceFlag(), 1);                      // u(1)
    WRITE_CODE(sei.getHashType(), 8);                             // u(8)
    WRITE_CODE(sei.getDecodedHighLevelHashPresentFlag(), 1);      // u(1)
    WRITE_CODE(sei.getDecodedAtlasHashPresentFlag(), 1);          // u(1)
    WRITE_CODE(sei.getDecodedAtlasB2pHashPresentFlag(), 1);       // u(1)
    WRITE_CODE(sei.getDecodedAtlasTilesHashPresentFlag(), 1);     // u(1)
    WRITE_CODE(sei.getDecodedAtlasTilesB2pHashPresentFlag(), 1);  // u(1)
    WRITE_CODE(0, 1);                                             // u(1)
    if (sei.getDecodedHighLevelHashPresentFlag()) {
      decodedHighLevelHash(bitstream, sei);
    }
    if (sei.getDecodedAtlasHashPresentFlag()) {
      decodedAtlasHash(bitstream, sei);
    }
    if (sei.getDecodedAtlasB2pHashPresentFlag()) {
      decodedAtlasB2pHash(bitstream, sei);
    }
    if (sei.getDecodedAtlasTilesHashPresentFlag()
        || sei.getDecodedAtlasTilesB2pHashPresentFlag()) {
      WRITE_UVLC(sei.getNumTilesMinus1());   // ue(v)
      WRITE_UVLC(sei.getTileIdLenMinus1());  // ue(v)
      for (size_t t = 0; t <= sei.getNumTilesMinus1(); t++) {
        WRITE_CODE(sei.getTileId(t),
                   sei.getTileIdLenMinus1() + 1);  // u(v)
      }
      while (!bitstream.byteAligned()) {
        WRITE_CODE(1, 1);  // f(1): equal to 1
      }
      for (size_t t = 0; t <= sei.getNumTilesMinus1(); t++) {
        size_t j = sei.getTileId(t);
        if (sei.getDecodedAtlasTilesHashPresentFlag()) {
          decodedAtlasTilesHash(bitstream, sei, j);
        }
        if (sei.getDecodedAtlasTilesB2pHashPresentFlag()) {
          decodedAtlasTilesB2pHash(bitstream, sei, j);
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.16.1 Decoded high level hash unit syntax
void
V3CWriter::decodedHighLevelHash(Bitstream& bitstream, SEI& seiAbs) {
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      WRITE_CODE(sei.getHighLevelMd5(i), 8);  // b(8)
    }
  } else if (hType == 1) {
    WRITE_CODE(sei.getHighLevelCrc(), 16);  // u(16)
  } else if (hType == 2) {
    WRITE_CODE(sei.getHighLevelCheckSum(), 32);  // u(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.16.2 Decoded atlas hash unit syntax
void
V3CWriter::decodedAtlasHash(Bitstream& bitstream, SEI& seiAbs) {
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      WRITE_CODE(sei.getAtlasMd5(i), 8);  // b(8)
    }
  } else if (hType == 1) {
    WRITE_CODE(sei.getAtlasCrc(), 16);  // u(16)
  } else if (hType == 2) {
    WRITE_CODE(sei.getAtlasCheckSum(), 32);  // u(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.16.3 Decoded atlas b2p hash unit syntax
void
V3CWriter::decodedAtlasB2pHash(Bitstream& bitstream, SEI& seiAbs) {
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      WRITE_CODE(sei.getAtlasB2pMd5(i), 8);  // b(8)
    }
  } else if (hType == 1) {
    WRITE_CODE(sei.getAtlasB2pCrc(), 16);  // u(16)
  } else if (hType == 2) {
    WRITE_CODE(sei.getAtlasB2pCheckSum(), 32);  // u(32)
  }
}

// F.2.16.4 Decoded atlas tile hash unit syntax
void
V3CWriter::decodedAtlasTilesHash(Bitstream& bitstream,
                                 SEI&       seiAbs,
                                 size_t     id) {
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      WRITE_CODE(sei.getAtlasTilesMd5(id, i), 8);  // b(8)
    }
  } else if (hType == 1) {
    WRITE_CODE(sei.getAtlasTilesCrc(id), 16);  // u(16)
  } else if (hType == 2) {
    WRITE_CODE(sei.getAtlasTilesCheckSum(id), 32);  // u(32)
  }
}

// F.2.16.5 Decoded atlas tile b2p hash unit syntax

void
V3CWriter::decodedAtlasTilesB2pHash(Bitstream& bitstream,
                                    SEI&       seiAbs,
                                    size_t     id) {
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      WRITE_CODE(sei.getAtlasTilesB2pMd5(id, i), 8);  // b(8)
    }
  } else if (hType == 1) {
    WRITE_CODE(sei.getAtlasTilesB2pCrc(id), 16);  // u(16)
  } else if (hType == 2) {
    WRITE_CODE(sei.getAtlasTilesB2pCheckSum(id), 32);  // u(32)
  }
}

// F.2.17 Time code SEI message syntax
void
V3CWriter::timeCode(Bitstream& bitstream, SEI& seiAbstract) {
  auto& sei = static_cast<SEITimeCode&>(seiAbstract);
  WRITE_CODE(sei.getNumUnitsInTick(), 32);    // u(32)
  WRITE_CODE(sei.getTimeScale(), 32);         // u(32)
  WRITE_CODE(sei.getCountingType(), 5);       // u(5)
  WRITE_CODE(sei.getFullTimestampFlag(), 1);  // u(1)
  WRITE_CODE(sei.getDiscontinuityFlag(), 1);  // u(1)
  WRITE_CODE(sei.getCntDroppedFlag(), 1);     // u(1)
  WRITE_CODE(sei.getNFrames(), 9);            // u(9)
  if (sei.getFullTimestampFlag()) {
    WRITE_CODE(sei.getSecondsValue(), 6);  // u(6)
    WRITE_CODE(sei.getMinutesValue(), 6);  // u(6)
    WRITE_CODE(sei.getHoursValue(), 5);    // u(5)
  } else {
    WRITE_CODE(sei.getSecondFlag(), 1);  // u(1)
    if (sei.getSecondFlag()) {
      WRITE_CODE(sei.getSecondsValue(), 6);  // u(6)
      WRITE_CODE(sei.getMinutesFlag(), 1);   // u(1)
      if (sei.getMinutesFlag()) {
        WRITE_CODE(sei.getMinutesValue(), 6);  // u(6)
        WRITE_CODE(sei.getHoursFlag(), 1);     // u(1)
        if (sei.getHoursFlag()) {
          WRITE_CODE(sei.getHoursValue(), 5);  // u(5)
        }
      }
    }
  }
  WRITE_CODE(sei.getTimeOffsetLength(), 5);  // u(5)
  if (sei.getTimeOffsetLength() > 0) {
    WRITE_CODES(sei.getTimeOffsetValue(),
                sei.getTimeOffsetLength());  // i(v)
  }
}

// H.20.2.17 Attribute transformation parameters SEI message syntax
void
V3CWriter::attributeTransformationParams(Bitstream& bitstream,
                                         SEI&       seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIAttributeTransformationParams&>(seiAbstract);
  WRITE_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (!sei.getCancelFlag()) {
    WRITE_UVLC(sei.getNumAttributeUpdates());  // ue(v)
    for (size_t j = 0; j < sei.getNumAttributeUpdates(); j++) {
      WRITE_CODE(sei.getAttributeIdx(j), 8);  // u(8)
      size_t index = sei.getAttributeIdx(j);
      WRITE_CODE(sei.getDimensionMinus1(index), 8);  // u(8)
      for (size_t i = 0; i < sei.getDimensionMinus1(index); i++) {
        WRITE_CODE(sei.getScaleParamsEnabledFlag(index, i), 1);   // u(1)
        WRITE_CODE(sei.getOffsetParamsEnabledFlag(index, i), 1);  // u(1)
        if (sei.getScaleParamsEnabledFlag(index, i)) {
          WRITE_CODE(sei.getAttributeScale(index, i), 32);  // u(32)
        }
        if (sei.getOffsetParamsEnabledFlag(index, i)) {
          WRITE_CODES(sei.getAttributeOffset(index, i), 32);  // i(32)
        }
      }
    }
    WRITE_CODE(sei.getPersistenceFlag(), 1);  // u(1)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.20.2.18 Occupancy synthesis SEI message syntax
void
V3CWriter::occupancySynthesis(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIOccupancySynthesis&>(seiAbstract);
  WRITE_CODE(sei.getPersistenceFlag(), 1);   //	u(1)
  WRITE_CODE(sei.getResetFlag(), 1);         //	u(1)
  WRITE_CODE(sei.getInstancesUpdated(), 8);  //	u(8)
  for (size_t i = 0; i < sei.getInstancesUpdated(); i++) {
    WRITE_CODE(sei.getInstanceIndex(i), 8);  // u(8)
    size_t k = sei.getInstanceIndex(i);
    WRITE_CODE(sei.getInstanceCancelFlag(k), 1);  //	u(1)
    if (!sei.getInstanceCancelFlag(k)) {
      WRITE_UVLC(sei.getMethodType(k));  // ue(v)
      if (sei.getMethodType(k) == 1) {
        WRITE_CODE(sei.getPbfLog2ThresholdMinus1(k), 2);  //	u(2)
        WRITE_CODE(sei.getPbfPassesCountMinus1(k), 2);    //	u(2)
        WRITE_CODE(sei.getPbfFilterSizeMinus1(k), 3);     //	u(3)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.20.2.19 Geometry smoothing SEI message syntax
void
V3CWriter::geometrySmoothing(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIGeometrySmoothing&>(seiAbstract);
  WRITE_CODE(sei.getPersistenceFlag(), 1);   //	u(1)
  WRITE_CODE(sei.getResetFlag(), 1);         //	u(1)
  WRITE_CODE(sei.getInstancesUpdated(), 8);  //	u(8)
  for (size_t i = 0; i < sei.getInstancesUpdated(); i++) {
    WRITE_CODE(sei.getInstanceIndex(i), 8);  // u(8)
    size_t k = sei.getInstanceIndex(i);
    WRITE_CODE(sei.getInstanceCancelFlag(k), 1);  //	u(1)
    if (!sei.getInstanceCancelFlag(k)) {
      WRITE_UVLC(sei.getMethodType(k));  // ue(v)
      if (sei.getMethodType(k) == 1) {
        WRITE_CODE(sei.getFilterEomPointsFlag(k), 1);  // u(1)
        WRITE_CODE(sei.getGridSizeMinus2(k), 7);       // u(7)
        WRITE_CODE(sei.getThreshold(k), 8);            // u(8)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.20.2.20 Attribute smoothing SEI message syntax
void
V3CWriter::attributeSmoothing(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIAttributeSmoothing&>(seiAbstract);
  WRITE_CODE(sei.getPersistenceFlag(), 1);    //	u(1)
  WRITE_CODE(sei.getResetFlag(), 1);          //	u(1)
  WRITE_UVLC(sei.getNumAttributesUpdated());  //	ue(v)
  for (size_t j = 0; j < sei.getNumAttributesUpdated(); j++) {
    WRITE_CODE(sei.getAttributeIdx(j), 7);  // u(7)
    size_t k = sei.getAttributeIdx(j);
    WRITE_CODE(sei.getAttributeSmoothingCancelFlag(k), 1);  // u(1)
    WRITE_CODE(sei.getInstancesUpdated(k), 8);              //	u(8)
    for (size_t i = 0; i < sei.getInstancesUpdated(k); i++) {
      WRITE_CODE(sei.getInstanceIndex(k, i), 8);  //	u(8)
      size_t m = sei.getInstanceIndex(k, i);
      WRITE_CODE(sei.getInstanceCancelFlag(k, m), 1);  // u(1)
      if (sei.getInstanceCancelFlag(k, m) != 1) {
        WRITE_UVLC(sei.getMethodType(k, m));  // ue(v)
        if (sei.getMethodType(k, m)) {
          WRITE_CODE(sei.getFilterEomPointsFlag(k, m), 1);  // u(1)
          WRITE_CODE(sei.getGridSizeMinus2(k, m), 5);       //	u(5)
          WRITE_CODE(sei.getThreshold(k, m), 8);            // u(8)
          WRITE_CODE(sei.getThresholdVariation(k, m), 8);   // u(8)
          WRITE_CODE(sei.getThresholdDifference(k, m), 8);  // u(8)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2 VUI syntax
// G.2.1 VUI parameters syntax
void
V3CWriter::vuiParameters(Bitstream& bitstream, VUIParameters& vp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(vp.getTimingInfoPresentFlag(), 1);  // u(1)
  if (vp.getTimingInfoPresentFlag()) {
    WRITE_CODE(vp.getNumUnitsInTick(), 32);              // u(32)
    WRITE_CODE(vp.getTimeScale(), 32);                   // u(32)
    WRITE_CODE(vp.getPocProportionalToTimingFlag(), 1);  // u(1)
    if (vp.getPocProportionalToTimingFlag()) {
      WRITE_UVLC(vp.getNumTicksPocDiffOneMinus1());  // ue(v)
    }
    WRITE_CODE(vp.getHrdParametersPresentFlag(), 1);  // u(1)
    if (vp.getHrdParametersPresentFlag()) {
      hrdParameters(bitstream, vp.getHrdParameters());
    }
  }
  WRITE_CODE(vp.getTileRestrictionsPresentFlag(), 1);  // u(1)
  if (vp.getTileRestrictionsPresentFlag()) {
    WRITE_CODE(vp.getFixedAtlasTileStructureFlag(), 1);          // u(1)
    WRITE_CODE(vp.getFixedVideoTileStructureFlag(), 1);          //	u(1)
    WRITE_UVLC(vp.getConstrainedTilesAcrossV3cComponentsIdc());  // ue(v)
    WRITE_UVLC(vp.getMaxNumTilesPerAtlasMinus1());               // 	ue(v)
  }
  WRITE_CODE(vp.getMaxCodedVideoResolutionPresentFlag(), 1);  // u(1)
  if (vp.getMaxCodedVideoResolutionPresentFlag()) {
    maxCodedVideoResolution(bitstream, vp.getMaxCodedVideoResolution());
  }
  WRITE_CODE(vp.getCoordinateSystemParametersPresentFlag(), 1);  // u(1)
  if (vp.getCoordinateSystemParametersPresentFlag()) {
    coordinateSystemParameters(bitstream, vp.getCoordinateSystemParameters());
  }
  WRITE_CODE(vp.getUnitInMetresFlag(), 1);           // u(1)
  WRITE_CODE(vp.getDisplayBoxInfoPresentFlag(), 1);  // u(1)
  if (vp.getDisplayBoxInfoPresentFlag()) {
    for (size_t d = 0; d < 3; d++) {
      WRITE_UVLC(vp.getDisplayBoxOrigin(d));  // ue(v)
      WRITE_UVLC(vp.getDisplayBoxSize(d));    // ue(v)
    }
    WRITE_CODE(vp.getAnchorPointPresentFlag(), 1);  // u(1)
    if (vp.getAnchorPointPresentFlag()) {
      for (size_t d = 0; d < 3; d++) {
        WRITE_UVLC(vp.getAnchorPoint(d));  // u(v)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2.2  HRD parameters syntax
void
V3CWriter::hrdParameters(Bitstream& bitstream, HrdParameters& hp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(hp.getNalParametersPresentFlag(), 1);  // u(1)
  WRITE_CODE(hp.getAclParametersPresentFlag(), 1);  // u(1)
  if (hp.getNalParametersPresentFlag() || hp.getAclParametersPresentFlag()) {
    WRITE_CODE(hp.getBitRateScale(), 4);  // u(4)
    WRITE_CODE(hp.getCabSizeScale(), 4);  // u(4)
  }
  for (size_t i = 0; i <= hp.getMaxNumSubLayersMinus1(); i++) {
    WRITE_CODE(hp.getFixedAtlasRateGeneralFlag(i), 1);  // u(1)
    if (!hp.getFixedAtlasRateGeneralFlag(i)) {
      WRITE_CODE(hp.getFixedAtlasRateWithinCasFlag(i), 1);  // u(1)
    }
    if (hp.getFixedAtlasRateWithinCasFlag(i)) {
      WRITE_CODE(hp.getElementalDurationInTcMinus1(i), 1);  // ue(v)
    } else {
      WRITE_CODE(hp.getLowDelayFlag(i), 1);  // u(1)
    }
    if (!hp.getLowDelayFlag(i)) {
      WRITE_CODE(hp.getCabCntMinus1(i), 1);  // ue(v)
    }
    if (hp.getNalParametersPresentFlag()) {
      hrdSubLayerParameters(
        bitstream, hp.getHdrSubLayerParameters(0, i), hp.getCabCntMinus1(i));
    }
    if (hp.getAclParametersPresentFlag()) {
      hrdSubLayerParameters(
        bitstream, hp.getHdrSubLayerParameters(1, i), hp.getCabCntMinus1(i));
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2.3  Sub-layer HRD parameters syntax
void
V3CWriter::hrdSubLayerParameters(Bitstream&             bitstream,
                                 HrdSubLayerParameters& hlsp,
                                 size_t                 cabCnt) {
  TRACE_BITSTREAM_IN("%s", __func__);
  for (size_t i = 0; i <= cabCnt; i++) {
    WRITE_UVLC(hlsp.getBitRateValueMinus1(i));  // ue(v)
    WRITE_UVLC(hlsp.getCabSizeValueMinus1(i));  // ue(v)
    WRITE_CODE(hlsp.getCbrFlag(i), 1);          // u(1)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2.4 Maximum coded video resolution syntax
void
V3CWriter::maxCodedVideoResolution(Bitstream&               bitstream,
                                   MaxCodedVideoResolution& mcvr) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(mcvr.getOccupancyResolutionPresentFlag(), 1);  // u(1)
  WRITE_CODE(mcvr.getGeometryResolutionPresentFlag(), 1);   // u(1)
  WRITE_CODE(mcvr.getAttributeResolutionPresentFlag(), 1);  // u(1)
  if (mcvr.getOccupancyResolutionPresentFlag()) {
    WRITE_UVLC(mcvr.getOccupancyWidth());   // ue(v)
    WRITE_UVLC(mcvr.getOccupancyHeight());  // ue(v)
  }
  if (mcvr.getGeometryResolutionPresentFlag()) {
    WRITE_UVLC(mcvr.getGeometryWidth());   // ue(v)
    WRITE_UVLC(mcvr.getGeometryHeight());  // ue(v)
  }
  if (mcvr.getAttributeResolutionPresentFlag()) {
    WRITE_UVLC(mcvr.getAttributeWidth());   // ue(v)
    WRITE_UVLC(mcvr.getAttributeHeight());  // ue(v)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2.5 Coordinate system parameters syntax
void
V3CWriter::coordinateSystemParameters(Bitstream&                  bitstream,
                                      CoordinateSystemParameters& csp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(csp.getForwardAxis(), 2);    // u(2)
  WRITE_CODE(csp.getDeltaLeftAxis(), 1);  // u(1)
  WRITE_CODE(csp.getForwardSign(), 1);    // u(1)
  WRITE_CODE(csp.getLeftSign(), 1);       // u(1)
  WRITE_CODE(csp.getUpSign(), 1);         // u(1)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.4.1	VPS V-PCC extension syntax
void
V3CWriter::vpsVpccExtension(Bitstream& bitstream, VpsVpccExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.1.1	ASPS V-PCC extension syntax
void
V3CWriter::aspsVpccExtension(Bitstream&                     bitstream,
                             AtlasSequenceParameterSetRbsp& asps,
                             AspsVpccExtension&             ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(ext.getRemoveDuplicatePointEnableFlag(), 1);  // u(1)
  if (asps.getPixelDeinterleavingFlag() || asps.getPLREnabledFlag()) {
    WRITE_CODE(ext.getSurfaceThicknessMinus1(), 7);  // u(?)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.1.2	ASPS MIV extension syntax
void
V3CWriter::aspsMivExtension(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.2.1	AFPS V-PCC extension syntax
void
V3CWriter::afpsVpccExtension(Bitstream& bitstream, AfpsVpccExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.2.1	AAPS V-PCC extension syntax
void
V3CWriter::aapsVpccExtension(Bitstream& bitstream, AapsVpccExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(ext.getCameraParametersPresentFlag(), 1);  // u(1);
  if (ext.getCameraParametersPresentFlag()) {
    atlasCameraParameters(bitstream, ext.getAtlasCameraParameters());
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.2.2	Atlas camera parameters syntax
void
V3CWriter::atlasCameraParameters(Bitstream&             bitstream,
                                 AtlasCameraParameters& acp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(acp.getCameraModel(), 8);  // u(8)
  if (acp.getCameraModel() == 1) {
    WRITE_CODE(acp.getScaleEnabledFlag(), 1);     // u(1)
    WRITE_CODE(acp.getOffsetEnabledFlag(), 1);    // u(1)
    WRITE_CODE(acp.getRotationEnabledFlag(), 1);  // u(1)
    if (acp.getScaleEnabledFlag()) {
      for (size_t i = 0; i < 3; i++) {
        WRITE_CODE(acp.getScaleOnAxis(i), 32);
      }  // u(32)
    }
    if (acp.getOffsetEnabledFlag()) {
      for (size_t i = 0; i < 3; i++) {
        WRITE_CODES(acp.getOffsetOnAxis(i), 32);
      }  // i(32)
    }
    if (acp.getRotationEnabledFlag()) {
      for (size_t i = 0; i < 3; i++) {
        WRITE_CODES(acp.getRotation(i), 16);
      }  // i(16)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}
