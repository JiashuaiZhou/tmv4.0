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
#include "videoBitstream.hpp"
#include "bitstream.hpp"
#include "v3cBitstream.hpp"
#include "atlasAdaptationParameterSetRbsp.hpp"
#include "rbsp.hpp"
#include "v3cReader.hpp"

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

#  define READ_CODE(VAR, LEN) \
    { \
      VAR = bitstream.read(LEN); \
      bitstream.traceBit( \
        format(#VAR), VAR, "u(" + std::to_string(LEN) + ")"); \
    }

#  define READ_CODE_CAST(VAR, LEN, CAST) \
    { \
      VAR = static_cast<CAST>(bitstream.read(LEN)); \
      bitstream.traceBit( \
        format(#VAR), VAR, "u(" + std::to_string(LEN) + ")"); \
    }

#  define READ_CODES(VAR, LEN) \
    { \
      VAR = bitstream.readS(LEN); \
      bitstream.traceBit( \
        format(#VAR), VAR, "s(" + std::to_string(LEN) + ")"); \
    }

#  define READ_UVLC(VAR) \
    { \
      VAR = bitstream.readUvlc(); \
      bitstream.traceBit(format(#VAR), VAR, "ue(v)"); \
    }
#  define READ_UVLC_CAST(VAR, CAST) \
    { \
      VAR = static_cast<CAST>(bitstream.readUvlc()); \
      bitstream.traceBit(format(#VAR), VAR, "ue(v)"); \
    }

#  define READ_SVLC(VAR) \
    { \
      VAR = bitstream.readSvlc(); \
      bitstream.traceBit(format(#VAR), VAR, "se(v)"); \
    }

#  define READ_FLOAT(VAR) \
    { \
      VAR = bitstream.readFloat(); \
      bitstream.traceBitFloat(format(#VAR), VAR, "f(32)"); \
    }

#  define READ_STRING(VAR) \
    { \
      VAR = bitstream.readString(); \
      bitstream.traceBitStr(format(#VAR), VAR, "str()"); \
    }

#  define READ_VECTOR(VAR, SIZE) \
    { \
      bitstream.read(VAR, SIZE); \
      bitstream.trace( \
        "Data stream: %s size = %zu \n", format(#VAR).c_str(), VAR.size()); \
    }
#  define READ_VIDEO(VAR, SIZE) \
    { \
      bitstream.read(VAR, SIZE); \
      bitstream.trace("Video stream: type = %s size = %zu \n", \
                      toString(VAR.type()).c_str(), \
                      SIZE); \
    }
#else
#  define READ_CODE(VAR, LEN) VAR = bitstream.read(LEN);
#  define READ_CODE_CAST(VAR, LEN, CAST) \
    VAR = static_cast<CAST>(bitstream.read(LEN));
#  define READ_CODES(VAR, LEN) VAR = bitstream.readS(LEN);
#  define READ_UVLC(VAR) VAR = bitstream.readUvlc();
#  define READ_UVLC_CAST(VAR, CAST) \
    VAR = static_cast<CAST>(bitstream.readUvlc());
#  define READ_SVLC(VAR) VAR = bitstream.readSvlc();
#  define READ_FLOAT(VAR) VAR = bitstream.readFloat();
#  define READ_STRING(VAR) VAR = bitstream.readString();
#  define READ_VECTOR(VAR, SIZE) bitstream.read(VAR, SIZE);
#  define READ_VIDEO(VAR, SIZE) bitstream.read(VAR, SIZE);
#endif

V3CReader::V3CReader() {}
V3CReader::~V3CReader() = default;

// B.2  Sample stream V3C unit syntax
size_t
V3CReader::read(Bitstream& bitstream, SampleStreamV3CUnit& ssvu) {
  size_t headerSize = 0;
  TRACE_BITSTREAM_IN("%s", "SampleStreamV3CUnits");
  sampleStreamV3CHeader(bitstream, ssvu);
  headerSize++;
  size_t unitCount = 0;
  while (bitstream.moreData()) {
    auto& v3cUnit = ssvu.addV3CUnit();
    sampleStreamV3CUnit(bitstream, ssvu, v3cUnit);
    unitCount++;
    headerSize += ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1;
  }
  TRACE_BITSTREAM_OUT("%s", "SampleStreamV3CUnits");
  return headerSize;
}

int32_t
V3CReader::decode(SampleStreamV3CUnit& ssvu, V3cBitstream& syntax) {
  int   numVPS       = 0;  // counter for the atlas information
  auto& bistreamStat = syntax.getBitstreamStat();
  bistreamStat.newGOF();
  while (ssvu.getV3CUnitCount() > 0) {
    auto& v3cNalu     = ssvu.front();
    auto  v3cUnitType = NUM_V3C_UNIT_TYPE;
    if (v3cNalu.getType() == V3C_VPS && (++numVPS > 1)) break;
    v3cUnit(syntax, v3cNalu, v3cUnitType);
    ssvu.popFront();
  }
  return 1;
}

void
V3CReader::videoSubStream(V3cBitstream& syntax,
                          Bitstream&    bitstream,
                          V3CUnitType&  V3CUnitType,
                          size_t        v3cPayloadSize) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& bistreamStat = syntax.getBitstreamStat();
  auto& video        = syntax.createVideoBitstream(V3CUnitType);
  READ_VIDEO(video, v3cPayloadSize);
  bistreamStat.setVideo(video);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.2 V3C unit syntax
// 8.3.2.1 General V3C unit syntax
void
V3CReader::v3cUnit(V3cBitstream& syntax,
                   V3CUnit&      v3cNalu,
                   V3CUnitType&  v3cUnitType) {
  Bitstream& bitstream = v3cNalu.getBitstream();
#if defined(BITSTREAM_TRACE)
  bitstream.setTrace(true);
  bitstream.setLogger(*logger_);
#endif
  TRACE_BITSTREAM_IN(
    "%s(%s)", __func__, toString(v3cNalu.getType()).c_str());
  auto position = static_cast<int32_t>(bitstream.size());
  v3cUnitHeader(syntax, bitstream, v3cUnitType);
  assert(v3cUnitType == v3cNalu.getType());
  v3cUnitPayload(syntax, bitstream, v3cUnitType, v3cNalu.getSize());
  syntax.getBitstreamStat().setV3CUnitSize(
    v3cUnitType, static_cast<int32_t>(bitstream.size()) - position);
  TRACE_BITSTREAM_OUT(
    "%s(%s)", __func__, toString(v3cNalu.getType()).c_str());    
}

// 8.3.2.2 V3C unit header syntax
void
V3CReader::v3cUnitHeader(V3cBitstream& syntax,
                         Bitstream&    bitstream,
                         V3CUnitType&  v3cUnitType) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  READ_CODE_CAST(v3cUnitType, 5, V3CUnitType);  // u(5)
  if (v3cUnitType == V3C_AVD || v3cUnitType == V3C_GVD
      || v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD
      || v3cUnitType == V3C_BMD) {
    auto& v3cUnitHeader = syntax.getV3CUnitHeader(v3cUnitType);
    READ_CODE(v3cUnitHeader.getV3CParameterSetId(), 4);  // u(4)
    READ_CODE(v3cUnitHeader.getAtlasId(), 6);            // u(6)
    syntax.getActiveVpsId() = v3cUnitHeader.getV3CParameterSetId();
    syntax.getAtlasIndex()  = v3cUnitHeader.getAtlasId();
  }
  if (v3cUnitType == V3C_AVD) {
    auto& v3cUnitHeader = syntax.getV3CUnitHeader(v3cUnitType);
    READ_CODE(v3cUnitHeader.getAttributeIndex(), 7);           // u(7)
    READ_CODE(v3cUnitHeader.getAttributeDimensionIndex(), 5);  // u(5)
    READ_CODE(v3cUnitHeader.getMapIndex(), 4);                 // u(4)
    READ_CODE(v3cUnitHeader.getAuxiliaryVideoFlag(), 1);       // u(1)
  } else if (v3cUnitType == V3C_GVD) {
    auto& vpcc = syntax.getV3CUnitHeader(v3cUnitType);
    READ_CODE(vpcc.getMapIndex(), 4);            // u(4)
    READ_CODE(vpcc.getAuxiliaryVideoFlag(), 1);  // u(1)
    READ_CODE(zero, 12);                         // u(12)
  } else if (v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD
             || v3cUnitType == V3C_BMD) {
    READ_CODE(zero, 17);  // u(17)
  } else {
    READ_CODE(zero, 27);  // u(27)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.2.3 V3C unit payload syntax
void
V3CReader::v3cUnitPayload(V3cBitstream& syntax,
                          Bitstream&    bitstream,
                          V3CUnitType&  v3cUnitType,
                          size_t        v3cUnitSize) {
  TRACE_BITSTREAM_IN("%s(%s)", __func__, toString(v3cUnitType).c_str());
  if (v3cUnitType == V3C_VPS) {
    auto& vps = syntax.addV3CParameterSet();
    v3cParameterSet(vps, syntax, bitstream);
  } else if (v3cUnitType == V3C_AD) {
    atlasSubStream(syntax, bitstream);
  } else if (v3cUnitType == V3C_BMD) {
    baseMeshSubStream(syntax, bitstream);
  } else if (v3cUnitType == V3C_OVD || v3cUnitType == V3C_GVD
             || v3cUnitType == V3C_AVD) {
    videoSubStream(syntax, bitstream, v3cUnitType, v3cUnitSize - 4);
  }
  TRACE_BITSTREAM_OUT("%s(%s)", __func__, toString(v3cUnitType).c_str());
}

// Base mesh sub-bitstream syntax
void
V3CReader::baseMeshSubStream(V3cBitstream& syntax, Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  size_t              sizeBitstream = bitstream.capacity();
  SampleStreamNalUnit ssnu;
  PCCSEI              sei;
  sampleStreamNalHeader(bitstream, ssnu);
  while (bitstream.size() < sizeBitstream) {
    ssnu.addNalUnit();
    sampleStreamNalUnit(
      syntax, false, bitstream, ssnu, ssnu.getNalUnitCount() - 1, sei);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh sequence parameter set Rbsp
void
V3CReader::baseMeshSequenceParameterSetRbsp(
  BaseMeshSequenceParameterSetRbsp& bmsps,
  V3cBitstream&                     syntax,
  Bitstream&                        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_UVLC(bmsps.getBaseMeshSequenceParameterSetId());         // ue(v)
  READ_UVLC(bmsps.getBaseMeshCodecId());                        // ue(v)
  READ_UVLC(bmsps.getQpPositionMinus1());                       // ue(v)
  READ_UVLC(bmsps.getQpTexCoordMinus1());                       // ue(v)
  READ_UVLC(bmsps.getLog2MaxBaseMeshFrameOrderCntLsbMinus4());  // ue(v)
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh frame parameter set Rbsp
void
V3CReader::baseMeshFrameParameterSetRbsp(BaseMeshFrameParameterSetRbsp& bmfps,
                                         V3cBitstream&                  syntax,
                                         Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  READ_UVLC(bmfps.getBaseMeshFrameParameterSetId());     // ue(v)
  READ_UVLC(bmfps.getBaseMeshSequenceParameterSetId());  // ue(v)
  auto& baseMesh = syntax.getBaseMesh();
  auto  bmspsId  = bmfps.getBaseMeshSequenceParameterSetId();
  auto& bmsps    = baseMesh.getBaseMeshSequenceParameterSet(bmspsId);
  auto& bmfi     = bmfps.getBaseMeshFrameTileInformation();
  baseMeshFrameTileInformation(bmfi, bmsps, bitstream);
  READ_CODE(bmfps.getOutputFlagPresentFlag(), 1);      // u(1)
  READ_UVLC(bmfps.getNumRefIdxDefaultActiveMinus1());  // ue(v)
  READ_UVLC(bmfps.getAdditionalLtAfocLsbLen());        // ue(v)
  READ_CODE(bmfps.getMotionGroupSize(), 8)             // u(8)
  READ_CODE(bmfps.getExtensionFlag(), 1);              // u(1)
  if (bmfps.getExtensionFlag()) {
    READ_CODE(bmfps.getExtension8Bits(), 8);  // u(8)
  }
  if (bmfps.getExtension8Bits()) {
    while (moreRbspData(bitstream)) { READ_CODE(zero, 1); }  // u(1)
  }
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh frame tile information syntax
void
V3CReader::baseMeshFrameTileInformation(
  BaseMeshFrameTileInformation&     bmfti,
  BaseMeshSequenceParameterSetRbsp& bmsps,
  Bitstream&                        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(bmfti.getSingleTileInBaseMeshFrameFlag(), 1);  // u(1)
  if (!bmfti.getSingleTileInBaseMeshFrameFlag()) {
    // TODO
  }
  READ_CODE(bmfti.getSignalledTileIdFlag(), 1);  // u(1)
  if (bmfti.getSignalledTileIdFlag()) {
    READ_UVLC(bmfti.getSignalledTileIdLengthMinus1());  // ue(v)
    for (size_t i = 0; i <= bmfti.getNumTilesInBaseMeshFrameMinus1(); i++) {
      uint8_t bitCount = bmfti.getSignalledTileIdLengthMinus1() + 1;
      READ_CODE(bmfti.getTileId(i), bitCount);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Base mesh tile layer
void
V3CReader::baseMeshTileLayerRbsp(BaseMeshTileLayer&  bmtl,
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

// Base mesh tile header
void
V3CReader::baseMeshTileHeader(BaseMeshTileHeader& bmth,
                              BaseMeshNalUnitType nalUnitType,
                              V3cBitstream&       syntax,
                              Bitstream&          bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  if (nalUnitType >= BASEMESH_NAL_BLA_W_LP
      && nalUnitType <= BASEMESH_NAL_RSV_BMCL_29) {
    READ_CODE(bmth.getNoOutputOfPriorBaseMeshFramesFlag(), 1);  // u(1)
  }
  READ_UVLC(bmth.getBaseMeshFrameParameterSetId());       // ue(v)
  READ_UVLC(bmth.getBaseMeshAdaptationParameterSetId());  // ue(v)
  auto&  baseMesh = syntax.getBaseMesh();
  size_t bmfpsId  = bmth.getBaseMeshFrameParameterSetId();
  auto&  bmfps    = baseMesh.getBaseMeshFrameParameterSet(bmfpsId);
  size_t bmspsId  = bmfps.getBaseMeshSequenceParameterSetId();
  auto&  bmsps    = baseMesh.getBaseMeshSequenceParameterSet(bmspsId);
  auto&  bmfti    = bmfps.getBaseMeshFrameTileInformation();
  if (bmfti.getSignalledTileIdFlag()) {
    READ_CODE(bmth.getBaseMeshId(),
              bmfti.getSignalledTileIdLengthMinus1() + 1);  // u(v)
  } else {
    if (bmfti.getNumTilesInBaseMeshFrameMinus1() != 0) {
      READ_CODE(
        bmth.getBaseMeshId(),
        ceilLog2(bmfti.getNumTilesInBaseMeshFrameMinus1() + 1));  // u(v)
    } else {
      bmth.getBaseMeshId() = 0;
    }
  }
  READ_UVLC_CAST(bmth.getBaseMeshType(), BaseMeshType);  // ue(v)
  if (bmfps.getOutputFlagPresentFlag()) {
    READ_CODE(bmth.getBaseMeshOutputFlag(), 1);  // u(1)
  } else {
    bmth.getBaseMeshOutputFlag() = false;
  }
  READ_CODE(bmth.getBaseMeshFrmOrderCntLsb(),
            bmsps.getLog2MaxBaseMeshFrameOrderCntLsbMinus4() + 4);  // u(v)
  if (bmsps.getNumRefBaseMeshFrameListsInAsps() > 0) {
    READ_CODE(bmth.getRefBaseMeshFrameListSpsFlag(), 1);  // u(1)
  } else {
    bmth.getRefBaseMeshFrameListSpsFlag() = false;
  }
  bmth.getRefBaseMeshFrameListIdx() = 0;
  if (static_cast<int>(bmth.getRefBaseMeshFrameListSpsFlag()) == 0) {
    refListStruct(bmth.getRefListStruct(), bmsps, bitstream);
  } else if (bmsps.getNumRefBaseMeshFrameListsInAsps() > 1) {
    size_t bitCount = ceilLog2(bmsps.getNumRefBaseMeshFrameListsInAsps());
    READ_CODE(bmth.getRefBaseMeshFrameListIdx(), bitCount);  // u(v)
  }
  if (bmth.getRefBaseMeshFrameListSpsFlag()) {
    bmth.getRefListStruct() =
      bmsps.getRefListStruct(bmth.getRefBaseMeshFrameListIdx());
  }
  uint8_t rlsIdx                   = bmth.getRefBaseMeshFrameListIdx();
  auto&   refList                  = bmth.getRefBaseMeshFrameListSpsFlag()
                                       ? bmsps.getRefListStruct(rlsIdx)
                                       : bmth.getRefListStruct();
  size_t  numLtrBaseMeshFrmEntries = 0;
  for (size_t i = 0; i < refList.getNumRefEntries(); i++) {
    if (!refList.getStRefAtlasFrameFlag(i)) { numLtrBaseMeshFrmEntries++; }
  }
  for (size_t j = 0; j < numLtrBaseMeshFrmEntries; j++) {
    READ_CODE(bmth.getAdditionalAfocLsbPresentFlag(j), 1);  // u(1)
    if (bmth.getAdditionalAfocLsbPresentFlag(j)) {
      uint8_t bitCount = bmfps.getAdditionalLtAfocLsbLen();
      READ_CODE(bmth.getAdditionalAfocLsbVal(j), bitCount);  // u(v)
    }
  }
  if (bmth.getBaseMeshType() == P_BASEMESH) {
    if (refList.getNumRefEntries() > 1) {
      READ_CODE(bmth.getNumRefIdxActiveOverrideFlag(), 1);  // u(1)
      if (bmth.getNumRefIdxActiveOverrideFlag()) {
        READ_UVLC(bmth.getNumRefIdxActiveMinus1());  // ue(v)
      }
    }
  }
  byteAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// Reference list structure syntax
void
V3CReader::refListStruct(RefListStruct&                    rls,
                         BaseMeshSequenceParameterSetRbsp& asps,
                         Bitstream&                        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_UVLC(rls.getNumRefEntries());  // ue(v)
  rls.allocate();
  for (size_t i = 0; i < rls.getNumRefEntries(); i++) {
    if (asps.getLongTermRefBaseMeshFramesFlag()) {
      READ_CODE(rls.getStRefAtlasFrameFlag(i), 1);  // u(1)
    } else {
      rls.getStRefAtlasFrameFlag(i) = true;
    }
    if (rls.getStRefAtlasFrameFlag(i)) {
      READ_UVLC(rls.getAbsDeltaAfocSt(i));  // ue(v)
      if (rls.getAbsDeltaAfocSt(i) > 0) {
        READ_CODE(rls.getStrafEntrySignFlag(i), 1);  // u(1)
      } else {
        rls.getStrafEntrySignFlag(i) = true;
      }
    } else {
      uint8_t bitCount = asps.getLog2MaxBaseMeshFrameOrderCntLsbMinus4() + 4;
      READ_CODE(rls.getAfocLsbLt(i), bitCount);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// base mesh tile data unit
void
V3CReader::baseMeshTileDataUnit(BaseMeshTileDataUnit& bmtdu,
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
V3CReader::baseMeshIntraTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                     BaseMeshTileHeader&   bmth,
                                     V3cBitstream&         syntax,
                                     Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&    bistreamStat = syntax.getBitstreamStat();
  uint32_t size         = bitstream.capacity() - bitstream.size() - 1;
  READ_VECTOR(bmtdu.getData().vector(), size);
  bistreamStat.setBaseMesh(I_BASEMESH, bmtdu.getData().vector().size());
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// base mesh inter tile data unit
void
V3CReader::baseMeshInterTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                     BaseMeshTileHeader&   bmth,
                                     V3cBitstream&         syntax,
                                     Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& bistreamStat = syntax.getBitstreamStat();
  READ_CODE(bmtdu.getMotionSkipFlag(), 1);  //u1
  if (bmtdu.getMotionSkipFlag()) {
    READ_CODE(bmtdu.getMotionSkipAll(), 1);  //u1
    if (!bmtdu.getMotionSkipAll()) {
      READ_CODE(bmtdu.getMotionSkipCount(), 7);  //u7
      bmtdu.getMotionSkipVextexIndices().resize(bmtdu.getMotionSkipCount());
      for (size_t i = 0; i < bmtdu.getMotionSkipCount(); i++)
        READ_CODE(bmtdu.getMotionSkipVextexIndices(i), 7);  //u7
    }
  }
  lengthAlignment(bitstream);
  uint32_t size = bitstream.capacity() - bitstream.size() - 1;
  READ_VECTOR(bmtdu.getData().vector(), size);
  bistreamStat.setBaseMesh(P_BASEMESH, bmtdu.getData().vector().size());
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// base mesh tile data unit
void
V3CReader::baseMeshSkipTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                    BaseMeshTileHeader&   bmth,
                                    V3cBitstream&         syntax,
                                    Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t size = bitstream.capacity() - bitstream.size() - 1;
  READ_VECTOR(bmtdu.getData().vector(), size);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// V3C VPS extension
void
V3CReader::vpsExtension(V3CParameterSet& vps,
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
    vps.getExtensionDataByte(index).resize(vps.getExtensionLength(index));
    for (size_t i = 0; i < vps.getExtensionLength(index); i++) {
      uint32_t data = 0;
      READ_CODE(data, 8);  // u(8)
      vps.setExtensionDataByte(index, i, data);
    }
  }
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// V3C VPS VDMC extension
void
V3CReader::vpsVdmcExtension(V3CParameterSet&  vps,
                            VpsVdmcExtension& ext,
                            Bitstream&        bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// ASPS VDMC extension syntax
void
V3CReader::aspsVdmcExtension(Bitstream&                     bitstream,
                             AtlasSequenceParameterSetRbsp& asps,
                             AspsVdmcExtension&             ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(ext.getSubdivisionIterationCount(), 4);        //u4
  READ_CODE(ext.getLodDisplacementQuantizationFlag(), 1);  //u1
  ext.getLiftingQuantizationParametersPerLevelOfDetails().clear();
  if (ext.getLodDisplacementQuantizationFlag()) {
    auto lodCount = ext.getSubdivisionIterationCount() + 1;
    ext.getLiftingQuantizationParametersPerLevelOfDetails().resize(lodCount);
    for (int32_t it = 0; it < lodCount; ++it) {
      READ_CODE(ext.getLiftingQuantizationParametersPerLevelOfDetails()[it][0],
                8);  //u8
      READ_CODE(ext.getLiftingQuantizationParametersPerLevelOfDetails()[it][1],
                8);  //u8
      READ_CODE(ext.getLiftingQuantizationParametersPerLevelOfDetails()[it][2],
                8);  //u8
    }
  } else {
    READ_CODE(ext.getLiftingQPs(0), 8);                         //u8
    READ_CODE(ext.getLiftingQPs(1), 8);                         //u8
    READ_CODE(ext.getLiftingQPs(2), 8);                         //u8
    READ_CODE(ext.getLiftingLevelOfDetailInverseScale(0), 64);  //f64
    READ_CODE(ext.getLiftingLevelOfDetailInverseScale(1), 64);  //f64
    READ_CODE(ext.getLiftingLevelOfDetailInverseScale(2), 64);  //f64
  }
  READ_CODE(ext.getInterpolateDisplacementNormals(), 1);  //u1
  READ_CODE(ext.getAddReconstructedNormals(), 1);         //u1
  READ_CODE(ext.getDisplacementReversePacking(), 1);      //u1
  READ_CODE(ext.getWidthDispVideo(), 16);                 //u16
  READ_CODE(ext.getHeightDispVideo(), 16);                //u16
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// AFPS V-DMC extension syntax
void
V3CReader::afpsVdmcExtension(Bitstream& bitstream, AfpsVdmcExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.2.1	AAPS V-PCC extension syntax
void
V3CReader::aapsVdmcExtension(Bitstream& bitstream, AapsVdmcExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.2.4 Atlas sub-bitstream syntax
void
V3CReader::atlasSubStream(V3cBitstream& syntax, Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  size_t              sizeBitstream = bitstream.capacity();
  PCCSEI              sei;
  SampleStreamNalUnit ssnu;
  sampleStreamNalHeader(bitstream, ssnu);
  while (bitstream.size() < sizeBitstream) {
    ssnu.addNalUnit();
    sampleStreamNalUnit(
      syntax, true, bitstream, ssnu, ssnu.getNalUnit().size() - 1, sei);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.3 Byte alignment syntax
void
V3CReader::byteAlignment(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t one  = 1;
  uint32_t zero = 0;
  READ_CODE(one, 1);  // f(1): equal to 1
  while (!bitstream.byteAligned()) {
    READ_CODE(zero, 1);  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4 Length alignment syntax
void
V3CReader::lengthAlignment(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  while (!bitstream.byteAligned()) {
    READ_CODE(zero, 1);  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4 V3C parameter set syntax
// 8.3.4.1 General V3C parameter set syntax
void
V3CReader::v3cParameterSet(V3CParameterSet& vps,
                           V3cBitstream&    syntax,
                           Bitstream&       bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  profileTierLevel(vps.getProfileTierLevel(), bitstream);
  READ_CODE(vps.getV3CParameterSetId(), 4);  // u(4)
  READ_CODE(zero, 8);                        // u(8)
  READ_CODE(vps.getAtlasCountMinus1(), 6);   // u(6)
  vps.allocateAtlas(vps.getAtlasCountMinus1() + 1);
  syntax.allocateAtlas(vps.getAtlasCountMinus1() + 1);
  syntax.allocateBaseMesh(1);
  for (uint32_t j = 0; j < vps.getAtlasCountMinus1() + 1; j++) {
    READ_CODE(vps.getAtlasId(j), 6);         // u(6)
    READ_UVLC(vps.getFrameWidth(j));         // ue(v)
    READ_UVLC(vps.getFrameHeight(j));        // ue(v)
    READ_CODE(vps.getMapCountMinus1(j), 4);  // u(4)
    vps.allocateMap(j);
    if (vps.getMapCountMinus1(j) > 0) {
      READ_CODE(vps.getMultipleMapStreamsPresentFlag(j), 1);  // u(1)
    }
    vps.getMapAbsoluteCodingEnableFlag(j, 0) = true;
    for (size_t i = 1; i <= vps.getMapCountMinus1(j); i++) {
      if (vps.getMultipleMapStreamsPresentFlag(j)) {
        READ_CODE(vps.getMapAbsoluteCodingEnableFlag(j, i), 1);  // u(1)
      } else {
        vps.getMapAbsoluteCodingEnableFlag(j, i) = true;
      }
      if (static_cast<int>(vps.getMapAbsoluteCodingEnableFlag(j, i)) == 0) {
        if (i > 0) {
          READ_UVLC(vps.getMapPredictorIndexDiff(j,
                                                 i));  // ue(v)
        } else {
          vps.getMapPredictorIndexDiff(j, i) = false;
        }
      }
    }
    READ_CODE(vps.getAuxiliaryVideoPresentFlag(j), 1);  // u(1)
    READ_CODE(vps.getOccupancyVideoPresentFlag(j), 1);  // u(1)
    READ_CODE(vps.getGeometryVideoPresentFlag(j), 1);   // u(1)
    READ_CODE(vps.getAttributeVideoPresentFlag(j), 1);  // u(1)
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
  READ_CODE(vps.getExtensionPresentFlag(), 1);  // u(1)
  if (vps.getExtensionPresentFlag()) {
    READ_CODE(vps.getExtensionCount(), 8);  // u(8)
  }
  if (vps.getExtensionCount()) {
    READ_UVLC(vps.getExtensionLengthMinus1());  // ue(v)
    for (size_t i = 0; i < vps.getExtensionCount(); i++) {
      READ_CODE(vps.getExtensionType(i), 8);     // u(8)
      READ_CODE(vps.getExtensionLength(i), 16);  // u(16)
      vpsExtension(vps, i, bitstream);
    }
  }
  byteAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.2 Profile, tier, and level syntax
void
V3CReader::profileTierLevel(ProfileTierLevel& ptl, Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  uint32_t ffff = 0xFFFF;
  READ_CODE(ptl.getTierFlag(), 1);                  // u(1)
  READ_CODE(ptl.getProfileCodecGroupIdc(), 7);      // u(7)
  READ_CODE(ptl.getProfileToolsetIdc(), 8);         // u(8)
  READ_CODE(ptl.getProfileReconstructionIdc(), 8);  // u(8)
  READ_CODE(zero, 16);                              // u(16)
  READ_CODE(ffff, 16);                              // u(16)
  READ_CODE(ptl.getLevelIdc(), 8);                  // u(8)
  READ_CODE(ptl.getNumSubProfiles(), 6);            // u(6)
  READ_CODE(ptl.getExtendedSubProfileFlag(), 1);    // u(1)
  ptl.allocate();
  for (size_t i = 0; i < ptl.getNumSubProfiles(); i++) {
    size_t v = ptl.getExtendedSubProfileFlag() == 0 ? 32 : 64;
    READ_CODE(ptl.getSubProfileIdc(i), v);  // u(v)
  }
  READ_CODE(ptl.getToolConstraintsPresentFlag(), 1);  // u(1)
  if (ptl.getToolConstraintsPresentFlag()) {
    profileToolsetConstraintsInformation(
      ptl.getProfileToolsetConstraintsInformation(), bitstream);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.3 Occupancy parameter set syntax
void
V3CReader::occupancyInformation(OccupancyInformation& oi,
                                Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(oi.getOccupancyCodecId(), 8);                    // u(8)
  READ_CODE(oi.getLossyOccupancyCompressionThreshold(), 8);  // u(8)
  READ_CODE(oi.getOccupancy2DBitdepthMinus1(), 5);           // u(5)
  READ_CODE(oi.getOccupancyMSBAlignFlag(), 1);               // u(1)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.4 Geometry parameter set syntax
void
V3CReader::geometryInformation(GeometryInformation& gi,
                               V3CParameterSet&     vps,
                               Bitstream&           bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  size_t atlasIndex = 0;
  READ_CODE(gi.getGeometryCodecId(), 8);                      // u(8)
  READ_CODE(gi.getGeometry2dBitdepthMinus1(), 5);             // u(5)
  READ_CODE(gi.getGeometryMSBAlignFlag(), 1);                 // u(1)
  READ_CODE(gi.getGeometry3dCoordinatesBitdepthMinus1(), 5);  // u(5)
  if (vps.getAuxiliaryVideoPresentFlag(atlasIndex)) {
    READ_CODE(gi.getAuxiliaryGeometryCodecId(), 8);  // u(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.5 Attribute information
void
V3CReader::attributeInformation(AttributeInformation& ai,
                                V3CParameterSet&      vps,
                                Bitstream&            bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  size_t atlasIndex = 0;
  READ_CODE(ai.getAttributeCount(), 7);  // u(7)
  ai.allocate(ai.getAttributeCount());
  for (uint32_t i = 0; i < ai.getAttributeCount(); i++) {
    READ_CODE(ai.getAttributeTypeId(i), 4);   // u(4)
    READ_CODE(ai.getAttributeCodecId(i), 8);  // u(8)
    if (vps.getAuxiliaryVideoPresentFlag(atlasIndex)) {
      READ_CODE(ai.getAuxiliaryAttributeCodecId(i), 8);  // u(8)
    }
    ai.getAttributeMapAbsoluteCodingPersistenceFlag(i) = true;
    if (vps.getMapCountMinus1(atlasIndex) > 0) {
      READ_CODE(ai.getAttributeMapAbsoluteCodingPersistenceFlag(i),
                1);  // u(1)
    }
    READ_CODE(ai.getAttributeDimensionMinus1(i), 6);  // u(6)
    if (ai.getAttributeDimensionMinus1(i) > 0) {
      READ_CODE(ai.getAttributeDimensionPartitionsMinus1(i), 6);  // u(6)
      int32_t remainingDimensions = ai.getAttributeDimensionMinus1(i);
      int32_t k = ai.getAttributeDimensionPartitionsMinus1(i);
      for (int32_t j = 0; j < k; j++) {
        if (k - j == remainingDimensions) {
          ai.getAttributePartitionChannelsMinus1(i, j) = 0;
        } else {
          READ_UVLC(ai.getAttributePartitionChannelsMinus1(i, j));  // ue(v)
        }
        remainingDimensions -=
          ai.getAttributePartitionChannelsMinus1(i, j) + 1;
      }
      ai.getAttributePartitionChannelsMinus1(i, k) = remainingDimensions;
    }
    READ_CODE(ai.getAttribute2dBitdepthMinus1(i), 5);  // u(5)
    READ_CODE(ai.getAttributeMSBAlignFlag(i), 1);      // u(1)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.4.6 Profile toolset constraints information syntax
void
V3CReader::profileToolsetConstraintsInformation(
  ProfileToolsetConstraintsInformation& ptci,
  Bitstream&                            bitstream) {
  uint32_t zero = 0;
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(ptci.getOneFrameOnlyFlag(), 1);                         // u(1)
  READ_CODE(ptci.getEOMContraintFlag(), 1);                         // u(1)
  READ_CODE(ptci.getMaxMapCountMinus1(), 4);                        // u(4)
  READ_CODE(ptci.getMaxAtlasCountMinus1(), 4);                      // u(4)
  READ_CODE(ptci.getMultipleMapStreamsConstraintFlag(), 1);         // u(1)
  READ_CODE(ptci.getPLRConstraintFlag(), 1);                        // u(1)
  READ_CODE(ptci.getAttributeMaxDimensionMinus1(), 6);              // u(6)
  READ_CODE(ptci.getAttributeMaxDimensionPartitionsMinus1(), 6);    // u(6)
  READ_CODE(ptci.getNoEightOrientationsConstraintFlag(), 1);        // u(1)
  READ_CODE(ptci.getNo45DegreeProjectionPatchConstraintFlag(), 1);  // u(1)
  READ_CODE(zero, 6);                                               // u(6)
  READ_CODE(ptci.getNumReservedConstraintBytes(), 8);               // u(8)
  ptci.allocate();
  for (size_t i = 0; i < ptci.getNumReservedConstraintBytes(); i++) {
    READ_CODE(ptci.getReservedConstraintByte(i), 8);  // u(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.5 NAL unit syntax
// 8.3.5.1 General NAL unit syntax
void
V3CReader::nalUnit(Bitstream& bitstream, NalUnit& nalUnit) {
  TRACE_BITSTREAM_IN("%s", __func__);
  nalUnitHeader(bitstream, nalUnit);
  for (size_t i = 2; i < nalUnit.getSize(); i++) {
    READ_CODE(nalUnit.getData(i), 8);  // b(8)
  }
}

// 8.3.5.2 NAL unit header syntax
void
V3CReader::nalUnitHeader(Bitstream& bitstream, NalUnit& nalUnit) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  READ_CODE(zero, 1);                                      // f(1)
  READ_CODE_CAST(nalUnit.getType(), 6, AtlasNalUnitType);  // u(6)
  READ_CODE(nalUnit.getLayerId(), 6);                      // u(6)
  READ_CODE(nalUnit.getTemporalyIdPlus1(), 3);             // u(3)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.1 Atlas sequence parameter set Rbsp
// 8.3.6.1.1 General Atlas sequence parameter set Rbsp
void
V3CReader::atlasSequenceParameterSetRbsp(AtlasSequenceParameterSetRbsp& asps,
                                         V3cBitstream&                  syntax,
                                         Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  READ_UVLC(asps.getAtlasSequenceParameterSetId());         // ue(v)
  READ_UVLC(asps.getFrameWidth());                          // ue(v)
  READ_UVLC(asps.getFrameHeight());                         // ue(v)
  READ_CODE(asps.getGeometry3dBitdepthMinus1(), 5);         // u(5)
  READ_CODE(asps.getGeometry2dBitdepthMinus1(), 5);         // u(5)
  READ_UVLC(asps.getLog2MaxAtlasFrameOrderCntLsbMinus4());  // ue(v)
  READ_UVLC(asps.getMaxDecAtlasFrameBufferingMinus1());     // ue(v)
  READ_CODE(asps.getLongTermRefAtlasFramesFlag(), 1);       // u(1)
  READ_UVLC(asps.getNumRefAtlasFrameListsInAsps());         // ue(v)
  asps.allocateRefListStruct();
  for (size_t i = 0; i < asps.getNumRefAtlasFrameListsInAsps(); i++) {
    refListStruct(asps.getRefListStruct(i), asps, bitstream);
  }
  READ_CODE(asps.getUseEightOrientationsFlag(), 1);       // u(1)
  READ_CODE(asps.getExtendedProjectionEnabledFlag(), 1);  // u(1)
  if (asps.getExtendedProjectionEnabledFlag()) {
    READ_UVLC(asps.getMaxNumberProjectionsMinus1());  // ue(v)
  }
  READ_CODE(asps.getNormalAxisLimitsQuantizationEnabledFlag(), 1);  // u(1)
  READ_CODE(asps.getNormalAxisMaxDeltaValueEnabledFlag(), 1);       // u(1)
  READ_CODE(asps.getPatchPrecedenceOrderFlag(), 1);                 // u(1)
  READ_CODE(asps.getLog2PatchPackingBlockSize(), 3);                // u(3)
  READ_CODE(asps.getPatchSizeQuantizerPresentFlag(), 1);            // u(1)
  READ_CODE(asps.getMapCountMinus1(), 4);                           // u(4)
  READ_CODE(asps.getPixelDeinterleavingFlag(), 1);                  // u(1)
  if (asps.getPixelDeinterleavingFlag()) {
    asps.allocatePixelDeinterleavingMapFlag();
    for (size_t i = 0; i < asps.getMapCountMinus1() + 1; i++) {
      READ_CODE(asps.getPixelDeinterleavingMapFlag(i), 1);  // u(1)
    }
  }
  READ_CODE(asps.getRawPatchEnabledFlag(), 1);  // u(1)
  READ_CODE(asps.getEomPatchEnabledFlag(), 1);  // u(1)
  if (asps.getEomPatchEnabledFlag() && asps.getMapCountMinus1() == 0) {
    READ_CODE(asps.getEomFixBitCountMinus1(), 4);  // u(4)
  }
  if (asps.getRawPatchEnabledFlag() || asps.getEomPatchEnabledFlag()) {
    READ_CODE(asps.getAuxiliaryVideoEnabledFlag(), 1);  // u(1)
  }
  READ_CODE(asps.getPLREnabledFlag(), 1);  // u(1)
  if (asps.getPLREnabledFlag()) { plrInformation(asps, syntax, bitstream); }
  READ_CODE(asps.getVuiParametersPresentFlag(), 1);  // u(1)
  if (asps.getVuiParametersPresentFlag()) {
    vuiParameters(bitstream, asps.getVuiParameters());
  }

  READ_CODE(asps.getExtensionFlag(), 1);  // u(1)
  if (asps.getExtensionFlag()) {
    READ_CODE(asps.getVpccExtensionFlag(), 1);  // u(1)
    READ_CODE(asps.getMivExtensionFlag(), 1);   // u(1)
    READ_CODE(asps.getVdmcExtensionFlag(), 1);  // u(1)
    READ_CODE(asps.getExtension5Bits(), 5);     // u(5)
    if (asps.getVpccExtensionFlag())
      aspsVpccExtension(bitstream, asps, asps.getAspsVpccExtension());
    if (asps.getVdmcExtensionFlag())
      aspsVdmcExtension(bitstream, asps, asps.getAspsVdmcExtension());
    if (asps.getExtension5Bits())
      while (moreRbspData(bitstream)) READ_CODE(zero, 1);  // u(1)
  }
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.1.2 Point local reconstruction information syntax
void
V3CReader::plrInformation(AtlasSequenceParameterSetRbsp& asps,
                          V3cBitstream&                  syntax,
                          Bitstream&                     bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  asps.allocatePLRInformation();
  for (size_t j = 0; j < asps.getMapCountMinus1() + 1; j++) {
    auto& plri = asps.getPLRInformation(j);
    READ_CODE(plri.getMapEnabledFlag(), 1);  // u(1)
    if (plri.getMapEnabledFlag()) {
      READ_CODE(plri.getNumberOfModesMinus1(), 4);  // u(4)
      plri.allocate();
      for (size_t i = 0; i < plri.getNumberOfModesMinus1(); i++) {
        READ_CODE(plri.getInterpolateFlag(i), 1);  // u(1)
        READ_CODE(plri.getFillingFlag(i), 1);      // u(1)
        READ_CODE(plri.getMinimumDepth(i), 2);     // u(2)
        READ_CODE(plri.getNeighbourMinus1(i), 2);  // u(2)
      }
      READ_CODE(plri.getBlockThresholdPerPatchMinus1(), 6);  // u(6)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.2 Specification of syntax functions and descriptors
bool
V3CReader::byteAligned(Bitstream& bitstream) {
  return bitstream.byteAligned();
}
bool
V3CReader::moreDataInPayload(Bitstream& bitstream) {
  return !bitstream.byteAligned();
}
bool
V3CReader::moreRbspData(Bitstream& bitstream) {
  uint32_t value = 0;
  // Return false if there is no more data.
  if (!bitstream.moreData()) { return false; }
  // Store bitstream state.
  auto position = bitstream.getPosition();
#if defined(CONFORMANCE_TRACE) || defined(BITSTREAM_TRACE)
  bitstream.setTrace(false);
#endif
  // Skip first bit. It may be part of a RBSP or a rbsp_one_stop_bit.
  READ_CODE(value, 1);  // u(1)
  while (bitstream.moreData()) {
    READ_CODE(value, 1);  // u(1)
    if (value) {
      // We found a one bit beyond the first bit. Restore bitstream state and return true.
      bitstream.setPosition(position);
#if defined(CONFORMANCE_TRACE) || defined(BITSTREAM_TRACE)
      bitstream.setTrace(true);
#endif
      return true;
    }
  }
  // We did not found a one bit beyond the first bit. Restore bitstream state and return false.
  bitstream.setPosition(position);
#if defined(CONFORMANCE_TRACE) || defined(BITSTREAM_TRACE)
  bitstream.setTrace(true);
#endif
  return false;
}

bool
V3CReader::moreRbspTrailingData(Bitstream& bitstream) {
  return false;
}
bool

V3CReader::moreDataInV3CUnit(Bitstream& bitstream) {
  return false;
}

bool
V3CReader::payloadExtensionPresent(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
  return false;
}

// 8.3.6.2 Atlas frame parameter set Rbsp syntax
// 8.3.6.2.1 General atlas frame parameter set Rbsp syntax
void
V3CReader::atlasFrameParameterSetRbsp(AtlasFrameParameterSetRbsp& afps,
                                      V3cBitstream&               syntax,
                                      Bitstream&                  bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  READ_UVLC(afps.getAtlasFrameParameterSetId());     // ue(v)
  READ_UVLC(afps.getAtlasSequenceParameterSetId());  // ue(v)
  auto& atlas = syntax.getAtlas();
  auto& asps =
    atlas.getAtlasSequenceParameterSet(afps.getAtlasSequenceParameterSetId());
  atlasFrameTileInformation(
    afps.getAtlasFrameTileInformation(), asps, bitstream);
  READ_CODE(afps.getOutputFlagPresentFlag(), 1);                // u(1)
  READ_UVLC(afps.getNumRefIdxDefaultActiveMinus1());            // ue(v)
  READ_UVLC(afps.getAdditionalLtAfocLsbLen());                  // ue(v)
  READ_CODE(afps.getLodModeEnableFlag(), 1);                    // u(1)
  READ_CODE(afps.getRaw3dOffsetBitCountExplicitModeFlag(), 1);  // u(1)
  READ_CODE(afps.getExtensionFlag(), 1);                        // u(1)
  if (afps.getExtensionFlag()) {
    READ_CODE(afps.getExtension8Bits(), 8);  // u(8)
  }
  if (afps.getExtension8Bits()) {
    while (moreRbspData(bitstream)) { READ_CODE(zero, 1); }  // u(1)
  }
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.2.2 Atlas frame tile information syntax
void
V3CReader::atlasFrameTileInformation(AtlasFrameTileInformation&     afti,
                                     AtlasSequenceParameterSetRbsp& asps,
                                     Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(afti.getSingleTileInAtlasFrameFlag(), 1);  // u(1)
  if (!afti.getSingleTileInAtlasFrameFlag()) {
    READ_CODE(afti.getUniformPartitionSpacingFlag(), 1);  // u(1)
    if (afti.getUniformPartitionSpacingFlag()) {
      READ_UVLC(afti.getPartitionColumnWidthMinus1(0));  //  ue(v)
      READ_UVLC(afti.getPartitionRowHeightMinus1(0));    //  ue(v)
      afti.getNumPartitionColumnsMinus1() =
        ceil(asps.getFrameWidth()
             / ((afti.getPartitionColumnWidthMinus1(0) + 1) * 64.0))
        - 1;
      afti.getNumPartitionRowsMinus1() =
        ceil(asps.getFrameHeight()
             / ((afti.getPartitionRowHeightMinus1(0) + 1) * 64.0))
        - 1;
    } else {
      READ_UVLC(afti.getNumPartitionColumnsMinus1());  //  ue(v)
      READ_UVLC(afti.getNumPartitionRowsMinus1());     //  ue(v)
      for (size_t i = 0; i < afti.getNumPartitionColumnsMinus1(); i++) {
        READ_UVLC(afti.getPartitionColumnWidthMinus1(i));  //  ue(v)
      }
      for (size_t i = 0; i < afti.getNumPartitionRowsMinus1(); i++) {
        READ_UVLC(afti.getPartitionRowHeightMinus1(i));  //  ue(v)
      }
    }
    READ_CODE(afti.getSinglePartitionPerTileFlag(), 1);  //  u(1)
    if (afti.getSinglePartitionPerTileFlag() == 0U) {
      uint32_t NumPartitionsInAtlasFrame =
        (afti.getNumPartitionColumnsMinus1() + 1)
        * (afti.getNumPartitionRowsMinus1() + 1);
      READ_UVLC(afti.getNumTilesInAtlasFrameMinus1());  // ue(v)
      for (size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++) {
        uint8_t bitCount = ceilLog2(NumPartitionsInAtlasFrame);
        READ_CODE(afti.getTopLeftPartitionIdx(i), bitCount);     // u(v)
        READ_UVLC(afti.getBottomRightPartitionColumnOffset(i));  // ue(v)
        READ_UVLC(afti.getBottomRightPartitionRowOffset(i));     // ue(v)
      }
    } else {
      afti.getNumTilesInAtlasFrameMinus1() =
        (afti.getNumPartitionColumnsMinus1() + 1)
          * (afti.getNumPartitionRowsMinus1() + 1)
        - 1;
      for (size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++) {
        afti.getTopLeftPartitionIdx(i)              = i;
        afti.getBottomRightPartitionColumnOffset(i) = 0;
        afti.getBottomRightPartitionRowOffset(i)    = 0;
      }
    }
  } else {
    afti.getNumTilesInAtlasFrameMinus1() = 0;
  }
  if (asps.getAuxiliaryVideoEnabledFlag()) {
    READ_UVLC(afti.getAuxiliaryVideoTileRowWidthMinus1());  // ue(v)
    for (size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++) {
      READ_UVLC(afti.getAuxiliaryVideoTileRowHeight(i));  // ue(v)
    }
  }
  READ_CODE(afti.getSignalledTileIdFlag(), 1);  // u(1)
  if (afti.getSignalledTileIdFlag()) {
    READ_UVLC(afti.getSignalledTileIdLengthMinus1());  // ue(v)
    for (size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++) {
      uint8_t bitCount = afti.getSignalledTileIdLengthMinus1() + 1;
      READ_CODE(afti.getTileId(i), bitCount);  // u(v)
    }
  } else {
    for (size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++) {
      afti.getTileId(i) = i;
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.3 Atlas adaptation parameter set RBSP syntax
// 8.3.6.3.1 General atlas adaptation parameter set RBSP syntax
void
V3CReader::atlasAdaptationParameterSetRbsp(
  AtlasAdaptationParameterSetRbsp& aaps,
  Bitstream&                       bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  READ_UVLC(aaps.getAtlasAdaptationParameterSetId());  // ue(v)
  READ_CODE(aaps.getExtensionFlag(), 1);               // u(1)
  if (aaps.getExtensionFlag()) {
    READ_CODE(aaps.getVpccExtensionFlag(), 1);  // u(1)
    READ_CODE(aaps.getMivExtensionFlag(), 1);   // u(1)
    READ_CODE(aaps.getVdmcExtensionFlag(), 1);  // u(1)
    READ_CODE(aaps.getExtension5Bits(), 5);     // u(5)
    if (aaps.getVpccExtensionFlag())
      aapsVpccExtension(bitstream, aaps.getAapsVpccExtension());
    if (aaps.getVdmcExtensionFlag())
      aapsVdmcExtension(bitstream, aaps.getAapsVdmcExtension());
    if (aaps.getExtension5Bits())
      while (moreRbspData(bitstream)) { READ_CODE(zero, 1); }  // u(1)
  }
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.4  Supplemental enhancement information Rbsp
void
V3CReader::seiRbsp(V3cBitstream&    syntax,
                   Bitstream&       bitstream,
                   AtlasNalUnitType nalUnitType,
                   PCCSEI&          sei) {
  TRACE_BITSTREAM_IN("%s", __func__);
  do {
    seiMessage(bitstream, syntax, nalUnitType, sei);
  } while (moreRbspData(bitstream));
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.5 Access unit delimiter RBSP syntax
void
V3CReader::accessUnitDelimiterRbsp(AccessUnitDelimiterRbsp& aud,
                                   V3cBitstream&            syntax,
                                   Bitstream&               bitstream) {
  READ_CODE(aud.getAframeType(), 3);  //	u(3)
  rbspTrailingBits(bitstream);
}
// 8.3.6.6 End of sequence RBSP syntax
void
V3CReader::endOfSequenceRbsp(EndOfSequenceRbsp& eosbsp,
                             V3cBitstream&      syntax,
                             Bitstream&         bitstream) {}

// 8.3.6.7 End of bitstream RBSP syntax
void
V3CReader::endOfAtlasSubBitstreamRbsp(EndOfAtlasSubBitstreamRbsp& eoasb,
                                      V3cBitstream&               syntax,
                                      Bitstream&                  bitstream) {}

// 8.3.6.8 Filler data RBSP syntax
void
V3CReader::fillerDataRbsp(FillerDataRbsp& fdrbsp,
                          V3cBitstream&   syntax,
                          Bitstream&      bitstream) {
  // while ( next_bits( 8 ) == 0xFF ) { uint32_t code; READ_CODE( code, 8 );  // f(8)
  rbspTrailingBits(bitstream);
}

// 8.3.6.9  Atlas tile group layer Rbsp syntax
void
V3CReader::atlasTileLayerRbsp(AtlasTileLayerRbsp& atgl,
                              V3cBitstream&       syntax,
                              AtlasNalUnitType    nalUnitType,
                              Bitstream&          bitstream) {
  // setFrameIndex
  TRACE_BITSTREAM_IN("%s", __func__);
  atlasTileHeader(atgl.getHeader(), syntax, nalUnitType, bitstream);
  atlasTileDataUnit(atgl.getDataUnit(), atgl.getHeader(), syntax, bitstream);
  rbspTrailingBits(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.10 RBSP trailing bit syntax
void
V3CReader::rbspTrailingBits(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t one  = 1;
  uint32_t zero = 0;
  READ_CODE(one, 1);  // f(1): equal to 1
  while (!bitstream.byteAligned()) {
    READ_CODE(zero, 1);  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.11  Atlas tile group header syntax
void
V3CReader::atlasTileHeader(AtlasTileHeader& ath,
                           V3cBitstream&    syntax,
                           AtlasNalUnitType nalUnitType,
                           Bitstream&       bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  if (nalUnitType >= ATLAS_NAL_BLA_W_LP
      && nalUnitType <= ATLAS_NAL_RSV_IRAP_ACL_29) {
    READ_CODE(ath.getNoOutputOfPriorAtlasFramesFlag(), 1);  // u(1)
  }
  if (nalUnitType == ATLAS_NAL_TRAIL_R) { ath.getTileNaluTypeInfo() = 1; }
  if (nalUnitType == ATLAS_NAL_TRAIL_N) { ath.getTileNaluTypeInfo() = 2; }
  READ_UVLC(ath.getAtlasFrameParameterSetId());       // ue(v)
  READ_UVLC(ath.getAtlasAdaptationParameterSetId());  // ue(v)
  auto&  atlas  = syntax.getAtlas();
  size_t afpsId = ath.getAtlasFrameParameterSetId();
  auto&  afps   = atlas.getAtlasFrameParameterSet(afpsId);
  size_t aspsId = afps.getAtlasSequenceParameterSetId();
  auto&  asps   = atlas.getAtlasSequenceParameterSet(aspsId);
  auto&  afti   = afps.getAtlasFrameTileInformation();
  if (afti.getSignalledTileIdFlag()) {
    READ_CODE(ath.getId(), afti.getSignalledTileIdLengthMinus1() + 1);  // u(v)
  } else {
    if (afti.getNumTilesInAtlasFrameMinus1() != 0) {
      READ_CODE(ath.getId(),
                ceilLog2(afti.getNumTilesInAtlasFrameMinus1() + 1));  // u(v)
    } else {
      ath.getId() = 0;
    }
  }
  READ_UVLC_CAST(ath.getType(), TileType);  // ue(v)
  if (afps.getOutputFlagPresentFlag()) {
    READ_CODE(ath.getAtlasOutputFlag(), 1);  // u(1)
  } else {
    ath.getAtlasOutputFlag() = false;
  }
  READ_CODE(ath.getAtlasFrmOrderCntLsb(),
            asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4);  // u(v)
  if (asps.getNumRefAtlasFrameListsInAsps() > 0) {
    READ_CODE(ath.getRefAtlasFrameListSpsFlag(), 1);  // u(1)
  } else {
    ath.getRefAtlasFrameListSpsFlag() = false;
  }
  ath.getRefAtlasFrameListIdx() = 0;
  if (static_cast<int>(ath.getRefAtlasFrameListSpsFlag()) == 0) {
    refListStruct(ath.getRefListStruct(), asps, bitstream);
  } else if (asps.getNumRefAtlasFrameListsInAsps() > 1) {
    size_t bitCount = ceilLog2(asps.getNumRefAtlasFrameListsInAsps());
    READ_CODE(ath.getRefAtlasFrameListIdx(), bitCount);  // u(v)
  }
  if (ath.getRefAtlasFrameListSpsFlag()) {
    ath.getRefListStruct() =
      asps.getRefListStruct(ath.getRefAtlasFrameListIdx());
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
    READ_CODE(ath.getAdditionalAfocLsbPresentFlag(j), 1);  // u(1)
    if (ath.getAdditionalAfocLsbPresentFlag(j)) {
      uint8_t bitCount = afps.getAdditionalLtAfocLsbLen();
      READ_CODE(ath.getAdditionalAfocLsbVal(j), bitCount);  // u(v)
    }
  }
  if (ath.getType() != SKIP_TILE) {
    if (asps.getNormalAxisLimitsQuantizationEnabledFlag()) {
      READ_CODE(ath.getPosMinDQuantizer(), 5);  // u(5)
      if (asps.getNormalAxisMaxDeltaValueEnabledFlag()) {
        READ_CODE(ath.getPosDeltaMaxDQuantizer(), 5);  // u(5)
      }
    }
    if (asps.getPatchSizeQuantizerPresentFlag()) {
      READ_CODE(ath.getPatchSizeXinfoQuantizer(), 3);  // u(3)
      READ_CODE(ath.getPatchSizeYinfoQuantizer(), 3);  // u(3)
    }
    if (afps.getRaw3dOffsetBitCountExplicitModeFlag()) {
      size_t bitCount = floorLog2(asps.getGeometry3dBitdepthMinus1() + 1);
      READ_CODE(ath.getRaw3dOffsetAxisBitCountMinus1(), bitCount);  // u(v)
    } else {
      ath.getRaw3dOffsetAxisBitCountMinus1() =
        std::max(0,
                 asps.getGeometry3dBitdepthMinus1()
                   - asps.getGeometry2dBitdepthMinus1())
        - 1;
    }
    if (ath.getType() == P_TILE && refList.getNumRefEntries() > 1) {
      READ_CODE(ath.getNumRefIdxActiveOverrideFlag(), 1);  // u(1)
      if (ath.getNumRefIdxActiveOverrideFlag()) {
        READ_UVLC(ath.getNumRefIdxActiveMinus1());  // ue(v)
      }
    }
  }
  byteAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.12  Reference list structure syntax
void
V3CReader::refListStruct(RefListStruct&                 rls,
                         AtlasSequenceParameterSetRbsp& asps,
                         Bitstream&                     bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_UVLC(rls.getNumRefEntries());  // ue(v)
  rls.allocate();
  for (size_t i = 0; i < rls.getNumRefEntries(); i++) {
    if (asps.getLongTermRefAtlasFramesFlag()) {
      READ_CODE(rls.getStRefAtlasFrameFlag(i), 1);  // u(1)
    } else {
      rls.getStRefAtlasFrameFlag(i) = true;
    }
    if (rls.getStRefAtlasFrameFlag(i)) {
      READ_UVLC(rls.getAbsDeltaAfocSt(i));  // ue(v)
      if (rls.getAbsDeltaAfocSt(i) > 0) {
        READ_CODE(rls.getStrafEntrySignFlag(i), 1);  // u(1)
      } else {
        rls.getStrafEntrySignFlag(i) = true;
      }
    } else {
      uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
      READ_CODE(rls.getAfocLsbLt(i), bitCount);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.1  General atlas tile group data unit syntax =patchTileDataUnit
void
V3CReader::atlasTileDataUnit(AtlasTileDataUnit& atdu,
                             AtlasTileHeader&   ath,
                             V3cBitstream&      syntax,
                             Bitstream&         bitstream) {
  TRACE_BITSTREAM_IN("%s(%s)", __func__, toString(ath.getType()).c_str());
  if (ath.getType() == SKIP_TILE) {
    skipPatchDataUnit(bitstream);
  } else {
    atdu.init();
    size_t patchIndex = 0;
    prevPatchSizeU_   = 0;
    prevPatchSizeV_   = 0;
    predPatchIndex_   = 0;
    uint8_t patchMode = 0;
    READ_UVLC(patchMode);  // ue(v)
    while ((patchMode != I_END) && (patchMode != P_END)) {
      auto& pid           = atdu.addPatchInformationData(patchMode);
      pid.getTileOrder()  = atdu.getTileOrder();
      pid.getPatchIndex() = patchIndex;
      patchIndex++;
      patchInformationData(pid, patchMode, ath, syntax, bitstream);
      READ_UVLC(patchMode);  // ue(v)
    }
    prevFrameIndex_ = atdu.getTileOrder();
  }
  TRACE_BITSTREAM_OUT("%s(%s)", __func__, toString(ath.getType()).c_str());
}

// 8.3.7.2  Patch information data syntax
void
V3CReader::patchInformationData(PatchInformationData& pid,
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
V3CReader::patchDataUnit(PatchDataUnit&   pdu,
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
  READ_UVLC(pdu.get2dPosX());                 // ue(v)
  READ_UVLC(pdu.get2dPosY());                 // ue(v)
  READ_UVLC(pdu.get2dSizeXMinus1());          // ue(v)
  READ_UVLC(pdu.get2dSizeYMinus1());          // ue(v)
  READ_CODE(pdu.get3dOffsetU(), bitCountUV);  // u(v)
  READ_CODE(pdu.get3dOffsetV(), bitCountUV);  // u(v)
  READ_CODE(pdu.get3dOffsetD(), bitCountD);   // u(v)
  if (asps.getNormalAxisMaxDeltaValueEnabledFlag()) {
    uint8_t bitCountForMaxDepth = std::min(asps.getGeometry2dBitdepthMinus1(),
                                           asps.getGeometry3dBitdepthMinus1())
                                  + 1 - ath.getPosDeltaMaxDQuantizer();
    READ_CODE(pdu.get3dRangeD(), bitCountForMaxDepth);  // u(v)
  } else {
    pdu.get3dRangeD() = 0;
  }

  uint32_t numProjection  = ceilLog2(asps.getMaxNumberProjectionsMinus1() + 1);
  uint32_t numOrientation = asps.getUseEightOrientationsFlag() ? 3 : 1;
  READ_CODE(pdu.getProjectionId(), numProjection);       // u(5 or 3)
  READ_CODE(pdu.getOrientationIndex(), numOrientation);  // u(3 or 1)
  if (afps.getLodModeEnableFlag()) {
    READ_CODE(pdu.getLodEnableFlag(), 1);  // u(1)
    if (pdu.getLodEnableFlag()) {
      READ_UVLC(pdu.getLodScaleXMinus1());  // ue(v)
      READ_UVLC(pdu.getLodScaleYIdc());     // ue(v)
    }
  } else {
    pdu.getLodEnableFlag()   = false;
    pdu.getLodScaleXMinus1() = 0;
    pdu.getLodScaleYIdc()    = 0;
  }
  if (asps.getPLREnabledFlag()) {
    auto& plrd = pdu.getPLRData();
    plrd.allocate(pdu.get2dSizeXMinus1() + 1, pdu.get2dSizeYMinus1() + 1);
    plrData(plrd, syntax, asps, bitstream);
  }
}

// 8.3.7.4  Skip patch data unit syntax
void
V3CReader::skipPatchDataUnit(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.5  Merge patch data unit syntax
void
V3CReader::mergePatchDataUnit(MergePatchDataUnit& mpdu,
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
  if (numRefIdxActive > 1) {
    READ_UVLC(mpdu.getRefIndex());  // ue(v)
  } else {
    mpdu.getRefIndex() = 0;
  }
  READ_CODE(mpdu.getOverride2dParamsFlag(), 1);  // u(1)
  if (mpdu.getOverride2dParamsFlag()) {
    READ_SVLC(mpdu.get2dPosX());        // se(v)
    READ_SVLC(mpdu.get2dPosY());        // se(v)
    READ_SVLC(mpdu.get2dDeltaSizeX());  // se(v)
    READ_SVLC(mpdu.get2dDeltaSizeY());  // se(v)
    if (asps.getPLREnabledFlag()) { overridePlrFlag = true; }
  } else {
    READ_CODE(mpdu.getOverride3dParamsFlag(), 1);  // u(1)
    if (mpdu.getOverride3dParamsFlag()) {
      READ_SVLC(mpdu.get3dOffsetU());  // se(v)
      READ_SVLC(mpdu.get3dOffsetV());  // se(v)
      READ_SVLC(mpdu.get3dOffsetD());  // se(v)
      if (asps.getNormalAxisMaxDeltaValueEnabledFlag()) {
        READ_SVLC(mpdu.get3dRangeD());  // se(v)
      } else mpdu.get3dRangeD() = 0;
      if (asps.getPLREnabledFlag()) {
        READ_CODE(mpdu.getOverridePlrFlag(), 1);  // u(1)
      }
    }
  }
  if (overridePlrFlag && asps.getPLREnabledFlag()) {
    auto& plrd = mpdu.getPLRData();
    plrd.allocate(prevPatchSizeU_ + mpdu.get2dDeltaSizeX(),
                  prevPatchSizeV_ + mpdu.get2dDeltaSizeY());
    plrData(plrd, syntax, asps, bitstream);
    prevPatchSizeU_ += mpdu.get2dDeltaSizeX();
    prevPatchSizeV_ += mpdu.get2dDeltaSizeY();
  }
}

// 8.3.7.6  Inter patch data unit syntax
void
V3CReader::interPatchDataUnit(InterPatchDataUnit& ipdu,
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
  size_t numRefIdxActive = atlas.getNumRefIdxActive(ath);
  if (numRefIdxActive > 1) {
    READ_UVLC(ipdu.getRefIndex());  // ue(v)
  } else {
    ipdu.getRefIndex() = 0;
  }
  READ_SVLC(ipdu.getRefPatchIndex());  // se(v)
  READ_SVLC(ipdu.get2dPosX());         // se(v)
  READ_SVLC(ipdu.get2dPosY());         // se(v)
  READ_SVLC(ipdu.get2dDeltaSizeX());   // se(v)
  READ_SVLC(ipdu.get2dDeltaSizeY());   // se(v)
  READ_SVLC(ipdu.get3dOffsetU());      // se(v)
  READ_SVLC(ipdu.get3dOffsetV());      // se(v)
  READ_SVLC(ipdu.get3dOffsetD());      // se(v)
  if (asps.getNormalAxisMaxDeltaValueEnabledFlag()) {
    READ_SVLC(ipdu.get3dRangeD());  // se(v)
  } else {
    ipdu.get3dRangeD() = 0;
  }
  if (asps.getPLREnabledFlag()) {
    auto&   atglPrev = atlas.getAtlasTileLayer(prevFrameIndex_);
    auto&   atghPrev = atglPrev.getHeader();
    auto&   atgdPrev = atglPrev.getDataUnit();
    auto&   pidPrev  = atgdPrev.getPatchInformationData(ipdu.getRefPatchIndex()
                                                     + predPatchIndex_);
    auto    patchModePrev = pidPrev.getPatchMode();
    int32_t sizeU         = ipdu.get2dDeltaSizeX();
    int32_t sizeV         = ipdu.get2dDeltaSizeY();
    if (atghPrev.getType() == P_TILE) {
      if (patchModePrev == P_MERGE) {
        auto& plrdPrev = pidPrev.getMergePatchDataUnit().getPLRData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      } else if (patchModePrev == P_INTER) {
        auto& plrdPrev = pidPrev.getInterPatchDataUnit().getPLRData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      } else if (patchModePrev == P_INTRA) {
        auto& plrdPrev = pidPrev.getPatchDataUnit().getPLRData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      }
    } else if (atghPrev.getType() == I_TILE) {
      if (patchModePrev == I_INTRA) {
        auto& plrdPrev = pidPrev.getPatchDataUnit().getPLRData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      }
    }
    auto& plrd = ipdu.getPLRData();
    plrd.allocate(sizeU, sizeV);
    plrData(plrd, syntax, asps, bitstream);
    prevPatchSizeU_ = sizeU;
    prevPatchSizeV_ = sizeV;
    predPatchIndex_ += ipdu.getRefPatchIndex() + 1;
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.7  Raw patch data unit syntax
void
V3CReader::rawPatchDataUnit(RawPatchDataUnit& rpdu,
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
    READ_CODE(rpdu.getPatchInAuxiliaryVideoFlag(), 1);  // u(1)
  } else {
    rpdu.getPatchInAuxiliaryVideoFlag() = 0;
  }
  READ_UVLC(rpdu.get2dPosX());               // ue(v)
  READ_UVLC(rpdu.get2dPosY());               // ue(v)
  READ_UVLC(rpdu.get2dSizeXMinus1());        // ue(v)
  READ_UVLC(rpdu.get2dSizeYMinus1());        // ue(v)
  READ_CODE(rpdu.get3dOffsetU(), bitCount);  // u(v)
  READ_CODE(rpdu.get3dOffsetV(), bitCount);  // u(v)
  READ_CODE(rpdu.get3dOffsetD(), bitCount);  // u(v)
  READ_UVLC(rpdu.getRawPointsMinus1());      // ue(v)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.6.x EOM patch data unit syntax
void
V3CReader::eomPatchDataUnit(EOMPatchDataUnit& epdu,
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
    READ_CODE(epdu.getPatchInAuxiliaryVideoFlag(), 1);  // u(1)
  } else {
    epdu.getPatchInAuxiliaryVideoFlag() = 0;
  }
  READ_UVLC(epdu.get2dPosX());            // ue(v)
  READ_UVLC(epdu.get2dPosY());            // ue(v)
  READ_UVLC(epdu.get2dSizeXMinus1());     // ue(v)
  READ_UVLC(epdu.get2dSizeYMinus1());     // ue(v)
  READ_UVLC(epdu.getPatchCountMinus1());  // ue(v)
  for (size_t i = 0; i < epdu.getPatchCountMinus1() + 1; i++) {
    READ_UVLC(epdu.getAssociatedPatchesIdx(i));  // ue(v)
    READ_UVLC(epdu.getPoints(i));                // ue(v)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.7.9 Point local reconstruction data syntax
void
V3CReader::plrData(PLRData&                       plrd,
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
        READ_CODE(plrd.getLevelFlag(), 1);  // u(1)
      } else {
        plrd.getLevelFlag() = 1;
      }
      if (!plrd.getLevelFlag()) {
        for (size_t i = 0; i < blockCount; i++) {
          READ_CODE(plrd.getBlockPresentFlag(i), 1);  // u(1)
          if (plrd.getBlockPresentFlag(i)) {
            READ_CODE(plrd.getBlockModeMinus1(i), bitCountMode);  // u(v)
          }
        }
      } else {
        READ_CODE(plrd.getPresentFlag(), 1);  // u(1)
        if (plrd.getPresentFlag()) {
          READ_CODE(plrd.getModeMinus1(), bitCountMode);  // u(v)
        }
      }
    }
  }
}

// 8.3.8 Supplemental enhancement information message syntax
void
V3CReader::seiMessage(Bitstream&       bitstream,
                      V3cBitstream&    syntax,
                      AtlasNalUnitType nalUnitType,
                      PCCSEI&          sei) {
  TRACE_BITSTREAM_IN("%s", __func__);
  int32_t payloadType = 0;
  int32_t payloadSize = 0;
  int32_t byte        = 0;
  do {
    READ_CODE(byte, 8);  // u(8)
    payloadType += byte;
  } while (byte == 0xff);
  do {
    READ_CODE(byte, 8);  // u(8)
    payloadSize += byte;
  } while (byte == 0xff);
  seiPayload(bitstream,
             syntax,
             nalUnitType,
             static_cast<SeiPayloadType>(payloadType),
             payloadSize,
             sei);
  // if ( static_cast<SeiPayloadType>( payloadType ) == DECODED_ATLAS_INFORMATION_HASH ) {
  //   syntax.getSeiHash().push_back(
  //       static_cast<SEIDecodedAtlasInformationHash&>( *( sei.getSeiSuffix().back().get() ) ) );
  // }
}

// C.2 Sample stream V3C unit syntax and semantics
// C.2.1 Sample stream V3C header syntax
void
V3CReader::sampleStreamV3CHeader(Bitstream&           bitstream,
                                 SampleStreamV3CUnit& ssvu) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  READ_CODE(ssvu.getSsvhUnitSizePrecisionBytesMinus1(), 3);  // u(3)
  READ_CODE(zero, 5);                                        // u(5)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// C.2.2 Sample stream NAL unit syntax
void
V3CReader::sampleStreamV3CUnit(Bitstream&           bitstream,
                               SampleStreamV3CUnit& ssvu,
                               V3CUnit&             v3cUnit) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(v3cUnit.getSize(),
            8 * (ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1));  // u(v)
  auto pos = bitstream.getPosition();
  READ_VECTOR(v3cUnit.getBitstream().vector(), v3cUnit.getSize());
  uint8_t v3cUnitType8 = v3cUnit.getBitstream().buffer()[0];
  auto    v3cUnitType  = static_cast<V3CUnitType>(v3cUnitType8 >> 3);
  v3cUnit.getType() = v3cUnitType;
  printf("sampleStreamV3CUnit => %s \n", toString( v3cUnit.getType()).c_str());
  fflush(stdout);
  TRACE_BITSTREAM_OUT(
    "%s type = %s ", __func__, toString(v3cUnit.getType()).c_str());
}

// D.2 Sample stream NAL unit syntax and semantics
// D.2.1 Sample stream NAL header syntax
void
V3CReader::sampleStreamNalHeader(Bitstream&           bitstream,
                                 SampleStreamNalUnit& ssnu) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  READ_CODE(ssnu.getSizePrecisionBytesMinus1(), 3);  // u(3)
  READ_CODE(zero, 5);                                // u(5)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// D.2.2 Sample stream NAL unit syntax
void
V3CReader::sampleStreamNalUnit(V3cBitstream&        syntax,
                               bool                 isAtlas,
                               Bitstream&           bitstream,
                               SampleStreamNalUnit& ssnu,
                               size_t               index,
                               PCCSEI&              prefixSEI) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& nalu = ssnu.getNalUnit(index);
  READ_CODE(nalu.getSize(),
            8 * (ssnu.getSizePrecisionBytesMinus1() + 1));  // u(v)
  nalu.allocate();
  Bitstream ssnuBitstream;
#if defined(CONFORMANCE_TRACE) || defined(BITSTREAM_TRACE)
  ssnuBitstream.setTrace(true);
  ssnuBitstream.setLogger(*logger_);
#endif
  bitstream.copyTo(ssnuBitstream, nalu.getSize());
  nalUnitHeader(ssnuBitstream, nalu);
  if (isAtlas) {
    auto  naluType = AtlasNalUnitType(nalu.getType());
    auto& atlas    = syntax.getAtlas();
    switch (naluType) {
    case ATLAS_NAL_ASPS:
      atlasSequenceParameterSetRbsp(
        atlas.addAtlasSequenceParameterSet(), syntax, ssnuBitstream);
      break;
    case ATLAS_NAL_AFPS:
      atlasFrameParameterSetRbsp(
        atlas.addAtlasFrameParameterSet(), syntax, ssnuBitstream);
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
        atlas.addAtlasTileLayer(), syntax, naluType, ssnuBitstream);
      atlas.getAtlasTileLayerList().back().getSEI().getSeiPrefix() =
        prefixSEI.getSeiPrefix();
      prefixSEI.getSeiPrefix().clear();
      break;
    case ATLAS_NAL_PREFIX_ESEI:
    case ATLAS_NAL_PREFIX_NSEI:
      seiRbsp(syntax, ssnuBitstream, naluType, prefixSEI);
      break;
    case ATLAS_NAL_SUFFIX_ESEI:
    case ATLAS_NAL_SUFFIX_NSEI:
      seiRbsp(syntax,
              ssnuBitstream,
              naluType,
              atlas.getAtlasTileLayerList().back().getSEI());
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
        baseMesh.addBaseMeshSequenceParameterSet(), syntax, ssnuBitstream);
      break;
    case BASEMESH_NAL_BMFPS:
      baseMeshFrameParameterSetRbsp(
        baseMesh.addBaseMeshFrameParameterSet(), syntax, ssnuBitstream);
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
        baseMesh.addBaseMeshTileLayer(), naluType, syntax, ssnuBitstream);
      break;
    default:
      fprintf(stderr,
              "sampleStreamNalUnit type = %d not supported\n",
              static_cast<int32_t>(nalu.getType()));
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2  SEI payload syntax
// F.2.1  General SEI message syntax
void
V3CReader::seiPayload(Bitstream&       bitstream,
                      V3cBitstream&    syntax,
                      AtlasNalUnitType nalUnitType,
                      SeiPayloadType   payloadType,
                      size_t           payloadSize,
                      PCCSEI&          seiList) {
  TRACE_BITSTREAM_IN("%s", __func__);
  SEI& sei = seiList.addSei(nalUnitType, payloadType);
  printf("        seiMessage: type = %d %s payloadSize = %zu \n",
         payloadType,
         toString(payloadType).c_str(),
         payloadSize);
  fflush(stdout);
  if (nalUnitType == ATLAS_NAL_PREFIX_ESEI
      || nalUnitType == ATLAS_NAL_PREFIX_NSEI) {
    if (payloadType == BUFFERING_PERIOD) {  // 0
      bufferingPeriod(bitstream, sei);
    } else if (payloadType == ATLAS_FRAME_TIMING) {  // 1
      assert(seiList.seiIsPresent(ATLAS_NAL_PREFIX_NSEI, BUFFERING_PERIOD));
      auto& bpsei =
        *seiList.getLastSei(ATLAS_NAL_PREFIX_NSEI, BUFFERING_PERIOD);
      atlasFrameTiming(bitstream, sei, bpsei, false);
    } else if (payloadType == FILLER_PAYLOAD) {  // 2
      fillerPayload(bitstream, sei, payloadSize);
    } else if (payloadType == USER_DATAREGISTERED_ITUTT35) {  // 3
      userDataRegisteredItuTT35(bitstream, sei, payloadSize);
    } else if (payloadType == USER_DATA_UNREGISTERED) {  // 4
      userDataUnregistered(bitstream, sei, payloadSize);
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
      reservedSeiMessage(bitstream, sei, payloadSize);
    }
  } else {
    if (payloadType == FILLER_PAYLOAD) {  // 2
      fillerPayload(bitstream, sei, payloadSize);
    } else if (payloadType == USER_DATAREGISTERED_ITUTT35) {  // 3
      userDataRegisteredItuTT35(bitstream, sei, payloadSize);
    } else if (payloadType == USER_DATA_UNREGISTERED) {  // 4
      userDataUnregistered(bitstream, sei, payloadSize);
    } else if (payloadType == DECODED_ATLAS_INFORMATION_HASH) {  // 21
      decodedAtlasInformationHash(bitstream, sei);
    } else {
      reservedSeiMessage(bitstream, sei, payloadSize);
    }
  }
  if (moreDataInPayload(bitstream)) {
    if (payloadExtensionPresent(bitstream)) {
      uint32_t zero = 0;
      READ_CODE(zero, 1);  // u(v)
    }
    byteAlignment(bitstream);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.2  Filler payload SEI message syntax
void
V3CReader::fillerPayload(Bitstream& bitstream, SEI& sei, size_t payloadSize) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t code = 0xFF;
  for (size_t k = 0; k < payloadSize; k++) {
    READ_CODE(code, 8);  // f(8) equal to 0xFF
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
void
V3CReader::userDataRegisteredItuTT35(Bitstream& bitstream,
                                     SEI&       seiAbstract,
                                     size_t     payloadSize) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIUserDataRegisteredItuTT35&>(seiAbstract);
  READ_CODE(sei.getCountryCode(), 8);  // b(8)
  payloadSize--;
  if (sei.getCountryCode() == 0xFF) {
    READ_CODE(sei.getCountryCodeExtensionByte(), 8);  // b(8)
    payloadSize--;
  }
  auto& payload = sei.getPayloadByte();
  payload.resize(payloadSize);
  for (auto& element : payload) {
    READ_CODE(element, 8);  // b(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.4  User data unregistered SEI message syntax
void
V3CReader::userDataUnregistered(Bitstream& bitstream,
                                SEI&       seiAbstract,
                                size_t     payloadSize) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIUserDataUnregistered&>(seiAbstract);
  for (size_t i = 0; i < 16; i++) {
    READ_CODE(sei.getUuidIsoIec11578(i), 8);  // u(128) <=> 16 * u(8)
  }
  payloadSize -= 16;
  sei.getUserDataPayloadByte().resize(payloadSize);
  for (size_t i = 0; i < payloadSize; i++) {
    READ_CODE(sei.getUserDataPayloadByte(i), 8);  // b(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.5  Recovery point SEI message syntax
void
V3CReader::recoveryPoint(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIRecoveryPoint&>(seiAbstract);
  READ_SVLC(sei.getRecoveryAfocCnt());    // se(v)
  READ_CODE(sei.getExactMatchFlag(), 1);  // u(1)
  READ_CODE(sei.getBrokenLinkFlag(), 1);  // u(1)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.6  No reconstruction SEI message syntax
void
V3CReader::noReconstruction(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.7  Reserved SEI message syntax
void
V3CReader::reservedSeiMessage(Bitstream& bitstream,
                              SEI&       seiAbstract,
                              size_t     payloadSize) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIReservedSeiMessage&>(seiAbstract);
  sei.getPayloadByte().resize(payloadSize);
  for (size_t i = 0; i < payloadSize; i++) {
    READ_CODE(sei.getPayloadByte(i), 8);  // b(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.8  SEI manifest SEI message syntax
void
V3CReader::seiManifest(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIManifest&>(seiAbstract);
  READ_CODE(sei.getNumSeiMsgTypes(), 16);  // u(16)
  sei.allocate();
  for (size_t i = 0; i < sei.getNumSeiMsgTypes(); i++) {
    READ_CODE(sei.getSeiPayloadType(i), 16);  // u(16)
    READ_CODE(sei.getSeiDescription(i), 8);   // u(8)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.9  SEI prefix indication SEI message syntax
void
V3CReader::seiPrefixIndication(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIPrefixIndication&>(seiAbstract);
  READ_CODE(sei.getPrefixSeiPayloadType(), 16);          // u(16)
  READ_CODE(sei.getNumSeiPrefixIndicationsMinus1(), 8);  // u(8)
  sei.getNumBitsInPrefixIndicationMinus1().resize(
    sei.getNumSeiPrefixIndicationsMinus1() + 1, 0);
  sei.getSeiPrefixDataBit().resize(sei.getNumSeiPrefixIndicationsMinus1() + 1);
  for (size_t i = 0; i <= sei.getNumSeiPrefixIndicationsMinus1(); i++) {
    READ_CODE(sei.getNumBitsInPrefixIndicationMinus1(i), 16);  // u(16)
    sei.getSeiPrefixDataBit(i).resize(
      sei.getNumBitsInPrefixIndicationMinus1(i), false);
    for (size_t j = 0; j <= sei.getNumBitsInPrefixIndicationMinus1(i); j++) {
      READ_CODE(sei.getSeiPrefixDataBit(i, j), 1);  // u(1)
    }
    while (!bitstream.byteAligned()) {
      uint32_t one = 0;
      READ_CODE(one, 1);  // f(1): equal to 1
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.10  Active substreams SEI message syntax
void
V3CReader::activeSubBitstreams(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIActiveSubBitstreams&>(seiAbstract);
  READ_CODE(sei.getActiveSubBitstreamsCancelFlag(), 1);  // u(1)
  if (!sei.getActiveSubBitstreamsCancelFlag()) {
    READ_CODE(sei.getActiveAttributesChangesFlag(), 1);    // u(1)
    READ_CODE(sei.getActiveMapsChangesFlag(), 1);          // u(1)
    READ_CODE(sei.getAuxiliarySubstreamsActiveFlag(), 1);  // u(1)
    if (sei.getActiveAttributesChangesFlag()) {
      READ_CODE(sei.getAllAttributesActiveFlag(), 1);  // u(1)
      if (!sei.getAllAttributesActiveFlag()) {
        READ_CODE(sei.getActiveAttributeCountMinus1(), 7);  // u(7)
        sei.getActiveAttributeIdx().resize(
          sei.getActiveAttributeCountMinus1() + 1, 0);
        for (size_t i = 0; i <= sei.getActiveAttributeCountMinus1(); i++) {
          READ_CODE(sei.getActiveAttributeIdx(i), 7);  // u(7)
        }
      }
    }
    if (sei.getActiveMapsChangesFlag()) {
      READ_CODE(sei.getAllMapsActiveFlag(), 1);  // u(1)
      if (!sei.getAllMapsActiveFlag()) {
        READ_CODE(sei.getActiveMapCountMinus1(), 4);  // u(4)
        sei.getActiveMapIdx().resize(sei.getActiveMapCountMinus1() + 1, 0);
        for (size_t i = 0; i <= sei.getActiveMapCountMinus1(); i++) {
          READ_CODE(sei.getActiveMapIdx(i), 4);  // u(4)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.11  Component codec mapping SEI message syntax
void
V3CReader::componentCodecMapping(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIComponentCodecMapping&>(seiAbstract);
  READ_CODE(sei.getComponentCodecCancelFlag(), 1);  // u(1)
  if (!sei.getComponentCodecCancelFlag()) {
    READ_CODE(sei.getCodecMappingsCountMinus1(), 8);  // u(8)
    sei.allocate();
    for (size_t i = 0; i <= sei.getCodecMappingsCountMinus1(); i++) {
      READ_CODE(sei.getCodecId(i), 8);                  // u(8)
      READ_STRING(sei.getCodec4cc(sei.getCodecId(i)));  // st(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.12  Volumetric Tiling SEI message syntax
// F.2.12.1 Scene object information SEI message syntax
void
V3CReader::sceneObjectInformation(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEISceneObjectInformation&>(seiAbstract);
  READ_CODE(sei.getPersistenceFlag(), 1);  // u(1)
  READ_CODE(sei.getResetFlag(), 1);        // u(1)
  READ_UVLC(sei.getNumObjectUpdates());    // ue(v)
  sei.allocateObjectIdx();
  if (sei.getNumObjectUpdates() > 0) {
    READ_CODE(sei.getSimpleObjectsFlag(), 1);  // u(1)
    if (static_cast<int>(sei.getSimpleObjectsFlag()) == 0) {
      READ_CODE(sei.getObjectLabelPresentFlag(), 1);       // u(1)
      READ_CODE(sei.getPriorityPresentFlag(), 1);          // u(1)
      READ_CODE(sei.getObjectHiddenPresentFlag(), 1);      // u(1)
      READ_CODE(sei.getObjectDependencyPresentFlag(), 1);  // u(1)
      READ_CODE(sei.getVisibilityConesPresentFlag(), 1);   // u(1)
      READ_CODE(sei.get3dBoundingBoxPresentFlag(), 1);     // u(1)
      READ_CODE(sei.getCollisionShapePresentFlag(), 1);    // u(1)
      READ_CODE(sei.getPointStylePresentFlag(), 1);        // u(1)
      READ_CODE(sei.getMaterialIdPresentFlag(), 1);        // u(1)
      READ_CODE(sei.getExtensionPresentFlag(), 1);         // u(1)
    } else {
      sei.getObjectLabelPresentFlag()      = false;
      sei.getPriorityPresentFlag()         = false;
      sei.getObjectHiddenPresentFlag()     = false;
      sei.getObjectDependencyPresentFlag() = false;
      sei.getVisibilityConesPresentFlag()  = false;
      sei.get3dBoundingBoxPresentFlag()    = false;
      sei.getCollisionShapePresentFlag()   = false;
      sei.getPointStylePresentFlag()       = false;
      sei.getMaterialIdPresentFlag()       = false;
      sei.getExtensionPresentFlag()        = false;
    }
    if (sei.get3dBoundingBoxPresentFlag()) {
      READ_CODE(sei.get3dBoundingBoxScaleLog2(), 5);        // u(5)
      READ_CODE(sei.get3dBoundingBoxPrecisionMinus8(), 5);  // u(5)
    }
    READ_CODE(sei.getLog2MaxObjectIdxUpdated(), 5);  // u(5)
    if (sei.getObjectDependencyPresentFlag()) {
      READ_CODE(sei.getLog2MaxObjectDependencyIdx(), 5);  // u(5)
    }
    for (size_t i = 0; i <= sei.getNumObjectUpdates(); i++) {
      assert(sei.getObjectIdx().size() >= sei.getNumObjectUpdates());
      READ_CODE(sei.getObjectIdx(i),
                sei.getLog2MaxObjectIdxUpdated());  // u(v)
      size_t k = sei.getObjectIdx(i);
      sei.allocate(k + 1);
      READ_CODE(sei.getObjectCancelFlag(k), 1);  // u(1)
      if (sei.getObjectCancelFlag(k)) {
        if (sei.getObjectLabelPresentFlag()) {
          READ_CODE(sei.getObjectLabelUpdateFlag(k), 1);  // u(1)
          if (sei.getObjectLabelUpdateFlag(k)) {
            READ_UVLC(sei.getObjectLabelIdx(k));  // ue(v)
          }
        }
        if (sei.getPriorityPresentFlag()) {
          READ_CODE(sei.getPriorityUpdateFlag(k), 1);  // u(1)
          if (sei.getPriorityUpdateFlag(k)) {
            READ_CODE(sei.getPriorityValue(k), 4);  // u(4)
          }
        }
        if (sei.getObjectHiddenPresentFlag()) {
          READ_CODE(sei.getObjectHiddenFlag(k), 1);  // u(1)
        }
        if (sei.getObjectDependencyPresentFlag()) {
          READ_CODE(sei.getObjectDependencyUpdateFlag(k), 1);  // u(1)
          if (sei.getObjectDependencyUpdateFlag(k)) {
            READ_CODE(sei.getObjectNumDependencies(k), 4);  // u(4)
            sei.allocateObjectNumDependencies(k,
                                              sei.getObjectNumDependencies(k));
            size_t bitCount =
              ceil(log2(sei.getObjectNumDependencies(k)) + 0.5);
            for (size_t j = 0; j < sei.getObjectNumDependencies(k); j++) {
              READ_CODE(sei.getObjectDependencyIdx(k, j), bitCount);  // u(v)
            }
          }
        }
        if (sei.getVisibilityConesPresentFlag()) {
          READ_CODE(sei.getVisibilityConesUpdateFlag(k), 1);  // u(1)
          if (sei.getVisibilityConesUpdateFlag(k)) {
            READ_CODE(sei.getDirectionX(k), 16);  // u(16)
            READ_CODE(sei.getDirectionY(k), 16);  // u(16)
            READ_CODE(sei.getDirectionZ(k), 16);  // u(16)
            READ_CODE(sei.getAngle(k), 16);       // u(16)
          }
        }  // cones

        if (sei.get3dBoundingBoxPresentFlag()) {
          READ_CODE(sei.get3dBoundingBoxUpdateFlag(k), 1);  // u(1)
          if (sei.get3dBoundingBoxUpdateFlag(k)) {
            READ_UVLC(sei.get3dBoundingBoxX(k));       // ue(v)
            READ_UVLC(sei.get3dBoundingBoxY(k));       // ue(v)
            READ_UVLC(sei.get3dBoundingBoxZ(k));       // ue(v)
            READ_UVLC(sei.get3dBoundingBoxDeltaX(k));  // ue(v)
            READ_UVLC(sei.get3dBoundingBoxDeltaY(k));  // ue(v)
            READ_UVLC(sei.get3dBoundingBoxDeltaZ(k));  // ue(v)
          }
        }  // 3dBB

        if (sei.getCollisionShapePresentFlag()) {
          READ_CODE(sei.getCollisionShapeUpdateFlag(k), 1);  // u(1)
          if (sei.getCollisionShapeUpdateFlag(k)) {
            READ_CODE(sei.getCollisionShapeId(k), 16);  // u(16)
          }
        }  // collision
        if (sei.getPointStylePresentFlag()) {
          READ_CODE(sei.getPointStyleUpdateFlag(k), 1);  // u(1)
          if (sei.getPointStyleUpdateFlag(k)) {
            READ_CODE(sei.getPointShapeId(k), 8);  // u(8)
            READ_CODE(sei.getPointSize(k), 16);    // u(16)
          }
        }  // pointstyle
        if (sei.getMaterialIdPresentFlag()) {
          READ_CODE(sei.getMaterialIdUpdateFlag(k), 1);  // u(1)
          if (sei.getMaterialIdUpdateFlag(k)) {
            READ_CODE(sei.getMaterialId(k), 16);  // u(16)
          }
        }  // materialid
      }    // sei.getObjectCancelFlag(k)
    }      // for(size_t i=0; i<=sei.getNumObjectUpdates(); i++)
  }        // if( sei.getNumObjectUpdates() > 0 )
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.12.2 Object label information SEI message syntax
void
V3CReader::objectLabelInformation(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&    sei  = static_cast<SEIObjectLabelInformation&>(seiAbstract);
  uint32_t zero = 0;
  READ_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (!sei.getCancelFlag()) {
    READ_CODE(sei.getLabelLanguagePresentFlag(), 1);  // u(1)
    if (sei.getLabelLanguagePresentFlag()) {
      while (!bitstream.byteAligned()) {
        READ_CODE(zero, 1);  // u(1)
      }
      READ_STRING(sei.getLabelLanguage());  // st(v)
    }
    READ_UVLC(sei.getNumLabelUpdates());  // ue(v)
    sei.allocate();
    for (size_t i = 0; i < sei.getNumLabelUpdates(); i++) {
      READ_UVLC(sei.getLabelIdx(i));           // ue(v)
      READ_CODE(sei.getLabelCancelFlag(), 1);  // u(1)
      if (!sei.getLabelCancelFlag()) {
        while (!bitstream.byteAligned()) {
          READ_CODE(zero, 1);  // u(1)
        }
        READ_STRING(sei.getLabel(sei.getLabelIdx(i)));  // st(v)
      }
    }
    READ_CODE(sei.getPersistenceFlag(), 1);  // u(1)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
};

// F.2.12.3 Patch information SEI message syntax
void
V3CReader::patchInformation(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIPatchInformation&>(seiAbstract);
  READ_CODE(sei.getPersistenceFlag(), 1);  // u(1)
  READ_CODE(sei.getResetFlag(), 1);        // u(1)
  READ_UVLC(sei.getNumTileUpdates());      // ue(v)
  if (sei.getNumTileUpdates() > 0) {
    READ_CODE(sei.getLog2MaxObjectIdxTracked(), 5);  // u(5)
    READ_CODE(sei.getLog2MaxPatchIdxUpdated(), 4);   // u(4)
  }
  for (size_t i = 0; i < sei.getNumTileUpdates(); i++) {
    READ_UVLC(sei.getTileId(i));  // ue(v)
    size_t j = sei.getTileId(i);
    READ_CODE(sei.getTileCancelFlag(j), 1);  // u(1)
    READ_UVLC(sei.getNumPatchUpdates(j));    // ue(v)
    for (size_t k = 0; k < sei.getNumPatchUpdates(j); k++) {
      READ_CODE(sei.getPatchIdx(j, k),
                sei.getLog2MaxPatchIdxUpdated());  // u(v)
      auto p = sei.getPatchIdx(j, k);
      READ_CODE(sei.getPatchCancelFlag(j, p), 1);  // u(1)
      if (!sei.getPatchCancelFlag(j, p)) {
        READ_UVLC(sei.getPatchNumberOfObjectsMinus1(j, p));  // ue(v)
        for (size_t n = 0; n < sei.getPatchNumberOfObjectsMinus1(j, p) + 1;
             n++) {
          READ_CODE(sei.getPatchObjectIdx(j, p, n),
                    sei.getLog2MaxObjectIdxTracked());  // u(v)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
};

// F.2.12.4 Volumetric rectangle information SEI message syntax
void
V3CReader::volumetricRectangleInformation(Bitstream& bitstream,
                                          SEI&       seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIVolumetricRectangleInformation&>(seiAbstract);
  READ_CODE(sei.getPersistenceFlag(), 1);    // u(1)
  READ_CODE(sei.getResetFlag(), 1);          // u(1)
  READ_UVLC(sei.getNumRectanglesUpdates());  // ue(v)
  if (sei.getNumRectanglesUpdates() > 0) {
    READ_CODE(sei.getLog2MaxObjectIdxTracked(), 5);     // u(5)
    READ_CODE(sei.getLog2MaxRectangleIdxUpdated(), 4);  // u(4)
  }
  for (size_t k = 0; k < sei.getNumRectanglesUpdates(); k++) {
    READ_CODE(sei.getRectangleIdx(k),
              sei.getLog2MaxRectangleIdxUpdated());  // u(v)
    auto p = sei.getRectangleIdx(k);
    READ_CODE(sei.getRectangleCancelFlag(p), 1);  // u(1)
    if (!sei.getRectangleCancelFlag(p)) {
      sei.allocate(p + 1);
      READ_CODE(sei.getBoundingBoxUpdateFlag(p), 1);  // u(1)
      if (sei.getBoundingBoxUpdateFlag(p)) {
        READ_UVLC(sei.getBoundingBoxTop(p));     // ue(v)
        READ_UVLC(sei.getBoundingBoxLeft(p));    // ue(v)
        READ_UVLC(sei.getBoundingBoxWidth(p));   // ue(v)
        READ_UVLC(sei.getBoundingBoxHeight(p));  // ue(v)
      }
      READ_UVLC(sei.getRectangleNumberOfObjectsMinus1(p));  // ue(v)
      sei.allocateRectangleObjectIdx(
        p, sei.getRectangleNumberOfObjectsMinus1(p) + 1);
      for (size_t n = 0; n < sei.getRectangleNumberOfObjectsMinus1(p) + 1;
           n++) {
        READ_CODE(sei.getRectangleObjectIdx(p, n),
                  sei.getLog2MaxObjectIdxTracked());  // u(v)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
};

// F.2.12.5 Atlas object information  SEI message syntax
void
V3CReader::atlasObjectInformation(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIAtlasInformation&>(seiAbstract);
  READ_CODE(sei.getPersistenceFlag(), 1);   // u(1)
  READ_CODE(sei.getResetFlag(), 1);         // u(1)
  READ_CODE(sei.getNumAtlasesMinus1(), 6);  // u(6)
  READ_UVLC(sei.getNumUpdates());           // ue(v)
  sei.allocateAltasId();
  sei.allocateObjectIdx();
  if (sei.getNumUpdates() > 0) {
    READ_CODE(sei.getLog2MaxObjectIdxTracked(), 5);  // u(5)
    for (size_t i = 0; i < sei.getNumAtlasesMinus1() + 1; i++) {
      READ_CODE(sei.getAtlasId(i), 5);  // u(6)
    }
    for (size_t i = 0; i < sei.getNumUpdates() + 1; i++) {
      READ_CODE(sei.getObjectIdx(i),
                sei.getLog2MaxObjectIdxTracked());  // u(v)
      for (size_t j = 0; j < sei.getNumAtlasesMinus1() + 1; j++) {
        READ_CODE(sei.getObjectInAtlasPresentFlag(i, j), 1);  // u(1)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.13  Buffering period SEI message syntax
void
V3CReader::bufferingPeriod(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIBufferingPeriod&>(seiAbstract);
  READ_CODE(sei.getNalHrdParamsPresentFlag(), 1);             // u(1)
  READ_CODE(sei.getAclHrdParamsPresentFlag(), 1);             // u(1)
  READ_CODE(sei.getInitialCabRemovalDelayLengthMinus1(), 5);  // u(5)
  READ_CODE(sei.getAuCabRemovalDelayLengthMinus1(), 5);       // u(5)
  READ_CODE(sei.getDabOutputDelayLengthMinus1(), 5);          // u(5)
  READ_CODE(sei.getIrapCabParamsPresentFlag(), 1);            // u(1)
  if (sei.getIrapCabParamsPresentFlag()) {
    READ_CODE(sei.getCabDelayOffset(),
              sei.getAuCabRemovalDelayLengthMinus1() + 1);  // u(v)
    READ_CODE(sei.getDabDelayOffset(),
              sei.getDabOutputDelayLengthMinus1() + 1);  // u(v)
  }
  READ_CODE(sei.getConcatenationFlag(), 1);  // u(1)
  READ_CODE(sei.getAtlasCabRemovalDelayDeltaMinus1(),
            sei.getAuCabRemovalDelayLengthMinus1() + 1);  // u(v)
  READ_CODE(sei.getMaxSubLayersMinus1(), 3);              // u(3)
  sei.allocate();
  int32_t bitCount = sei.getInitialCabRemovalDelayLengthMinus1() + 1;
  for (size_t i = 0; i <= sei.getMaxSubLayersMinus1(); i++) {
    READ_CODE(sei.getHrdCabCntMinus1(i), 3);  // u(3)
    if (sei.getNalHrdParamsPresentFlag()) {
      for (size_t j = 0; j < sei.getHrdCabCntMinus1(i) + 1; j++) {
        READ_CODE(sei.getNalInitialCabRemovalDelay(i, j), bitCount);   // u(v)
        READ_CODE(sei.getNalInitialCabRemovalOffset(i, j), bitCount);  // u(v)
        if (sei.getIrapCabParamsPresentFlag()) {
          READ_CODE(sei.getNalInitialAltCabRemovalDelay(i, j),
                    bitCount);  // u(v)
          READ_CODE(sei.getNalInitialAltCabRemovalOffset(i, j),
                    bitCount);  // u(v)
        }
      }
    }
    if (sei.getAclHrdParamsPresentFlag()) {
      for (size_t j = 0; j < sei.getHrdCabCntMinus1(i) + 1; j++) {
        READ_CODE(sei.getAclInitialCabRemovalDelay(i, j), bitCount);   // u(v)
        READ_CODE(sei.getAclInitialCabRemovalOffset(i, j), bitCount);  // u(v)
        if (sei.getIrapCabParamsPresentFlag()) {
          READ_CODE(sei.getAclInitialAltCabRemovalDelay(i, j),
                    bitCount);  // u(v)
          READ_CODE(sei.getAclInitialAltCabRemovalOffset(i, j),
                    bitCount);  // u(v)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.14  Atlas frame timing SEI message syntax
void
V3CReader::atlasFrameTiming(Bitstream& bitstream,
                            SEI&       seiAbstract,
                            SEI&       seiBufferingPeriodAbstract,
                            bool       cabDabDelaysPresentFlag) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei   = static_cast<SEIAtlasFrameTiming&>(seiAbstract);
  auto& bpsei = static_cast<SEIBufferingPeriod&>(seiBufferingPeriodAbstract);
  if (cabDabDelaysPresentFlag) {
    for (uint32_t i = 0; i <= bpsei.getMaxSubLayersMinus1(); i++) {
      READ_CODE(sei.getAftCabRemovalDelayMinus1(i),
                bpsei.getAuCabRemovalDelayLengthMinus1() + 1);  // u(v)
      READ_CODE(sei.getAftDabOutputDelay(i),
                bpsei.getDabOutputDelayLengthMinus1() + 1);  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.15.1 Viewport camera parameters SEI messages syntax
void
V3CReader::viewportCameraParameters(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIViewportCameraParameters&>(seiAbstract);
  READ_CODE(sei.getCameraId(), 10);   // u(10)
  READ_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (sei.getCameraId() > 0 && !sei.getCancelFlag()) {
    READ_CODE(sei.getPersistenceFlag(), 1);              // u(1)
    READ_CODE(sei.getCameraType(), 3);                   // u(3)
    if (sei.getCameraType() == 0) {                      // equirectangular
      READ_CODE(sei.getErpHorizontalFov(), 32);          // u(32)
      READ_CODE(sei.getErpVerticalFov(), 32);            // u(32)
    } else if (sei.getCameraType() == 1) {               // perspective
      READ_FLOAT(sei.getPerspectiveAspectRatio());       // fl(32)
      READ_CODE(sei.getPerspectiveHorizontalFov(), 32);  // u(32)
    } else if (sei.getCameraType() == 2) {               /* orthographic */
      READ_FLOAT(sei.getOrthoAspectRatio());             // fl(32)
      READ_FLOAT(sei.getOrthoHorizontalSize());          // fl(32)
    }
    READ_FLOAT(sei.getClippingNearPlane());  // fl(32)
    READ_FLOAT(sei.getClippingFarPlane());   // fl(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.15.2 Viewport position SEI messages syntax
void
V3CReader::viewportPosition(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIViewportPosition&>(seiAbstract);
  READ_UVLC(sei.getViewportId());                      // ue(v)
  READ_CODE(sei.getCameraParametersPresentFlag(), 1);  // u(1)
  if (sei.getCameraParametersPresentFlag()) {
    READ_CODE(sei.getViewportId(), 10);  // u(10)
  }
  READ_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (!sei.getCancelFlag()) {
    READ_CODE(sei.getPersistenceFlag(), 1);  // u(1)
    for (size_t d = 0; d < 3; d++) {
      READ_FLOAT(sei.getPosition(d));  // fl(32)
    }
    READ_CODE(sei.getRotationQX(), 16);     // i(16)
    READ_CODE(sei.getRotationQY(), 16);     // i(16)
    READ_CODE(sei.getRotationQZ(), 16);     // i(16)
    READ_CODE(sei.getCenterViewFlag(), 1);  //  u(1)
    if (!sei.getCenterViewFlag()) {
      READ_CODE(sei.getLeftViewFlag(), 1);  // u(1)
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.16 Decoded Atlas Information Hash SEI message syntax
void
V3CReader::decodedAtlasInformationHash(Bitstream& bitstream,
                                       SEI&       seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&    sei  = static_cast<SEIDecodedAtlasInformationHash&>(seiAbstract);
  uint32_t zero = 0;
  READ_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (!sei.getCancelFlag()) {
    READ_CODE(sei.getPersistenceFlag(), 1);                      // u(1)
    READ_CODE(sei.getHashType(), 8);                             // u(8)
    READ_CODE(sei.getDecodedHighLevelHashPresentFlag(), 1);      // u(1)
    READ_CODE(sei.getDecodedAtlasHashPresentFlag(), 1);          // u(1)
    READ_CODE(sei.getDecodedAtlasB2pHashPresentFlag(), 1);       // u(1)
    READ_CODE(sei.getDecodedAtlasTilesHashPresentFlag(), 1);     // u(1)
    READ_CODE(sei.getDecodedAtlasTilesB2pHashPresentFlag(), 1);  // u(1)
    READ_CODE(zero, 1);                                          // u(1)
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
      READ_UVLC(sei.getNumTilesMinus1());   // ue(v)
      READ_UVLC(sei.getTileIdLenMinus1());  // ue(v)
      sei.allocateAtlasTilesHash(sei.getNumTilesMinus1() + 1);
      for (size_t t = 0; t <= sei.getNumTilesMinus1(); t++) {
        READ_CODE(sei.getTileId(t), sei.getTileIdLenMinus1() + 1);  // u(v)
      }
      while (!bitstream.byteAligned()) {
        uint32_t one = 1;
        READ_CODE(one, 1);  // f(1): equal to 1
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

void
V3CReader::decodedHighLevelHash(Bitstream& bitstream, SEI& seiAbs) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      READ_CODE(sei.getHighLevelMd5(i), 8);  // b(8)
    }
  } else if (hType == 1) {
    READ_CODE(sei.getHighLevelCrc(), 16);  // u(16)
  } else if (hType == 2) {
    READ_CODE(sei.getHighLevelCheckSum(), 32);  // u(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

void
V3CReader::decodedAtlasHash(Bitstream& bitstream, SEI& seiAbs) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      READ_CODE(sei.getAtlasMd5(i), 8);  // b(8)
    }
  } else if (hType == 1) {
    READ_CODE(sei.getAtlasCrc(), 16);  // u(16)
  } else if (hType == 2) {
    READ_CODE(sei.getAtlasCheckSum(), 32);  // u(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

void
V3CReader::decodedAtlasB2pHash(Bitstream& bitstream, SEI& seiAbs) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      READ_CODE(sei.getAtlasB2pMd5(i), 8);  // b(8)
    }
  } else if (hType == 1) {
    READ_CODE(sei.getAtlasB2pCrc(), 16);  // u(16)
  } else if (hType == 2) {
    READ_CODE(sei.getAtlasB2pCheckSum(), 32);  // u(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

void
V3CReader::decodedAtlasTilesHash(Bitstream& bitstream,
                                 SEI&       seiAbs,
                                 size_t     id) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      READ_CODE(sei.getAtlasTilesMd5(id, i), 8);  // b(8)
    }
  } else if (hType == 1) {
    READ_CODE(sei.getAtlasTilesCrc(id), 16);  // u(16)
  } else if (hType == 2) {
    READ_CODE(sei.getAtlasTilesCheckSum(id), 32);  // u(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

void
V3CReader::decodedAtlasTilesB2pHash(Bitstream& bitstream,
                                    SEI&       seiAbs,
                                    size_t     id) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto&   sei   = static_cast<SEIDecodedAtlasInformationHash&>(seiAbs);
  uint8_t hType = sei.getHashType();
  if (hType == 0) {
    for (size_t i = 0; i < 16; i++) {
      READ_CODE(sei.getAtlasTilesB2pMd5(id, i), 8);  // b(8)
    }
  } else if (hType == 1) {
    READ_CODE(sei.getAtlasTilesB2pCrc(id), 16);  // u(16)
  } else if (hType == 2) {
    READ_CODE(sei.getAtlasTilesB2pCheckSum(id), 32);  // u(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// F.2.17 Time code SEI message syntax
void
V3CReader::timeCode(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEITimeCode&>(seiAbstract);
  READ_CODE(sei.getNumUnitsInTick(), 32);    // u(32)
  READ_CODE(sei.getTimeScale(), 32);         // u(32)
  READ_CODE(sei.getCountingType(), 5);       // u(5)
  READ_CODE(sei.getFullTimestampFlag(), 1);  // u(1)
  READ_CODE(sei.getDiscontinuityFlag(), 1);  // u(1)
  READ_CODE(sei.getCntDroppedFlag(), 1);     // u(1)
  READ_CODE(sei.getNFrames(), 9);            // u(9)
  if (sei.getFullTimestampFlag()) {
    READ_CODE(sei.getSecondsValue(), 6);  // u(6)
    READ_CODE(sei.getMinutesValue(), 6);  // u(6)
    READ_CODE(sei.getHoursValue(), 5);    // u(5)
  } else {
    READ_CODE(sei.getSecondFlag(), 1);  // u(1)
    if (sei.getSecondFlag()) {
      READ_CODE(sei.getSecondsValue(), 6);  // u(6)
      READ_CODE(sei.getMinutesFlag(), 1);   // u(1)
      if (sei.getMinutesFlag()) {
        READ_CODE(sei.getMinutesValue(), 6);  // u(6)
        READ_CODE(sei.getHoursFlag(), 1);     // u(1)
        if (sei.getHoursFlag()) {
          READ_CODE(sei.getHoursValue(), 5);  // u(5)
        }
      }
    }
  }
  READ_CODE(sei.getTimeOffsetLength(), 5);  // u(5)
  if (sei.getTimeOffsetLength() > 0) {
    READ_CODES(sei.getTimeOffsetValue(), sei.getTimeOffsetLength());  // i(v)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.20.2.17 Attribute transformation parameters SEI message syntax
void
V3CReader::attributeTransformationParams(Bitstream& bitstream,
                                         SEI&       seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIAttributeTransformationParams&>(seiAbstract);
  READ_CODE(sei.getCancelFlag(), 1);  // u(1)
  if (!sei.getCancelFlag()) {
    READ_UVLC(sei.getNumAttributeUpdates());  // ue(v)
    sei.allocate();
    for (size_t j = 0; j < sei.getNumAttributeUpdates(); j++) {
      READ_CODE(sei.getAttributeIdx(j), 8);  // u(8)
      size_t index = sei.getAttributeIdx(j);
      READ_CODE(sei.getDimensionMinus1(index), 8);  // u(8)
      sei.allocate(index);
      for (size_t i = 0; i < sei.getDimensionMinus1(index); i++) {
        READ_CODE(sei.getScaleParamsEnabledFlag(index, i), 1);   // u(1)
        READ_CODE(sei.getOffsetParamsEnabledFlag(index, i), 1);  // u(1)
        if (sei.getScaleParamsEnabledFlag(index, i)) {
          READ_CODE(sei.getAttributeScale(index, i), 32);  // u(32)
        }
        if (sei.getOffsetParamsEnabledFlag(index, i)) {
          READ_CODES(sei.getAttributeOffset(index, i), 32);  // i(32)
        }
      }
    }
    READ_CODE(sei.getPersistenceFlag(), 1);  // u(1)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.20.2.18 Occupancy synthesis SEI message syntax
void
V3CReader::occupancySynthesis(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIOccupancySynthesis&>(seiAbstract);
  READ_CODE(sei.getPersistenceFlag(), 1);   // u(1)
  READ_CODE(sei.getResetFlag(), 1);         // u(1)
  READ_CODE(sei.getInstancesUpdated(), 8);  // u(8)
  sei.allocate();
  for (size_t i = 0; i < sei.getInstancesUpdated(); i++) {
    READ_CODE(sei.getInstanceIndex(i), 8);  // u(8)
    size_t k = sei.getInstanceIndex(i);
    READ_CODE(sei.getInstanceCancelFlag(k), 1);  // u(1)
    if (!sei.getInstanceCancelFlag(k)) {
      READ_UVLC(sei.getMethodType(k));  // ue(v)
      if (sei.getMethodType(k) == 1) {
        READ_CODE(sei.getPbfLog2ThresholdMinus1(k), 2);  // u(2)
        READ_CODE(sei.getPbfPassesCountMinus1(k), 2);    // u(2)
        READ_CODE(sei.getPbfFilterSizeMinus1(k), 3);     // u(3)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.20.2.19 Geometry smoothing SEI message syntax
void
V3CReader::geometrySmoothing(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIGeometrySmoothing&>(seiAbstract);
  READ_CODE(sei.getPersistenceFlag(), 1);   // u(1)
  READ_CODE(sei.getResetFlag(), 1);         // u(1)
  READ_CODE(sei.getInstancesUpdated(), 8);  // u(8)
  sei.allocate();
  for (size_t i = 0; i < sei.getInstancesUpdated(); i++) {
    READ_CODE(sei.getInstanceIndex(i), 8);  // u(8)
    size_t k = sei.getInstanceIndex(i);
    READ_CODE(sei.getInstanceCancelFlag(k), 1);  // u(1)
    if (!sei.getInstanceCancelFlag(k)) {
      READ_UVLC(sei.getMethodType(k));  // ue(v)
      if (sei.getMethodType(k) == 1) {
        READ_CODE(sei.getFilterEomPointsFlag(k), 1);  // u(1)
        READ_CODE(sei.getGridSizeMinus2(k), 7);       // u(7)
        READ_CODE(sei.getThreshold(k), 8);            // u(8)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.20.2.20 Attribute smoothing SEI message syntax
void
V3CReader::attributeSmoothing(Bitstream& bitstream, SEI& seiAbstract) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& sei = static_cast<SEIAttributeSmoothing&>(seiAbstract);
  READ_CODE(sei.getPersistenceFlag(), 1);    // u(1)
  READ_CODE(sei.getResetFlag(), 1);          // u(1)
  READ_UVLC(sei.getNumAttributesUpdated());  // ue(v)
  sei.allocate();
  for (size_t j = 0; j < sei.getNumAttributesUpdated(); j++) {
    READ_CODE(sei.getAttributeIdx(j), 7);  // u(7)
    size_t k = sei.getAttributeIdx(j);
    READ_CODE(sei.getAttributeSmoothingCancelFlag(k), 1);  // u(1)
    READ_CODE(sei.getInstancesUpdated(k), 8);              // u(8)
    for (size_t i = 0; i < sei.getInstancesUpdated(k); i++) {
      size_t m = 0;
      READ_CODE(m, 8);  // u(8)
      sei.allocate(k + 1, m + 1);
      sei.getInstanceIndex(k, i) = m;
      READ_CODE(sei.getInstanceCancelFlag(k, m), 1);  // u(1)
      if (sei.getInstanceCancelFlag(k, m) != 1) {
        READ_UVLC(sei.getMethodType(k, m));  // ue(v)
        if (sei.getMethodType(k, m)) {
          READ_CODE(sei.getFilterEomPointsFlag(k, m), 1);  // u(1)
          READ_CODE(sei.getGridSizeMinus2(k, m), 5);       // u(5)
          READ_CODE(sei.getThreshold(k, m), 8);            // u(8)
          READ_CODE(sei.getThresholdVariation(k, m), 8);   // u(8)
          READ_CODE(sei.getThresholdDifference(k, m), 8);  // u(8)
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2  VUI syntax
// G.2.1  VUI parameters syntax
void
V3CReader::vuiParameters(Bitstream& bitstream, VUIParameters& vp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(vp.getTimingInfoPresentFlag(), 1);  // u(1)
  if (vp.getTimingInfoPresentFlag()) {
    READ_CODE(vp.getNumUnitsInTick(), 32);              // u(32)
    READ_CODE(vp.getTimeScale(), 32);                   // u(32)
    READ_CODE(vp.getPocProportionalToTimingFlag(), 1);  // u(1)
    if (vp.getPocProportionalToTimingFlag()) {
      READ_UVLC(vp.getNumTicksPocDiffOneMinus1());  // ue(v)
    }
    READ_CODE(vp.getHrdParametersPresentFlag(), 1);  // u(1)
    if (vp.getHrdParametersPresentFlag()) {
      hrdParameters(bitstream, vp.getHrdParameters());
    }
  }
  READ_CODE(vp.getTileRestrictionsPresentFlag(), 1);  // u(1)
  if (vp.getTileRestrictionsPresentFlag()) {
    READ_CODE(vp.getFixedAtlasTileStructureFlag(), 1);          // u(1)
    READ_CODE(vp.getFixedVideoTileStructureFlag(), 1);          //	u(1)
    READ_UVLC(vp.getConstrainedTilesAcrossV3cComponentsIdc());  // ue(v)
    READ_UVLC(vp.getMaxNumTilesPerAtlasMinus1());               // ue(v)
  }
  READ_CODE(vp.getCoordinateSystemParametersPresentFlag(), 1);  // u(1)
  if (vp.getCoordinateSystemParametersPresentFlag()) {
    coordinateSystemParameters(bitstream, vp.getCoordinateSystemParameters());
  }
  READ_CODE(vp.getUnitInMetresFlag(), 1);           // u(1)
  READ_CODE(vp.getDisplayBoxInfoPresentFlag(), 1);  // u(1)
  if (vp.getDisplayBoxInfoPresentFlag()) {
    for (size_t d = 0; d < 3; d++) {
      READ_UVLC(vp.getDisplayBoxOrigin(d));  // ue(v)
      READ_UVLC(vp.getDisplayBoxSize(d));    // ue(v)
    }
    READ_CODE(vp.getAnchorPointPresentFlag(), 1);  // u(1)
    if (vp.getAnchorPointPresentFlag()) {
      for (size_t d = 0; d < 3; d++) {
        READ_UVLC(vp.getAnchorPoint(d));  // u(v)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2.2  HRD parameters syntax
void
V3CReader::hrdParameters(Bitstream& bitstream, HrdParameters& hp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(hp.getNalParametersPresentFlag(), 1);  // u(1)
  READ_CODE(hp.getAclParametersPresentFlag(), 1);  // u(1)
  if (hp.getNalParametersPresentFlag() || hp.getAclParametersPresentFlag()) {
    READ_CODE(hp.getBitRateScale(), 4);  // u(4)
    READ_CODE(hp.getCabSizeScale(), 4);  // u(4)
  }
  for (size_t i = 0; i <= hp.getMaxNumSubLayersMinus1(); i++) {
    READ_CODE(hp.getFixedAtlasRateGeneralFlag(i), 1);  // u(1)
    if (!hp.getFixedAtlasRateGeneralFlag(i)) {
      READ_CODE(hp.getFixedAtlasRateWithinCasFlag(i), 1);  // u(1)
    }
    if (hp.getFixedAtlasRateWithinCasFlag(i)) {
      READ_CODE(hp.getElementalDurationInTcMinus1(i), 1);  // ue(v)
    } else {
      READ_CODE(hp.getLowDelayFlag(i), 1);  // u(1)
    }
    if (!hp.getLowDelayFlag(i)) {
      READ_CODE(hp.getCabCntMinus1(i), 1);  // ue(v)
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
V3CReader::hrdSubLayerParameters(Bitstream&             bitstream,
                                 HrdSubLayerParameters& hlsp,
                                 size_t                 cabCnt) {
  TRACE_BITSTREAM_IN("%s", __func__);
  hlsp.allocate(cabCnt + 1);
  for (size_t i = 0; i <= cabCnt; i++) {
    READ_UVLC(hlsp.getBitRateValueMinus1(i));  // ue(v)
    READ_UVLC(hlsp.getCabSizeValueMinus1(i));  // ue(v)
    READ_CODE(hlsp.getCbrFlag(i), 1);          // u(1)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2.4 Maximum coded video resolution syntax
void
V3CReader::maxCodedVideoResolution(Bitstream&               bitstream,
                                   MaxCodedVideoResolution& mcvr) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(mcvr.getOccupancyResolutionPresentFlag(), 1);  // u(1)
  READ_CODE(mcvr.getGeometryResolutionPresentFlag(), 1);   // u(1)
  READ_CODE(mcvr.getAttributeResolutionPresentFlag(), 1);  // u(1)
  if (mcvr.getOccupancyResolutionPresentFlag()) {
    READ_UVLC(mcvr.getOccupancyWidth());   // ue(v)
    READ_UVLC(mcvr.getOccupancyHeight());  // ue(v)
  }
  if (mcvr.getGeometryResolutionPresentFlag()) {
    READ_UVLC(mcvr.getGeometryWidth());   // ue(v)
    READ_UVLC(mcvr.getGeometryHeight());  // ue(v)
  }
  if (mcvr.getAttributeResolutionPresentFlag()) {
    READ_UVLC(mcvr.getAttributeWidth());   // ue(v)
    READ_UVLC(mcvr.getAttributeHeight());  // ue(v)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// G.2.5 Coordinate system parameters syntax
void
V3CReader::coordinateSystemParameters(Bitstream&                  bitstream,
                                      CoordinateSystemParameters& csp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(csp.getForwardAxis(), 2);    // u(2)
  READ_CODE(csp.getDeltaLeftAxis(), 1);  // u(1)
  READ_CODE(csp.getForwardSign(), 1);    // u(1)
  READ_CODE(csp.getLeftSign(), 1);       // u(1)
  READ_CODE(csp.getUpSign(), 1);         // u(1)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.4.1 VPS V-PCC extension syntax
void
V3CReader::vpsVpccExtension(Bitstream& bitstream, VpsVpccExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.1.1 ASPS V-PCC extension syntax
void
V3CReader::aspsVpccExtension(Bitstream&                     bitstream,
                             AtlasSequenceParameterSetRbsp& asps,
                             AspsVpccExtension&             ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(ext.getRemoveDuplicatePointEnableFlag(), 1);
  if (asps.getPixelDeinterleavingFlag() || asps.getPLREnabledFlag()) {
    READ_CODE(ext.getSurfaceThicknessMinus1(), 7);
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.2.1 AFPS V-PCC extension syntax
void
V3CReader::afpsVpccExtension(Bitstream& bitstream, AfpsVpccExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.2.1 AAPS V-PCC extension syntax
void
V3CReader::aapsVpccExtension(Bitstream& bitstream, AapsVpccExtension& ext) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(ext.getCameraParametersPresentFlag(), 1);  // u(1);
  if (ext.getCameraParametersPresentFlag()) {
    atlasCameraParameters(bitstream, ext.getAtlasCameraParameters());
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// H.7.3.6.2.2 Atlas camera parameters syntax
void
V3CReader::atlasCameraParameters(Bitstream&             bitstream,
                                 AtlasCameraParameters& acp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(acp.getCameraModel(), 8);  // u(8)
  if (acp.getCameraModel() == 1) {
    READ_CODE(acp.getScaleEnabledFlag(), 1);     // u(1)
    READ_CODE(acp.getOffsetEnabledFlag(), 1);    // u(1)
    READ_CODE(acp.getRotationEnabledFlag(), 1);  // u(1)
    if (acp.getScaleEnabledFlag()) {
      for (size_t i = 0; i < 3; i++) {
        READ_CODE(acp.getScaleOnAxis(i), 32);  // u(32)
      }
    }
    if (acp.getOffsetEnabledFlag()) {
      for (size_t i = 0; i < 3; i++) {
        READ_CODES(acp.getOffsetOnAxis(i), 32);  // i(32)
      }
    }
    if (acp.getRotationEnabledFlag()) {
      for (size_t i = 0; i < 3; i++) {
        READ_CODES(acp.getRotation(i), 16);  // i(16)
      }
    }
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}
