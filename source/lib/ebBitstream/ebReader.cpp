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
#include "ebBitstream.h"
#include "ebReader.hpp"
#include "meshCoding.hpp"
#include "meshCodingHeader.hpp"
#include "meshPositionEncodingParameters.hpp"
#include "meshPositionDequantizeParameters.hpp"
#include "meshAttributesEncodingParameters.hpp"
#include "meshAttributesDequantizeParameters.hpp"
#include "meshPositionCodingPayload.hpp"
#include "meshPositionDeduplicateInformation.hpp"
#include "meshAttributeCodingPayload.hpp"
#include "meshAttributeExtraData.hpp"
#include "meshTexCoordStretchExtraData.hpp"
#include "meshAttributeDeduplicateInformation.hpp"

using namespace eb;

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

#  define READ_VU(VAR) \
    { \
      VAR = bitstream.readVu(); \
      bitstream.traceBit(format(#VAR), VAR, "vu(v)"); \
    }

#  define READ_VI(VAR) \
    { \
      VAR = bitstream.readVi(); \
      bitstream.traceBit(format(#VAR), VAR, "vi(v)"); \
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
#  define READ_VU(VAR) VAR = bitstream.readVu();
#  define READ_VI(VAR) VAR = bitstream.readVi();
#  define READ_VECTOR(VAR, SIZE) bitstream.read(VAR, SIZE);
#  define READ_VIDEO(VAR, SIZE) bitstream.read(VAR, SIZE);
#endif

EbReader::EbReader() {}
EbReader::~EbReader() = default;

void
EbReader::read(Bitstream& bitstream, MeshCoding& mc) {
  TRACE_BITSTREAM_IN("%s", __func__);
  setGlobal(mc);
  meshCoding(bitstream, mc);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.3 Byte alignment syntax
void
EbReader::byteAlignment(Bitstream& bitstream) {
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
EbReader::lengthAlignment(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  while (!bitstream.byteAligned()) {
    READ_CODE(zero, 1);  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.1	General Mesh coding syntax
void
EbReader::meshCoding(Bitstream& bitstream, MeshCoding& mc) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& mch  = mc.getMeshCodingHeader();
  auto& macp = mc.getMeshAttributeCodingPayload();
  auto& mpcp = mc.getMeshPositionCodingPayload();
  meshCodingHeader(bitstream, mch);
  meshPositionCodingPayload(bitstream, mpcp, mch);
  meshAttributeCodingPayload(bitstream, macp, mch);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.2	Mesh coding header syntax
void
EbReader::meshCodingHeader(Bitstream& bitstream, MeshCodingHeader& mch) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE_CAST(mch.getMeshCodecType(), 2, MeshCodecType);  // u(2)
  meshPositionEncodingParameters(bitstream,
                                 mch.getMeshPositionEncodingParameters());
  READ_CODE(mch.getMeshPositionDequantizeFlag(), 1);  // u(1)

  if (mch.getMeshPositionDequantizeFlag()) {
    meshPositionDequantizeParameters(
      bitstream, mch.getMeshPositionDequantizeParameters());
  }
  READ_CODE(mch.getMeshAttributeCount(), 5);  // u(5)

  mch.allocateAttributes(mch.getMeshAttributeCount());
  for (uint32_t i = 0; i < mch.getMeshAttributeCount(); i++) {
      READ_CODE_CAST(mch.getMeshAttributeType()[i], 3, MeshAttributeType);  // u(3)
    if (mch.getMeshAttributeType()[i] == MeshAttributeType::MESH_ATTR_GENERIC) {
      READ_CODE(mch.getMeshAttributeNumComponentsMinus1()[i], 2);  // u(2)
    }
    meshAttributesEncodingParameters(
      bitstream, mch.getMeshAttributesEncodingParameters()[i]);
    READ_CODE(mch.getMeshAttributeDequantizeFlag()[i], 1);  // u(1)
    if (mch.getMeshAttributeDequantizeFlag()[i])
      meshAttributesDequantizeParameters(bitstream, mch, i);
  }
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.3	Mesh position encoding parameters syntax
void
EbReader::meshPositionEncodingParameters(
  Bitstream&                      bitstream,
  MeshPositionEncodingParameters& mpep) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(mpep.getMeshPositionBitDepthMinus1(), 5);           // u(5)
  READ_UVLC_CAST(mpep.getMeshClersSymbolsEncodingMethod(),      MeshClersSymbolsEncodingMethod);       // ue(v)
  READ_UVLC_CAST(mpep.getMeshPositionPredictionMethod(),        MeshPositionPredictionMethod);         // ue(v)
  READ_UVLC_CAST(mpep.getMeshPositionResidualsEncodingMethod(), MeshPositionResidualsEncodingMethod);  // ue(v)
  READ_UVLC_CAST(mpep.getMeshPositionDeduplicateMethod(),       MeshPositionDeduplicateMethod);        // ue(v)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.4	Mesh position dequantize parameters syntax
void
EbReader::meshPositionDequantizeParameters(
  Bitstream&                        bitstream,
  MeshPositionDequantizeParameters& mpdp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  for (uint32_t i = 0; i < 3; i++) {
    READ_FLOAT(mpdp.getMeshPositionMin(i));  // fl(32)
    READ_FLOAT(mpdp.getMeshPositionMax(i));  // fl(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.5	Mesh attributes encoding parameters syntax
void
EbReader::meshAttributesEncodingParameters(
  Bitstream&                        bitstream,
  MeshAttributesEncodingParameters& maep) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_CODE(maep.getMeshAttributeBitDepthMinus1(), 5);        // u(5)
  READ_CODE(maep.getMeshAttributePerFaceFlag(), 1);           // u(1)
  if (!maep.getMeshAttributePerFaceFlag()) {
    READ_CODE(maep.getMeshAttributeSeparateIndexFlag(), 1);   // u(1)
    if (!maep.getMeshAttributeSeparateIndexFlag())
      READ_UVLC(maep.getMeshAttributeReferenceIndexPlus1());  // ue(v)
  }
  READ_UVLC(maep.getMeshAttributePredictionMethod());         // ue(v)
  READ_UVLC(maep.getMeshAttributeResidualsEncodingMethod());  // ue(v)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.6	Mesh attributes dequantize parameters syntax
void
EbReader::meshAttributesDequantizeParameters(Bitstream&        bitstream,
                                             MeshCodingHeader& mch,
                                             uint32_t          index) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& madp = mch.getMeshAttributesDequantizeParameters(index);
  for (uint32_t i = 0; i < mch.getNumComponents(index); i++) {
    READ_FLOAT(madp.getMeshAttributeMin(i));  // fl(32)
    READ_FLOAT(madp.getMeshAttributeMax(i));  // fl(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.7	Mesh position coding payload syntax
void
EbReader::meshPositionCodingPayload(Bitstream&                 bitstream,
                                    MeshPositionCodingPayload& mpcp,
                                    MeshCodingHeader&          mch) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& NumHandles =       getGlobal().getNumHandles();
  auto& NumPositionStart = getGlobal().getNumPositionStart();
  READ_VU(mpcp.getMeshVertexCount());         //	vu(v)
  READ_VU(mpcp.getMeshClersCount());          //	vu(v)
  READ_VU(mpcp.getMeshCcCount());             //	vu(v)
  READ_VU(mpcp.getMeshVirtualVertexCount());  //	vu(v)
  auto& meshVirtualIndexDelta = mpcp.getMeshVirtualIndexDelta();
  meshVirtualIndexDelta.resize(mpcp.getMeshVirtualVertexCount());
  for (uint32_t i = 0; i < mpcp.getMeshVirtualVertexCount(); i++) {
    READ_VU(meshVirtualIndexDelta[i]);  // vu(v)
  }
  READ_VU(mpcp.getMeshCcWithHandlesCount());  //	vu(v)
  auto&    meshHandlesCcOffset = mpcp.getMeshHandlesCcOffset();
  auto&    meshHandlesCount    = mpcp.getMeshHandlesCount();
  meshHandlesCcOffset.resize(mpcp.getMeshCcWithHandlesCount());
  meshHandlesCount.resize(mpcp.getMeshCcWithHandlesCount());
  NumHandles = 0;
  for (uint32_t i = 0; i < mpcp.getMeshCcWithHandlesCount(); i++) {
    READ_VU(meshHandlesCcOffset[i]);  // vu(v)
    READ_VU(meshHandlesCount[i]);     // vu(v)
    NumHandles += meshHandlesCount[i];
  }
  auto& meshHandleIndexFirstDelta  = mpcp.getMeshHandleIndexFirstDelta();
  auto& meshHandleIndexSecondDelta = mpcp.getMeshHandleIndexSecondDelta();
  meshHandleIndexFirstDelta.resize(NumHandles);
  meshHandleIndexSecondDelta.resize(NumHandles);
  for (size_t i = 0; i < NumHandles; i++) {
    READ_VI(meshHandleIndexFirstDelta[i]);   // vi(v)
    READ_VI(meshHandleIndexSecondDelta[i]);  // vi(v)
  }
  READ_VU(mpcp.getMeshCodedHandleIndexSecondShiftSize());  //	vu(v)
  READ_VECTOR(mpcp.getMeshHandleIndexSecondShift(),
              mpcp.getMeshCodedHandleIndexSecondShiftSize());
  lengthAlignment(bitstream);

  READ_VU(mpcp.getMeshCodedClersSymbolsSize());  //	vu(v)
  READ_VECTOR(mpcp.getMeshClersSymbol(), mpcp.getMeshCodedClersSymbolsSize());
  lengthAlignment(bitstream);

  auto& mpdi = mpcp.getMeshPositionDeduplicateInformation();
  auto& mpep = mch.getMeshPositionEncodingParameters();
  meshPositionDeduplicateInformation(bitstream, mpdi, mpep, mpcp);

  auto& meshPositionStart = mpcp.getMeshPositionStart();
  meshPositionStart.resize(NumPositionStart);
  for (uint32_t i = 0; i < NumPositionStart; i++) {
    meshPositionStart[i].resize(3);
    for (uint32_t j = 0; j < 3; j++) {
      READ_CODE(meshPositionStart[i][j],
                mpep.getMeshPositionBitDepthMinus1() + 1);  // u(v)
    }
  }
  lengthAlignment(bitstream);
  READ_VU(mpcp.getMeshCodedPositionResidualsSize());  //	vu(v)
  READ_VECTOR(mpcp.getMeshPositionResidual(),
              mpcp.getMeshCodedPositionResidualsSize());
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.8	Mesh position deduplicate information syntax
void
EbReader::meshPositionDeduplicateInformation(
  Bitstream&                          bitstream,
  MeshPositionDeduplicateInformation& mpdi,
  MeshPositionEncodingParameters&     mpep,
  MeshPositionCodingPayload&          mpcp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& NumPositionStart = getGlobal().getNumPositionStart();
  if (mpep.getMeshPositionDeduplicateMethod() == MeshPositionDeduplicateMethod::MESH_POSITION_DEDUP_DEFAULT) {
    READ_VU(mpdi.getMeshPositionDeduplicateCount());  //vu(v)
    if( mpdi.getMeshPositionDeduplicateCount() > 0 ){
      auto& meshPositionDeduplicateIdx = mpdi.getMeshPositionDeduplicateIdx();
      meshPositionDeduplicateIdx.resize(mpdi.getMeshPositionDeduplicateCount());
      for (uint32_t i = 0; i < mpdi.getMeshPositionDeduplicateCount(); i++) {
        READ_VU(meshPositionDeduplicateIdx[i]);  //vu(v)
      }
      READ_VU(mpdi.getMeshPositionDeduplicateStartPositions());  //vu(v)
      READ_VU(mpdi.getMeshPositionIsDuplicateSize());            //vu(v)
      READ_VECTOR(mpdi.getMeshPositionIsDuplicateFlag(),
                  mpdi.getMeshPositionIsDuplicateSize());
      lengthAlignment(bitstream);
      NumPositionStart = mpdi.getMeshPositionDeduplicateStartPositions();
    }
    else {
        NumPositionStart = mpcp.getMeshCcCount();
    }
  }
  else {
      NumPositionStart = mpcp.getMeshCcCount();
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.9	Mesh attribute coding payload syntax
void
EbReader::meshAttributeCodingPayload(Bitstream&                  bitstream,
                                     MeshAttributeCodingPayload& macp,
                                     MeshCodingHeader&           mch) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& NumPositionStart = getGlobal().getNumPositionStart();
  auto& NumAttributeStart = getGlobal().getNumAttributeStart();
  NumAttributeStart.resize(mch.getMeshAttributeCount());
  macp.allocate(mch.getMeshAttributeCount());
  for (uint32_t i = 0; i < mch.getMeshAttributeCount(); i++) {
    auto& maep          = mch.getMeshAttributesEncodingParameters(i);
    auto  numComponents = mch.getNumComponents(i);
    auto  bitDepth      = maep.getMeshAttributeBitDepthMinus1() + 1;
    if (!maep.getMeshAttributePerFaceFlag()
        && maep.getMeshAttributeSeparateIndexFlag()) {
      READ_VU(macp.getMeshAttributeSeamsCount()[i]);  // vu(v)
      READ_VU(macp.getMeshCodedAttributeSeamsSize()[i]);   // vu(v)
      READ_VECTOR(macp.getMeshAttributeSeam()[i],
                  macp.getMeshCodedAttributeSeamsSize()[i]);
      lengthAlignment(bitstream);
    }
    if (maep.getMeshAttributeSeparateIndexFlag()) {
      READ_VU(macp.getMeshAttributeStartCount()[i]);  // vu(v)
      NumAttributeStart[i] = macp.getMeshAttributeStartCount()[i];
    }
    else {
      if (maep.getMeshAttributeReferenceIndexPlus1() == 0)
        NumAttributeStart[i] = NumPositionStart; // TODO FIX CASE WITH REF TO OTHER THAN POS
      else // add check < i
        NumAttributeStart[i] = NumAttributeStart[maep.getMeshAttributeReferenceIndexPlus1() - 1];
    }
    auto& meshAttributeStart = macp.getMeshAttributeStart()[i];
    meshAttributeStart.resize(NumAttributeStart[i]);
    for (uint32_t j = 0; j < NumAttributeStart[i]; j++) {
      meshAttributeStart[j].resize(numComponents);
      for (uint32_t k = 0; k < numComponents; k++) {
        READ_CODE(meshAttributeStart[j][k], bitDepth)  // u(v)
      }
    }
    lengthAlignment(bitstream);

    READ_VU(macp.getMeshAttributeResidualsCount()[i]);  // vu(v)
    if ( macp.getMeshAttributeResidualsCount()[i] > 0 ){
      READ_VU(macp.getMeshCodedAttributeResidualsSize()[i]);  // vu(v)
      READ_VECTOR(macp.getMeshAttributeResidual()[i],
                  macp.getMeshCodedAttributeResidualsSize()[i])
    }
    lengthAlignment(bitstream);

    if (maep.getMeshAttributeSeparateIndexFlag()) {
      auto& madi = macp.getMeshAttributeDeduplicateInformation()[i];
      meshAttributeDeduplicateInformation(bitstream, madi);
    }

    /*	extra data dependent on the selected prediction scheme	*/
    auto& mead = macp.getMeshAttributeExtraData()[i];
    meshAttributeExtraData(bitstream, mead, mch, maep, i );
  }
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.10	Mesh extra attribute data syntax
void
EbReader::meshAttributeExtraData(Bitstream&                        bitstream,
                                 MeshAttributeExtraData&           mead,
                                 MeshCodingHeader&                 mch,
                                 MeshAttributesEncodingParameters& maep,
                                 uint32_t                          index) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto type = mch.getMeshAttributeType()[index];
  if (type == MeshAttributeType::MESH_ATTR_TEXCOORD) {
    if (maep.getMeshAttributePredictionMethod() == (uint8_t)MeshAttributePredictionMethod_TEXCOORD::MESH_TEXCOORD_STRETCH) {
      meshTexCoordStretchExtraData(bitstream,
                                   mead.getMeshTexCoordStretchExtraData());
    } else if (type == MeshAttributeType::MESH_ATTR_NORMAL) {
      /* No extra data defined for specified prediction methods applied on normals */
    } else if (type == MeshAttributeType::MESH_ATTR_COLOR) {
      /* No extra data defined for specified prediction methods applied on colors */
    } else if (type == MeshAttributeType::MESH_ATTR_MATERIAL_ID) {
      /* No extra data defined for specified prediction methods applied on material ids */
    } else if (type == MeshAttributeType::MESH_ATTR_GENERIC) {
      /* No extra data defined for specified prediction methods applied on generic*/
    }
    TRACE_BITSTREAM_OUT("%s", __func__);
  }
}

// 1.2.11	Mesh texcoord stretch extra data syntax
void
EbReader::meshTexCoordStretchExtraData(Bitstream&                    bitstream,
                                       MeshTexCoordStretchExtraData& mtcsed) {
  TRACE_BITSTREAM_IN("%s", __func__);
  READ_VU(mtcsed.getMeshTexCoordStretchOrientationsCount());  // vu(v)
  if (mtcsed.getMeshTexCoordStretchOrientationsCount() > 0) {
    READ_VU(mtcsed.getMeshCodedTexCoordStretchOrientationsSize());  // vu(v)
    READ_VECTOR(mtcsed.getMeshTexCoordStretchOrientation(),
                mtcsed.getMeshCodedTexCoordStretchOrientationsSize());
  }
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.12	Mesh attribute deduplicate information syntax
void
EbReader::meshAttributeDeduplicateInformation(
  Bitstream&                           bitstream,
  MeshAttributeDeduplicateInformation& mc) {
  TRACE_BITSTREAM_IN("%s", __func__);
  // nothing to code
  TRACE_BITSTREAM_OUT("%s", __func__);
}
