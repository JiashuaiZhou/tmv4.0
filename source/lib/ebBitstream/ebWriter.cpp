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
#include "ebWriter.hpp"
#include "ebBitstream.h"
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

#  define WRITE_VU(VAR) \
    { \
      bitstream.writeVu(VAR); \
      bitstream.traceBit(format(#VAR), VAR, "vu(v)"); \
    }

#  define WRITE_VI(VAR) \
    { \
      bitstream.writeVi(VAR); \
      bitstream.traceBit(format(#VAR), VAR, "vi(v)"); \
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
#  define WRITE_VU(VAR) bitstream.writeVu(VAR);
#  define WRITE_VI(VAR) bitstream.writeVi(VAR);
#  define WRITE_VECTOR(VAR) bitstream.write(VAR);
#  define WRITE_VIDEO(VAR) bitstream.write(VAR);
#endif

EbWriter::EbWriter()  = default;
EbWriter::~EbWriter() = default;

void
EbWriter::write(Bitstream& bitstream, MeshCoding& mc) {
  TRACE_BITSTREAM_IN("%s", __func__);
  setGlobal(mc);
  meshCoding( bitstream, mc );
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 8.3.3 Byte alignment syntax
void
EbWriter::byteAlignment(Bitstream& bitstream) {
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
EbWriter::lengthAlignment(Bitstream& bitstream) {
  TRACE_BITSTREAM_IN("%s", __func__);
  uint32_t zero = 0;
  while (!bitstream.byteAligned()) {
    WRITE_CODE(zero, 1);  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.1	General Mesh coding syntax
void
EbWriter::meshCoding(Bitstream& bitstream, MeshCoding& mc) {
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
EbWriter::meshCodingHeader(Bitstream& bitstream, MeshCodingHeader& mch) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(mch.getMeshCodecType(), 2);  // u(2)
  meshPositionEncodingParameters(bitstream,
                                 mch.getMeshPositionEncodingParameters());
  WRITE_CODE(mch.getMeshPositionDequantizeFlag(), 1);  // u(1)

  if (mch.getMeshPositionDequantizeFlag()) {
    meshPositionDequantizeParameters(
      bitstream, mch.getMeshPositionDequantizeParameters());
  }
  WRITE_CODE(mch.getMeshAttributeCount(), 5);  // u(5)

  for (uint32_t i = 0; i < mch.getMeshAttributeCount(); i++) {
    WRITE_CODE(mch.getMeshAttributeType()[i], 3);  // u(3)
    if (mch.getMeshAttributeType()[i] == MeshAttributeType::MESH_ATTR_GENERIC) {
      WRITE_CODE(mch.getMeshAttributeNumComponentsMinus1()[i], 2);  // u(2)
    }
    meshAttributesEncodingParameters(
      bitstream, mch.getMeshAttributesEncodingParameters()[i]);
    WRITE_CODE(mch.getMeshAttributeDequantizeFlag()[i], 1);  // u(1)
    if (mch.getMeshAttributeDequantizeFlag()[i])
      meshAttributesDequantizeParameters(bitstream, mch, i);
  }
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.3	Mesh position encoding parameters syntax
void
EbWriter::meshPositionEncodingParameters(
  Bitstream&                      bitstream,
  MeshPositionEncodingParameters& mpep) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(mpep.getMeshPositionBitDepthMinus1(), 5);        // u(5)
  WRITE_UVLC(mpep.getMeshClersSymbolsEncodingMethod());       // ue(v)
  WRITE_UVLC(mpep.getMeshPositionPredictionMethod());         // ue(v)
  WRITE_UVLC(mpep.getMeshPositionResidualsEncodingMethod());  // ue(v)
  WRITE_UVLC(mpep.getMeshPositionDeduplicateMethod());        // ue(v)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.4	Mesh position dequantize parameters syntax
void
EbWriter::meshPositionDequantizeParameters(
  Bitstream&                        bitstream,
  MeshPositionDequantizeParameters& mpdp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  for (uint32_t i = 0; i < 3; i++) {
    WRITE_FLOAT(mpdp.getMeshPositionMin(i));  // fl(32)
    WRITE_FLOAT(mpdp.getMeshPositionMax(i));  // fl(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.5	Mesh attributes encoding parameters syntax
void
EbWriter::meshAttributesEncodingParameters(
  Bitstream&                        bitstream,
  MeshAttributesEncodingParameters& maep) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_CODE(maep.getMeshAttributeBitDepthMinus1(), 5);        // u(5)
  WRITE_CODE(maep.getMeshAttributePerFaceFlag(), 1);           // u(1)
  if (!maep.getMeshAttributePerFaceFlag()) {
    WRITE_CODE(maep.getMeshAttributeSeparateIndexFlag(), 1);   // u(1)
    if (!maep.getMeshAttributeSeparateIndexFlag())
      WRITE_UVLC(maep.getMeshAttributeReferenceIndexPlus1());  // ue(v)
  }
  WRITE_UVLC(maep.getMeshAttributePredictionMethod());         // ue(v)
  WRITE_UVLC(maep.getMeshAttributeResidualsEncodingMethod());  // ue(v)
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.6	Mesh attributes dequantize parameters syntax
void
EbWriter::meshAttributesDequantizeParameters(Bitstream&        bitstream,
                                             MeshCodingHeader& mch,
                                             uint32_t          index) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& madp = mch.getMeshAttributesDequantizeParameters(index);
  for (uint32_t i = 0; i < mch.getNumComponents(index); i++) {
    WRITE_FLOAT(madp.getMeshAttributeMin(i));  // fl(32)
    WRITE_FLOAT(madp.getMeshAttributeMax(i));  // fl(32)
  }
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.7	Mesh position coding payload syntax
void
EbWriter::meshPositionCodingPayload(Bitstream&                 bitstream,
                                    MeshPositionCodingPayload& mpcp,
                                    MeshCodingHeader&          mch) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& NumHandles = getGlobal().getNumHandles();
  auto& NumPositionStart = getGlobal().getNumPositionStart();
  WRITE_VU(mpcp.getMeshVertexCount());         //	vu(v)
  WRITE_VU(mpcp.getMeshClersCount());          //	vu(v)
  WRITE_VU(mpcp.getMeshCcCount());             //	vu(v)
  WRITE_VU(mpcp.getMeshVirtualVertexCount());  //	vu(v)

  auto& meshVirtualIndexDelta = mpcp.getMeshVirtualIndexDelta();
  for (uint32_t i = 0; i < mpcp.getMeshVirtualVertexCount(); i++) {
    WRITE_VU(meshVirtualIndexDelta[i]);  // vu(v)
  }
  WRITE_VU(mpcp.getMeshCcWithHandlesCount());  //	vu(v)
  auto&    meshHandlesCcOffset = mpcp.getMeshHandlesCcOffset();
  auto&    meshHandlesCount    = mpcp.getMeshHandlesCount();


  NumHandles = 0;
  for (uint32_t i = 0; i < mpcp.getMeshCcWithHandlesCount(); i++) {
    WRITE_VU(meshHandlesCcOffset[i]);  // vu(v)
    WRITE_VU(meshHandlesCount[i]);     // vu(v)
    NumHandles += meshHandlesCount[i];
  }
  auto& meshHandleIndexFirstDelta  = mpcp.getMeshHandleIndexFirstDelta();
  auto& meshHandleIndexSecondDelta = mpcp.getMeshHandleIndexSecondDelta();
  
  
  for (size_t i = 0; i < NumHandles; i++) {
    WRITE_VI(meshHandleIndexFirstDelta[i]);   // vi(v)
    WRITE_VI(meshHandleIndexSecondDelta[i]);  // vi(v)
  }
  WRITE_VU(mpcp.getMeshCodedHandleIndexSecondShiftSize());  //	vu(v)
  WRITE_VECTOR(mpcp.getMeshHandleIndexSecondShift());
  
  lengthAlignment(bitstream);

  WRITE_VU(mpcp.getMeshCodedClersSymbolsSize());  //	vu(v)
  WRITE_VECTOR(mpcp.getMeshClersSymbol());
  lengthAlignment(bitstream);

  auto& mpdi = mpcp.getMeshPositionDeduplicateInformation();
  auto& mpep = mch.getMeshPositionEncodingParameters();
  meshPositionDeduplicateInformation(bitstream, mpdi, mpep, mpcp);

  auto& meshPositionStart = mpcp.getMeshPositionStart();
  for (uint32_t i = 0; i < NumPositionStart; i++) {    
    for (uint32_t j = 0; j < 3; j++) {
      WRITE_CODE(meshPositionStart[i][j],
                 mpep.getMeshPositionBitDepthMinus1() + 1);  // u(v)
    }
  }
  lengthAlignment(bitstream);
  WRITE_VU(mpcp.getMeshCodedPositionResidualsSize());  //	vu(v)
  WRITE_VECTOR(mpcp.getMeshPositionResidual());
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.8	Mesh position deduplicate information syntax
void
EbWriter::meshPositionDeduplicateInformation(
  Bitstream&                          bitstream,
  MeshPositionDeduplicateInformation& mpdi,
  MeshPositionEncodingParameters&     mpep,
  MeshPositionCodingPayload&          mpcp) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& NumPositionStart = getGlobal().getNumPositionStart();
  if (mpep.getMeshPositionDeduplicateMethod() == MeshPositionDeduplicateMethod::MESH_POSITION_DEDUP_DEFAULT) {
    WRITE_VU(mpdi.getMeshPositionDeduplicateCount());  //vu(v)
    if (mpdi.getMeshPositionDeduplicateCount() > 0) {
      auto& meshPositionDeduplicateIdx = mpdi.getMeshPositionDeduplicateIdx();
      for (uint32_t i = 0; i < mpdi.getMeshPositionDeduplicateCount(); i++) {
        WRITE_VU(meshPositionDeduplicateIdx[i]);  //vu(v)
      }
      WRITE_VU(mpdi.getMeshPositionDeduplicateStartPositions());  //vu(v)
      WRITE_VU(mpdi.getMeshPositionIsDuplicateSize());            //vu(v)
      WRITE_VECTOR(mpdi.getMeshPositionIsDuplicateFlag());
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
EbWriter::meshAttributeCodingPayload(Bitstream&                  bitstream,
                                     MeshAttributeCodingPayload& macp,
                                     MeshCodingHeader&           mch) {
  TRACE_BITSTREAM_IN("%s", __func__);
  auto& NumPositionStart = getGlobal().getNumPositionStart();
  auto& NumAttributeStart = getGlobal().getNumAttributeStart();
  NumAttributeStart.resize(mch.getMeshAttributeCount());
  for (uint32_t i = 0; i < mch.getMeshAttributeCount(); i++) {
    auto& maep          = mch.getMeshAttributesEncodingParameters(i);
    auto  numComponents = mch.getNumComponents(i);
    auto  bitDepth      = maep.getMeshAttributeBitDepthMinus1() + 1;
    if (!maep.getMeshAttributePerFaceFlag()
        && maep.getMeshAttributeSeparateIndexFlag()) {
      WRITE_VU(macp.getMeshAttributeSeamsCount()[i]);      // vu(v)
      WRITE_VU(macp.getMeshCodedAttributeSeamsSize()[i]);  // vu(v)
      WRITE_VECTOR(macp.getMeshAttributeSeam()[i]);
      lengthAlignment(bitstream);
    }
    if (maep.getMeshAttributeSeparateIndexFlag()) {
      WRITE_VU(macp.getMeshAttributeStartCount()[i]);  // vu(v)
      NumAttributeStart[i] = macp.getMeshAttributeStartCount()[i];
    }
    else {
      if (maep.getMeshAttributeReferenceIndexPlus1() == 0)
        NumAttributeStart[i] = NumPositionStart; // TODO FIX CASE WITH REF TO OTHER THAN POS
      else // add check < i
        NumAttributeStart[i] = NumAttributeStart[maep.getMeshAttributeReferenceIndexPlus1() - 1];
    }
    auto& meshAttributeStart = macp.getMeshAttributeStart()[i];
    for (uint32_t j = 0; j < NumAttributeStart[i]; j++) {
      for (uint32_t k = 0; k < numComponents; k++) {
        WRITE_CODE(meshAttributeStart[j][k], bitDepth)  // u(v)
      }
    }
    lengthAlignment(bitstream);

    WRITE_VU(macp.getMeshAttributeResidualsCount()[i]);  // vu(v)
    if (macp.getMeshAttributeResidualsCount()[i] > 0) {
      WRITE_VU(macp.getMeshCodedAttributeResidualsSize()[i]);  // vu(v)
      WRITE_VECTOR(macp.getMeshAttributeResidual()[i])
    }
    lengthAlignment(bitstream);

    if (maep.getMeshAttributeSeparateIndexFlag()) {
      auto& madi = macp.getMeshAttributeDeduplicateInformation()[i];
      meshAttributeDeduplicateInformation(bitstream, madi);
    }

    /*	extra data dependent on the selected prediction scheme	*/
    auto& mead = macp.getMeshAttributeExtraData()[i];
    meshAttributeExtraData(bitstream, mead, mch, maep, i);
  }
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.10	Mesh extra attribute data syntax
void
EbWriter::meshAttributeExtraData(Bitstream&                        bitstream,
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
EbWriter::meshTexCoordStretchExtraData(Bitstream&                    bitstream,
                                       MeshTexCoordStretchExtraData& mtcsed) {
  TRACE_BITSTREAM_IN("%s", __func__);
  WRITE_VU(mtcsed.getMeshTexCoordStretchOrientationsCount());  // vu(v)
  if (mtcsed.getMeshTexCoordStretchOrientationsCount() > 0) {
    WRITE_VU(mtcsed.getMeshCodedTexCoordStretchOrientationsSize());  // vu(v)
    WRITE_VECTOR(mtcsed.getMeshTexCoordStretchOrientation());
  }
  lengthAlignment(bitstream);
  TRACE_BITSTREAM_OUT("%s", __func__);
}

// 1.2.12	Mesh attribute deduplicate information syntax
void
EbWriter::meshAttributeDeduplicateInformation(
  Bitstream&                           bitstream,
  MeshAttributeDeduplicateInformation& mc) {
  TRACE_BITSTREAM_IN("%s", __func__);
  // nothing to code
  TRACE_BITSTREAM_OUT("%s", __func__);
}
