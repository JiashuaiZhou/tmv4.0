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

#include <vector>
#include <map>

#if defined(BITSTREAM_TRACE)
#include "util/logger.hpp"
#endif

namespace eb {

class Bitstream;
class MeshCoding;
class MeshCodingHeader;
class MeshPositionEncodingParameters;
class MeshPositionDequantizeParameters;
class MeshAttributesEncodingParameters;
class MeshAttributesDequantizeParameters;
class MeshPositionCodingPayload;
class MeshPositionDeduplicateInformation;
class MeshAttributeCodingPayload;
class MeshAttributeExtraData;
class MeshTexCoordStretchExtraData;
class MeshAttributeDeduplicateInformation;

class EbWriter {
public:
  EbWriter();
  ~EbWriter();

  void write(Bitstream& bitstream, MeshCoding& meshCoding);

private:
  // 8.3.3 Byte alignment syntax
  void byteAlignment(Bitstream& bitstream);

  // 8.3.4 Length alignment syntax
  void lengthAlignment(Bitstream& bitstream);

  // 1.2.1	General Mesh coding syntax
  void meshCoding(Bitstream& bitstream, MeshCoding& mc);

  // 1.2.2	Mesh coding header syntax
  void meshCodingHeader(Bitstream& bitstream, MeshCodingHeader& mch);

  // 1.2.3	Mesh position encoding parameters syntax
  void meshPositionEncodingParameters(Bitstream& bitstream,
                                      MeshPositionEncodingParameters& mpep);

  // 1.2.4	Mesh position dequantize parameters syntax
  void
  meshPositionDequantizeParameters(Bitstream&                        bitstream,
                                   MeshPositionDequantizeParameters& mpdp);

  // 1.2.5	Mesh attributes encoding parameters syntax
  void
  meshAttributesEncodingParameters(Bitstream&                        bitstream,
                                   MeshAttributesEncodingParameters& maep);
                                   
  // 1.2.6	Mesh attributes dequantize parameters syntax
  void meshAttributesDequantizeParameters(Bitstream&        bitstream,
                                          MeshCodingHeader& mch,
                                          uint32_t          index);

  // 1.2.7	Mesh position coding payload syntax
  void meshPositionCodingPayload(Bitstream&                 bitstream,
                                 MeshPositionCodingPayload& mpcp,
                                 MeshCodingHeader&          mch);

  // 1.2.8	Mesh position deduplicate information syntax
  void
  meshPositionDeduplicateInformation(Bitstream& bitstream,
                                     MeshPositionDeduplicateInformation& mpdi,
                                     MeshPositionEncodingParameters&     mpep,
                                     MeshPositionCodingPayload&          mpcp);

  // 1.2.9	Mesh attribute coding payload syntax
  void meshAttributeCodingPayload(Bitstream&                        bitstream,
                                  MeshAttributeCodingPayload&       macp,
                                  MeshCodingHeader&                 mch);

  // 1.2.10	Mesh extra attribute data syntax
  void meshAttributeExtraData(Bitstream&                        bitstream,
                              MeshAttributeExtraData&           mead,
                              MeshCodingHeader&                 mch,
                              MeshAttributesEncodingParameters& maep,
                              uint32_t                          index);

  // 1.2.11	Mesh texcoord stretch extra data syntax
  void meshTexCoordStretchExtraData(Bitstream&                    bitstream,
                                    MeshTexCoordStretchExtraData& mtcsed);

  // 1.2.12	Mesh attribute deduplicate information syntax
  void
  meshAttributeDeduplicateInformation(Bitstream& bitstream,
                                      MeshAttributeDeduplicateInformation& mc);

#if defined(BITSTREAM_TRACE)
public:
  void setLogger(Logger& logger) { logger_ = &logger; }

private:
  Logger* logger_ = nullptr;
#endif

private:
    MeshCoding* global_ = nullptr;
public:
    MeshCoding& getGlobal() { return *global_; }
    void        setGlobal(MeshCoding& meshCoding) { global_ = &meshCoding; }

};

};  // namespace eb
