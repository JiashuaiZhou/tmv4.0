/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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
#include "util/mesh.hpp"
#include "mpegGeometryEncoder.hpp"

#include "ebModel.h"
#include "ebBasicEncoder.h"
#include "ebBasicDecoder.h"

namespace vmesh {

template<typename T>
MpegGeometryEncoder<T>::MpegGeometryEncoder() = default;
template<typename T>
MpegGeometryEncoder<T>::~MpegGeometryEncoder() = default;

//============================================================================

template<typename T>
void
convert(const TriangleMesh<T>& src, eb::Model& dst) {
    dst.reset();
    const auto& points = src.points();
    dst.vertices.resize(points.size() * 3);
    for (size_t i = 0; i < points.size(); i++) {
        dst.vertices[3 * i + 0] = points[i][0];
        dst.vertices[3 * i + 1] = points[i][1];
        dst.vertices[3 * i + 2] = points[i][2];
    }
    if (!src.texCoords().empty()) {
        const auto& texCoords = src.texCoords();
        dst.uvcoords.resize(texCoords.size() * 2);
        for (size_t i = 0; i < texCoords.size(); i++) {
            dst.uvcoords[2 * i + 0] = texCoords[i][0];
            dst.uvcoords[2 * i + 1] = texCoords[i][1];
        }
    }
    const auto& triangles = src.triangles();
    dst.triangles.resize(triangles.size() * 3);
    for (size_t i = 0; i < triangles.size(); i++) {
        dst.triangles[3 * i + 0] = triangles[i][0];
        dst.triangles[3 * i + 1] = triangles[i][1];
        dst.triangles[3 * i + 2] = triangles[i][2];
    }
    if (!src.texCoordTriangles().empty()) {
        const auto& texCoordsTri = src.texCoordTriangles();
        dst.trianglesuv.resize(texCoordsTri.size() * 3);
        for (size_t i = 0; i < texCoordsTri.size(); i++) {
            dst.trianglesuv[3 * i + 0] = texCoordsTri[i][0];
            dst.trianglesuv[3 * i + 1] = texCoordsTri[i][1];
            dst.trianglesuv[3 * i + 2] = texCoordsTri[i][2];
        }
    }
    if (!src.normals().empty()) {
        const auto& normals = src.normals();
        dst.normals.resize(normals.size() * 3);
        for (size_t i = 0; i < normals.size(); i++) {
            dst.normals[3 * i + 0] = normals[i][0];
            dst.normals[3 * i + 1] = normals[i][1];
            dst.normals[3 * i + 2] = normals[i][2];
        }
    }
}

template<typename T>
void
convert(const eb::Model& src, TriangleMesh<T>& dst) {
    dst.resizePoints(src.vertices.size() / 3);
    auto& points = dst.points();
    for (size_t i = 0; i < points.size(); i++) {
        points[i][0] = src.vertices[3 * i + 0];
        points[i][1] = src.vertices[3 * i + 1];
        points[i][2] = src.vertices[3 * i + 2];
    }
    if (!src.uvcoords.empty()) {
        dst.resizeTexCoords(src.uvcoords.size() / 2);
        auto& texCoords = dst.texCoords();
        for (size_t i = 0; i < texCoords.size(); i++) {
            texCoords[i][0] = src.uvcoords[2 * i + 0];
            texCoords[i][1] = src.uvcoords[2 * i + 1];
        }
    }
    dst.resizeTriangles(src.triangles.size() / 3);
    auto& triangles = dst.triangles();
    for (size_t i = 0; i < triangles.size(); i++) {
        triangles[i][0] = src.triangles[3 * i + 0];
        triangles[i][1] = src.triangles[3 * i + 1];
        triangles[i][2] = src.triangles[3 * i + 2];
    }
    if (!src.trianglesuv.empty()) {
        dst.resizeTexCoordTriangles(src.trianglesuv.size() / 3);
        auto& texCoordsTri = dst.texCoordTriangles();
        for (size_t i = 0; i < texCoordsTri.size(); i++) {
            texCoordsTri[i][0] = src.trianglesuv[3 * i + 0];
            texCoordsTri[i][1] = src.trianglesuv[3 * i + 1];
            texCoordsTri[i][2] = src.trianglesuv[3 * i + 2];
        }
    }
    else if (!src.uvcoords.empty())
    {
        dst.resizeTexCoordTriangles(dst.triangles().size());
        auto& texCoordsTri = dst.texCoordTriangles();
        auto& triangles = dst.triangles();
        for (size_t i = 0; i < texCoordsTri.size(); i++) {
            texCoordsTri[i] = triangles[i];
        }
    }
    if (!src.normals.empty()) {
        dst.resizeNormals(src.normals.size() / 3);
        auto& normals = dst.normals();
        for (size_t i = 0; i < normals.size(); i++) {
            normals[i][0] = src.normals[3 * i + 0];
            normals[i][1] = src.normals[3 * i + 1];
            normals[i][2] = src.normals[3 * i + 2];
        }
    }
}

template<typename T>
void
MpegGeometryEncoder<T>::encode(TriangleMesh<T>&           src,
                               GeometryEncoderParameters& params,
                               std::vector<uint8_t>&      bitstream,
                               TriangleMesh<T>&           rec)

{
  // convert source model
  eb::Model srcModel;
  convert(src, srcModel);

  // prepare encoder parameters
  eb::EBBasicEncoder encoder;
  encoder.cfg.intAttr = true;
  encoder.qp = params.qp_;
  encoder.qt = params.qt_;

  encoder.cfg.posPred   = eb::EBConfig::PosPred::MPARA;
  encoder.cfg.uvPred    = eb::EBConfig::UvPred::STRETCH;
  encoder.cfg.predCoder = eb::EBConfig::ECName::DIRAC;
  encoder.cfg.topoCoder = eb::EBConfig::ECName::DIRAC;

  // No alternative AC coders in this first release aligned with the syntax
  if (params.predCoder_ == "dirac")
      encoder.cfg.predCoder = eb::EBConfig::ECName::DIRAC;
  //else if (params.predCoder_ == "rans")
  //    encoder.cfg.predCoder = eb::EBConfig::ECName::RANS;
  else // default
      encoder.cfg.predCoder = eb::EBConfig::ECName::DIRAC;

  if (params.topoCoder_ == "dirac")
      encoder.cfg.topoCoder = eb::EBConfig::ECName::DIRAC;
  //else if (params.topoCoder_ == "rans")
  //    encoder.cfg.topoCoder = eb::EBConfig::ECName::RANS;
  else // default
      encoder.cfg.topoCoder = eb::EBConfig::ECName::DIRAC;

  // deduplication is deactivated by default
  encoder.cfg.deduplicate = params.baseMeshDeduplicatePositions_;

  // compress
  encoder.encode(srcModel);
  
  // serialize to bitstream
  eb::Bitstream bs;
#if defined(BITSTREAM_TRACE)
  eb::Logger loggerMeb;
  if (!params.logFileName_.empty())
  {
      loggerMeb.initilalize(params.logFileName_ + "_meb", true);
      bs.setLogger(loggerMeb);
      bs.setTrace(true);
  }
#endif
  encoder.serialize(bs);

  // Decode for rec
  // parsing 
  bs.beginning();
  eb::MeshCoding meshCoding; // Top level syntax element
  eb::EbReader   ebReader;
#if defined(BITSTREAM_TRACE)
  eb::Logger loggerMebDec;
  if (!params.logFileName_.empty())
  {
      loggerMebDec.initilalize(params.logFileName_ + "_meb");
      bs.setLogger(loggerMebDec);
  }
#endif
  ebReader.read(bs, meshCoding);

  const auto& mch = meshCoding.getMeshCodingHeader();
  // Codec Variant
  const auto& method = mch.getMeshCodecType();

  // instanciate decoder - single variant - will be moved inside the library
  eb::EBBasicDecoder decoder;

  // deserialize the bitstream
  decoder.unserialize(meshCoding);

  // decode to model
  eb::Model recModel;
  decoder.decode(recModel);

  // convert and unify vertices
  convert(recModel, rec);

// set bitstream
  std::swap(bitstream, bs.vector());
  bitstream.resize(bs.size()); // assumes the bitstream is byte aligned
}

template class MpegGeometryEncoder<float>;
template class MpegGeometryEncoder<double>;

}  // namespace vmesh

