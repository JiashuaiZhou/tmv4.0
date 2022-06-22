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
#if defined(USE_DRACO_GEOMETRY_CODEC)
#  include "util/mesh.hpp"
#  include "dracoLibGeometryEncoder.hpp"

#  include "draco/compression/encode.h"
#  include "draco/compression/decode.h"
#  include "draco/core/data_buffer.h"
#  include "draco/mesh/triangle_soup_mesh_builder.h"
#include "draco/io/obj_encoder.h"

namespace vmesh {

template<typename T>
DracoLibGeometryEncoder<T>::DracoLibGeometryEncoder()
{}
template<typename T>
DracoLibGeometryEncoder<T>::~DracoLibGeometryEncoder()
{}

template<typename T>
std::unique_ptr<draco::Mesh>
convert(TriangleMesh<T>& src)
{
  const int32_t triCount = src.triangleCount();
  const int32_t pointCount = src.pointCount();
  const int32_t texCoordCount = src.texCoordCount();
  const int32_t normalCount = src.normalCount();
  const int32_t colourCount = src.colourCount();
  int posAtt = -1, nrmAtt = -1, colAtt = -1, texAtt = -1;

  // Add attributes if they are present in the input data.
  const bool use_identity_mapping = false;
  auto mesh = std::unique_ptr<draco::Mesh>(new draco::Mesh());
  mesh->SetNumFaces(triCount);
  mesh->set_num_points(3 * triCount);

  draco::GeometryAttribute va;
  va.Init(draco::GeometryAttribute::POSITION, nullptr, 3, draco::DT_INT32, false,
          sizeof(int32_t) * 3, 0);
  posAtt = mesh->AddAttribute(va, use_identity_mapping, pointCount);

  if (normalCount > 0) {
    draco::GeometryAttribute va;
    va.Init(
      draco::GeometryAttribute::NORMAL, nullptr, 3, draco::DT_INT32, false,
      sizeof(int32_t) * 3, 0);
    nrmAtt = mesh->AddAttribute(va, use_identity_mapping, normalCount);
  }
  if (colourCount > 0) {
    draco::GeometryAttribute va;
    va.Init(
      draco::GeometryAttribute::NORMAL, nullptr, 3, draco::DT_INT32, false,
      sizeof(int32_t) * 3, 0);
    colAtt = mesh->AddAttribute(va, use_identity_mapping, colourCount);
  }
  if (texCoordCount > 0) {
    draco::GeometryAttribute va;
    va.Init(
      draco::GeometryAttribute::TEX_COORD, nullptr, 2, draco::DT_INT32, false,
      sizeof(int32_t) * 2, 0);
    texAtt = mesh->AddAttribute(va, use_identity_mapping, texCoordCount);
  }

  draco::AttributeValueIndex posIndex(0);
  draco::AttributeValueIndex colIndex(0);
  draco::AttributeValueIndex nrmIndex(0);
  draco::AttributeValueIndex texIndex(0);
  for (int32_t i = 0; i < pointCount; i++) {
    Vec3<int32_t> pos(src.point(i));
    mesh->attribute(posAtt)->SetAttributeValue(posIndex++, pos.data());
  }
  if (normalCount > 0) {
    for (int32_t i = 0; i < normalCount; i++) {
      Vec3<int32_t> nrm(src.normal(i));
      mesh->attribute(nrmAtt)->SetAttributeValue(nrmIndex++, nrm.data());
    }
  }
  if (colourCount > 0) {
    for (int32_t i = 0; i < colourCount; i++) {
      Vec3<int32_t> col(src.colour(i));
      mesh->attribute(colAtt)->SetAttributeValue(colIndex++, col.data());
    }
  }
  if (texCoordCount > 0) {
    for (int32_t i = 0; i < texCoordCount; i++) {
      Vec2<int32_t> tex(src.texCoord(i));
      mesh->attribute(texAtt)->SetAttributeValue(texIndex++, tex.data());
    }
  }
  for (int32_t i = 0; i < triCount; i++) {
    draco::Mesh::Face face;
    Vec3<int32_t> tri(src.triangle(i));
    for (int c = 0; c < 3; ++c) {
      face[c] = 3 * i + c;
      mesh->attribute(posAtt)->SetPointMapEntry(
        face[c], draco::AttributeValueIndex(tri[c]));
    }
    if (texCoordCount > 0) {
      Vec3<int32_t> tex(src.texCoordTriangle(i));
      for (int c = 0; c < 3; ++c)
        mesh->attribute(texAtt)->SetPointMapEntry(
          face[c], draco::AttributeValueIndex(tex[c]));
    }
    if (normalCount > 0) {
      Vec3<int32_t> nrm(src.normalTriangle(i));
      for (int c = 0; c < 3; ++c)
        mesh->attribute(nrmAtt)->SetPointMapEntry(
          face[c], draco::AttributeValueIndex(nrm[c]));
    }
    mesh->SetFace(draco::FaceIndex(i), face);
  }
  mesh->DeduplicatePointIds();
  return mesh;
}

template<typename T>
void
convert(std::unique_ptr<draco::Mesh>& src, TriangleMesh<T>& dst)
{
  draco::Mesh& mesh = *src;
  auto posAtt = mesh.GetNamedAttribute(draco::GeometryAttribute::POSITION);
  auto nrmAtt = mesh.GetNamedAttribute(draco::GeometryAttribute::NORMAL);
  auto colAtt = mesh.GetNamedAttribute(draco::GeometryAttribute::COLOR);
  auto texAtt = mesh.GetNamedAttribute(draco::GeometryAttribute::TEX_COORD);
  // position
  if (posAtt) {
    for (draco::AttributeValueIndex i(0);
         i < static_cast<uint32_t>(posAtt->size()); ++i) {
      std::array<int32_t, 3> value;
      if (!posAtt->ConvertValue<int32_t, 3>(i, &value[0])) {
        return;
      }
      dst.addPoint(value[0], value[1], value[2]);
    }
  }
  // Normal
  if (nrmAtt) {
    for (draco::AttributeValueIndex i(0);
         i < static_cast<uint32_t>(nrmAtt->size()); ++i) {
      std::array<int32_t, 3> value;
      if (!nrmAtt->ConvertValue<int32_t, 3>(i, &value[0])) {
        return;
      }
      dst.addNormal(value[0], value[1], value[2]);
    }
  }
  // Color
  if (colAtt) {
    for (draco::AttributeValueIndex i(0);
         i < static_cast<uint32_t>(colAtt->size()); ++i) {
      std::array<int32_t, 3> value;
      if (!colAtt->ConvertValue<int32_t, 3>(i, &value[0])) {
        return;
      }
      dst.addColour(value[0], value[1], value[2]);
    }
  }
  // Texture coordinate
  if (texAtt) {
    for (draco::AttributeValueIndex i(0);
         i < static_cast<uint32_t>(texAtt->size()); ++i) {
      std::array<int32_t, 2> value;
      if (!texAtt->ConvertValue<int32_t, 2>(i, &value[0])) {
        return;
      }
      dst.addTexCoord(value[0], value[1]);
    }
  }
  for (draco::FaceIndex i(0); i < mesh.num_faces(); ++i) {
    auto& face = mesh.face(i);
    const int32_t idx0 = posAtt->mapped_index(face[0]).value();
    const int32_t idx1 = posAtt->mapped_index(face[1]).value();
    const int32_t idx2 = posAtt->mapped_index(face[2]).value();
    dst.addTriangle(idx0, idx1, idx2);
    if (texAtt && texAtt->size() > 0){
      const int32_t tex0 = texAtt->mapped_index(face[0]).value();
      const int32_t tex1 = texAtt->mapped_index(face[1]).value();
      const int32_t tex2 = texAtt->mapped_index(face[2]).value();
      dst.addTexCoordTriangle(tex0, tex1, tex2);
    }
    if (nrmAtt && nrmAtt->size() > 0){
      const int32_t nrm0 = nrmAtt->mapped_index(face[0]).value();
      const int32_t nrm1 = nrmAtt->mapped_index(face[1]).value();
      const int32_t nrm2 = nrmAtt->mapped_index(face[2]).value();
      dst.addNormalTriangle(nrm0, nrm1, nrm2);
    }
  }
}

template<typename T>
void
DracoLibGeometryEncoder<T>::encode(
  TriangleMesh<T>& src,
  GeometryEncoderParameters& params,
  std::vector<uint8_t>& bitstream,
  TriangleMesh<T>& rec)

{  
  // Load draco mesh
  auto mesh = convert(src);

  // Encode
  draco::Encoder encoder;
  encoder.SetSpeedOptions(10 - params.cl_, 10 - params.cl_);
  encoder.SetEncodingMethod(
    draco::MeshEncoderMethod::MESH_EDGEBREAKER_ENCODING);
  encoder.SetAttributeQuantization(
    draco::GeometryAttribute::POSITION, params.qp_);
  encoder.SetAttributeQuantization(
    draco::GeometryAttribute::TEX_COORD, params.qt_);
  if (params.qn_ > 0) {
    encoder.SetAttributeQuantization(
      draco::GeometryAttribute::NORMAL, params.qn_);
  }
  if (params.qg_ > 0) {
    encoder.SetAttributeQuantization(
      draco::GeometryAttribute::COLOR, params.qg_);
  }
  draco::EncoderBuffer buffer;
  const draco::Status status =
    encoder.EncodeMeshToBuffer(*(mesh.get()), &buffer);
  if (!status.ok()) {
    printf("Failed to encode the mesh: %s\n", status.error_msg());
    exit(-1);
  }
  printf("EncodeMeshToBuffer => buffer size = %zu \n", buffer.size());
  fflush(stdout);

  // Copy bitstream
  bitstream.resize(buffer.size());
  std::copy(buffer.data(), buffer.data() + buffer.size(), bitstream.data());

  // Decode  
  draco::DecoderBuffer decBuffer;
  decBuffer.Init((const char*)bitstream.data(), bitstream.size());
  auto type = draco::Decoder::GetEncodedGeometryType(&decBuffer);
  if (!type.ok()) {
    printf("Failed GetEncodedGeometryType: %s.\n", type.status().error_msg());
    exit(-1);
  }
  if (type.value() == draco::TRIANGULAR_MESH) {
    draco::Decoder decoder;
    auto status = decoder.DecodeMeshFromBuffer(&decBuffer);
    if (!status.ok()) {
      printf(
        "Failed DecodeMeshFromBuffer: %s.\n", status.status().error_msg());
      exit(-1);
    }
    std::unique_ptr<draco::Mesh> decMesh = std::move(status).value();
    if (decMesh) {
      convert(decMesh, rec);
    } else {
      printf("Failed no in mesh  \n");
      exit(-1);
    }
  } else {
    printf("Failed no mesh type not supported.\n");
    exit(-1);
  }
}

template class DracoLibGeometryEncoder<float>;
template class DracoLibGeometryEncoder<double>;

}  // namespace vmesh

#endif
