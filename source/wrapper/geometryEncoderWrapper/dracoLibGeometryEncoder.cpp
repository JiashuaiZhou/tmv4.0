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
#  include "mesh.hpp"
#  include "dracoLibGeometryEncoder.hpp"

#  include "draco/compression/encode.h"
#  include "draco/io/file_utils.h"
#  include "draco/io/mesh_io.h"
#  include "draco/compression/decode.h"
#  include "draco/io/obj_encoder.h"
#  include "draco/io/parser_utils.h"
#  include "draco/core/data_buffer.h"
#  include "draco/mesh/triangle_soup_mesh_builder.h"

namespace vmesh {

template<typename T>
dracoLibGeometryEncoder<T>::dracoLibGeometryEncoder()
{}
template<typename T>
dracoLibGeometryEncoder<T>::~dracoLibGeometryEncoder()
{}

template<typename T>
std::unique_ptr<draco::Mesh>
convert(vmesh::TriangleMesh<T>& src)
{
  draco::TriangleSoupMeshBuilder meshBuilder;

  const int32_t triCount = src.triangleCount();
  meshBuilder.Start(src.triangleCount());

  const auto hasTexCoords = src.texCoordTriangleCount() > 0;
  const auto hasNormals = src.normalCount() > 0;
  const auto hasColors = src.colourCount() > 0;

  int posAtt = -1, nrmAtt = -1, colAtt = -1, texAtt = -1;
  posAtt = meshBuilder.AddAttribute(
    draco::GeometryAttribute::POSITION, 3, draco::DT_INT32);
  if (hasNormals) {
    nrmAtt = meshBuilder.AddAttribute(
      draco::GeometryAttribute::NORMAL, 3, draco::DT_INT32);
  }
  if (hasColors) {
    colAtt = meshBuilder.AddAttribute(
      draco::GeometryAttribute::COLOR, 3, draco::DT_INT32);
  }
  if (hasTexCoords) {
    texAtt = meshBuilder.AddAttribute(
      draco::GeometryAttribute::TEX_COORD, 2, draco::DT_INT32);
  }

  auto addFace = [&](const Vec3<int>& indices, draco::FaceIndex index) {
    int32_t idx0 = indices.x();
    int32_t idx1 = indices.y();
    int32_t idx2 = indices.z();
    Vec3<int32_t> pos0(src.point(idx0));
    Vec3<int32_t> pos1(src.point(idx1));
    Vec3<int32_t> pos2(src.point(idx2));
    meshBuilder.SetAttributeValuesForFace(
      posAtt, index, pos0.data(), pos1.data(), pos2.data());
    if (hasNormals) {
      Vec3<int32_t> nrm0(src.normal(idx0));
      Vec3<int32_t> nrm1(src.normal(idx1));
      Vec3<int32_t> nrm2(src.normal(idx2));
      meshBuilder.SetAttributeValuesForFace(
        nrmAtt, index, nrm0.data(), nrm1.data(), nrm2.data());
    }
    if (hasColors) {
      Vec3<int32_t> col0(src.colour(idx0));
      Vec3<int32_t> col1(src.colour(idx1));
      Vec3<int32_t> col2(src.colour(idx2));
      meshBuilder.SetAttributeValuesForFace(
        colAtt, index, col0.data(), col0.data(), col0.data());
    }
    if (hasTexCoords) {
      Vec2<int32_t> tex0(src.texCoord(idx0));
      Vec2<int32_t> tex1(src.texCoord(idx1));
      Vec2<int32_t> tex2(src.texCoord(idx2));
      meshBuilder.SetAttributeValuesForFace(
        texAtt, index, tex0.data(), tex1.data(), tex2.data());
    }
  };
  draco::FaceIndex face;
  for (int32_t i = 0; i < triCount; i++) {
    addFace(src.triangle(i), face++);
  }
  return meshBuilder.Finalize();
}

template<typename T>
void 
convert( std::unique_ptr<draco::Mesh>& src, vmesh::TriangleMesh<T>& dst ) {
  draco::Mesh& mesh = *src;
  auto posAtt = mesh.GetNamedAttribute(draco::GeometryAttribute::POSITION);
  auto nrmAtt = mesh.GetNamedAttribute(draco::GeometryAttribute::NORMAL);
  auto colAtt = mesh.GetNamedAttribute(draco::GeometryAttribute::COLOR);
  auto texAtt = mesh.GetNamedAttribute(draco::GeometryAttribute::TEX_COORD);

  // position
  for ( draco::AttributeValueIndex i( 0 ); i < static_cast<uint32_t>( posAtt->size() ); ++i ) {
    std::array<int32_t, 3> value;
    if ( !posAtt->ConvertValue<int32_t, 3>( i, &value[0] ) ) { return; }
    dst.addPoint( value[0], value[1], value[2]);
  }  
  // Normal
  for ( draco::AttributeValueIndex i( 0 ); i < static_cast<uint32_t>( nrmAtt->size() ); ++i ) {
    std::array<int32_t, 3> value;
    if ( !nrmAtt->ConvertValue<int32_t, 3>( i, &value[0] ) ) { return; }
    dst.addNormal( value[0], value[1], value[2]);
  }
  // Color
  for ( draco::AttributeValueIndex i( 0 ); i < static_cast<uint32_t>( colAtt->size() ); ++i ) {
    std::array<int32_t, 3> value;
    if ( !colAtt->ConvertValue<int32_t, 3>( i, &value[0] ) ) { return; }
    dst.addColour( value[0], value[1], value[2]);
  }
  // Texture coordinate
  for ( draco::AttributeValueIndex i( 0 ); i < static_cast<uint32_t>( texAtt->size() ); ++i ) {
    std::array<int32_t, 2> value;
    if ( !texAtt->ConvertValue<int32_t, 3>( i, &value[0] ) ) { return; }
    dst.addTexCoord( value[0], value[1] );
  }
  for (draco::FaceIndex i(0); i < mesh.num_faces(); ++i) {
    auto& face = mesh.face(i);
    const int32_t idx0 = face[0].value();
    const int32_t idx1 = face[1].value();
    const int32_t idx2 = face[2].value();
    dst.addTriangle(idx0, idx1, idx2);
    if (texAtt->size() > 0)
      dst.addTexCoordTriangle(idx0, idx1, idx2);
    if (nrmAtt->size() > 0)
      dst.addNormalTriangle(idx0, idx1, idx2);
  }
}

template<typename T>
void
dracoLibGeometryEncoder<T>::encode(
  TriangleMesh<T>& src,
  GeometryEncoderParameters& params,
  std::vector<uint8_t>& bitstream,
  TriangleMesh<T>& rec)
{
  // Load draco mesh
  // printf( "encoding %s\n", params.srcFileName_.c_str() );
  // auto maybe_mesh = draco::ReadMeshFromFile( params.srcFileName_, false );
  // if ( !maybe_mesh.ok() ) {
  //   printf( "Failed loading the input mesh %s: %s.\n", params.srcFileName_.c_str(), maybe_mesh.status().error_msg() );
  //   exit( -1 );
  // }
  // auto mesh = maybe_mesh.value();
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
  encoder.SetAttributeQuantization(
    draco::GeometryAttribute::NORMAL, params.qn_);
  encoder.SetAttributeQuantization(
    draco::GeometryAttribute::COLOR, params.qc_);
  draco::EncoderBuffer buffer;
  const draco::Status status =
    encoder.EncodeMeshToBuffer(*(mesh.get()), &buffer);
  if (!status.ok()) {
    printf("Failed to encode the mesh: %s\n", status.error_msg());
    exit(-1);
  }

  // Copy bitstream
  bitstream.resize(buffer.size());
  std::copy(buffer.data(), buffer.data() + buffer.size(), bitstream.data());

  // Decode

  draco::DecoderBuffer decBuffer;
  decBuffer.Init( (const char*)bitstream.data(), bitstream.size() );
  auto type = draco::Decoder::GetEncodedGeometryType( &decBuffer );
  if ( !type.ok() ) {
    printf( "Failed GetEncodedGeometryType: %s.\n", type.status().error_msg() );
    exit( -1 );
  }
  if ( type.value() == draco::TRIANGULAR_MESH ) {
    draco::Decoder decoder;
    auto           status = decoder.DecodeMeshFromBuffer( &decBuffer );
    if ( !status.ok() ) {
      printf( "Failed DecodeMeshFromBuffer: %s.\n", status.status().error_msg() );
      exit( -1 );
    }
    std::unique_ptr<draco::Mesh> decMesh = std::move( status ).value();
    if ( decMesh ) {
      convert( decMesh, rec );
    } else {
      printf( "Failed no in mesh  \n" );
      exit( -1 );
    }
  } else {
    printf( "Failed no mesh type not supported.\n" );
    exit( -1 );
  }    
}

template class vmesh::dracoLibGeometryEncoder<uint8_t>;
template class vmesh::dracoLibGeometryEncoder<uint16_t>;

}  // namespace vmesh

#endif
