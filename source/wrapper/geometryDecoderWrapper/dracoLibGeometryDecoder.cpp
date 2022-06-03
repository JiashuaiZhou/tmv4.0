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

#include "mesh.hpp"
#include "dracoLibGeometryDecoder.hpp"

#include "draco/compression/decode.h"
#include "draco/core/data_buffer.h"

namespace vmesh {

template<typename T>
dracoLibGeometryDecoder<T>::dracoLibGeometryDecoder()
{}
template<typename T>
dracoLibGeometryDecoder<T>::~dracoLibGeometryDecoder()
{}

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
dracoLibGeometryDecoder<T>::decode(
  std::vector<uint8_t>& bitstream,
  TriangleMesh<T>& dec){
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
      convert( decMesh, dec );
    } else {
      printf( "Failed no in mesh  \n" );
      exit( -1 );
    }
  } else {
    printf( "Failed no mesh type not supported.\n" );
    exit( -1 );
  }    
}

template class vmesh::dracoLibGeometryDecoder<uint8_t>;
template class vmesh::dracoLibGeometryDecoder<uint16_t>;

}  // namespace vmesh

#endif
