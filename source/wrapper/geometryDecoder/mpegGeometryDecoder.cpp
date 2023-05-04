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
#include "mpegGeometryDecoder.hpp"

#include "ebModel.h"
#include "ebBasicDecoder.h"

namespace vmesh {

template<typename T>
MpegGeometryDecoder<T>::MpegGeometryDecoder() = default;
template<typename T>
MpegGeometryDecoder<T>::~MpegGeometryDecoder() = default;

// NOTE : duplicated in  mpegLibGeometryEncoder for reconstruction
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
MpegGeometryDecoder<T>::decode(const std::vector<uint8_t>& bitstream,
                               GeometryDecoderParameters&  params,
                               TriangleMesh<T>&            dec) {

    // mm bitstream
    eb::Bitstream bs;
    // init buffer from input
    std::swap(const_cast<std::vector<uint8_t>&>(bitstream), bs.buffer);

    // instanciate decoder
    eb::EBBasicDecoder decoder;

    // deserialize the bitstream
    decoder.unserialize(bs);

    // decode to model
    eb::Model decModel;
    decoder.decode(decModel);

    // convert back to triangle mesh
    convert(decModel, dec);

    // revert bitstream
    std::swap(const_cast<std::vector<uint8_t>&>(bitstream), bs.buffer);
}

template class MpegGeometryDecoder<float>;
template class MpegGeometryDecoder<double>;

}  // namespace vmesh

