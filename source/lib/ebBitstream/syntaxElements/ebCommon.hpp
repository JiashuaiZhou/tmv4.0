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

namespace eb {

enum class MeshCodecType {
	CODEC_TYPE_FORWARD = 0,  // 0
	CODEC_TYPE_RESERVED_1,   // 1
	CODEC_TYPE_RESERVED_2,   // 2
	CODEC_TYPE_RESERVED_3    // 3
};

enum class MeshClersSymbolsEncodingMethod {
	MESH_CLERS_AC_DEFAULT = 0,  // 0
	// MESH_CLERS_AC_RESERVED >0
};

enum class MeshPositionPredictionMethod {
	MESH_POSITION_MPARA = 0,  // 0
	// MESH_POSITION_RESERVED >0
};

enum class MeshPositionResidualsEncodingMethod {
	MESH_POSITION_AC_DEFAULT = 0,  // 0
	// MESH_POSITION_AC_RESERVED >0
};

enum class MeshPositionDeduplicateMethod {
	MESH_POSITION_DEDUP_NONE = 0,  // 0
	MESH_POSITION_DEDUP_DEFAULT   // 1
	// MESH_POSITION_DEDUP_RESERVED > 1
};

enum class MeshAttributeType {
	MESH_ATTR_TEXCOORD = 0,  // 0
	MESH_ATTR_NORMAL,        // 1
	MESH_ATTR_COLOR,         // 2
	MESH_ATTR_MATERIAL_ID,   // 3
	MESH_ATTR_GENERIC,       // 4
	MESH_ATTR_RESERVED_5,    // 5
	MESH_ATTR_RESERVED_6,    // 6
	MASH_ATTR_UNSPECIFIED    // 7
};

enum class MeshAttributePredictionMethod_TEXCOORD {
	MESH_TEXCOORD_STRETCH = 0  // 0
	// MESH_TEXCOORD_RESERVED >0
};

enum class MeshAttributePredictionMethod_NORMAL {
	MESH_NORMAL_DEFAULT = 0  // 0
	// MESH_NORMAL_RESERVED >0
};

enum class MeshAttributePredictionMethod_COLOR {
	MESH_COLOR_DEFAULT = 0  // 0
	// MESH_COLOR_RESERVED >0
};

enum class MeshAttributePredictionMethod_MATERIALID {
	MESH_MATERIALID_DEFAULT = 0  // 0
	// MESH_MATERIALID_RESERVED >0
};

enum class MeshAttributePredictionMethod_GENERIC {
	MESH_GENERIC_DEFAULT = 0  // 0
	// MESH_GENERIC_RESERVED >0
};

enum class MeshAttributeResidualsEncodingMethod_TEXCOORD {
	MESH_TEXCOORD_AC_DEFAULT = 0  // 0
	// MESH_TEXCOORD_AC_RESERVED >0
};

enum class MeshAttributeResidualsEncodingMethod_NORMAL {
	MESH_NORMAL_AC_DEFAULT = 0  // 0
	// MESH_NORMAL_AC_RESERVED >0
};

enum class MeshAttributeResidualsEncodingMethod_COLOR {
	MESH_COLOR_AC_DEFAULT = 0  // 0
	// MESH_COLOR_AC_RESERVED >0
};

enum class MeshAttributeResidualsEncodingMethod_MATERIALID {
	MESH_MATERIALID_AC_DEFAULT = 0  // 0
	// MESH_MATERIALID_AC_RESERVED >0
};

enum class MeshAttributeResidualsEncodingMethod_GENERIC {
	MESH_GENERIC_AC_DEFAULT = 0  // 0
	// MESH_GENERIC_AC_RESERVED >0
};



// no !! cleanup
template <MeshAttributeType a> class MeshAttributePrediction {
};

template<> class MeshAttributePrediction<MeshAttributeType::MESH_ATTR_TEXCOORD> {
	typedef enum MeshAttributePredictionMethod_TEXCOORD Method;
};
template<> class MeshAttributePrediction<MeshAttributeType::MESH_ATTR_NORMAL> {
	typedef enum MeshAttributePredictionMethod_NORMAL Method;
};
template<> class MeshAttributePrediction<MeshAttributeType::MESH_ATTR_COLOR> {
	typedef enum MeshAttributePredictionMethod_COLOR Method;
};
template<> class MeshAttributePrediction<MeshAttributeType::MESH_ATTR_MATERIAL_ID> {
	typedef enum MeshAttributePredictionMethod_MATERIALID Method;
};
template<> class MeshAttributePrediction<MeshAttributeType::MESH_ATTR_GENERIC> {
	typedef enum MeshAttributePredictionMethod_GENERIC Method;
};

};  // namespace eb
