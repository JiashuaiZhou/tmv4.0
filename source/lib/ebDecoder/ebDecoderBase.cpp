// The copyright in this software is being made available under the BSD
// Licence, included below.  This software may be subject to other third
// party and contributor rights, including patent rights, and no such
// rights are granted under this licence.
// 
// Copyright (c) 2022, InterDigital
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//  
// * Neither the name of the InterDigital nor the names of its contributors
//   may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//
// Author: jean-eudes.marvie@interdigital.com
// *****************************************************************

#include <iostream>
#include <algorithm>
#include <fstream>
#include <unordered_map>
#include <time.h>
#include <math.h>
#include <tuple>
#include <bitset>

// internal headers
#include "ebIO.h"
#include "ebModel.h"
#include "ebGeometry.h"
#include "ebColor.h"
#include "ebDecoderBase.h"

using namespace eb;

bool EBDecoderBase::dequantize(void) {

    // quantize position
    auto& positions = _ovTable.positions;
    if (!positions.empty() && qp >= 7) {
        
        const int32_t   maxPositionQuantizedValue = (1u << static_cast<uint32_t>(qp)) - 1;
        const glm::vec3 diag = maxPos - minPos;
        const float     range = std::max(std::max(diag.x, diag.y), diag.z);
        const float     scale = range / maxPositionQuantizedValue;

        for (size_t i = 0; i < positions.size(); i++) {
            positions[i] = positions[i] * scale + minPos;
        }
    }

    // quantize UV coordinates
    auto& uvcoords = _ovTable.uvcoords;
    if (!uvcoords.empty() && qt >= 7) {

        const int32_t   maxUVcoordQuantizedValue = (1u << static_cast<uint32_t>(qt)) - 1;
        const glm::vec2 diag = maxUv - minUv;
        const float     range = std::max(diag.x, diag.y);
        float           scale = range / maxUVcoordQuantizedValue;

        for (size_t i = 0; i < uvcoords.size(); i++) {
            uvcoords[i] = uvcoords[i] * scale + minUv;
        }
    }

    // quantize normals
    auto& normals = _ovTable.normals;
    if (!normals.empty() && qn >= 7) {

        const int32_t   maxNormalQuantizedValue = (1u << static_cast<uint32_t>(qn)) - 1;
        const glm::vec3 diag = maxNrm - minNrm;
        const float     range = std::max(std::max(diag.x, diag.y), diag.z);
        const float     scale = range / maxNormalQuantizedValue;

        for (size_t i = 0; i < normals.size(); i++) {
            normals[i] = normals[i] * scale + minNrm;
        }
    }

    // quantize colors
    // TODO

    return true;
}
