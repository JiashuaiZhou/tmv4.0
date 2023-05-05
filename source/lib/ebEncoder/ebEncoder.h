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

#ifndef _EB_ENCODER_H_
#define _EB_ENCODER_H_

#include <fstream>
#include <stack>

// internal headers
#include "ebModel.h"
#include "ebCTMesh.h"
#include "ebConfig.h"
#include "ebBitstream.h"

namespace eb {

    class EBEncoder {
        
    public:

        EBConfig cfg;

        // parameters
        int  qp = 12, qt = 11, qn = 7, qc = 8, gshift = 2;
        bool yuv = false; // not yet implemented

     public:
        
        virtual ~EBEncoder() = default;

        virtual void encode(const Model& input) = 0;
        virtual bool save(std::string fileName) = 0;
        virtual bool serialize(Bitstream& bitstream) = 0;

    protected:
        
        // corner table representation of the mesh
        CTMesh _ovTable;

        // bounding boxes generated and used by quantization
        // will be encoded in bitstream in case if quantization is enabled
        glm::vec3 minPos, maxPos;
        glm::vec3 minCol, maxCol;
        glm::vec3 minNrm, maxNrm;
        glm::vec2 minUv,  maxUv;

        // quantizes the CTModel
        // yuv set to true means convert colors to yuv before quantization
        bool quantize(size_t skipLastNb=0);
    };


};  // namespace mm

#endif
