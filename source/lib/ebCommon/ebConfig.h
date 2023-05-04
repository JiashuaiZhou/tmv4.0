// The copyright in this software is being made available under the BSD
// Licence, included below.  This software may be subject to other third
// party and contributor rights, including patent rights, and no such
// rights are granted under this licence.
// 
// Copyright (c) 2023 - InterDigital
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
// * Neither the name of the ISO/IEC nor the names of its contributors
//   may be used to endorse or promote products derived from this
//   software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: jean-eudes.marvie@interdigital.com
// *****************************************************************

#ifndef _MM_EB_CONFIG_H_
#define _MM_EB_CONFIG_H_

namespace eb {

    class EBConfig  {

    public:

        // Entropy Coder name
        enum class ECName {
            DIRAC = 2,
            RANS = 1, 
        };

        // name of the prediction mode for positions
        enum class PosPred {
            NONE = 0,   // use global coordinates, no delta
            MPARA = 2,  // use multi- parallelogram rule
        };

        // name of the prediction mode for positions
        enum class UvPred {
            NONE = 0,   // use global coordinates, no delta
            STRETCH = 2 // uv stretch
        };

    public:

        // predictions entropy encoder
        ECName predCoder = ECName::DIRAC;
        // topology entropy encoder
        ECName topoCoder = ECName::DIRAC;
        // position prediction mode
        PosPred posPred = PosPred::MPARA;
        // uv coords prediction mode
        UvPred uvPred = UvPred::STRETCH;
        // Consider input attributes (pos and uv) as integers sotered on the number opf bits resp. defined by --qp and --qt
        bool intAttr = false;
        // Deduplicate vertices positions added to handle non - manifold meshes on decode(adds related information in the bitstream)
        bool deduplicate = false;
        // flag options
        uint32_t optionFlags = 0;

    };

};  // namespace mm

#endif
