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
#include "ebChrono.h"
#include "ebModel.h"
#include "ebGeometry.h"
#include "ebColor.h"

#include "ebDecoder.h"
#include "ebBasicDecoder.h"

#include "ebReader.hpp"
#include "syntaxElements/meshCoding.hpp"

using namespace eb;

bool EBDecoder::load(std::string fileName) {

    delete _eb; _eb = 0;

    Bitstream bitstream;

    auto t = now();
    if (!bitstream.load(fileName)) // loads the full file content including format & method
        return false;

    std::cout << "  File read time " << elapsed(t) << "sec" << std::endl;

    std::cout << "  Loaded bistream byte size = " << bitstream.size() << std::endl;

    t = now();
    MeshCoding meshCoding; // Top level syntax element
    EbReader   ebReader;
    ebReader.read(bitstream, meshCoding);
    std::cout << "  Parsing time " << elapsed(t) << "sec" << std::endl;

    // Check codec variant
    const auto& mch = meshCoding.getMeshCodingHeader();
    const auto& method = mch.getMeshCodecType();
    // single variant in this release
    if (method == MeshCodecType::CODEC_TYPE_FORWARD) {
        _eb = new EBBasicDecoder;
    }
    else
        std::cout << "Error: invalid mesh coding variant " << (uint8_t)(method) << std::endl;

    //
    t = now();
    bool res = _eb->unserialize(meshCoding);
    std::cout << "  Unserialize time " << elapsed(t) << "sec" << std::endl;
    return res;
}

bool EBDecoder::decode(Model& output) {
    if (_eb)
        return _eb->decode(output);
    else
        return false;
}
