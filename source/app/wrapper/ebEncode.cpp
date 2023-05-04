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

// argument parsing
#include <cxxopts.hpp>

#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"

// internal headers
#include "ebIO.h"
#include "ebModel.h"
#include "ebEncoder.h"
#include "ebBasicEncoder.h"

int main(int argc, char* argv[]) {
    
    const char* name = "ebEncode";
    const char* brief = "MPEG Mesh compression using edge breaker.";

    eb::EBEncoder* eb = 0;

    std::string method;
    std::string posPredStr;
    std::string uvPredStr;
    std::string predCoderStr;
    std::string topoCoderStr;
    std::string inputModelFilename;
    std::string outputModelFilename;

    // command line parameters
    try {
        cxxopts::Options options(name, brief);
        options.add_options()
            ("h,help", "Print usage")
            ("i,inputModel", "path to input model (obj or ply file)",
                cxxopts::value<std::string>())
            ("o,outputModel", "path to output encoded file (.eb)",
                cxxopts::value<std::string>())
            ("qp", "Positions quantization bits in [7,20]. Special qp=0 means integer input, no quantization; qp=-1 means skip. Other values will be clamped.",
                cxxopts::value<int>()->default_value("11"))
            ("qt", "UVCoords quantization bits in [7,20]. Special qt=0 means integer input, no quantization; qt=-1 means skip. Other will be clamped.",
                cxxopts::value<int>()->default_value("10"))
            ("qn", "Normals quantization bits in [7,20]. Special qn=0 means integer input, no quantization; qn=-1 means skip. Other will be clamped.",
                cxxopts::value<int>()->default_value("7"))
            ("qc", "Colors  quantization bits in [7,8]. Special qc=0 means integer input, no quantization; qc=-1 means skip. Other will be clamped.",
                cxxopts::value<int>()->default_value("8"))
            ("yuv", "Colors space convertion to yuv prior to quantization if any.",
                cxxopts::value<bool>()->default_value("false"))
            ("method", "edge breaker method in [basic]",
                cxxopts::value<std::string>()->default_value("basic"))
            ("format", "output write format in [binary].",
                cxxopts::value<std::string>()->default_value("binary"))
            ("posPred", "Positions predictor in [none, mpara]",
                cxxopts::value<std::string>()->default_value("mpara"))
            ("uvPred", "UV coordinates predictor in [none, stretch]",
                cxxopts::value<std::string>()->default_value("stretch"))
            ("predCoder", "Entropy coder used to encode predictions in [dirac, rans]",
                cxxopts::value<std::string>()->default_value("dirac"))
            ("topoCoder", "Entopy coder used to encode topology in [dirac, rans]",
                cxxopts::value<std::string>()->default_value("dirac"))
            ("gshift", "Golomb Coding Shift value in [0,10]. Used when dirac is selected",
                cxxopts::value<int>()->default_value("2"))
            ("intAttr", "Consider input attributes (pos and uv) as integers stored on the number of bits resp. defined by --qp and --qt",
                cxxopts::value<bool>()->default_value("false"))
            ("deduplicate", "Deduplicate vertices positions added to handle non-manifold meshes on decode (adds related information in the bitstream)",
                cxxopts::value<bool>()->default_value("false"))
            ("optionFlags", "Flags for options during development",
                cxxopts::value<uint32_t>()->default_value("0"))
            ;

        auto result = options.parse(argc, argv);

        // Analyse the options
        if (result.count("help") || result.arguments().size() == 0) {
            std::cout << options.help() << std::endl;
            return false;
        }
        // Analyse the options
        method = result["method"].as<std::string>();
        if (method != "basic") {
            std::cerr << "Error: invalid --method \"" << method << "\"" << std::endl;
            return false;
        }

        // allocates the encoder
        if (method == "basic") {
            eb = new eb::EBBasicEncoder();
        }

        eb->cfg.intAttr = result["intAttr"].as<bool>();
        eb->cfg.deduplicate = result["deduplicate"].as<bool>();

        eb->cfg.optionFlags = result["optionFlags"].as<uint32_t>();

        eb->qp = result["qp"].as<int>();
        if (eb->qp != 0 && eb->qp != -1)
            glm::clamp(eb->qp, 7, 20);

        eb->qt = result["qt"].as<int>();
        if (eb->qt != 0 && eb->qt != -1)
            glm::clamp(eb->qt, 7, 20);

        eb->qn = result["qn"].as<int>();
        if (eb->qn != 0 && eb->qn != -1)
            glm::clamp(eb->qn, 7, 20);

        eb->qc = result["qc"].as<int>();
        if (eb->qc != 0 && eb->qc != -1)
            glm::clamp(eb->qc, 7, 8);

        eb->gshift = result["gshift"].as<int>();
        glm::clamp(eb->gshift, 0, 10);

        std::string format = result["format"].as<std::string>();
        if (format != "binary")
        {
            std::cerr << "Error: invalid --format \"" << format << "\"" << std::endl;
            return false;
        }

        posPredStr = result["posPred"].as<std::string>();
        if (posPredStr == "none")
            eb->cfg.posPred = eb::EBConfig::PosPred::NONE;
        else if (posPredStr == "mpara")
            eb->cfg.posPred = eb::EBConfig::PosPred::MPARA;
        else
        {
            std::cerr << "Error: invalid --posPred \"" << posPredStr << "\"" << std::endl;
            return false;
        }

        uvPredStr = result["uvPred"].as<std::string>();
        if (uvPredStr == "none")
            eb->cfg.uvPred = eb::EBConfig::UvPred::NONE;
        else if (uvPredStr == "stretch")
            eb->cfg.uvPred = eb::EBConfig::UvPred::STRETCH;
        else
        {
            std::cerr << "Error: invalid --uvPred \"" << uvPredStr << "\"" << std::endl;
            return false;
        }

        predCoderStr = result["predCoder"].as<std::string>();
        if (predCoderStr == "dirac")
            eb->cfg.predCoder = eb::EBConfig::ECName::DIRAC;
        else if (predCoderStr == "rans")
            eb->cfg.predCoder = eb::EBConfig::ECName::RANS;
        else
        {
            std::cerr << "Error: invalid --predCoder \"" << predCoderStr << "\"" << std::endl;
            return false;
        }

        topoCoderStr = result["topoCoder"].as<std::string>();
        if (topoCoderStr == "dirac")
            eb->cfg.topoCoder = eb::EBConfig::ECName::DIRAC;
        else if (topoCoderStr == "RANS")
            eb->cfg.topoCoder = eb::EBConfig::ECName::RANS;
        else
        {
            std::cerr << "Error: invalid --topoCoder \"" << topoCoderStr << "\"" << std::endl;
            return false;
        }

        if (result.count("inputModel"))
            inputModelFilename = result["inputModel"].as<std::string>();
        else {
            std::cerr << "Error: missing inputModel parameter" << std::endl;
            std::cout << options.help() << std::endl;
            return false;
        }

        //
        if (result.count("outputModel"))
            outputModelFilename = result["outputModel"].as<std::string>();
        else {
            std::cerr << "Warning: missing outputModel parameter, compressed model will not be saved" << std::endl;
        }
    }
    catch (const cxxopts::OptionException& e) {
        std::cout << "error parsing options: " << e.what() << std::endl;
        return false;
    }

    // this is mandatory to print floats with full precision
    std::cout.precision(std::numeric_limits<float>::max_digits10);

    // the input
    eb::Model inputModel;

    if (!eb::IO::loadModel(inputModelFilename, inputModel)) {
        return false;
    }
    if (inputModel.vertices.size() == 0) {
        std::cout << "Error: invalid input model (missing vertices) from " << inputModelFilename << std::endl;
        return false;
    }
    if (inputModel.triangles.size() == 0) {
        std::cout << "Error: invalid input model (not a mesh) from " << inputModelFilename << std::endl;
        return false;
    }

    // Perform the processings
    clock_t t1 = clock();

    std::cout << "Starting EB compression of the model" << std::endl;
    std::cout << "  qp = " << eb->qp << std::endl;
    std::cout << "  qt = " << eb->qt << std::endl;
    std::cout << "  qn = " << eb->qn << " not used yet" << std::endl;
    std::cout << "  qc = " << eb->qc << " not used yet" << std::endl;
    std::cout << "  yuv = " << eb->yuv << " not used yet" << std::endl;
    std::cout << "  method = " << method << std::endl;
    std::cout << "  predCoder = " << predCoderStr << std::endl;
    std::cout << "  topoCoder = " << topoCoderStr << std::endl;

    eb->encode(inputModel);

    clock_t t2 = clock();
    std::cout << "Time on processing: " << ((float)(t2 - t1)) / CLOCKS_PER_SEC << " sec." << std::endl;

    // save the result
    eb->save(outputModelFilename);

    delete eb;
        
    return 0;
}