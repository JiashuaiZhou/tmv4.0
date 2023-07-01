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
#include <time.h>
#include <math.h>
#include <tuple>
#include <vector>

// internal headers
#include "ebIO.h"
#include "ebChrono.h"
#include "ebModel.h"
#include "ebGeometry.h"
#include "ebColor.h"
#include "ebModelConverter.h"

#include "ebEntropy.h"
#include "ebEntropyContext.h"
#include "ebBasicDecoder.h"
#include "ebRansCodecUtils.h"
#include "ebRansSymbolCodec.h"
#include "ebRansBinaryCodec.h"

using namespace eb;

bool EBBasicDecoder::decode(Model& output) {

    std::cout << "  Basic Edgebreaker method selected" << std::endl;
    auto t = now();
    initDecompression();
    startDecompression();
    std::cout << "  EB decoding " << elapsed(t) << "sec" << std::endl;

    t = now();
    if (!cfg.intAttr)
        dequantize();
    std::cout << "  Dequantize time " << elapsed(t) << "sec" << std::endl;

    t = now();
    postProcess(output);
    std::cout << "  Cleanup dummy time " << elapsed(t) << "sec" << std::endl;

    return true;
}

//
void EBBasicDecoder::initDecompression()
{

    auto& G = _ovTable.positions;
    auto& O = _ovTable.O;
    auto& V = _ovTable.V;
    auto& TC = _ovTable.TC;
    auto& OTC = _ovTable.OTC;
    if (sVertices.size()) // mpara case with separate table
        numOfVertices = iVertices.size() + sVertices.size() + iDummyVertices.size()+ numAddedDupVert;
    else
        numOfVertices = iVertices.size()+ numAddedDupVert;

    // Allocate & Init Memory for Tables
    V.resize(3 * numOfTriangles, 0);		// table of vertex Ids for each corner
    O.resize(3 * numOfTriangles, -3);		// table of opposite corner Ids for each corner or free edge orientation
    TC.resize(3 * numOfTriangles, -1);
    OTC.resize(3 * numOfTriangles, -3);

    if (hasSeparateUvIndex)
        numOfUVCoords = iUVCoords.size() + sUVCoords.size();
    else
        numOfUVCoords = numOfVertices;

    // Allocate & Init memory for M and U tables
    _D.resize(numOfVertices, 0);
    _M.resize(numOfVertices, 0);
    _U.resize(numOfTriangles, 0);
    if (hasSeparateUvIndex)
        _MC.resize(numOfUVCoords, 0);

    // Init D table 
    for (const auto& d : iDummyVertices)
    {
        _D[d] = 1;
    }
    // Allocate memory for geometry array
    G.resize(numOfVertices);

    if (iUVCoords.size() != 0)
        _ovTable.uvcoords.resize(numOfUVCoords); 
}

//
void EBBasicDecoder::startDecompression()
{

    // init some global variables
    startCorner = 0;
    startSymbol = 0;
    startHandle = 0;
    startTriangle = 0;
    startVertex = 0;
    startUVCoord = 0;
    ccCount = 0;
    Vcur = 0;
    Dcur = 0;

    do {

        // shortcuts
        auto G = &(_ovTable.positions.data()[startVertex]);
        auto O = &(_ovTable.O.data()[startCorner]);
        auto OTC = &(_ovTable.OTC.data()[startCorner]);
        auto V = &(_ovTable.V.data()[startCorner]);
        auto TC = &(_ovTable.TC.data()[startCorner]);
        auto U = &(_U.data()[startTriangle]);
        auto M = &(_M.data()[startVertex]);
        auto H = &(iHandles.data()[startHandle]);

        perCCduplicates.clear();
        // assign handle opposite for this cc
        for (auto i = A; i < iHandleSizes[ccCount]; i += 2)
        {
            O[iHandles[i] - startSymbol * 3] = iHandles[i + 1] - startSymbol * 3;
            O[iHandles[i + 1] - startSymbol * 3] = iHandles[i] - startSymbol * 3;
        }        
        
        T = 0;
        A = 0;
        N = 0;

        V[0] = 0;
        V[1] = 1;
        V[2] = 2;

        O[1] = -1;
        O[2] = -1;

        // start connectivity decompression
        N = 2; // because we already added the first triangle        
        processedCorners.push_back(0); // caution here !!
        DecompressConnectivity(0);

        // estimate 1st vertex
        decodeMainIndexAttributes(0, 0);
        M[0] = 1;

        // estimate third vertex and mark it as visited
        decodeMainIndexAttributes(1, 1);
        M[1] = 1;

        // estimate second vertex and mark it as visited
        decodeMainIndexAttributes(2, 2);
        M[2] = 1;

        // paint the 1st triangle and go to opposite corner
        U[0] = 1;

        // reinit N for decompressVertices
        N = 2;
        // start vertices decompression from opposite corner
        DecompressVertices(O[0]);

        if (hasSeparateUvIndex)
        {
            decodeSeparateIndexAttributes();
        }

        // shift indices
        for (auto i = 0; i < (T + 1) * 3; ++i) {
            V[i] = V[i] + startVertex;
            TC[i] = TC[i] + startUVCoord;
        }

        if (cfg.deduplicate)
        {
            for (auto dup : perCCduplicates)
            {
                auto curCorner = dup.first;
                auto splitVtxToVertex = dup.second;

                V[curCorner] = splitVtxToVertex;
                auto next_c = curCorner;
                while (next_c = _ovTable.n(O[_ovTable.n(next_c)]),
                    next_c != curCorner && next_c >= 0) {
                    V[next_c] = splitVtxToVertex;
                }
                if (next_c < 0)
                {
                    next_c = curCorner;
                    while (next_c = _ovTable.p(O[_ovTable.p(next_c)]),
                        next_c != curCorner && next_c >= 0) {
                        V[next_c] = splitVtxToVertex;
                    }
                }
            }
        }

        // update start indices for next connected component
        startCorner = startCorner + (T + 1) * 3;

        if (startCorner < _ovTable.V.size()) {

            startTriangle = _ovTable.t(startCorner);
            startSymbol = startSymbol + T;
            startVertex = startVertex + N + 1;
            startHandle = startHandle + A;
            startUVCoord = startUVCoord + NTC;
            ccCount++;
        }
        processedCorners.clear();

    } while (startCorner < _ovTable.V.size());
    std::cout << "zipnum=" << zipnum << std::endl;
}

//
void EBBasicDecoder::DecompressConnectivity(int c)
{
    auto& ov = _ovTable;
    auto C = &(iClers.data()[startSymbol]);
    auto O = &(_ovTable.O.data()[startCorner]);
    auto V = &(_ovTable.V.data()[startCorner]);

    //Loop builds triangle tree and zips it up
    do {
        //new triangle
        T++;

        //attach new triangle, link opposite corners
        O[c] = 3 * T;
        O[3 * T] = c;

        //enter vertex Ids for shared vertices
        V[3 * T + 1] = V[ov.p(c)];
        V[3 * T + 2] = V[ov.n(c)];
 
        //move corner to new triangle
        c = ov.n(O[c]);

        processedCorners.push_back(ov.p(c));

        //select operation based on next symbol
        switch (C[T - 1]) {
        case 'C':
            //C: left edge is free, store ref to new vertex			
            O[ov.n(c)] = -1;
            V[3 * T] = ++N;
            break;
        case 'L':
            //L: orient free edge
            if (O[ov.n(c)] < 0) 
                O[ov.n(c)] = -2;
            //check for handles, if non, try to zip
            if (!CheckHandle(ov.n(c)))
                Zip(ov.n(c));
            break;
        case 'R':
            //R: orient free edge, check for handles, go left 
            if (O[c]<0)
                O[c] = -2;
            CheckHandle(c);
            c = ov.n(c);
            break;
        case'S':
            //S: recursion going right, then go left
            DecompressConnectivity(c);
            c = ov.n(c);
            //if the triangle to the left was visited, then return	
            if (O[c] >= 0)
                return;
            break;
        case 'E':
            //E: left and right edges are  free
            if (O[c] < 0)
                O[c] = -2;
            if (O[ov.n(c)] < 0)
                O[ov.n(c)] = -2;
            //check for handles on the right
            CheckHandle(c);
            //check for handles on the left, if non, try to zip
            if (!CheckHandle(ov.n(c)))
                Zip(ov.n(c));
            //pop 	
            return;
            break;
        }
    } while (true);
}

//
bool EBBasicDecoder::CheckHandle(int c)
{
    auto& ov = _ovTable;
    auto C = &(iClers.data()[startSymbol]);
    auto H = &(iHandles.data()[startHandle]);
    auto O = &(_ovTable.O.data()[startCorner]);
    auto V = &(_ovTable.V.data()[startCorner]);

    //check if this is a handle
    if (A >= iHandleSizes[ccCount] || c != (H[A + 1] - startSymbol*3))
        return false;
    else {
        //link opposite corners
        O[c] = H[A] - startSymbol * 3;
        O[H[A] - startSymbol*3] = c;

        //find corner of next free edge if any 
        int a = ov.p(c);
        while ((O[a] >= 0) && (a != (H[A] - startSymbol * 3)))
            a = ov.p(O[a]);

        //zip if found cw edge
        if (O[a] == -2)
            Zip(a);

        //find corner of next free edge if any
        a = ov.p(O[c]);
        while ((O[a] >= 0) && (a != c))
            a = ov.p(O[a]);

        //zip if found cw edge
        if (O[a] == -2)
            Zip(a);

        //next handle
        A += 2;
        return true;
    }
}

//
void EBBasicDecoder::Zip(int c)
{
    auto& ov = _ovTable;
    auto G = &(_ovTable.positions.data()[startVertex]);
    auto O = &(_ovTable.O.data()[startCorner]);
    auto V = &(_ovTable.V.data()[startCorner]);

    //tries to zip free edges opposite c
    int b = ov.n(c);

    //search clockwise for free edge
    while (O[b] >= 0 && O[b] != c) {
        b = ov.n(O[b]);
    }

    //pop if no zip possible
    if (O[b] != -1) {
        return;
    }

    //link opposite corners
    O[c] = b; O[b] = c;

    //assign co-incident corners
    int a = ov.n(c);
    V[ov.n(a)] = V[ov.n(b)];
    while (O[a] >= 0 && a != b) {
        a = ov.n(O[a]);
        V[ov.n(a)] = V[ov.n(b)];
    }

    //find corner of next free edge on right
    c = ov.p(c);
    while (O[c] >= 0 && c != b) {
        c = ov.p(O[c]);
    }

    //try to zip again
    if (O[c] == -2) {
        zipnum++;
        Zip(c);
    }
}

//
void EBBasicDecoder::DecompressVertices(int c)
{
    auto& ov = _ovTable;
    auto G = &(_ovTable.positions.data()[startVertex]);
    auto O = &(_ovTable.O.data()[startCorner]);
    auto V = &(_ovTable.V.data()[startCorner]);
    auto U = &(_U.data()[startTriangle]);
    auto M = &(_M.data()[startVertex]);

    //start traversal for triangle tree
    do {
        //mark the triangle as visited
        U[ov.t(c)] = 1;

        //test whether tip vertex was visited
        if (M[V[c]] == 0) {
            //update new vertex
            N++;
            decodeMainIndexAttributes(c, N);
            //mark tip vertex as visited
            M[V[c]] = 1;
            //continue with the right neighbor
            c = ov.r(c, O);
        }
        else
            //test whether right triangle was visited
            if (U[ov.t(ov.r(c, O))] == 1) {
                //test whether left triangle was visited
                if (U[ov.t(ov.l(c, O))] == 1)
                {
                    //E, pop
                    return;
                }
                else
                {
                    //R,move to left triangle
                    c = ov.l(c, O);
                }
            }
            else
                //test whether left triangle was visited
                if (U[ov.t(ov.l(c, O))] == 1) {
                    //L, move to right triangle
                    c = ov.r(c, O);
                }
                else {
                    //S, recursive call to visit right branch first
                    DecompressVertices(ov.r(c, O));
                    //move to left triangle
                    c = ov.l(c, O);
                    //if the triangle to the left was visited, then return
                    if (U[ov.t(c)] > 0)
                        return;
                }
    } while (true);
}

//
void EBBasicDecoder::decodeMainIndexAttributes(int c, int v)
{
    const auto V = &(_ovTable.V.data()[startCorner]);
    const bool predictUVs = (iUVCoords.size() != 0) && (iSeams.size() == 0);
    bool dispdup = false;
    if (cfg.deduplicate && isVertexDup.size())
    {
        if (isVertexDup[Vcur++])
        {
            auto splitIdx = iDuplicateSplitVertexIdx[Dcur++];
            auto dupIt = processedDupIdx.find(splitIdx);
            if (dupIt != processedDupIdx.end())
            {
                perCCduplicates.push_back(std::make_pair(c, dupIt->second));
                posSkipDup++; //?
                auto G = &(_ovTable.positions.data()[startVertex]);
                G[v] = _ovTable.positions[dupIt->second];
                if (predictUVs)
                {
                    auto UV = &(_ovTable.uvcoords.data()[startVertex]);
                    UV[v] = _ovTable.uvcoords[dupIt->second];
                }
                dispdup = true;
            }
            else
            {
                processedDupIdx[splitIdx] = V[c] + startVertex;
            }
        }
    }

    if (!dispdup) {
        // positions
        switch (cfg.posPred) {
        case EBConfig::PosPred::NONE:
        {
            auto G = &(_ovTable.positions.data()[startVertex]);
            G[v] = iVertices[startVertex + V[c]];
        }
        break;
        case EBConfig::PosPred::MPARA:
            posDecodeWithPrediction(c, v);
            break;
        }
        // UVCoords
        if (predictUVs) {
            switch (cfg.uvPred) {
            case EBConfig::UvPred::NONE:
            {
                auto UV = &(_ovTable.uvcoords.data()[startVertex]);
                UV[v] = iUVCoords[startVertex + V[c]];
            }
            break;
            case EBConfig::UvPred::STRETCH:
                uvDecodeWithPrediction(c, v);
                break;
            }
        }
    }
}

//
void EBBasicDecoder::posDecodeWithPrediction(int c, int v)
{
    const auto MAX_PARALLELOGRAMS = 4;
    auto& ov = _ovTable;
    auto  G = &(_ovTable.positions.data()[startVertex]); // use CC shifting
    auto  UV = &(_ovTable.uvcoords.data()[startVertex]);  // use CC shifting
    auto  O = &(_ovTable.O.data()[startCorner]);         // use CC shifting
    auto  V = &(_ovTable.V.data()[startCorner]);         // use CC shifting
    auto  M = &(_M.data()[startVertex]);                 // use CC shifting
    auto  D = &(_D.data()[startVertex]);                 // use CC shifting

    if ((v == 0) || D[V[c]])                   // if start point or dummy point
    {
        if (v == 0)
            G[v] = sVertices[posSvalueIdx++];  // reads the separate table for start vertex
        else
            posSkipNum++;                      // or skip and we track the number of skipped dummy points
        return;
    }

    // Otherwise, reads the value for this vertex, may be delta or prediction residuals
    auto& pos = iVertices[startVertex + V[c] - posSvalueIdx - posSkipNum - posSkipDup];

    switch (v) {
    case 1:                                          // delta
    case 2:                                          // delta
        G[v] = G[v - 1] + pos; break;
    default:                                        // multi-parallelogram 
        int count = 0;
        bool last = false;
        glm::vec3 predPos(0);
        auto altC = c;
        auto prevIsDummy = D[V[ov.p(c)]];
        // loop through corners attached to the current vertex
        do
        {
            if (count >= MAX_PARALLELOGRAMS) break;
            if (((!D[V[O[altC]]]) && (!D[V[ov.p(altC)]]) && (!D[V[ov.n(altC)]])) &&
                ((M[V[O[altC]]] > 0) && (M[V[ov.p(altC)]] > 0) && (M[V[ov.n(altC)]] > 0)))
            {
                const auto estG = G[V[ov.p(altC)]] + G[V[ov.n(altC)]] - G[V[O[altC]]];
                predPos += estG;
                ++count;
            }
            altC = ov.p(O[ov.p(altC)]);
        } while (altC != c);

        if (count > 0)                              // use parallelogram prediction when possible
            predPos = glm::round(predPos / glm::vec3(count));
        else                                        // or fallback to delta with available values
            predPos = prevIsDummy ? G[V[ov.n(c)]] : G[V[ov.p(c)]];

        G[v] = predPos + pos;                       // store the reconstructed value = prediction + residual
    } // end of switch

}


//
void EBBasicDecoder::predictUV(const int c, const int* indices, const int v, glm::vec2& uv, bool predWithDummies, bool prevIsDummy)
{

    auto& ov = _ovTable;
    auto G = &(_ovTable.positions.data()[startVertex]);
    auto V = &(_ovTable.V.data()[startCorner]);
    auto UV = &(_ovTable.uvcoords.data()[hasSeparateUvIndex ? startUVCoord : startVertex]);
    const auto& IDX = indices;

    // accumulate uv predictions 
    const glm::dvec2 uvPrev = UV[IDX[ov.p(c)]];
    const glm::dvec2 uvNext = UV[IDX[ov.n(c)]];
    const glm::dvec3 gPrev = G[V[ov.p(c)]];
    const glm::dvec3 gNext = G[V[ov.n(c)]];
    const glm::dvec3 gCurr = G[V[c]];

    const glm::dvec3 gNgP = gPrev - gNext;
    const glm::dvec3 gNgC = gCurr - gNext;
    const glm::dvec2 uvNuvP = uvPrev - uvNext;
    const double gNgP_dot_gNgC = glm::dot(gNgP, gNgC);
    const double d2_gNgP = glm::dot(gNgP, gNgP);
    if (d2_gNgP > 0)
    {
        const glm::dvec2 uvProj = uvNext + uvNuvP * (gNgP_dot_gNgC / d2_gNgP);
        const glm::dvec3 gProj = gNext + gNgP * (gNgP_dot_gNgC / d2_gNgP);
        const double d2_gProj_gCurr = glm::dot(gCurr - gProj, gCurr - gProj);
        const glm::dvec2 uvProjuvCurr = glm::dvec2(uvNuvP.y, -uvNuvP.x) * std::sqrt(d2_gProj_gCurr / d2_gNgP);
        const glm::dvec2 predUV0(uvProj + uvProjuvCurr);
        const glm::dvec2 predUV1(uvProj - uvProjuvCurr);
        glm::dvec2 predUVd = (iOrientations[orientationIdx++]) ? predUV0 : predUV1;
        glm::vec2 predUV = glm::vec2(std::round(predUVd.x), std::round(predUVd.y));
        UV[v] = predUV + uv;
    }
    else
    {
        if (predWithDummies) {
            glm::vec2 predUV = prevIsDummy ? UV[IDX[ov.n(c)]] : UV[IDX[ov.p(c)]];
            UV[v] = predUV + uv;
        }
        else {
            glm::vec2 predUV = (glm::vec2(UV[IDX[ov.n(c)]]) + glm::vec2(UV[IDX[ov.p(c)]])) / 2.0f;
            predUV = glm::vec2(std::round(predUV.x), std::round(predUV.y));
            UV[v] = predUV + uv;
        }
    }
}

//
void EBBasicDecoder::uvDecodeWithPrediction(int c, int v)
{
    auto& ov = _ovTable;
    auto G = &(_ovTable.positions.data()[startVertex]);
    auto UV = &(_ovTable.uvcoords.data()[startVertex]);
    auto O = &(_ovTable.O.data()[startCorner]);
    auto V = &(_ovTable.V.data()[startCorner]);
    auto M = &(_M.data()[startVertex]);
    auto D = &(_D.data()[startVertex]);

    // use a separate table for start and dummy vertices
    if ((v == 0) || D[V[c]])
    {
        if (v == 0)
            UV[v] = sUVCoords[uvSvalueIdx++];
        else
            uvSkipNum++;
        return;
    }

    // following may be deltas
    auto& uv = iUVCoords[startVertex + V[c] - uvSvalueIdx - uvSkipNum - posSkipDup];

    switch (v) {
    case 0: // global
        UV[v] = uv;
        break;
    case 1: // delta
    case 2: // delta
        UV[v] = UV[v - 1] + uv;
        break;
    default: // parallelogram
        const glm::vec3& prev = G[V[ov.p(c)]];
        const glm::vec3& next = G[V[ov.n(c)]];
        const auto& prevIsDummy = D[V[ov.p(c)]];
        const auto& nextIsDummy = D[V[ov.n(c)]];
        if (prevIsDummy || nextIsDummy)
        {
            // multi stretch prediction for positions
            int count = 0;
            bool last = false;
            glm::vec3 predPos(0);
            auto altC = c;

            // loop through corners attached to the current vertex
            do
            {
                if (count >= 1) break; // no multiple predictions yet stop on first
                // break on max count ?
                if ((c != altC) && (D[V[altC]]))
                { // stop after looping in both directions or if a complete turn achieved
                    if (last) break;
                    altC = c;
                    last = true;
                }
                else
                {
                    // ensure that p.n from altC with same V[altC] are already decoded and are not dummies
                    const auto& PaltCIsDummy = D[V[ov.p(altC)]];
                    const auto& NaltCIsDummy = D[V[ov.n(altC)]];
                    if (((!PaltCIsDummy) && (!NaltCIsDummy))
                        && ((M[V[ov.p(altC)]] > 0) && (M[V[ov.n(altC)]] > 0)))
                    {
                        predictUV(altC, V, v, uv, true, prevIsDummy);
                        ++count;
                    }
                }
                // swing right or left
                altC = (!last) ? ov.p(O[ov.p(altC)]) : ov.n(O[ov.n(altC)]); 
            } while (altC != c);

            if (count == 0) // nok !!
            {
                glm::vec2 predUV = prevIsDummy ? UV[V[ov.n(c)]] : UV[V[ov.p(c)]];
                UV[v] = predUV + uv;
            }
        }
        else
        {
            predictUV(c, V, v, uv, false);
        }
        break;
    };
}

//
void EBBasicDecoder::decodeSeparateIndexAttributes(void)
{
    const auto& ov = _ovTable;
    auto G = &(_ovTable.positions.data()[startVertex]);
    auto O = &(_ovTable.O.data()[startCorner]);
    auto V = &(_ovTable.V.data()[startCorner]);
    auto D = &(_D.data()[startVertex]);             // dummies
    auto MC = &(_MC.data()[startUVCoord]);
    auto OTC = &(_ovTable.OTC.data()[startCorner]);
    auto TC = &(_ovTable.TC.data()[startCorner]);

    for (auto corner : processedCorners)
    {
        const int corners[3] = { corner, ov.n(corner), ov.p(corner) };

        // do not process dummy triangles
        bool isDummy = (D[V[corners[0]]]) || (D[V[corners[1]]]) || (D[V[corners[2]]]);
        if (isDummy)
        { // no need to assign TC index on dummy triangles
            OTC[corners[0]] = 0;
            OTC[corners[1]] = 0;
            OTC[corners[2]] = 0;
            continue;
        }
        const int src_face_id = ov.t(corner);
        for (int c = 0; c < 3; ++c) {
            const int cur_corner = corners[c];
            const int opp_corner = O[cur_corner];
            bool isOppDummy = (D[V[opp_corner]]);
            // no explicit seams on boundary edges
            if (isOppDummy) continue;
            const int opp_face_id = ov.t(opp_corner);
            // encode only one side 
            if (opp_face_id < src_face_id) continue;
            // use pre evaluated seam code
            const auto is_seam = iSeams[seamsIdx++];
            const auto fval = (is_seam) ? -2 : -1;
            OTC[cur_corner] = fval;
            OTC[opp_corner] = fval;
        }
    }

    NTC = 0;
    for (auto corner : processedCorners)
    {
        const int corners[3] = { corner, ov.n(corner), ov.p(corner) };
        for (int c = 0; c < 3; ++c) {
            const int cur_corner = corners[c];
            if (TC[cur_corner] >= 0) continue;
            if (OTC[cur_corner] >= 0) continue;
            const auto newTC = NTC++;
            TC[cur_corner] = newTC;
            // swing to right most extent 
            auto movC = cur_corner;
            while (OTC[ov.n(movC)] == -1)
            {
                movC = ov.n(O[ov.n(movC)]);
                TC[movC] = newTC;
                if (movC == cur_corner) break;
            }
            // then the other side
            movC = cur_corner;
            while (OTC[ov.p(movC)] == -1)
            {
                movC = ov.p(O[ov.p(movC)]);
                TC[movC] = newTC;
                if (movC == cur_corner) break;
            }
        }
    }

    auto uvIdx = 0;
    for (auto corner : processedCorners)
    {
        const int corners[3] = { corner, ov.n(corner), ov.p(corner) };
        for (int c = 0; c < 3; ++c) {
            const int cur_corner = corners[c];
            //
            if ((TC[cur_corner] < 0) || MC[TC[cur_corner]] > 0) continue;

            MC[TC[cur_corner]] = 1;

            switch (cfg.uvPred) {
            case EBConfig::UvPred::NONE:
            {
                auto UV = &(_ovTable.uvcoords.data()[startUVCoord]);
                UV[TC[cur_corner]] = iUVCoords[startUVCoord + uvIdx];
            }
            break;
            case EBConfig::UvPred::STRETCH:
                uvSepDecodeWithPrediction(cur_corner, uvIdx);
                break;
            }
            uvIdx++;
        }
    }
}

//
void EBBasicDecoder::uvSepDecodeWithPrediction(int c, int uvIdx)
{
    const auto& OV = _ovTable;
    const auto OTC = &(_ovTable.OTC.data()[startCorner]);
    const auto O = &(_ovTable.O.data()[startCorner]);
    const auto TC = &(_ovTable.TC.data()[startCorner]);
    const auto MC = &(_MC.data()[startUVCoord]);
    auto UV = &(_ovTable.uvcoords.data()[startUVCoord]);

    // following may be deltas
    auto& uv = iUVCoords[startUVCoord + uvIdx - uvSvalueIdx];

    bool hasUVn = TC[OV.n(c)] >= 0 && MC[TC[OV.n(c)]];
    bool hasUVp = TC[OV.p(c)] >= 0 && MC[TC[OV.p(c)]];

    int predC = c;
    int minn = (hasUVn)+(hasUVp);
    if (!hasUVn || !hasUVp)
    {
        auto altC = c;
        while (OTC[OV.n(altC)] == -1)
        {
            altC = OV.n(O[OV.n(altC)]);
            if (altC == c) break;
            const bool hasUVna = TC[OV.n(altC)] >= 0 && MC[TC[OV.n(altC)]];
            const bool hasUVpa = TC[OV.p(altC)] >= 0 && MC[TC[OV.p(altC)]];
            int curn = (hasUVna)+(hasUVpa);
            if (curn > minn)
            {
                predC = altC;
                minn = curn;
                hasUVn = hasUVna;
                hasUVp = hasUVpa;
                if (minn == 2) break;
            }
        }
        if (altC != c && minn < 2)
        {
            altC = c;
            while (OTC[OV.p(altC)] == -1)
            {
                altC = OV.p(O[OV.p(altC)]);
                if (altC == c) break;
                const bool hasUVna = TC[OV.n(altC)] >= 0 && MC[TC[OV.n(altC)]];
                const bool hasUVpa = TC[OV.p(altC)] >= 0 && MC[TC[OV.p(altC)]];
                int curn = (hasUVna)+(hasUVpa);
                if (curn > minn)
                {
                    predC = altC;
                    minn = curn;
                    hasUVn = hasUVna;
                    hasUVp = hasUVpa;
                    if (minn == 2) break;
                }
            }
        }
    }

    // use a separate table for start and dummy vertices.. then less unique symbols
    if (!hasUVn && !hasUVp)
    {
        UV[TC[predC]] = sUVCoords[uvSvalueIdx++];
        return;
    }
    else if (!hasUVn)
    {
        UV[TC[predC]] = uv + UV[TC[OV.p(predC)]];
    }
    else if (!hasUVp)
    {
        UV[TC[predC]] = uv + UV[TC[OV.n(predC)]];
    }
    else
    {
        predictUV(predC, TC, TC[predC], uv, false);
    }
}

void EBBasicDecoder::postProcess(Model& decoded)
{
    const auto& G = _ovTable.positions;
    const auto& UV = _ovTable.uvcoords;
    auto& V = _ovTable.V;
    auto& TC = _ovTable.TC;

    std::vector<bool> trimask = std::vector<bool>(numOfTriangles, true);

    // mark triangles to be discarded and modify indices...
    int dummy_count = iDummyVertices.size();

    std::vector<int> dc = std::vector<int>(dummy_count);
    copy(iDummyVertices.begin(), iDummyVertices.end(), dc.begin());

    for (int i = 0; i < dummy_count; i++) {
        for (int j = 0; j < numOfTriangles; j++) {
            if (trimask[j] == false)
                continue;
            if ((V[3 * j] == dc[i]) || (V[3 * j + 1] == dc[i]) || (V[3 * j + 2] == dc[i]))
                trimask[j] = false;
            else {
                for (int k = 0; k < 3; k++) {
                    if (V[3 * j + k] > dc[i])
                        V[3 * j + k] -= 1;
                }
            }
        }
        for (int j = i + 1; j < dummy_count; j++) {
            dc[j] -= 1;
        }
    }

    // now populate the mesh...
    if (dummy_count > 0) {
        int from = 0;
        int to;
        for (int i = 0; i < dummy_count; i++) {
            to = iDummyVertices[i];
            for (int j = from; j < to; j++) {
                const auto& pos = G[j];
                decoded.vertices.push_back(pos.x);
                decoded.vertices.push_back(pos.y);
                decoded.vertices.push_back(pos.z);
                if (!hasSeparateUvIndex && (UV.size() != 0)) {
                    const auto& uv = UV[j];
                    decoded.uvcoords.push_back(uv.x);
                    decoded.uvcoords.push_back(uv.y);
                }
            }
            from = to + 1;
        }
        if (from < numOfVertices) {
            to = numOfVertices;
            for (int j = from; j < to; j++) {
                const auto& pos = G[j];
                decoded.vertices.push_back(pos.x);
                decoded.vertices.push_back(pos.y);
                decoded.vertices.push_back(pos.z);
                if (!hasSeparateUvIndex && (UV.size() != 0)) {
                    const auto& uv = UV[j];
                    decoded.uvcoords.push_back(uv.x);
                    decoded.uvcoords.push_back(uv.y);
                }
            }
        }
    }
    else {
        for (int i = 0; i < numOfVertices; i++) {
            const auto& pos = G[i];
            decoded.vertices.push_back(pos.x);
            decoded.vertices.push_back(pos.y);
            decoded.vertices.push_back(pos.z);
            if (!hasSeparateUvIndex && (UV.size() != 0)) {
                const auto& uv = UV[i];
                decoded.uvcoords.push_back(uv.x);
                decoded.uvcoords.push_back(uv.y);
            }
        }
    }

    if (hasSeparateUvIndex) {
        for (int i = 0; i < numOfUVCoords; i++) {
            const auto& uv = UV[i];
            decoded.uvcoords.push_back(uv.x);
            decoded.uvcoords.push_back(uv.y);
        }
    }
    for (int i = 0; i < numOfTriangles; i++) {
        if (trimask[i]) {
            decoded.triangles.push_back(V[3 * i + 0]);
            decoded.triangles.push_back(V[3 * i + 1]);
            decoded.triangles.push_back(V[3 * i + 2]);
            if (hasSeparateUvIndex) {
                decoded.trianglesuv.push_back(TC[3 * i + 0]);
                decoded.trianglesuv.push_back(TC[3 * i + 1]);
                decoded.trianglesuv.push_back(TC[3 * i + 2]);
            }
        }
    }

    if (cfg.deduplicate)
    {
        // remove duplicate vertices
        int idx = 0;
        int unref = 0;
        std::vector<int> mapping(decoded.vertices.size() / 3, -1);
        for (int i = 0; i < decoded.triangles.size(); ++i) {
            if (mapping[decoded.triangles[i]] < 0)
            {
                mapping[decoded.triangles[i]] = idx++;
            }
        }
        for (int i = 0; i < decoded.triangles.size(); ++i) {
            decoded.triangles[i] = mapping[decoded.triangles[i]];
        }
        std::vector<float> newVertices(decoded.vertices.size());
        std::vector<float> newUVCoords(decoded.uvcoords.size());
        for (int i = 0; i < mapping.size(); ++i) {
            if (mapping[i] >= 0) 
            {
                newVertices[3 * mapping[i] + 0] = decoded.vertices[3 * i + 0];
                newVertices[3 * mapping[i] + 1] = decoded.vertices[3 * i + 1];
                newVertices[3 * mapping[i] + 2] = decoded.vertices[3 * i + 2];
                if (!hasSeparateUvIndex) {
                    newUVCoords[2 * mapping[i] + 0] = decoded.uvcoords[2 * i + 0];
                    newUVCoords[2 * mapping[i] + 1] = decoded.uvcoords[2 * i + 1];
                }
            }
        }
        newVertices.resize(3 * idx);
        std::swap(decoded.vertices, newVertices);
        if (!hasSeparateUvIndex)
        {
            newUVCoords.resize(2 * idx);
            std::swap(decoded.uvcoords, newUVCoords);
        }
    }
}

// Converts a single unsigned integer symbol back to a signed value.
int32_t ConvertSymbolToSignedInt(uint32_t val) {
    const bool is_positive = !static_cast<bool>(val & 1);
    val >>= 1;
    if (is_positive) {
        return (val);
    }
    int32_t ret = (val);
    ret = -ret - 1;
    return ret;
}

bool EBBasicDecoder::unserialize(MeshCoding& meshCoding) {

    auto NumVertices = 0;
    auto NumPositionStart = 0;
    auto NumPredictedPositions = 0;

    // Extracting Mesh Coding Header
    auto& mch = meshCoding.getMeshCodingHeader();
    
    auto& method = (uint8_t&)mch.getMeshCodecType();

    // Position Encoding Parameters
    auto& mpep = mch.getMeshPositionEncodingParameters();
    qp = mpep.getMeshPositionBitDepthMinus1() = qp + 1;

    cfg.topoCoder = (EBConfig::ECName)mpep.getMeshClersSymbolsEncodingMethod();
    cfg.predCoder = (EBConfig::ECName)mpep.getMeshPositionPredictionMethod();// UPDATE SPLIT PRED CODER FOR ATTRIBS
    cfg.posPred = EBConfig::PosPred::MPARA; // UPDATE ENUMS(EBConfig::PosPred)mpep.getMeshPositionResidualsEncodingMethod();
    cfg.deduplicate = !(mpep.getMeshPositionDeduplicateMethod() == MeshPositionDeduplicateMethod::MESH_POSITION_DEDUP_NONE);
    // Position Dequantization
    cfg.intAttr = !(mch.getMeshPositionDequantizeFlag());
    if (mch.getMeshPositionDequantizeFlag())
    {
        auto& mpdp = mch.getMeshPositionDequantizeParameters();
        minPos.x = mpdp.getMeshPositionMin(0);
        minPos.y = mpdp.getMeshPositionMin(1);
        minPos.z = mpdp.getMeshPositionMin(2);
        maxPos.x = mpdp.getMeshPositionMax(0);
        maxPos.y = mpdp.getMeshPositionMax(1);
        maxPos.z = mpdp.getMeshPositionMax(2);
    }
    // Attributes
    for (auto i = 0; i < mch.getMeshAttributeCount(); i++)
    {
        mch.getMeshAttributeType()[i] = MeshAttributeType::MESH_ATTR_TEXCOORD; // TO BE MODIFIED - simplification as tex coord only considered now
        auto& maep = mch.getMeshAttributesEncodingParameters(i);

        qt = maep.getMeshAttributeBitDepthMinus1() + 1; // caution single attribute here

        if (!maep.getMeshAttributePerFaceFlag()) // CAUTION always true in current version
        {
            hasSeparateUvIndex = maep.getMeshAttributeSeparateIndexFlag(); // caution single attribute here
            if (!maep.getMeshAttributeSeparateIndexFlag())
                maep.getMeshAttributeReferenceIndexPlus1();// CAUTION not used for now
        }
        // !! something to revise to ba able to better write the following
        cfg.uvPred = EBConfig::UvPred::STRETCH; // TO BE UDATED MeshAttributePredictionMethod_TEXCOORD::MESH_TEXCOORD_STRETCH;
        // TO ADD SPLIT AC METHOD PER ATTRIB MeshAttributeResidualsEncodingMethod_TEXCOORD::MESH_TEXCOORD_AC_DEFAULT;

        // TODO SPLIT DEQUANT GLOBAL AND PER ATTRIBUTE COMPONENT
        if (mch.getMeshAttributeDequantizeFlag()[i])
        {
            // CAUTION single UV attrib here
            auto& madp = mch.getMeshAttributesDequantizeParameters()[i];
            /*
            for (auto j = 0; j < mch.getNumComponents(i); j++)
            {
                madp.getMeshAttributeMin(j) = mmm;
                madp.getMeshAttributeMax(j) = MMM;
            }
            */
            minUv.x = madp.getMeshAttributeMin(0);
            minUv.y = madp.getMeshAttributeMin(1);
            maxUv.x = madp.getMeshAttributeMax(0);
            maxUv.y = madp.getMeshAttributeMax(1);
        }
    }


    // Payload

    // Extracting Mesh Position Coding Payload
    auto& mpcp = meshCoding.getMeshPositionCodingPayload();

    iVertices.resize(mpcp.getMeshVertexCount());//!! check alignment with semantics
    ccCount = mpcp.getMeshCcCount();
    iDummyVertices.resize(mpcp.getMeshVirtualVertexCount());
    int shift = 0;
    for (auto i = 0; i < mpcp.getMeshVirtualVertexCount(); ++i) {
        shift += mpcp.getMeshVirtualIndexDelta()[i];
        iDummyVertices[i] = shift;
    }
    auto NumHandles = 0;
    int index = 0;
    int size = 0;
    iHandleSizes.resize(ccCount, 0);
    for (auto i = 0; i < mpcp.getMeshCcWithHandlesCount(); i++) {
        index += mpcp.getMeshHandlesCcOffset()[i];
        NumHandles += mpcp.getMeshHandlesCount()[i];
        iHandleSizes[index] = 2*NumHandles;
    }
    iHandles.resize(2*NumHandles);
    uint32_t curHandle1 = 0;
    uint32_t curHandle2 = 0;
    mpcp.getMeshHandleIndexFirstDelta().resize(NumHandles);
    mpcp.getMeshHandleIndexSecondDelta().resize(NumHandles);
    if (NumHandles)
    {
        rans::RansBinaryDecoder ad;
        ad.init(mpcp.getMeshHandleIndexSecondShift());
        for (auto i = 0; i < NumHandles; i++) {
            const bool orient = ad.decodeNextBit();
            curHandle1 += mpcp.getMeshHandleIndexFirstDelta()[i];
            curHandle2 += mpcp.getMeshHandleIndexSecondDelta()[i];
            iHandles[2 * i + 0] = 3 * curHandle1 + 2;
            iHandles[2 * i + 1] = 3 * curHandle2 + (orient ? 1 : 2);
        }
    }

    {
        const char codeToChar[8] = { 'C','S',0,'L',0,'R',0,'E' };
        const uint32_t& topoByteCount = mpcp.getMeshCodedClersSymbolsSize();

        EntropyDecoder   ad;
        AdaptiveBitModel ctx_isNotC[32]; // not all 32 contexts used - code to be rewitten
        AdaptiveBitModel ctx_bit1[32];
        AdaptiveBitModel ctx_bit02[32];
        AdaptiveBitModel ctx_bit12[32];
        const auto* const  bufferPtr =
            reinterpret_cast<const char*>(&mpcp.getMeshClersSymbol()[0]);
        ad.setBuffer(topoByteCount, bufferPtr);
        ad.start();
        iClers.resize(mpcp.getMeshClersCount());
        int pS = 0; //'C'
        bool useExtended = (mpcp.getMeshClersCount() > 3000); // fixed threshold is this basic variant
        for (auto i = 0; i < mpcp.getMeshClersCount(); ++i) {
            int32_t value = ad.decode(ctx_isNotC[pS]);
            if (value)
            {
                const auto bit1 = ad.decode(ctx_bit1[pS]);
                const auto bit2 = ad.decode(bit1 ? ctx_bit12[pS] : ctx_bit02[pS]);
                value |= bit1 << 1;
                value |= bit2 << 2;

            }
            if (!useExtended)
                pS = value; // 5 contexts
            else
                pS = value + ((pS & 1) << 3) + ((pS & 4) << 2); // extended contexts (4*5)

            iClers[i] = codeToChar[value];
        }
    }

    numOfTriangles = mpcp.getMeshClersCount() + mpcp.getMeshCcCount();
    numSplitVert = 0;
    numAddedDupVert = 0;
    auto& mpdi = mpcp.getMeshPositionDeduplicateInformation();

    // TODO - rewrite using globals from parsing to avoid code replication
    if (mpep.getMeshPositionDeduplicateMethod() == MeshPositionDeduplicateMethod::MESH_POSITION_DEDUP_DEFAULT) {
        iDuplicateSplitVertexIdx.resize(mpdi.getMeshPositionDeduplicateCount());
        unsigned int NumSplitVertex = 0;
        if (mpdi.getMeshPositionDeduplicateCount() > 0)
        {
            for (auto i = 0; i < mpdi.getMeshPositionDeduplicateCount(); ++i) {
                iDuplicateSplitVertexIdx[i] = mpdi.getMeshPositionDeduplicateIdx()[i];
                NumSplitVertex = std::max(NumSplitVertex, mpdi.getMeshPositionDeduplicateIdx()[i] + 1);
            }
            NumPositionStart = mpdi.getMeshPositionDeduplicateStartPositions();
            auto NumAddedDuplicatedVertex = mpdi.getMeshPositionDeduplicateCount() - NumSplitVertex;
            NumVertices = mpcp.getMeshVertexCount()
                + mpdi.getMeshPositionDeduplicateStartPositions() // should not be sVertices ?
                + mpcp.getMeshVirtualVertexCount()
                + NumAddedDuplicatedVertex;
            numSplitVert = NumSplitVertex;
            numAddedDupVert = NumAddedDuplicatedVertex;
        }
        else
            NumPositionStart = mpcp.getMeshCcCount();
    }
    else
        NumPositionStart = mpcp.getMeshCcCount();

    sVertices.resize(NumPositionStart); // used, this is sVertices when different from ccount
    for (auto i = 0; i < NumPositionStart; i++) {
        mpcp.getMeshPositionStart()[i].resize(3);
        for (auto j = 0; j < 3; j++) {
            sVertices[i][j] = mpcp.getMeshPositionStart()[i][j];
        }
    }

    // !! is it true if we skip predictions and if vertex count not set to iVerts but total input verts ? 
    NumPredictedPositions = mpcp.getMeshVertexCount() - mpcp.getMeshCcCount();

    // Extracting Mesh Attribute Coding Payload

    auto& macp = meshCoding.getMeshAttributeCodingPayload();
    hasSeparateUvIndex = false;
    auto& NumAttributeStart = meshCoding.getNumAttributeStart();
    for (auto i = 0; i < mch.getMeshAttributeCount(); i++) {
        //macp.get
        auto mesh_attribute_per_face_flag = mch.getMeshAttributesEncodingParameters(i).getMeshAttributePerFaceFlag();
        auto mesh_attribute_separate_index_flag = mch.getMeshAttributesEncodingParameters(i).getMeshAttributeSeparateIndexFlag();
        if (!mesh_attribute_per_face_flag && mesh_attribute_separate_index_flag) {
            hasSeparateUvIndex = true;
            auto& nbSeams = macp.getMeshAttributeSeamsCount()[i];
            iSeams.resize(macp.getMeshAttributeSeamsCount()[i]);
            AdaptiveBitModel ctx;
            EntropyDecoder     ad;
            const auto* const  bufferPtr =
                reinterpret_cast<const char*>(&macp.getMeshAttributeSeam()[i][0]);
            ad.setBuffer(macp.getMeshCodedAttributeSeamsSize()[i], bufferPtr);
            ad.start();
            for (uint32_t j = 0; j < nbSeams; j++)
                iSeams[j] = ad.decode(ctx);
        }

        sUVCoords.resize(NumAttributeStart[i]);
        for (auto j = 0; j < NumAttributeStart[i]; j++) {
            macp.getMeshAttributeStart()[i][j].resize(3);
            for (auto k = 0; k < mch.getNumComponents(i); k++) {
                sUVCoords[j][k] = macp.getMeshAttributeStart()[i][j][k];
            }
        }

        iUVCoords.resize(macp.getMeshAttributeResidualsCount()[i]);

        if (mesh_attribute_separate_index_flag)
        {
            auto& madi = macp.getMeshAttributeDeduplicateInformation()[i]; //EMPTY 
        }

        {
            auto& maed = macp.getMeshAttributeExtraData()[i];
            if ((mch.getMeshAttributeType()[i] == MeshAttributeType::MESH_ATTR_TEXCOORD)
                && (mch.getMeshAttributesEncodingParameters()[i].getMeshAttributePredictionMethod()
                    == (uint8_t)MeshAttributePredictionMethod_TEXCOORD::MESH_TEXCOORD_STRETCH))
            {
                auto& mtced = maed.getMeshTexCoordStretchExtraData();
                auto& nbOrientations = mtced.getMeshTexCoordStretchOrientationsCount();
                iOrientations.resize(nbOrientations);
                rans::RansBinaryDecoder ad;
                ad.init(mtced.getMeshTexCoordStretchOrientation());
                for (uint32_t j = 0; j < nbOrientations; j++)
                    iOrientations[j] = ad.decodeNextBit();
            }
        }

    }

    // Decode deduplication info
    numOfVertices = NumVertices;
    isVertexDup.resize(NumVertices);
    if (NumVertices)
    {
        AdaptiveBitModel ctx;
        EntropyDecoder     ad;
        const auto* const  bufferPtr =
            reinterpret_cast<const char*>(&mpdi.getMeshPositionIsDuplicateFlag()[0]);
        ad.setBuffer(mpdi.getMeshPositionIsDuplicateSize(), bufferPtr);
        ad.start();
        for (uint32_t i = 0; i < numOfVertices; i++)
            isVertexDup[i] = ad.decode(ctx);
    }
    // Decode position info
    {
        uint32_t posByteCount = mpcp.getMeshCodedPositionResidualsSize();
        uint8_t gshift = 2;
        AdaptiveBitModel ctxCoeffRemPrefix[12];
        AdaptiveBitModel ctxCoeffRemSuffix[12];
        const auto bias = 3;
        const auto bctx = 2;
        AdaptiveBitModel ctxz;
        AdaptiveBitModel ctxs;
        AdaptiveBitModel ctxlb[2][bctx];
        EntropyDecoder     ad;
        const auto* const  bufferPtr = reinterpret_cast<const char*>(&mpcp.getMeshPositionResidual()[0]);
        ad.setBuffer(posByteCount, bufferPtr);
        ad.start();
        const auto biasExp = 2;
        bool islow = true;
        //for (auto i = 0; i < verticesSize; ++i) {
        for (int32_t k = 0; k < 3; ++k) { // better in this case to loop through all x then y then z ... or use sep ctx for those
            for (auto i = 0; i < iVertices.size(); ++i) {
                int value = 0;
                {
                    bool ispos = true;
                    if (!ad.decode(ctxz))
                    {
                        ++value; // implied by not is z
                        ispos = !ad.decode(ctxs);
                        for (int k = 0; k < bias; ++k)
                        {
                            const auto ctx = std::min(k, bctx - 1);
                            if (!ad.decode(ctxlb[islow][ctx]))
                                break;
                            ++value;
                        }
                        islow = value < (bias + 1); // +1 cause by +1 as not Z
                    }
                    else
                        islow = true;
                    if (value == (bias + 1))
                        value += ad.decodeExpGolomb(gshift, ctxCoeffRemPrefix, ctxCoeffRemSuffix);
                    value = ispos ? value : -value;
                }
                iVertices[i][k] = value;
            }
        }
        ad.stop();
    }

    // Decode uv info
    {
        uint32_t uvByteCount = macp.getMeshCodedAttributeResidualsSize()[0];
        uint8_t gshift = 2;

        AdaptiveBitModel ctxCoeffRemPrefix[12];
        AdaptiveBitModel ctxCoeffRemSuffix[12];
        EntropyDecoder     ad;
        const auto bias = 3;
        const auto bctx = 2;
        AdaptiveBitModel ctxz;
        AdaptiveBitModel ctxs;
        AdaptiveBitModel ctxlb[2][bctx];
        const auto* const  bufferPtr = reinterpret_cast<const char*>(&macp.getMeshAttributeResidual()[0][0]);
        ad.setBuffer(uvByteCount, bufferPtr);
        ad.start();
        bool islow = true;
        for (auto i = 0; i < iUVCoords.size(); ++i) {
            for (int32_t k = 0; k < 2; ++k) {
                int value = 0;
                {
                    bool ispos = true;
                    if (!ad.decode(ctxz))
                    {
                        ++value; // implied by not is z
                        ispos = !ad.decode(ctxs);
                        for (int k = 0; k < bias; ++k)
                        {
                            const auto ctx = std::min(k, bctx - 1);
                            if (!ad.decode(ctxlb[islow][ctx]))
                                break;
                            ++value;
                        }
                        islow = value < (bias + 1); // +1 cause by +1 as not Z
                    }
                    else
                        islow = true;
                    if (value == (bias + 1))
                        value += ad.decodeExpGolomb(gshift, ctxCoeffRemPrefix, ctxCoeffRemSuffix);
                    value = ispos ? value : -value;
                }
                iUVCoords[i][k] = value;
            }
        }
        ad.stop();
    }

    return true;
}
