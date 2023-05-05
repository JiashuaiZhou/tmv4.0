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
#include "ebModelConverter.h"

#include "ebBasicEncoder.h"
#include "ebEntropy.h"
#include "ebEntropyContext.h"
#include "ebRansCodecUtils.h"
#include "ebRansSymbolCodec.h"
#include "ebRansBinaryCodec.h"

#include "ebBitstream.h"
#include "ebWriter.hpp"
#include "syntaxElements/meshCoding.hpp"

using namespace eb;

void EBBasicEncoder::encode(const Model& input)
{

    // Cleanup the input model to convert non-manifold elements to manifold

    // A.1 - Convert input mesh into OVTable
    auto t = now();
    ModelConverter::convertModelToCTMesh(input, _ovTable, cfg.deduplicate);
    std::cout << "  Convert time " << elapsed(t) << "sec" << std::endl;

    // A.2 - triangulate boundaries if needed, by adding dummy points and triangles
    t = now();
    int nbLoops = _ovTable.closeBoundaries(dummyVertices);
    std::cout << "  Hole fill time " << elapsed(t) << "sec" << std::endl;

    if (nbLoops != 0) {
        std::cout << "  -> Detected " << nbLoops << " loops that were triangulated adding " << dummyVertices.size()
            << " dumy vertices and " << _ovTable.getTriangleCount() - input.getTriangleCount() << " triangles" << std::endl;
    }

    // A.3 - Quantize the CT mesh if needed*
    t = now();
    if (!cfg.intAttr)
        quantize(dummyVertices.size()); // do not include dummy values in bbox computation

    std::cout << "  Quantize time " << elapsed(t) << "sec" << std::endl;

    // B - init some little stuffs
    // then these allocations shall better be performed in initCompression
    t = now();

    numOfVertices  = _ovTable.getPositionCount(); // pb is allocated after dummies inserted 
    numOfTriangles = _ovTable.getTriangleCount();
    numOfUVCoords  = _ovTable.getUVCount();

    // Allocate memory for vertex array
    G_est.resize(numOfVertices);
    // Allocate memory for M and U tables
    M.resize(numOfVertices);	 // Table for marking visited vertices
    U.resize(numOfTriangles);  // Table for marking visited triangles
    MC.resize(numOfUVCoords);	 // Table for marking visited corners

    //init tables for marking visited vertices and triangles
    for (auto i = 0; i < numOfVertices; i++)  M[i] = 0;
    for (auto i = 0; i < numOfTriangles; i++) U[i] = 0;
    std::fill(MC.begin(), MC.end(), 0);

    // C - do the job, CC per CC

    // T id of the last symbol compressed so far
    T = 0;
    Vcur = 0;
    ccCount = 0;
    posSkipDup = 0;
    int startCorner = 0;
    int startHandle = 0;
    while (startCorner != -1) {

        startCompression(startCorner);

        int size = oHandles.size() - startHandle;
        if (size != 0) {                              // we store only if non null
            oHandleSizes.push_back(ccCount);          // the index of the CC 
            oHandleSizes.push_back(oHandles.size());  // the size of the handle
            startHandle = oHandles.size();
        }

        ccCount++;
        startCorner = findRestartCorner();
    }

    //
    std::cout << "  EB encoding time " << elapsed(t) << "sec" << std::endl;
}

// 
int EBBasicEncoder::findRestartCorner() {
    for (auto t = 0; t < U.size(); ++t) {
        if (U[t] == 0)
            return  t * 3;
    }
    return -1;
}

//
void EBBasicEncoder::startCompression(int c)
{
    // std::cout << "initCompression " << c << std::endl;
    const auto& ov = _ovTable;
    const auto& G = _ovTable.positions;
    const auto& V = _ovTable.V;
    const auto& O = _ovTable.O;

    // estimate 1st  vertex
    encodeMainIndexAttributes(c, 0);
    M[V[c]] = 1;

    //estimate third vertex and mark it as visited
    encodeMainIndexAttributes(ov.n(c), 1);
    M[V[ov.n(c)]] = 1;

    //estimate second vertex and mark it as visited
    encodeMainIndexAttributes(ov.p(c), 2);
    M[V[ov.p(c)]] = 1;
    processedCorners.push_back(c);

    // paint the triangle 
    U[ov.t(c)] = 1; // mark the triangle as visited

    // traverse triangles incident on the first vertex
    int a = O[c];

    // we keep a count of number of triangles incident on the first corner
    int count = 1;

    //first traverse 'C' triangles 
    while (a != ov.p(O[ov.p(c)])) {

        // increment count for number of triangles incident on the first corner
        count++;

        // paint the triangle, increment # of triangles 
        U[ov.t(a)] = 1;
        T++;

        // estimate next vertex and mark it as visited
        processedCorners.push_back(a);
        encodeMainIndexAttributes(a, -1);
        M[ov.V[a]] = 1;

        // continue with the right neighbor 
        a = O[ov.n(a)];
    }

    // traverse 'R' triangle incident on first vertex 
    processedCorners.push_back(a);
    U[ov.t(a)] = 1;
    T++;
    count++;

    // Initializes the CLRES table
    // Append(I - 2) Cs and 1R to the beginning of the array
    // First write I-2 C's
    for (int i = 0; i < (count - 2); i++) {
        oClers.push_back('C');
    }
    // now write one R
    oClers.push_back('R');

    // Start CC compression
    CompressRec(O[ov.p(a)]);

    // Finally encode attributes with separate index table
    encodeSeparateIndexAttributes();
}

//
void EBBasicEncoder::CompressRec(int c)
{
    const auto& ov = _ovTable;
    const auto& G = _ovTable.positions;
    const auto& V = _ovTable.V;
    const auto& O = _ovTable.O;

    //start traversal for triangle tree
    do
    {
        processedCorners.push_back(c);
        //mark the triangle as visited
        U[ov.t(c)] = 1;
        T++;

        //check for handles
        CheckHandle(c);

        //test whether tip vertex was visited
        if (M[V[c]] == 0)
        {
            //append encoding of C to clers
            oClers.push_back('C');

            //estimate next vertex and mark it as visited
            encodeMainIndexAttributes(c, -1);
            M[V[c]] = 1;

            //continue with the right neighbor
            c = ov.r(c);
        }
        else
            //test whether right triangle was visited
            if (U[ov.t(ov.r(c))] > 0)
            {
                //test whether left triangle was visited
                if (U[ov.t(ov.l(c))] > 0)
                {
                    //append code for E and pop
                    oClers.push_back('E');
                    return;
                }
                else
                {
                    //append code for R, move to left triangle
                    oClers.push_back('R');
                    c = ov.l(c);
                }
            }
            else
                //test whether left triangle was visited
                if (U[ov.t(ov.l(c))] > 0)
                {
                    //append code for L, move to right triangle
                    oClers.push_back('L');
                    c = ov.r(c);
                }
                else
                {
                    //store corner number in decompression, to support handles
                    U[ov.t(c)] = T * 3 + 2;

                    //append code for S
                    oClers.push_back('S');

                    //recursive call to visit right branch first
                    CompressRec(ov.r(c));

                    //move to left triangle
                    c = ov.l(c);

                    //if the triangle to the left was visited, then  return
                    if (U[ov.t(c)] > 0)
                        return;
                }
    } while (true);

}

// we use triangles indices only and binary indication on direction, then 3T+1 or 3T+2 values
void EBBasicEncoder::CheckHandle(const int c)
{
    const auto& ov = _ovTable;

    //check for handles from the right
    if (U[ov.t(ov.O[ov.n(c)])] > 1)
    {
        //store opposite corners for handle triangles
        oHandles.push_back(U[ov.t(ov.O[ov.n(c)])]);
        oHandles.push_back(T * 3 + 1);
    }

    //check for handles from the left
    if (U[ov.t(ov.O[ov.p(c)])] > 1)
    {
        //store opposite corners for handle triangles
        oHandles.push_back(U[ov.t(ov.O[ov.p(c)])]);
        oHandles.push_back(T * 3 + 2);
    }
}

//
void EBBasicEncoder::encodeMainIndexAttributes(const int c, const int v)
{
    const auto& V = _ovTable.V;
    const auto& G = _ovTable.positions;
    const auto& UV = _ovTable.uvcoords;
    const auto& OTC = _ovTable.OTC;
    const bool predictUVs = (UV.size() && !OTC.size()); // predict UVs in first pass if no separate index

    bool bypasspos = false;
    if (cfg.deduplicate)
    {
        // check for duplicated positions
        const auto dupIt = _ovTable.duplicatesMap.find(_ovTable.V[c]);
        if (dupIt != _ovTable.duplicatesMap.end())
        {
            isVertexDup.push_back(true);
            oDuplicateSplitVertexIdx.push_back(dupIt->second);
        }
        else
            isVertexDup.push_back(false);

        Vcur++;

        // early return if duplicate already coded
        if (dupIt != _ovTable.duplicatesMap.end())
        {
            if (processedDupIdx.find(dupIt->second) != processedDupIdx.end())
            {
                bypasspos = true; // no need to encode as already processed
                posSkipDup++;
            }
            else
                processedDupIdx.insert(dupIt->second);
        }
        // then use duplicate pos info to replicate the value when decoding      
    }

    // reindex the dummy vertives
    if (isCornerVertexDummy(c))
    {
        // reindexation is dependent on how we encode some values separately
        if (cfg.posPred != EBConfig::PosPred::MPARA)
            oDummies.push_back(oVertices.size());
        else
            oDummies.push_back(oVertices.size() + sVertices.size() + oDummies.size() + posSkipDup);
    }

    if (!bypasspos)
    {
        // positions
        switch (cfg.posPred) {
        case EBConfig::PosPred::NONE:
            oVertices.push_back(G[V[c]]);
            break;
        case EBConfig::PosPred::MPARA:
            posEncodeWithPrediction(c, v);
            break;
        }
        // UVCoords
        if (predictUVs) {
            switch (cfg.uvPred) {
            case EBConfig::UvPred::NONE:
                oUVCoords.push_back(UV[V[c]]);
                break;
            case EBConfig::UvPred::STRETCH:
                uvEncodeWithPrediction(c, v);
                break;
            }
        }
    }
}

inline bool EBBasicEncoder::isCornerVertexDummy(const int c)
{
    return ((dummyVertices.size() > 0) && (_ovTable.V[c] >= dummyVertices[0]));
}

void EBBasicEncoder::posEncodeWithPrediction(const int c, const int v)
{
    const auto MAX_PARALLELOGRAMS = 4;
    const auto& OV = _ovTable;
    const auto& V = _ovTable.V;                 // NO CC SHIFTING
    const auto& O = _ovTable.O;                 // NO CC SHIFTING
    const auto& G = _ovTable.positions;         // NO CC SHIFTING
    // use a separate table for start and dummy vertices => less unique symbols for entropy coding
    if ((v == 0) || isCornerVertexDummy(c))
    {
        if (v == 0)
            sVertices.push_back(G[V[c]]);                     // start point, store as global coordinate
        return;
    }
    switch (v) {                                              // case 0 already handled along with dummies
    case 1:                                                   // store delta, 
    case 2:                                                   // store delta, 
        oVertices.push_back(G[V[c]] - G[V[OV.p(c)]]); break;
    default:                                                  // store parallelogram estimation
        bool prevIsDummy = isCornerVertexDummy(OV.p(c));
        // search for some parallelogram estimations around the vertex of the corner
        int  count = 0;
        int  altC = c;
        glm::vec3 predPos(0, 0, 0);                       // the predicted position
        do                                                // loop through corners attached to the current vertex 
        {
            if (count >= MAX_PARALLELOGRAMS) break;
            if (((!isCornerVertexDummy(O[altC])) &&
                (!isCornerVertexDummy(OV.p(altC))) &&
                (!isCornerVertexDummy(OV.n(altC)))) &&
                ((M[V[O[altC]]] > 0) && (M[V[OV.p(altC)]] > 0) && (M[V[OV.n(altC)]] > 0)))
            {
                // parallelogram prediction estG = prevG + nextG - oppoGd
                glm::vec3 estG = G[V[OV.p(altC)]] + G[V[OV.n(altC)]] - G[V[O[altC]]];
                predPos += estG;                      // accumulate parallelogram predictions
                ++count;
            }
            altC = OV.p(O[OV.p(altC)]);              // swing around the triangle fan
        } while (altC != c);

        if (count > 0)                                             // use parallelogram prediction when possible
            predPos = glm::round(predPos / glm::vec3(count));      // divide and round each component of vector predPos
        else                                                       // or fallback to delta with available values
            // G[V[OV.n(c)]] cannot be dummy if prevIsDummy and is necessarly marked
            predPos = prevIsDummy ? G[V[OV.n(c)]] : G[V[OV.p(c)]];

        oVertices.push_back(G[V[c]] - predPos);    // store the residual = position - predicted
    }                                              // end of switch
}

// c corner to use for the prediction, 
// predWithDummies boolean to predict using the dummy branch
// prevIsDummy set to true if previous corner is a dummy point (default to false)
void EBBasicEncoder::predictUV(const int c, const std::vector<int>& indices, bool predWithDummies, bool prevIsDummy)
{
    const auto& OV = _ovTable;
    const auto& V = _ovTable.V;
    const auto& G = _ovTable.positions;
    const auto& UV = _ovTable.uvcoords;
    const auto& IDX = indices;

    // we do not accumulate uv predictions, stop on first 
    glm::dvec2 uvPrev = UV[IDX[OV.p(c)]];
    glm::dvec2 uvNext = UV[IDX[OV.n(c)]];
    glm::dvec2 uvCurr = UV[IDX[c]];
    glm::dvec3 gPrev = G[V[OV.p(c)]];
    glm::dvec3 gNext = G[V[OV.n(c)]];
    glm::dvec3 gCurr = G[V[c]];
    glm::dvec3 gNgP = gPrev - gNext;
    glm::dvec3 gNgC = gCurr - gNext;
    glm::dvec2 uvNuvP = uvPrev - uvNext;
    double gNgP_dot_gNgC = glm::dot(gNgP, gNgC);
    double d2_gNgP = glm::dot(gNgP, gNgP); 
    if (d2_gNgP > 0)
    {
        glm::dvec2 uvProj = uvNext + uvNuvP * (gNgP_dot_gNgC / d2_gNgP);
        glm::dvec3 gProj = gNext + gNgP * (gNgP_dot_gNgC / d2_gNgP);
        double d2_gProj_gCurr = glm::dot(gCurr - gProj, gCurr - gProj);              
        const glm::dvec2 uvProjuvCurr = glm::dvec2(uvNuvP.y, -uvNuvP.x) * std::sqrt(d2_gProj_gCurr / d2_gNgP);
        glm::dvec2 predUV0(uvProj + uvProjuvCurr);
        glm::dvec2 predUV1(uvProj - uvProjuvCurr);
        bool orientation = length(uvCurr - predUV0) < length(uvCurr - predUV1); 
        glm::vec2 predUV = round(orientation ? predUV0 : predUV1);
        glm::vec2 resUV = UV[IDX[c]] - predUV;
        oUVCoords.push_back(resUV);
        orientations.push_back(orientation);
    }
    else
    {
        glm::vec2 predUV(0.0, 0.0);
        if (predWithDummies)                                    // if next or prev corner is a dummy point 
        {
            predUV = prevIsDummy ? UV[IDX[OV.n(c)]] : UV[IDX[OV.p(c)]];
        }
        else                                                      // else average the two predictions
        {
            predUV = round((UV[IDX[OV.n(c)]] + UV[IDX[OV.p(c)]]) / 2.0f);
        }
        glm::vec2 resUV = UV[IDX[c]] - predUV;
        oUVCoords.push_back(resUV);
    }
}


void EBBasicEncoder::uvEncodeWithPrediction(const int c, const int v) {
    auto& OV = _ovTable;
    auto& V = _ovTable.V;
    auto& O = _ovTable.O;
    auto& G = _ovTable.positions;
    auto& UV = _ovTable.uvcoords;

    // use a separate table for start and dummy uv coords => less unique symbols for entopy coding
    if ((v == 0) || isCornerVertexDummy(c))
    {
        if (v == 0)
            sUVCoords.push_back(UV[V[c]]);                          // start point, store as global coordinate
        // this introduces a shift to be handled when decoding as no related oUVCoords.push_back exist
        return;
    }
    // switch on vertex index. case 0 already handled with dummies
    switch (v) {
    case 1:                                                       // delta, 
    case 2:                                                       // delta, 
        oUVCoords.push_back(UV[V[c]] - UV[V[OV.p(c)]]); break;
    default:                                                      // parallelogram
        bool prevIsDummy = isCornerVertexDummy(OV.p(c));
        bool nextIsDummy = isCornerVertexDummy(OV.n(c));
        bool predWithDummies = nextIsDummy || prevIsDummy;
        if (predWithDummies)
        {
            int  count = 0;
            bool last = false;
            int  altC = c;
            glm::vec3 predPos(0.0, 0.0, 0.0);
            do                                   // loop through corners attached to the current vertex
            {
                if (count >= 1) break;           // no multiple predictions, stop on first corner found
                if ((c != altC) && isCornerVertexDummy(altC))
                {   // stop after looping in both directions or if a complete turn achieved
                    if (last) break;
                    altC = c;
                    last = true;
                }
                // ensure that p.n from altC with same V[altC] are already decoded and are not dummies
                else if (((!isCornerVertexDummy(OV.p(altC))) && (!isCornerVertexDummy(OV.n(altC))))
                    && ((M[V[OV.p(altC)]] > 0) && (M[V[OV.n(altC)]] > 0)))
                {
                    predictUV(altC, V, true, prevIsDummy);
                    ++count;
                }
                altC = (!last) ? OV.p(O[OV.p(altC)]) : OV.n(O[OV.n(altC)]); // swing right or left
            } while (altC != c);

            if (count == 0) // no corner found
            {
                glm::vec2 predUV = prevIsDummy ? UV[V[OV.n(c)]] : UV[V[OV.p(c)]];
                const auto resUV = UV[V[c]] - predUV;
                oUVCoords.push_back(resUV);
            }
        }
        else
        {
            predictUV(c, V, false);
        }
        break;
    } // end of switch
}

void EBBasicEncoder::encodeSeparateIndexAttributes(void)
{

    const auto& ov = _ovTable;

    if (ov.OTC.size()) // if hasUV and separate table
    {
        // here we should have the list of the processed corners and be able to define seams
        for (auto corner : processedCorners)
        {
            const int corners[3] = { corner, ov.n(corner), ov.p(corner) };

            for (int i = 0; i < 3; ++i) {
                const int cur_corner = corners[i];
                const int opp_corner = ov.O[cur_corner];
                // no explicit seams on boundary edges
                if (ov.OTC[cur_corner] < -2) continue;
                const bool is_seam = ov.OTC[cur_corner] == -2;
                seams.push_back(is_seam);
                const int updval = is_seam ? -3 : -4; // used to identify seams when predicting uvs
                _ovTable.OTC[cur_corner] = updval;
                _ovTable.OTC[opp_corner] = updval;
            }
        }

        const auto& TC = _ovTable.TC;
        const auto& UV = _ovTable.uvcoords;

        for (auto corner : processedCorners)
        {
            const int corners[3] = { corner, ov.n(corner), ov.p(corner) };
            for (int i = 0; i < 3; ++i) {
                const int cur_corner = corners[i];
                if ((TC[cur_corner] < 0) || MC[TC[cur_corner]] > 0) continue;

                switch (cfg.uvPred) {
                case EBConfig::UvPred::NONE:
                    oUVCoords.push_back(UV[TC[cur_corner]]);
                    break;
                case EBConfig::UvPred::STRETCH:
                    uvSepEncodeWithPrediction(cur_corner);
                    break;
                }
                MC[TC[cur_corner]] = 1;
            }
        }

    }

}

//
void EBBasicEncoder::uvSepEncodeWithPrediction(const int c) {
    // add check opposite decoded and no seam ? -> then no need to send orientation but not important
    auto& OV = _ovTable;
    auto& OTC = _ovTable.OTC;
    auto& O = _ovTable.O;
    auto& UV = _ovTable.uvcoords;
    auto& TC = _ovTable.TC;
    auto& V = _ovTable.V;
    auto& G = _ovTable.positions;
    bool hasUVn = TC[OV.n(c)] >= 0 && MC[TC[OV.n(c)]];
    bool hasUVp = TC[OV.p(c)] >= 0 && MC[TC[OV.p(c)]];
    int predC = c;
    int minn = (hasUVn)+(hasUVp);

    // selection of the triangle used for prediction
    if (!hasUVn || !hasUVp) 
    {
        auto altC = c;
        while (OTC[OV.n(altC)] == -4)
        {
            altC = OV.n(O[OV.n(altC)]);
            if (altC == c) break;
            bool hasUVna = TC[OV.n(altC)] >= 0 && MC[TC[OV.n(altC)]];
            bool hasUVpa = TC[OV.p(altC)] >= 0 && MC[TC[OV.p(altC)]];
            int curn = (hasUVna)+(hasUVpa);
            if (curn > minn)
            {
                predC = altC; minn = curn;
                hasUVn = hasUVna; hasUVp = hasUVpa;
                if (minn == 2) break;
            }
        }
        if (altC != c && minn < 2)
        {
            altC = c;
            while (OTC[OV.p(altC)] == -4)
            {
                altC = OV.p(O[OV.p(altC)]);
                if (altC == c) break;
                bool hasUVna = TC[OV.n(altC)] >= 0 && MC[TC[OV.n(altC)]];
                bool hasUVpa = TC[OV.p(altC)] >= 0 && MC[TC[OV.p(altC)]];
                int curn = (hasUVna)+(hasUVpa);
                if (curn > minn)
                {
                    predC = altC; minn = curn;
                    hasUVn = hasUVna; hasUVp = hasUVpa;
                    if (minn == 2) break;
                }
            }
        }
    }

    if (!hasUVn && !hasUVp)                                         // start point, store as global coord
    {   // use a separate table for start and dummy uv coords => less unique symbols for entropy coding
        // this introduces a shift to be handled when decoding as no related oUVCoords.push_back
        sUVCoords.push_back(UV[TC[predC]]);
    }
    else if (!hasUVn)                                               // cannot predict, use delta
        oUVCoords.push_back(UV[TC[predC]] - UV[TC[OV.p(predC)]]);
    else if (!hasUVp)                                               // cannot predict, use delta
        oUVCoords.push_back(UV[TC[predC]] - UV[TC[OV.n(predC)]]);
    else
        predictUV(predC, TC, false); // use corner for prediction and skip dummy branch (false)

}

uint32_t ConvertSignedIntToSymbol(int32_t val) {
    if (val >= 0) {
        return (val) << 1;
    }
    val = -(val + 1);
    uint32_t ret = (val);
    ret <<= 1;
    ret |= 1;
    return ret;
}

bool EBBasicEncoder::save(std::string fileName) {

    Bitstream bitstream;

    auto t = now();
    if (!serialize(bitstream))
        return false;
    std::cout << "  Serialize time " << elapsed(t) << "sec" << std::endl;

    if (!bitstream.save(fileName)) {
        std::cerr << "Error: can't save compressed bitstream!\n";
        return false;
    }

    std::cout << "  Saved bitsream byte size = " << bitstream.size() << std::endl;

    return true;
}

// entropy encoding and serialization into a bitstream
bool EBBasicEncoder::serialize(Bitstream& bitstream) {


    // AC coded buffers
    std::vector<uint8_t> meshHandleIndexSecondShiftBuffer;
    std::vector<uint8_t> meshClersSymbolBuffer;
    std::vector<uint8_t> meshPositionIsDuplicateFlagBuffer;
    std::vector<uint8_t> meshPositionResidualsBuffer;
    std::vector<uint8_t> meshAttributeSeamBuffer;
    std::vector<uint8_t> meshAttributeResidualBuffer;
    std::vector<uint8_t> meshTexCoordStretchOrientationBuffer;

    if (_ovTable.uvcoords.empty()) // or sUVCoords or oUVCoords empy
        qt = -1; // override user setting if no uv in bitstream - used for decoding

    const bool hasSeparateUvIndex = _ovTable.OTC.size();

    size_t posLength = 0;
    // cfg.predCoder == EBConfig::ECName::DIRAC - single alternative .. TO BE ADDED
    {
        AdaptiveBitModel ctxCoeffRemPrefix[12];
        AdaptiveBitModel ctxCoeffRemSuffix[12];
        const auto bias = 3;
        const auto bctx = 2;
        AdaptiveBitModel ctxz;
        AdaptiveBitModel ctxs;
        AdaptiveBitModel ctxlb[2][bctx];
        EntropyEncoder     ac;
        const auto         maxAcBufLen = oVertices.size() * 3 * 4 + 1024;
        ac.setBuffer(maxAcBufLen, nullptr);
        ac.start();
        bool islow = true;
        //for (auto i = 0; i < oVertices.size(); ++i) {
        for (int32_t k = 0; k < 3; ++k) {
            for (auto i = 0; i < oVertices.size(); ++i) { // may be better to use x then y then z, or use 3 sets of rem pref and suff
                {
                    const int value = (int)(oVertices[i][k]);
                    int pval;
                    const bool isz = (value == 0);
                    ac.encode(isz, ctxz);
                    if (isz) {
                        islow = true; continue;
                    }
                    ac.encode(value < 0, ctxs);
                    pval = abs(value) - 1;
                    for (int k = 0; k < bias; ++k)
                    {
                        const auto ctx = std::min(k, bctx - 1);
                        if (pval <= k) {
                            ac.encode(0, ctxlb[islow][ctx]);
                            break;
                        }
                        ac.encode(1, ctxlb[islow][ctx]);
                    }
                    islow = pval < bias;
                    if (pval >= bias)
                        ac.encodeExpGolomb(pval - bias, gshift, ctxCoeffRemPrefix, ctxCoeffRemSuffix);
                }
            }
        }
        posLength = ac.stop();
        const auto byteCount = uint32_t(posLength);
        // assign position residuals buffer
        meshPositionResidualsBuffer.assign(ac.buffer(), ac.buffer() + byteCount);
    }
    std::cout << "  Positions bytes = " << posLength << ", bpv = " << 8.0 * (float)posLength / oVertices.size() << std::endl;

    size_t uvLength = 0;
    if (qt >= 0)
    {
        //cfg.predCoder == EBConfig::ECName::DIRAC - single alternative ..TO BE ADDED
        {
            AdaptiveBitModel ctxCoeffRemPrefix[12];
            AdaptiveBitModel ctxCoeffRemSuffix[12];
            EntropyEncoder     ac;
            const auto bias = 3;
            const auto bctx = 2;
            AdaptiveBitModel ctxz;
            AdaptiveBitModel ctxs;
            AdaptiveBitModel ctxlb[2][bctx];
            const auto         maxAcBufLen = oUVCoords.size() * 2 * 4 + 1024;
            ac.setBuffer(maxAcBufLen, nullptr);
            ac.start();
            bool islow = true;
            for (auto i = 0; i < oUVCoords.size(); ++i) { // no gain when using x then y or using separate ctx for those ?
                for (int32_t k = 0; k < 2; ++k) {
                    {
                        const int value = (int)(oUVCoords[i][k]);
                        int pval;
                        const bool isz = (value == 0);
                        ac.encode(isz, ctxz);
                        if (isz) {
                            islow = true; continue;
                        }
                        ac.encode(value < 0, ctxs);
                        pval = abs(value) - 1;
                        for (int k = 0; k < bias; ++k)
                        {
                            auto ctx = std::min(k, bctx - 1);
                            if (pval <= k) {
                                ac.encode(0, ctxlb[islow][ctx]);
                                break;
                            }
                            ac.encode(1, ctxlb[islow][ctx]);
                        }
                        islow = pval < bias;
                        if (pval >= bias)
                            ac.encodeExpGolomb(pval - bias, gshift, ctxCoeffRemPrefix, ctxCoeffRemSuffix);
                    }
                }
            }
            uvLength = ac.stop();
            const auto byteCount = uint32_t(uvLength);
            // assign attributes residuals buffer
            meshAttributeResidualBuffer.assign(ac.buffer(), ac.buffer()+ byteCount);
        }
        std::cout << "  UVCoods bytes = " << uvLength << ", bpv = " << 8.0 * (float)uvLength / oUVCoords.size() << std::endl;

        // extra data for mpara 
        if (cfg.uvPred == EBConfig::UvPred::STRETCH)
        {
            size_t uvOrientationLength = 0;
            { // single coder selection for uv orientation .. TO BE ADDED
                std::vector<uint8_t> orientBuffer;
                rans::RansBinaryEncoder orientEncoder;
                for (auto it = orientations.begin(); it != orientations.end(); ++it) {
                    orientEncoder.append(*it);
                }
                orientEncoder.encode(orientBuffer);
                uvOrientationLength = orientBuffer.size();
                const auto byteCount = uint32_t(uvOrientationLength);
                const auto nbOrientations = uint32_t(orientations.size());
                // assign uv strech orientation buffr
                meshTexCoordStretchOrientationBuffer = std::move(orientBuffer);
            }
            std::cout << "  UVCoods auxiliary orientation selection bytes = " << uvOrientationLength << ", bpv = " << 8.0 * (float)uvOrientationLength / oUVCoords.size() << std::endl;
        }

        // extra data for uv coord seams when separte uv indices - should be replicated for all non geo attributes
        if (hasSeparateUvIndex)
        {
            size_t uvSeamsLength = 0;
            { // single coder selection for seams.. TO BE ADDED
                // using dirac
                AdaptiveBitModel ctx;
                EntropyEncoder   ac;
                ac.setBuffer(seams.size() + 1024, nullptr);
                ac.start();
                for (auto i = 0; i < seams.size(); ++i) {
                    ac.encode(seams[i], ctx);
                }
                uvSeamsLength = ac.stop();
                const auto byteCount = uint32_t(uvSeamsLength);
                const auto nbSeams = uint32_t(seams.size());
                // assign attribute seams buffer
                meshAttributeSeamBuffer.assign(ac.buffer(), ac.buffer() + byteCount);
            }
            std::cout << "  UVCoords auxiliary seam bytes = " << uvSeamsLength << ", bpv = " << 8.0 * (float)uvSeamsLength / oUVCoords.size() << std::endl;
        }
    }
    else
    {
        std::cout << "  no UVCoods " << std::endl;
    }
    // CLERS table
    std::map<char, uint32_t> clersToInt = { {'C',0}, {'S',1}, {'L',3}, {'R',5},{'E',7} };
    std::vector<uint32_t> clersUI32;
    clersUI32.reserve(oClers.size());
    for (auto i = 0; i < oClers.size(); ++i) {
        const auto symb = clersToInt.find(oClers[i])->second;
        clersUI32.push_back(symb);
    }
    size_t topoLength = 0;
    // cfg.topoCoder == EBConfig::ECName::DIRAC - single alternative ..TO BE ADDED
    {
        // alternative using binary coding for clers symbols and three bit contexts
        // adding context selection based on previous symbol
        const auto       maxAcBufLent = oClers.size() + 1024;
        AdaptiveBitModel ctx_isNotC[32]; // not all 32 contexts used - code to be rewitten
        AdaptiveBitModel ctx_bit1[32];
        AdaptiveBitModel ctx_bit02[32];
        AdaptiveBitModel ctx_bit12[32];

        EntropyEncoder   ac;
        ac.setBuffer(maxAcBufLent, nullptr);
        ac.start();
        int pS = 0; // 'C'
        bool useExtended = (clersUI32.size() > 3000); // fixed threshold is this basic variant
        for (auto i = 0; i < clersUI32.size(); ++i) {
            const auto value = clersUI32[i];
            const bool isNotC = (value & 1);
            ac.encode(isNotC, ctx_isNotC[pS]);
            if (isNotC)
            {
                const bool bit1 = (value & 2);
                const bool bit2 = (value & 4);
                ac.encode(bit1, ctx_bit1[pS]);
                if (bit1)
                    ac.encode(bit2, ctx_bit12[pS]);
                else
                    ac.encode(bit2, ctx_bit02[pS]);
            }
            if (!useExtended)
                pS = value; // 5 contexts
            else
                pS = value + ((pS & 1) << 3) + ((pS & 4) << 2); // extended contexts (4*5)
        }
        topoLength = ac.stop();
        const auto byteCount = uint32_t(topoLength);
        std::cout << "  Topology bytes = " << topoLength << ", bpf = " << 8.0 * (float)topoLength / oClers.size() << std::endl;
        // assign topology symbols buffer
        meshClersSymbolBuffer.assign(ac.buffer(), ac.buffer() + byteCount);
    }


    { // handles
        size_t handleOrientationLength = 0;
        if (oHandles.size()) {
            std::vector<uint8_t> handleBuffer;
            rans::RansBinaryEncoder handleEncoder;
            for (auto i = 0; i < oHandles.size() / 2; ++i) {
                handleEncoder.append((oHandles[2 * i + 1] % 3) == 1);
            }
            handleEncoder.encode(handleBuffer);
            handleOrientationLength = handleBuffer.size();
            const auto byteCount = uint32_t(handleOrientationLength);
            std::cout << "  Handles auxiliary orientation selection bytes = " << handleOrientationLength << ", bph = " << 8.0 * (float)handleOrientationLength / oHandles.size() / 2 << std::endl;
            // assign handle shift buffer
            meshHandleIndexSecondShiftBuffer = std::move(handleBuffer);
        }
    }

    if (cfg.deduplicate)
    {
        if (oDuplicateSplitVertexIdx.size())
        {
            AdaptiveBitModel ctx;
            EntropyEncoder   ac;
            ac.setBuffer(isVertexDup.size() + 10, nullptr); // to resize better
            ac.start();
            for (auto i = 0; i < isVertexDup.size(); ++i) {
                ac.encode(isVertexDup[i], ctx);
            }
            auto dupLength = ac.stop();
            const auto byteCount = uint32_t(dupLength);
            std::cout << "  binary signallisation of duplicate indices = " << byteCount << std::endl;

            // assign duplicate position information buffer
            meshPositionIsDuplicateFlagBuffer.assign(ac.buffer(), ac.buffer() + byteCount);
        }
    }

    // Using bitstream writer to fill syntax tables 
    MeshCoding meshCoding; // Top level syntax element
    EbWriter   ebWriter;

    // Filling Mesh Coding Header
    auto& mch = meshCoding.getMeshCodingHeader();
    // Codec Variant
    mch.getMeshCodecType() = MeshCodecType::CODEC_TYPE_FORWARD;
    // Position Encoding Parameters
    auto& mpep = mch.getMeshPositionEncodingParameters();
    mpep.getMeshPositionBitDepthMinus1() = qp - 1;
    mpep.getMeshClersSymbolsEncodingMethod() = MeshClersSymbolsEncodingMethod::MESH_CLERS_AC_DEFAULT;
    mpep.getMeshPositionPredictionMethod() = MeshPositionPredictionMethod::MESH_POSITION_MPARA;
    mpep.getMeshPositionResidualsEncodingMethod() = MeshPositionResidualsEncodingMethod::MESH_POSITION_AC_DEFAULT;
    mpep.getMeshPositionDeduplicateMethod() = cfg.deduplicate ? MeshPositionDeduplicateMethod::MESH_POSITION_DEDUP_DEFAULT : MeshPositionDeduplicateMethod::MESH_POSITION_DEDUP_NONE;
    // Position Dequantization
    mch.getMeshPositionDequantizeFlag() = (qp >= 7 && !cfg.intAttr); // this condition to be modified through a more flexible ebEncode interface
    if (mch.getMeshPositionDequantizeFlag())
    {
        auto& mpdp = mch.getMeshPositionDequantizeParameters();
        mpdp.getMeshPositionMin(0) = minPos.x;
        mpdp.getMeshPositionMin(1) = minPos.y;
        mpdp.getMeshPositionMin(2) = minPos.z;
        mpdp.getMeshPositionMax(0) = maxPos.x;
        mpdp.getMeshPositionMax(1) = maxPos.y;
        mpdp.getMeshPositionMax(2) = maxPos.z;
    }
    // Attributes
    uint8_t attributeCount = (qt >= 0) ? 1 : 0; // only tex coords
    mch.allocateAttributes(attributeCount);
    mch.getMeshAttributeCount() = attributeCount;
    for (auto i = 0; i < mch.getMeshAttributeCount(); i++)
    {
        mch.getMeshAttributeType()[i] = MeshAttributeType::MESH_ATTR_TEXCOORD; // TO BE MODIFIED - simplification as tex coord only considered now
        auto& maep = mch.getMeshAttributesEncodingParameters(i);

        maep.getMeshAttributeBitDepthMinus1() = qt - 1; // caution single attribute here
        maep.getMeshAttributePerFaceFlag() = false;  // caution single attribute here
        if (!maep.getMeshAttributePerFaceFlag())
        {
            maep.getMeshAttributeSeparateIndexFlag() = hasSeparateUvIndex; // caution single attribute here
            if (!maep.getMeshAttributeSeparateIndexFlag())
                maep.getMeshAttributeReferenceIndexPlus1() = 0; // we always use position as the principal attibute for now 
        }
        // !! something to revise to ba able to better write the following
        maep.getMeshAttributePredictionMethod() = (uint8_t)MeshAttributePredictionMethod_TEXCOORD::MESH_TEXCOORD_STRETCH;
        maep.getMeshAttributeResidualsEncodingMethod() = (uint8_t)MeshAttributeResidualsEncodingMethod_TEXCOORD::MESH_TEXCOORD_AC_DEFAULT;

        mch.getMeshAttributeDequantizeFlag()[i] = (qt >= 7 && !_ovTable.uvcoords.empty() && !cfg.intAttr); // this condition to be modified through a more flexible ebEncode interface
        if (mch.getMeshAttributeDequantizeFlag()[i])
        {
            auto& madp = mch.getMeshAttributesDequantizeParameters()[i];
            /*
            for (auto j = 0; j < mch.getNumComponents(i); j++)
            {
                madp.getMeshAttributeMin(j) = mmm;
                madp.getMeshAttributeMax(j) = MMM;
            }
            */
            madp.getMeshAttributeMin(0) = minUv.x;
            madp.getMeshAttributeMin(1) = minUv.y;
            madp.getMeshAttributeMax(0) = maxUv.x;
            madp.getMeshAttributeMax(1) = maxUv.y;
        }
    }


    // Payload

    // Filling Mesh Position Coding Payload
    auto& mpcp = meshCoding.getMeshPositionCodingPayload();

    mpcp.getMeshVertexCount() = oVertices.size(); //!! check alignment with semantics
    mpcp.getMeshClersCount() = oClers.size();
    mpcp.getMeshCcCount() = ccCount;
    mpcp.getMeshVirtualVertexCount() = oDummies.size();
    mpcp.getMeshVirtualIndexDelta().resize(mpcp.getMeshVirtualVertexCount());
    for (auto i = 0; i < mpcp.getMeshVirtualVertexCount(); ++i) {
        mpcp.getMeshVirtualIndexDelta()[i] = (i > 0) ? oDummies[i] - oDummies[i - 1] : oDummies[0];
    }
    mpcp.getMeshCcWithHandlesCount() = oHandleSizes.size() >> 1;
    mpcp.getMeshHandlesCcOffset().resize(mpcp.getMeshCcWithHandlesCount());
    mpcp.getMeshHandlesCount().resize(mpcp.getMeshCcWithHandlesCount());
    auto NumHandles = 0;
    for (auto i = 0; i < mpcp.getMeshCcWithHandlesCount(); i++) {
        mpcp.getMeshHandlesCcOffset()[i] = oHandleSizes[2 * i + 0] - (i ? oHandleSizes[2 * i - 2] : 0);
        mpcp.getMeshHandlesCount()[i] = (oHandleSizes[2 * i + 1] - (i ? oHandleSizes[2 * i - 1] : 0))>>1;
        NumHandles += mpcp.getMeshHandlesCount()[i];
    }
    mpcp.getMeshHandleIndexFirstDelta().resize(NumHandles);
    mpcp.getMeshHandleIndexSecondDelta().resize(NumHandles);
    for (auto i = 0; i < NumHandles; i++) {
        mpcp.getMeshHandleIndexFirstDelta()[i] = (int)oHandles[2 * i + 0] / 3 - (i ? oHandles[2 * i - 2] / 3 : 0);
        mpcp.getMeshHandleIndexSecondDelta()[i] = (int)oHandles[2 * i + 1] / 3 - (i ? oHandles[2 * i - 1] / 3 : 0);
    }
    mpcp.getMeshCodedHandleIndexSecondShiftSize() = meshHandleIndexSecondShiftBuffer.size();
    mpcp.getMeshHandleIndexSecondShift() = meshHandleIndexSecondShiftBuffer;
    mpcp.getMeshCodedClersSymbolsSize() = meshClersSymbolBuffer.size();
    mpcp.getMeshClersSymbol() = meshClersSymbolBuffer;

    auto NumPositionStart = 0;
    auto& mpdi = mpcp.getMeshPositionDeduplicateInformation();

    if (mpep.getMeshPositionDeduplicateMethod() == MeshPositionDeduplicateMethod::MESH_POSITION_DEDUP_DEFAULT) {
        mpdi.getMeshPositionDeduplicateCount() = oDuplicateSplitVertexIdx.size();
        unsigned int NumSplitVertex = 0;
        if (mpdi.getMeshPositionDeduplicateCount() > 0)
        {
            mpdi.getMeshPositionDeduplicateIdx().resize(mpdi.getMeshPositionDeduplicateCount());
            for (auto i = 0; i < mpdi.getMeshPositionDeduplicateCount(); ++i) {
                mpdi.getMeshPositionDeduplicateIdx()[i] = oDuplicateSplitVertexIdx[i];
                NumSplitVertex = std::max(NumSplitVertex, mpdi.getMeshPositionDeduplicateIdx()[i] + 1);
            }
            mpdi.getMeshPositionDeduplicateStartPositions() = sVertices.size(); // used, this is sVertices when different from ccount
            NumPositionStart = mpdi.getMeshPositionDeduplicateStartPositions();
            auto NumAddedDuplicatedVertex = mpdi.getMeshPositionDeduplicateCount() - NumSplitVertex;
            auto NumVertices = isVertexDup.size();
            auto NumVerticesCheck = mpcp.getMeshVertexCount()
                + mpdi.getMeshPositionDeduplicateStartPositions() // should not be sVertices ?
                + mpcp.getMeshVirtualVertexCount()
                + NumAddedDuplicatedVertex;
            // = iVertices.size() + sVertices.size() + iDummyVertices.size()+ NumAddedDuplicatedVertex ;
            // where NumAddedDuplicatedVertex  = mpdi.getMeshPositionDeduplicateCount() - NumSplitVertex
            // !! to check if we keep mesh_cc count as NumPositionStart always (coding start values for duplicactes on 1st corner of component)
            // or if we use this vu(v) to store a delta that reduces
            // !!! RECHECK IF HAVE CASES WITH DUPLICATES ON 1st CORNER OF CC AND HOW IT WORKS NOW
            mpdi.getMeshPositionIsDuplicateSize() = meshPositionIsDuplicateFlagBuffer.size();
            mpdi.getMeshPositionIsDuplicateFlag() = meshPositionIsDuplicateFlagBuffer;

        }
        else
            NumPositionStart = mpcp.getMeshCcCount();
    }
    else
        NumPositionStart = mpcp.getMeshCcCount();

    mpcp.getMeshPositionStart().resize(NumPositionStart);
    for (auto i = 0; i < NumPositionStart; i++) {
        mpcp.getMeshPositionStart()[i].resize(3);
        for (auto j = 0; j < 3; j++) {
            mpcp.getMeshPositionStart()[i][j] = sVertices[i][j];
        }
    }

    // !! is it true if we skip predictions and if vertex count not set to iVerts but total input verts ? 
    auto NumPredictedPositions = mpcp.getMeshVertexCount() - mpcp.getMeshCcCount();

    mpcp.getMeshCodedPositionResidualsSize() = meshPositionResidualsBuffer.size();
    mpcp.getMeshPositionResidual() = meshPositionResidualsBuffer;


    // Filling Mesh Attribute Coding Payload
    auto& macp = meshCoding.getMeshAttributeCodingPayload();
    macp.allocate(mch.getMeshAttributeCount());
    for (auto i = 0; i < mch.getMeshAttributeCount(); i++) {
        //macp.get
        auto mesh_attribute_per_face_flag = mch.getMeshAttributesEncodingParameters(i).getMeshAttributePerFaceFlag();
        auto mesh_attribute_separate_index_flag = mch.getMeshAttributesEncodingParameters(i).getMeshAttributeSeparateIndexFlag();
        if (!mesh_attribute_per_face_flag && mesh_attribute_separate_index_flag) {
            macp.getMeshAttributeSeamsCount()[i] = seams.size();
            macp.getMeshCodedAttributeSeamsSize()[i] = meshAttributeSeamBuffer.size();
            macp.getMeshAttributeSeam()[i] = meshAttributeSeamBuffer;
        }

        // TODO !! REWRITE SYNTAX
        if (mesh_attribute_separate_index_flag)
            macp.getMeshAttributeStartCount()[i] = sUVCoords.size();

        // TO BE EXTENDED FOR CASES WITH MULTIPLE ATTRIBUTES
        macp.getMeshAttributeStart()[i].resize(sUVCoords.size());
        for (auto j = 0; j < sUVCoords.size(); j++) {
            macp.getMeshAttributeStart()[i][j].resize(3);
            for (auto k = 0; k < mch.getNumComponents(i); k++) {
                macp.getMeshAttributeStart()[i][j][k] = sUVCoords[j][k];
            }
        }

        macp.getMeshAttributeResidualsCount()[i] = oUVCoords.size();
        macp.getMeshCodedAttributeResidualsSize()[i] = meshAttributeResidualBuffer.size();
        macp.getMeshAttributeResidual()[i] = meshAttributeResidualBuffer;

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

                mtced.getMeshTexCoordStretchOrientationsCount() = orientations.size();
                mtced.getMeshCodedTexCoordStretchOrientationsSize() = meshTexCoordStretchOrientationBuffer.size();
                mtced.getMeshTexCoordStretchOrientation() = meshTexCoordStretchOrientationBuffer;
            }
        }

    }

    //serialize to bitstream
    ebWriter.write(bitstream, meshCoding);

    return true;
}

