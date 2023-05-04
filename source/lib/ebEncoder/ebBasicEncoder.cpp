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

    std::ofstream out(fileName, std::ios::binary);
    if (!out)
        return false;

    eb::Bitstream bs;

    auto t = now();
    if (!serialize(bs))
        return false;
    std::cout << "  AC coding time " << elapsed(t) << "sec" << std::endl;

    t = now();
    bs.save(out);
    out.close();
    std::cout << "  File saving time " << elapsed(t) << "sec" << std::endl;

    std::cout << "  Saved bitsream byte size = " << bs.size() << std::endl;

    return true;
}

// entroy encoding and serialization into a bitstream
bool EBBasicEncoder::serialize(Bitstream& bs) {

    if (_ovTable.uvcoords.empty()) // or sUVCoords or oUVCoords empy
        qt = -1; // override user setting if no uv in bitstream - used for decoding

    bs.write('b'); // basic encoder
    bs.write(uint8_t(cfg.posPred));
    bs.write(uint8_t(cfg.uvPred));
    bs.write(uint8_t(cfg.predCoder));
    bs.write(uint8_t(cfg.topoCoder));

    const bool hasSeparateUvIndex = _ovTable.OTC.size();

    const bool bopt0 = (cfg.optionFlags & (1 << 0));
    const bool bopt1 = (cfg.optionFlags & (1 << 1));
    const bool bopt2 = (cfg.optionFlags & (1 << 2));
    const bool bopt3 = (cfg.optionFlags & (1 << 3));
    const bool bopt4 = (cfg.optionFlags & (1 << 4));
    uint8_t flags = (uint8_t(cfg.intAttr) << 0)
        | (uint8_t(hasSeparateUvIndex) << 1)
        | (uint8_t(cfg.deduplicate) << 2)
        | (uint8_t(bopt0) << 3)
        | (uint8_t(bopt1) << 4)
        | (uint8_t(bopt2) << 5)
        | (uint8_t(bopt3) << 6)
        | (uint8_t(bopt4) << 7);

    bs.write(flags);
    bs.write(uint8_t(qp + 1));
    bs.write(uint8_t(qt + 1));
    bs.write(uint8_t(qn + 1));
    bs.write(uint8_t(qc + 1));
    bs.writeVarUint(oVertices.size());
    bs.writeVarUint(oUVCoords.size());

    if (qp >= 7 && !cfg.intAttr) {
        bs.writeRaw(minPos.x);
        bs.writeRaw(minPos.y);
        bs.writeRaw(minPos.z);
        bs.writeRaw(maxPos.x);
        bs.writeRaw(maxPos.y);
        bs.writeRaw(maxPos.z);
    }
    if (qt >= 7 && !_ovTable.uvcoords.empty() && !cfg.intAttr) {
        bs.writeRaw(minUv.x);
        bs.writeRaw(minUv.y);
        bs.writeRaw(maxUv.x);
        bs.writeRaw(maxUv.y);
    }
    //
    bs.writeVarUint(numOfTriangles);
    bs.writeVarUint(oClers.size());
    bs.writeVarUint(ccCount);
    bs.writeVarUint(oHandleSizes.size());
    bs.writeVarUint(oDummies.size());

    // Payload

    // absolute start positions
    if (cfg.posPred == EBConfig::PosPred::MPARA)
    {
        bs.writeVarUint(sVertices.size()); // not always redundant with cccount (when deduplication)
        if (sVertices.size())
        {
            const auto aposStart = bs.buffer.size();
            // CAUTION requires qp to be correctly set
            std::vector<uint8_t> tempBS;
            BitstreamRef test(tempBS);
            for (auto i = 0; i < sVertices.size(); ++i)
                for (auto k = 0; k < 3; ++k)
                    test.write((int)sVertices[i][k], qp);

            while (!test.byteAligned()) test.write(0, 1);
            const auto aposLength = test.getPosition().bytes_;

            const auto offset = bs.size();
            bs.resize(offset + aposLength);
            std::copy(test.buffer(), test.buffer() + aposLength, bs.buffer.begin() + offset);
            std::cout << "  Absolute Positions bytes = " << aposLength << ", bpv = " << 8.0 * (float)aposLength / sVertices.size() << std::endl;
        }
    }
    // absolute start uv coords
    if ((qt >= 0) && cfg.uvPred == EBConfig::UvPred::STRETCH)
    {
        if (cfg.posPred != EBConfig::PosPred::MPARA || hasSeparateUvIndex)
            bs.writeVarUint(sUVCoords.size());
        if (sUVCoords.size())
        {
            const auto auvStart = bs.buffer.size();
            // CAUTION requires qt to be correctly set
            std::vector<uint8_t> tempBS;
            BitstreamRef test(tempBS); // TODO replace with extended bitstream reader/writer
            for (auto i = 0; i < sUVCoords.size(); ++i)
                for (auto k = 0; k < 2; ++k)
                    test.write((int)sUVCoords[i][k], qt);

            while (!test.byteAligned()) test.write(0, 1);
            const auto auvLength = test.getPosition().bytes_;

            const auto offset = bs.size();
            bs.resize(offset + auvLength);
            std::copy(test.buffer(), test.buffer() + auvLength, bs.buffer.begin() + offset);
            std::cout << "  Absolute UVCoords bytes = " << auvLength << ", bpv = " << 8.0 * (float)auvLength / sUVCoords.size() << std::endl;
        }
    }

    size_t posLength = 0;
    if (cfg.predCoder == EBConfig::ECName::DIRAC) {
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
                const bool basicStyle = bopt0;
                if (basicStyle) {
                    const int value = (int)(oVertices[i][k]);
                    ac.encodeExpGolomb(ConvertSignedIntToSymbol(value), gshift, ctxCoeffRemPrefix, ctxCoeffRemSuffix);
                }
                else {
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
        bs.writeVarUint(byteCount);
        bs.write((uint8_t)gshift);
        const auto offset = bs.size();
        bs.resize(offset + byteCount);
        std::copy(ac.buffer(), ac.buffer() + byteCount, bs.buffer.begin() + offset);
    }
    else if (cfg.predCoder == EBConfig::ECName::RANS) {
        std::vector<uint32_t> vertsUI32;
        vertsUI32.reserve(3 * oVertices.size());
        for (auto i = 0; i < oVertices.size(); ++i) {
            for (int32_t k = 0; k < 3; ++k) {
                const int value = (int)(oVertices[i][k]);
                vertsUI32.push_back(ConvertSignedIntToSymbol(value));
            }
        }
        rans::RansSymbolEncoder ransVertEncoder;
        std::vector<uint8_t> ransVertEncodedBuffer;
        ransVertEncoder.encode(&vertsUI32[0], (int)oVertices.size() * 3, ransVertEncodedBuffer, 2);
        posLength = ransVertEncodedBuffer.size();
        const auto byteCount = uint32_t(posLength);
        bs.writeVarUint(byteCount);
        const auto offset = bs.size();
        bs.resize(offset + byteCount);
        std::copy(ransVertEncodedBuffer.begin(), ransVertEncodedBuffer.begin() + byteCount, bs.buffer.begin() + offset);
    }
    std::cout << "  Positions bytes = " << posLength << ", bpv = " << 8.0 * (float)posLength / oVertices.size() << std::endl;

    size_t uvLength = 0;
    if (qt >= 0)
    {
        if (cfg.predCoder == EBConfig::ECName::DIRAC) {
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
                    const bool basicStyle = bopt0;
                    if (basicStyle) {
                        const int value = (int)(oUVCoords[i][k]);
                        ac.encodeExpGolomb(ConvertSignedIntToSymbol(value), gshift, ctxCoeffRemPrefix, ctxCoeffRemSuffix);
                    }
                    else {
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
            bs.writeVarUint(byteCount);
            bs.write((uint8_t)gshift);
            const auto offset = bs.size();
            bs.resize(offset + byteCount);
            std::copy(ac.buffer(), ac.buffer() + byteCount, bs.buffer.begin() + offset);
        }
        else if (cfg.predCoder == EBConfig::ECName::RANS) {
            std::vector<uint32_t> uvUI32;
            uvUI32.reserve(2 * oUVCoords.size());
            for (auto i = 0; i < oUVCoords.size(); ++i) {
                for (int32_t k = 0; k < 2; ++k) {
                    const int value = (int)(oUVCoords[i][k]);
                    uvUI32.push_back(ConvertSignedIntToSymbol(value));
                }
            }

            rans::RansSymbolEncoder ransUVEncoder;
            std::vector<uint8_t> ransUVEncodedBuffer;
            ransUVEncoder.encode(&uvUI32[0], oUVCoords.size() * 2, ransUVEncodedBuffer, 0);
            uvLength = ransUVEncodedBuffer.size();
            const auto byteCount = uint32_t(uvLength);
            bs.writeVarUint(byteCount);
            const auto offset = bs.size();
            bs.resize(offset + byteCount);
            std::copy(ransUVEncodedBuffer.begin(), ransUVEncodedBuffer.begin() + byteCount, bs.buffer.begin() + offset);
        }
        std::cout << "  UVCoods bytes = " << uvLength << ", bpv = " << 8.0 * (float)uvLength / oUVCoords.size() << std::endl;

        // extra data for mpara 
        if (cfg.uvPred == EBConfig::UvPred::STRETCH)
        {
            size_t uvOrientationLength = 0;
            { // no coder selection yet for uv orientation .. TO BE ADDED
                std::vector<uint8_t> orientBuffer;
                rans::RansBinaryEncoder orientEncoder;
                for (auto it = orientations.begin(); it != orientations.end(); ++it) {
                    orientEncoder.append(*it);
                }
                orientEncoder.encode(orientBuffer);
                uvOrientationLength = orientBuffer.size();
                const auto byteCount = uint32_t(uvOrientationLength);
                const auto nbOrientations = uint32_t(orientations.size());
                bs.writeVarUint(nbOrientations);
                if (nbOrientations)
                {
                    bs.writeVarUint(byteCount);
                    const auto offset = bs.size();
                    bs.resize(offset + byteCount);
                    std::copy(orientBuffer.begin(), orientBuffer.begin() + byteCount, bs.buffer.begin() + offset);
                }
            }
            std::cout << "  UVCoods auxiliary orientation selection bytes = " << uvOrientationLength << ", bpv = " << 8.0 * (float)uvOrientationLength / oUVCoords.size() << std::endl;
        }

        // extra data for uv coord seams when separte uv indices - should be replicated for all non geo attributes
        if (hasSeparateUvIndex)
        {
            size_t uvSeamsLength = 0;
            { // no coder selection yet for seams
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
                bs.writeVarUint(nbSeams);
                if (nbSeams)
                {
                    bs.writeVarUint(byteCount);
                    const auto offset = bs.size();
                    bs.resize(offset + byteCount);
                    std::copy(ac.buffer(), ac.buffer() + byteCount, bs.buffer.begin() + offset);
                }
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
    if (cfg.topoCoder == EBConfig::ECName::DIRAC) {
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
        bs.writeVarUint(byteCount);
        const auto offset = bs.size();
        bs.resize(offset + byteCount);
        std::copy(ac.buffer(), ac.buffer() + byteCount, bs.buffer.begin() + offset);
    }
    else if (cfg.topoCoder == EBConfig::ECName::RANS) {
        rans::RansSymbolEncoder ransClersEncoder;
        std::vector<uint8_t> buffer;
        ransClersEncoder.encode(&clersUI32[0], clersUI32.size(), buffer, 0);
        topoLength = buffer.size();
        const auto byteCount = uint32_t(topoLength);
        bs.writeVarUint(byteCount);
        const auto offset = bs.size();
        bs.resize(offset + byteCount);
        std::copy(buffer.data(), buffer.data() + byteCount, bs.buffer.begin() + offset);
    }

    std::cout << "  Topology bytes = " << topoLength << ", bpf = " << 8.0 * (float)topoLength / oClers.size() << std::endl;

    { // handles
        const auto handleStart = bs.buffer.size();
        size_t handleOrientationLength = 0;
        if (oHandles.size()) {
            for (auto i = 0; i < oHandleSizes.size() / 2; ++i) {
                bs.writeVarUint(oHandleSizes[2 * i + 0] - (i ? oHandleSizes[2 * i - 2] : 0)); // cc offset
                bs.writeVarUint(oHandleSizes[2 * i + 1] - (i ? oHandleSizes[2 * i - 1] : 0)); // handle index offset
            }
            std::vector<uint8_t> handleBuffer;
            rans::RansBinaryEncoder handleEncoder;
            for (auto i = 0; i < oHandles.size() / 2; ++i) {
                handleEncoder.append((oHandles[2 * i + 1] % 3) == 1);
            }
            handleEncoder.encode(handleBuffer);
            handleOrientationLength = handleBuffer.size();
            const auto byteCount = uint32_t(handleOrientationLength);
            bs.writeVarUint(byteCount);
            const auto offset = bs.size();
            bs.resize(offset + byteCount);
            std::copy(handleBuffer.begin(), handleBuffer.begin() + byteCount, bs.buffer.begin() + offset);
            std::cout << "  Handles auxiliary orientation selection bytes = " << handleOrientationLength << ", bph = " << 8.0 * (float)handleOrientationLength / oHandles.size() / 2 << std::endl;
        }

        for (auto i = 0; i < oHandles.size() / 2; ++i)
        {
            int value;
            value = (int)oHandles[2 * i + 0] / 3 - (i ? oHandles[2 * i - 2] / 3 : 0);
            bs.writeVarUint(ConvertSignedIntToSymbol(value));
            value = (int)oHandles[2 * i + 1] / 3 - (i ? oHandles[2 * i - 1] / 3 : 0);
            bs.writeVarUint(ConvertSignedIntToSymbol(value));
        }
        const auto handleLength = bs.buffer.size() - handleStart;
        std::cout << "  Handle bytes = " << handleLength << ", bph = " << 8.0 * (float)handleLength / oHandles.size() << std::endl;
    }
    for (auto i = 0; i < oDummies.size(); ++i) {
        ;
        const auto value = (i > 0) ? oDummies[i] - oDummies[i - 1] : oDummies[0];
        bs.writeVarUint(value);
    }
    if (cfg.deduplicate)
    {
        const auto dupStart = bs.buffer.size();
        bs.writeVarUint(oDuplicateSplitVertexIdx.size());
        if (oDuplicateSplitVertexIdx.size())
        {
            for (auto i = 0; i < oDuplicateSplitVertexIdx.size(); ++i)
                bs.writeVarUint(oDuplicateSplitVertexIdx[i]);

            AdaptiveBitModel ctx;
            EntropyEncoder   ac;
            ac.setBuffer(isVertexDup.size() + 10, nullptr); // to resize better
            ac.start();
            for (auto i = 0; i < isVertexDup.size(); ++i) {
                ac.encode(isVertexDup[i], ctx);
            }
            auto dupLength = ac.stop();
            const auto byteCount = uint32_t(dupLength);
            bs.writeVarUint(byteCount);
            const auto offset = bs.size();
            bs.resize(offset + byteCount);
            std::copy(ac.buffer(), ac.buffer() + byteCount, bs.buffer.begin() + offset);

            std::cout << "  binary signallisation of duplicate indices = " << byteCount << std::endl;

            const auto totalDupLength = bs.buffer.size() - dupStart;
            std::cout << "  Duplicates bytes = " << totalDupLength << std::endl;
        }
    }
    return true;
}

