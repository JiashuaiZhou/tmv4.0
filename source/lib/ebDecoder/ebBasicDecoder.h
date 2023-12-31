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

#ifndef _EB_BASIC_DECODER_H_
#define _EB_BASIC_DECODER_H_

#include <fstream>
#include <stack>

// internal headers
#include "ebModel.h"
#include "ebCTMesh.h"
#include "ebDecoderBase.h"

namespace eb {

    class EBBasicDecoder : public EBDecoderBase {

    public: 

        EBBasicDecoder() {};

        virtual bool unserialize(MeshCoding& meshCoding);
        virtual bool decode(Model& output);

    private:

        //
        void initClersTable();

        // EB loaded data
        std::vector<char> iClers;
        std::vector<int>  iHandles;         // handle corners array
        std::vector<int>  iHandleSizes;     // handle corners array, handles count for each CC
        std::vector<glm::vec3> iVertices;   // contains values or deltas
        std::vector<glm::vec2> iUVCoords;   // contains values or deltas
        std::vector<glm::vec3> sVertices;   // contains positions of start or dummy positions
        std::vector<glm::vec2> sUVCoords;   // contains positions or start or dummy uv coords
        std::vector<int> iDummyVertices;    //
        std::vector<bool> iOrientations;    //
        std::vector<bool> iSeams;           //

        // used for vertex deduplication
        // from which unique split vertex is the duplicated 
        // point originating added duplicateCornerIdx
        std::vector<int> iDuplicateSplitVertexIdx;  
        // associates a unique duplicate index with a vertex index on 
        // first occurence of the duplicate - used to replace indices - could be vector
        std::map<int, int> processedDupIdx; 
        // per connected component list of corners for which indexing 
        // has to be fixed and the corresponding fixed vertex index
        std::vector<std::pair<int, int>> perCCduplicates; 
        std::vector<bool> isVertexDup;       // per vertex duplicated status
        int Vcur = 0;  					             // current decoded vertex count
        int Dcur = 0;  					             // current duplicate index
        int numSplitVert = 0;                // number of unique split vertex which have bee duplicated
        int numAddedDupVert = 0;             // number of added vertex after deduplication

        std::vector<int> processedCorners;	 // processed corners

        int numOfTriangles;                  // deduced from clers table
        int numOfVertices;                   // deduced from vertices table
        int numOfUVCoords;                   // deduced from uv coords table

        // Decompression variables
        int T = 0;					    // symbols count
        int N = 0;					    // vertices count
        int NTC = 0;			      // uvcoords count
        int A = 0;					    // handles count
        std::vector<int> _M;	  // Vertex marking array
        std::vector<int> _U;	  // Triangles marking array
        std::vector<int> _D;	  // Dummy marking array
        std::vector<int> _MC;	  // uv coords marking array TO RENAME
        int startCorner;        // first corner index of the current connected component
        int startVertex;        // first vertex index of the current connected component
        int startUVCoord;       // first uvcoord index of the current connected component
        int startSymbol;        // first symbol index of the current connected component
        int startTriangle;      // first triangle index of the current connected component
        int startHandle;        // first handle index of the current connected component
        int ccCount;            // the current connected component
        int orientationIdx = 0; // the current orientation index
        int seamsIdx = 0;       // the current seams index
        int posSvalueIdx = 0;   // the next start position index
        int uvSvalueIdx = 0;    // the next start uv index
        int posSkipNum = 0;     // the current number of skipped dummies
        int posSkipDup = 0;     // the current number of skipped dummies
        int uvSkipNum = 0;      // the current number of skipped dummies
        int zipnum = 0;

        // Decompression methods
        void initDecompression();
        void startDecompression();
        void DecompressConnectivity(int c);
        bool CheckHandle(int c);
        void Zip(int c);

        //
        void DecompressVertices(int c);
        void decodeMainIndexAttributes(int c, int v);
        void decodeSeparateIndexAttributes(void);
        void posDecodeWithPrediction(int c, int v);
        // use min stretch to predict uv
        void predictUV(const int c, glm::vec2& predUV, glm::dvec2& firstpredUV,
            const int* indices, const int v,
            bool predWithDummies, bool first, bool prevIsDummy = false);
        // decode Uv using main index
        void uvDecodeWithPrediction(int c, int v);
        // decode Uv using separate index
        void uvSepDecodeWithPrediction(int c, int uvIdx);

        // recreate holes, optionaly finalize de-duplicates, and build final mesh to output
        void postProcess(Model& decoded);

    };
};  // namespace mm

#endif
