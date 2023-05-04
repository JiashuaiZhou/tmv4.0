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

#ifndef _EB_BASIC_ENCODER_H_
#define _EB_BASIC_ENCODER_H_

#include <fstream>
#include <stack>

// internal headers
#include "ebModel.h"
#include "ebCTMesh.h"
#include "ebEncoder.h"
#include "ebBitstream.h"

namespace eb {

    class EBBasicEncoder: public EBEncoder {

    public: // methods

        EBBasicEncoder() {};

        virtual void encode(const Model& input);
        virtual bool serialize(Bitstream& bs);
        virtual bool save(std::string fileName);

    private:

        // Internal options

        // preprocessing data
        std::vector<int> dummyVertices;

        // Variables for storing Input OVTable and geometry
        
        std::vector<glm::vec3> G_est;		  // Input Geometry estimates

        int numOfVertices;
        int numOfTriangles;
        int numOfUVCoords;

        // Compression variables
        int T;    					               // triangles count
        int Vcur;  					               // current coded vertex count
        std::vector<int> M;	               // Vertex marking array
        std::vector<int> U;	               // Triangles marking array
        std::vector<int> MC;               // Corners marking array
        int ccCount;                       // index of current CC, number of CCs at the end

        // EB output data storage before Arithmetic Coding
        std::vector<char> oClers;
        std::vector<int> oHandles;          // handle corners array
        std::vector<int> oHandleSizes;      // handle corners non null array sizes
        std::vector<glm::vec3> oVertices;   // contains positions or deltas
        std::vector<glm::vec2> oUVCoords;   // contains positions or deltas
        std::vector<glm::vec3> sVertices;   // contains positions of start or dummy positions
        std::vector<glm::vec2> sUVCoords;   // contains positions or start or dummy uv coords

        std::vector<int> oDummies;          // indices of artificially inserted vertices
        std::vector<bool> orientations;
        std::vector<bool> seams;            // for separate uv indices table generation
        std::vector<int>  processedCorners;	// processed corners

        std::vector<int> oDuplicateSplitVertexIdx;  // from which unique split vertex is the duplicated point originating added
        std::vector<bool> oDuplicatesOnStart;       // are duplicates inserted as the first 2 vertices of the first triangle of a connected component
        std::set<int> processedDupIdx;
        std::vector<bool> isVertexDup;
        int posSkipDup;                             // number of duplicates for which position is not predicted - nb added vertices in the end

        // start compression for corner c
        void startCompression(int c);
        // select next corner to start encoding next CC
        int findRestartCorner();
        // compress the CC
        void CompressRec(int c);
        void CheckHandle(const int c);
        // encodes attributes of corner c, only those using main index table 
        void encodeMainIndexAttributes(const int c, const int v);
        // encodes all the attributes that uses a separate index table, of all the corners
        void encodeSeparateIndexAttributes(void);

        void posEncodeWithPrediction(const int c, const int v);
        // use min stretch to predict uv
        void predictUV(const int c, const std::vector<int>& indices, 
            bool predWithDummies, bool prevIsDummy=false);
        // encode Uv using main index
        void uvEncodeWithPrediction(const int c, const int v);
        // encode Uv using separate index
        void uvSepEncodeWithPrediction(const int c);
        
        // helper function
        inline bool isCornerVertexDummy(const int corner);
        
    };

};  // namespace mm

#endif
