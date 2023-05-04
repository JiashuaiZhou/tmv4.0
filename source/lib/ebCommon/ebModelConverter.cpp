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
#include <set>
#include <map>
#include <list>
#include <stack>
#include <vector>
#include <algorithm>
#include <functional>
#include <iostream>
#include <time.h>
// mathematics
#include <glm/vec3.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
// internal
#include "ebModelConverter.h"

using namespace eb;

// output shall be pristine
// TODO - reserve output vectors memory fo better push_back performance
bool ModelConverter::convertCTMeshToModel(const CTMesh& input, Model& output)
{
    auto numpts = input.positions.size();

    if (numpts < 3) // nothing to do, no mesh
        return false;

    const bool hasNormal = input.normals.size() == numpts;
    const bool hasColor = input.colors.size() == numpts;
    const bool hasUvCoord = input.uvcoords.size() == numpts;

    output.vertices.resize(3 * numpts);
    if (hasUvCoord)
        output.uvcoords.resize(2 * numpts);
    if (hasColor)
        output.colors.resize(3 * numpts);
    if (hasNormal)
        output.normals.resize(3 * numpts);

    const auto& G = input.positions;
    for (int i = 0; i < G.size(); i++) {
        const auto& p = G[i];
        output.storePosition(i, G[i]);
        if (hasUvCoord)
            output.storeUv(i, input.uvcoords[i]);
        if (hasColor)
            output.storeColor(i, input.colors[i]);
        if (hasNormal)
            output.storeNormal(i, input.normals[i]);
    }

    const auto& V = input.V;
    for (int i = 0; i < V.size() / 3; i++) {
        output.triangles.push_back(V[3 * i]);
        output.triangles.push_back(V[3 * i + 1]);
        output.triangles.push_back(V[3 * i + 2]);
    }

    return true;
}

// output shall be pristine
bool ModelConverter::convertModelToCTMesh(const Model& input, CTMesh& output, bool keepTrackOfVertexDeduplications) {

    int  i, j, c;

    const auto nverts = input.getPositionCount();
    const auto nuv    = input.getUvCount();
    const auto ntrigs = input.getTriangleCount();

    bool hasUv = false;
    bool hasSeparateUvIndex = false;
    if (input.getUvCount() == input.getPositionCount()) {
        if (input.trianglesuv.size() == 0) {
            hasUv = true;
        }
        // costly path, there are two separate tables         
        else
        {
            hasUv = true;
            // but they are similar we can work using the position one
            hasSeparateUvIndex = (input.trianglesuv != input.triangles);
        }
    }
    // we can still have
    else if (input.trianglesuv.size() == input.triangles.size())
    {
        // this is a case with separate index - we use TC
        hasUv = true;
        hasSeparateUvIndex = true;
    }

    // we should replicate uv handling for all attributes where separate tables could be used
    // for now restricted to no separate index for other attributes
    const bool hasNrm = (input.getNormalCount() == input.getPositionCount());
    const bool hasCol = (input.getColorCount() == input.getPositionCount());

    output.V.resize(3 * ntrigs, -1);
    output.TC.resize(3 * ntrigs, -1);
    output.O.resize(3 * ntrigs, -2);
    output.positions.resize(nverts);
    if (hasUv)
    {
        output.uvcoords.resize(nuv);
        if (hasSeparateUvIndex)
            output.OTC.resize(3 * ntrigs, -2);
    }
    if (hasNrm)
        output.normals.resize(nverts);
    if (hasCol)
        output.colors.resize(nverts);

    for (i = 0; i < nverts; i++)
    {
        output.positions[i] = input.fetchPosition(i);
        if (hasNrm)
            output.normals[i] = input.fetchNormal(i);
        if (hasCol)
            output.colors[i] = input.fetchColor(i);
    }
    if (hasUv)
    {
        for (i = 0; i < nuv; i++)
        {
            output.uvcoords[i] = input.fetchUv(i);
        }
    }

    // Assigns corner to vertices 
    for (i = 0; i < ntrigs; i++)
    {
        output.V[3 * i + 0] = input.triangles[i * 3];
        output.V[3 * i + 1] = input.triangles[i * 3 + 1];
        output.V[3 * i + 2] = input.triangles[i * 3 + 2];
        if (hasSeparateUvIndex) { // V used instead of TC if not hasSeparateUvIndex
            output.TC[3 * i + 0] = input.trianglesuv[i * 3];
            output.TC[3 * i + 1] = input.trianglesuv[i * 3 + 1];
            output.TC[3 * i + 2] = input.trianglesuv[i * 3 + 2];
        }
        // we should extend with other attributes there - similarly for normal ...
    }

    // Get vertices' stars
    std::multimap<std::pair<int, int>, int> adjacency;

    // attempt to pair matching half hedges
    for (c = 0; c < 3 * ntrigs; c++)
    {
        i = output.V[output.n(c)];
        j = output.V[output.p(c)];
        const auto k = output.V[c];
        // skip degenerated // but should be disconsidered later also !!? ... or will result in separate coding of degenerate tri
        if ((i == j) || (i == k) || (j == k))
        {
            std::cout << "!!! CAUTION degenerated" << std::endl;
            continue;
        }
        // if ordered edges do match, set opposite - as done in draco - but wrong (non optimal) pairing could result
        // should we break all and reconnect later ?
        auto pos = adjacency.find(std::pair<int, int>(j, i)); // should have opposite winding
        // matching corner ?
        if (pos != adjacency.end())
        { 
            // skip mirror case
            if (k == output.V[pos->second])
                continue;
            output.O[c] = pos->second;
            output.O[output.O[c]] = c;
            adjacency.erase(pos); // would be faster with big vector ?
        }
        else
        {
            // multi map requird here for completeness - or vector of half edge data
            adjacency.insert({ std::pair<int, int>(i, j), c });
        }
    }
    adjacency.clear();

    //////////////////////////////////////////////////////
    // FOLLOWING PORTION OF CODE IS ADAPTED AND EXTENDED FROM GOOGLE DRACO
    // https://github.com/google/draco/blob/master/LICENSE
    // related license fro draco is replicated at the end of this file

    // break remaining manifold by analysing vertex indices in 1-rings around corners
    std::vector<int> MC(output.V.size(), false); // Corners marking array
    if (true)
    {
        std::vector<std::pair<int, int>> sink_vertices; // vert , corner
        bool mesh_connectivity_updated = false;
        do {
            mesh_connectivity_updated = false;
            for (c = 0; c < 3 * ntrigs; c++)
            {
                if (MC[c]) continue;
                sink_vertices.clear();
                int first_c = c;
                int current_c = c;
                int next_c;
                while (next_c = output.n(output.O[output.n(current_c)]),
                    next_c != first_c && next_c >= 0 && !MC[next_c]) {
                    current_c = next_c;
                }
                first_c = current_c;
                do {
                    MC[current_c] = true;
                    const int sink_c = output.n(current_c);
                    const int sink_v = output.V[sink_c];

                    const int edge_corner = output.p(current_c);
                    bool vertex_connectivity_updated = false;
                    for (auto&& attached_sink_vertex : sink_vertices) {
                        if (attached_sink_vertex.first == sink_v) {
                            const int other_edge_corner = attached_sink_vertex.second;
                            const int opp_edge_corner = output.O[edge_corner];
                            if (opp_edge_corner == other_edge_corner) continue;

                            const int opp_other_edge_corner = output.O[other_edge_corner];
                            if (opp_edge_corner >= 0)
                                output.O[opp_edge_corner] = -2; 
                            if (opp_other_edge_corner >= 0)
                                output.O[opp_other_edge_corner] = -2;                                
                            if (edge_corner >= 0)
                                output.O[edge_corner] = -2;
                            if (other_edge_corner >= 0)
                                output.O[other_edge_corner] = -2;
                            vertex_connectivity_updated = true;
                            break;
                        }
                    }
                    if (vertex_connectivity_updated) {
                        mesh_connectivity_updated = true;
                        break;
                    }
                    std::pair<int, int> new_sink_vert;
                    new_sink_vert.first = output.V[output.p(current_c)];
                    new_sink_vert.second = sink_c;
                    sink_vertices.push_back(new_sink_vert);
                    current_c = output.p(output.O[output.p(current_c)]);
                } while (current_c != first_c && current_c >= 0);

            }
        } while (mesh_connectivity_updated);
    }

    // duplicate points where needed
    std::vector<bool> M(output.positions.size(),false);	 // Vertex marking array
    MC.assign(output.V.size(), false);                   // resets corners marked
    std::vector<std::pair<int, int>> matchedOpposites;
    int pairedAfterManifoldSplit = 0;
    auto numVertex = output.positions.size();
    int addedVertices = 0;
    int numSplitVertices = 0;
    output.duplicatesMap.clear();
    for (c = 0; c < 3 * ntrigs; c++)
    {
        bool manifoldVertex = false;
        if (MC[c]) continue;
        auto vertexIndex = output.V[c];
        if (M[vertexIndex])
        {
            manifoldVertex = true;
            M.push_back(false);
            output.positions.push_back(output.positions[vertexIndex]);
            if (hasUv && !hasSeparateUvIndex)
            {
                output.uvcoords.push_back(output.uvcoords[vertexIndex]);
            }
            vertexIndex = numVertex++;
            // THIS IS ADDED FOR VERTEX DEDUPLICATION
            if (keepTrackOfVertexDeduplications)
            {
                const auto firstadd = output.duplicatesMap.find(output.V[c]);
                const auto dupIdx = (firstadd == output.duplicatesMap.end()) ? numSplitVertices : firstadd->second;
                if (firstadd == output.duplicatesMap.end())
                {
                    output.duplicatesMap.insert({ output.V[c],  dupIdx });
                    numSplitVertices++;
                }
                output.duplicatesMap.insert({ vertexIndex,dupIdx });
            }
            addedVertices++;
        }
        M[vertexIndex] = true;
        auto curc = c;
        int lefm = curc;
        int rigm = curc;
        while (curc >= 0)
        {
            MC[curc] = true;
            if (manifoldVertex && curc >= 0)
                output.V[curc] = vertexIndex;
            if (curc>=0 && output.O[output.n(curc)]<0)
                lefm = curc; 
            curc = output.n(output.O[output.n(curc)]);
            if (curc == c) break; 
        }
        if (curc < 0)
        {
            curc = output.p(output.O[output.p(c)]);
            while (curc >= 0)
            {
                MC[curc] = true;                
                if (manifoldVertex && curc >= 0)
                    output.V[curc] = vertexIndex;
                if (curc >= 0 && output.O[output.p(curc)] < 0)
                    rigm = curc; 
                curc = output.p(output.O[output.p(curc)]);
                if (curc == c) break;
            }
        }
        // THIS IS ADDED FOR EDGE MERGING 
        // test appairing directly
        if (lefm != rigm && lefm > 0 && rigm > 0)
        {
            auto oln = output.O[output.n(lefm)];
            auto orp = output.O[output.p(rigm)];
            auto vl = output.V[output.p(lefm)];
            auto vr = output.V[output.n(rigm)];
            // same fan center V[c] same other edge vertex means opposites should be registered 
            if ((vl == vr) && (oln < 0) && (orp < 0))
                // TO CHECK when could we could reconnect in the loop partly ?
                matchedOpposites.push_back(std::pair<int,int>(output.n(lefm), output.p(rigm)));
        }
    }

    // END OF CODE INSPIRED FROM DRACO
    //////////////////////////////////

    // THIS CODE IS ADDED FOR EDGE MERGING 
    for (const auto& opp : matchedOpposites)
    {
        auto oln = output.O[opp.first];
        auto orp = output.O[opp.second];
        auto vl = output.V[output.n(opp.first)];
        auto vr = output.V[output.p(opp.second)];
        if ((vl == vr) && (oln < 0) && (orp < 0))
        {
            output.O[opp.first] = opp.second;
            output.O[opp.second] = opp.first;
            pairedAfterManifoldSplit++;
        }
    }
    
    if (keepTrackOfVertexDeduplications)
        std::cout << "Manifold handling, split vtx= " << numSplitVertices << ", added vtx = " << addedVertices << ", paired = " << pairedAfterManifoldSplit << std::endl;
    else
        std::cout << "Manifold handling, added vtx = " << addedVertices << ", paired = " << pairedAfterManifoldSplit << std::endl;

    // uv analysis now, based on correct O table
    if (hasSeparateUvIndex) // and if uv index separate!!
    {
        MC.assign(output.V.size(), false); // resets corners marked
        // NOK we should replicate O from geo to split components and add index test for seams - then second pass to add missing indices
        for (c = 0; c < 3 * ntrigs; c++)
        {
            if (MC[c]) continue;
            const auto oppc = output.O[c];
            MC[c] = true;
            if (oppc >= 0)
            {
                const auto& i = output.TC[output.n(c)];
                const auto& j = output.TC[output.p(c)];
                const auto& k = output.TC[c];
                if ((i == j) || (i == k) || (j == k))
                {
                    // degenerated // but should be disconsidered later also !!?
                    std::cout << "!! DEGEN UV" << std::endl;
                }
                const auto& io = output.TC[output.n(oppc)];
                const auto& jo = output.TC[output.p(oppc)]; // pb if opp is denegerated no ?!!
                const bool isNotSeam = (i == jo) && (j == io);
                if (isNotSeam)
                {
                    output.OTC[c] = oppc;
                    output.OTC[oppc] = c;
                }
                MC[oppc] = true;
            }
        }

        MC.assign(output.V.size(), false); // resets corners marked
        M.assign(output.uvcoords.size(), false);
        auto numUVcoords = output.uvcoords.size();
        for (c = 0; c < 3 * ntrigs; c++)
        {
            bool manifoldUV = false;
            if (MC[c]) continue;
            auto uvIndex = output.TC[c];
            if (M[uvIndex])
            {
                // already visited - is manifold
                manifoldUV = true;
                M.push_back(false);
                // duplicate the uvcoord
                output.uvcoords.push_back(output.uvcoords[uvIndex]);
                uvIndex = numUVcoords++;
            }
            M[uvIndex] = true;
            auto curc = c;
            while (curc >= 0)
            {
                MC[curc] = true;
                if (manifoldUV && curc >= 0)
                    output.TC[curc] = uvIndex;
                curc = output.n(output.OTC[output.n(curc)]);
                if (curc == c) break;
            }
            if (curc < 0)
            {
                curc = output.p(output.OTC[output.p(c)]);
                while (curc >= 0)
                {
                    MC[curc] = true;
                    if (manifoldUV && curc >= 0)
                        output.TC[curc] = uvIndex;
                    curc = output.p(output.OTC[output.p(curc)]);
                    if (curc == c) break;
                }
            }
        }
    }

    return true;
}

// License related to Google Draco inspired code
//Apache License
//Version 2.0, January 2004
//http://www.apache.org/licenses/
//
//TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION
//
//1. Definitions.
//
//"License" shall mean the terms and conditions for use, reproduction,
//and distribution as defined by Sections 1 through 9 of this document.
//
//"Licensor" shall mean the copyright owner or entity authorized by
//the copyright owner that is granting the License.
//
//"Legal Entity" shall mean the union of the acting entity and all
//other entities that control, are controlled by, or are under common
//control with that entity.For the purposes of this definition,
//"control" means(i) the power, direct or indirect, to cause the
//direction or management of such entity, whether by contract or
//otherwise, or (ii)ownership of fifty percent(50 %) or more of the
//outstanding shares, or (iii)beneficial ownership of such entity.
//
//"You" (or "Your") shall mean an individual or Legal Entity
//exercising permissions granted by this License.
//
//"Source" form shall mean the preferred form for making modifications,
//including but not limited to software source code, documentation
//source, and configuration files.
//
//"Object" form shall mean any form resulting from mechanical
//transformation or translation of a Source form, including but
//not limited to compiled object code, generated documentation,
//and conversions to other media types.
//
//"Work" shall mean the work of authorship, whether in Source or
//Object form, made available under the License, as indicated by a
//copyright notice that is included in or attached to the work
//(an example is provided in the Appendix below).
//
//"Derivative Works" shall mean any work, whether in Source or Object
//form, that is based on(or derived from) the Workand for which the
//editorial revisions, annotations, elaborations, or other modifications
//represent, as a whole, an original work of authorship.For the purposes
//of this License, Derivative Works shall not include works that remain
//separable from, or merely link(or bind by name) to the interfaces of,
//the Workand Derivative Works thereof.
//
//"Contribution" shall mean any work of authorship, including
//the original version of the Workand any modifications or additions
//to that Work or Derivative Works thereof, that is intentionally
//submitted to Licensor for inclusion in the Work by the copyright owner
//or by an individual or Legal Entity authorized to submit on behalf of
//the copyright owner.For the purposes of this definition, "submitted"
//means any form of electronic, verbal, or written communication sent
//to the Licensor or its representatives, including but not limited to
//communication on electronic mailing lists, source code control systems,
//and issue tracking systems that are managed by, or on behalf of, the
//Licensor for the purpose of discussingand improving the Work, but
//excluding communication that is conspicuously marked or otherwise
//designated in writing by the copyright owner as "Not a Contribution."
//
//"Contributor" shall mean Licensor and any individual or Legal Entity
//on behalf of whom a Contribution has been received by Licensor and
//subsequently incorporated within the Work.
//
//2. Grant of Copyright License.Subject to the terms and conditions of
//this License, each Contributor hereby grants to You a perpetual,
//worldwide, non - exclusive, no - charge, royalty - free, irrevocable
//copyright license to reproduce, prepare Derivative Works of,
//publicly display, publicly perform, sublicense, and distribute the
//Workand such Derivative Works in Source or Object form.
//
//3. Grant of Patent License.Subject to the terms and conditions of
//this License, each Contributor hereby grants to You a perpetual,
//worldwide, non - exclusive, no - charge, royalty - free, irrevocable
//(except as stated in this section) patent license to make, have made,
//use, offer to sell, sell, import, and otherwise transfer the Work,
//where such license applies only to those patent claims licensable
//by such Contributor that are necessarily infringed by their
//Contribution(s) alone or by combination of their Contribution(s)
//with the Work to which such Contribution(s) was submitted.If You
//institute patent litigation against any entity(including a
//    cross - claim or counterclaim in a lawsuit) alleging that the Work
//    or a Contribution incorporated within the Work constitutes direct
//    or contributory patent infringement, then any patent licenses
//    granted to You under this License for that Work shall terminate
//    as of the date such litigation is filed.
//
//    4. Redistribution.You may reproduce and distribute copies of the
//    Work or Derivative Works thereof in any medium, with or without
//    modifications, and in Source or Object form, provided that You
//    meet the following conditions :
//
//(a)You must give any other recipients of the Work or
//Derivative Works a copy of this License; and
//
//(b)You must cause any modified files to carry prominent notices
//stating that You changed the files; and
//
//(c)You must retain, in the Source form of any Derivative Works
//that You distribute, all copyright, patent, trademark, and
//attribution notices from the Source form of the Work,
//excluding those notices that do not pertain to any part of
//the Derivative Works; and
//
//(d)If the Work includes a "NOTICE" text file as part of its
//distribution, then any Derivative Works that You distribute must
//include a readable copy of the attribution notices contained
//within such NOTICE file, excluding those notices that do not
//pertain to any part of the Derivative Works, in at least one
//of the following places : within a NOTICE text file distributed
//as part of the Derivative Works; within the Source form or
//documentation, if provided along with the Derivative Works; or ,
//within a display generated by the Derivative Works, ifand
//wherever such third - party notices normally appear.The contents
//of the NOTICE file are for informational purposes onlyand
//do not modify the License.You may add Your own attribution
//notices within Derivative Works that You distribute, alongside
//or as an addendum to the NOTICE text from the Work, provided
//that such additional attribution notices cannot be construed
//as modifying the License.
//
//You may add Your own copyright statement to Your modificationsand
//may provide additional or different license terms and conditions
//for use, reproduction, or distribution of Your modifications, or
//for any such Derivative Works as a whole, provided Your use,
//reproduction, and distribution of the Work otherwise complies with
//the conditions stated in this License.
//
//5. Submission of Contributions.Unless You explicitly state otherwise,
//any Contribution intentionally submitted for inclusion in the Work
//by You to the Licensor shall be under the termsand conditions of
//this License, without any additional terms or conditions.
//Notwithstanding the above, nothing herein shall supersede or modify
//the terms of any separate license agreement you may have executed
//with Licensor regarding such Contributions.
//
//6. Trademarks.This License does not grant permission to use the trade
//names, trademarks, service marks, or product names of the Licensor,
//except as required for reasonableand customary use in describing the
//origin of the Workand reproducing the content of the NOTICE file.
//
//7. Disclaimer of Warranty.Unless required by applicable law or
//agreed to in writing, Licensor provides the Work(and each
//    Contributor provides its Contributions) on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
//    implied, including, without limitation, any warranties or conditions
//    of TITLE, NON - INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
//    PARTICULAR PURPOSE.You are solely responsible for determining the
//    appropriateness of using or redistributing the Work and assume any
//    risks associated with Your exercise of permissions under this License.
//
//    8. Limitation of Liability.In no event and under no legal theory,
//    whether in tort(including negligence), contract, or otherwise,
//    unless required by applicable law(such as deliberateand grossly
//        negligent acts) or agreed to in writing, shall any Contributor be
//    liable to You for damages, including any direct, indirect, special,
//    incidental, or consequential damages of any character arising as a
//    result of this License or out of the use or inability to use the
//    Work(including but not limited to damages for loss of goodwill,
//        work stoppage, computer failure or malfunction, or any and all
//        other commercial damages or losses), even if such Contributor
//    has been advised of the possibility of such damages.
//
//    9. Accepting Warranty or Additional Liability.While redistributing
//    the Work or Derivative Works thereof, You may choose to offer,
//    and charge a fee for, acceptance of support, warranty, indemnity,
//    or other liability obligations and /or rights consistent with this
//    License.However, in accepting such obligations, You may act only
//    on Your own behalfand on Your sole responsibility, not on behalf
//    of any other Contributor, and only if You agree to indemnify,
//    defend, and hold each Contributor harmless for any liability
//    incurred by, or claims asserted against, such Contributor by reason
//    of your accepting any such warranty or additional liability.
//
//    END OF TERMS AND CONDITIONS
//
//    APPENDIX : How to apply the Apache License to your work.
//
//    To apply the Apache License to your work, attach the following
//    boilerplate notice, with the fields enclosed by brackets "[]"
//    replaced with your own identifying information. (Don't include
//        the brackets!)  The text should be enclosed in the appropriate
//    comment syntax for the file format.We also recommend that a
//    file or class nameand description of purpose be included on the
//    same "printed page" as the copyright notice for easier
//    identification within third - party archives.
//
//    Copyright[yyyy][name of copyright owner]
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//http ://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.
