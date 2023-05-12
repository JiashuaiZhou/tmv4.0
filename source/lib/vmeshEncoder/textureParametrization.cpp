/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2022, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//============================================================================

#include <chrono>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <memory>
#include "encoder.hpp"
#include "textureParametrization.hpp"

#include "util/checksum.hpp"

// NB: these must come after the standard headers as they define prohibited
//     macros that conflict with the system implementation
#include <DirectXMath.h>
#include <DirectXMesh.h>

#include <UVAtlas.h>

#define DEBUG_ORTHO_CREATE_PATCH 0
#define DEBUG_ORTHO_CREATE_PATCH_VERBOSE 0
#define DEBUG_ORTHO_PATCH_PACKING 0
#define DEBUG_ORTHO_PATCH_PACKING_VERBOSE 0

namespace vmesh {

//============================================================================

HRESULT __cdecl UVAtlasCallback(float fPercentDone) {
  // static auto prev = std::chrono::steady_clock::now();
  // const auto  tick = std::chrono::steady_clock::now();
  // if (tick - prev > std::chrono::seconds(1)) {
  //   std::cout << fPercentDone * 100. << "%   \r" << std::flush;
  //   prev = tick;
  // }
  return S_OK;
}

//============================================================================

bool
TextureParametrization::generate(const TriangleMesh<MeshType>& decimate,
                                 TriangleMesh<MeshType>&            decimateTexture,
                                 const VMCEncoderParameters& params,
                                 std::vector<ConnectedComponent<MeshType>>& curCC,
                                 std::vector<ConnectedComponent<MeshType>>& previousCC, 
                                 std::string keepFilesPathPrefix) {
    switch (params.textureParameterizationType) {
    case 1 /*ORTHO*/:
        return generate_orthoAtlas(decimate, decimateTexture, curCC, previousCC, params, keepFilesPathPrefix);
    default:
    case 0 /*UVATLAS*/:
        return generate_UVAtlas(decimate, decimateTexture, params, keepFilesPathPrefix);
    }
    return false;
}
//============================================================================
bool
TextureParametrization::generate_orthoAtlas(const TriangleMesh<MeshType>& decimate,
                                            TriangleMesh<MeshType>& decimateTexture,
                                            std::vector<ConnectedComponent<MeshType>>& curCC,
                                            std::vector<ConnectedComponent<MeshType>>& previousCC,
                                            const VMCEncoderParameters& params, std::string keepFilesPathPrefix) {
    TriangleMesh<float> input;
    input.convert(decimate);
    TriangleMesh<MeshType> mesh;
    mesh.convert(input);

    // Remove unwanted mesh components
    mesh.displacements().clear();
    mesh.colours().clear();
    mesh.texCoords().clear();
    mesh.texCoordTriangles().clear();
    mesh.normals().clear();
    mesh.normalTriangles().clear();

    std::cout << mesh.pointCount() << " vertices, " << mesh.triangleCount()
        << " faces\n";

    if (!mesh.pointCount() || !mesh.triangleCount()) {
        std::cerr << "ERROR: Invalid mesh\n";
        return false;
    }

    std::vector<std::shared_ptr<TriangleMesh<MeshType>>> projectedCCList;
    std::vector<int32_t> projectedConnectedComponentsCategories;
    const auto startCreate = std::chrono::steady_clock::now();
    create_patches(mesh, projectedCCList, projectedConnectedComponentsCategories, params, keepFilesPathPrefix);
    namespace chrono = std::chrono;
    auto endCreate = chrono::steady_clock::now();
    auto deltaCreatems = chrono::duration_cast<chrono::milliseconds>(endCreate - startCreate);
#if DEBUG_ORTHO_CREATE_PATCH
    std::cout << "Patch Creation time: " << deltaCreatems.count() << " ms\n";
#endif
    const auto startPack = std::chrono::steady_clock::now();
    std::vector<ConnectedComponent<MeshType>> packedCCList;
    if (previousCC.size() == 0) {
        double adjustScaling = 1.0;
        switch (params.packingType) {
        default:
        case 0: // default packing 
        {
            while (!pack_patches_with_patch_scale_and_rotation(mesh, projectedCCList, projectedConnectedComponentsCategories, packedCCList, params, adjustScaling, keepFilesPathPrefix)) {
                adjustScaling *= params.packingScaling;
#if DEBUG_ORTHO_PATCH_PACKING
                std::cout << "Packing did not fit, adjusting the scale of patches to " << adjustScaling << std::endl;
#endif
            }
            break;
        }
        case 1: // tetris packing 
        {
            while (!tetris_packing_patches_with_patch_scale_and_rotation(mesh, projectedCCList, projectedConnectedComponentsCategories, packedCCList, params, adjustScaling, keepFilesPathPrefix)) {
                adjustScaling *= params.packingScaling;
#if DEBUG_ORTHO_PATCH_PACKING
                std::cout << "Packing did not fit, adjusting the scale of patches to " << adjustScaling << std::endl;
#endif
            }
            break;
        }
        case 2: // projection packing 
        {
            while (!projection_packing_patches_with_patch_scale_and_rotation(mesh, projectedCCList, projectedConnectedComponentsCategories, packedCCList, params, adjustScaling, keepFilesPathPrefix)) {
                adjustScaling *= params.packingScaling;
#if DEBUG_ORTHO_PATCH_PACKING
                std::cout << "Packing did not fit, adjusting the scale of patches to " << adjustScaling << std::endl;
#endif
            }
            break;
        }
        }
    }
    else {
        //extract connected components from past frame
        auto& previousFrameProjectedCCList = previousCC;
        //now place the new patches, considering the past ones
        double adjustScaling = 1.0;
        while (!pack_patches_with_patch_scale_and_rotation_and_temporal_stabilization(mesh, projectedCCList, projectedConnectedComponentsCategories, packedCCList, previousFrameProjectedCCList, params, adjustScaling, keepFilesPathPrefix)) {
            adjustScaling *= params.packingScaling;
#if DEBUG_ORTHO_PATCH_PACKING
            std::cout << "Packing did not fit, adjusting the scale of patches to " << adjustScaling << std::endl;
#endif
        }
    }
    namespace chrono = std::chrono;
    auto endPack = chrono::steady_clock::now();
    auto deltaPackms = chrono::duration_cast<chrono::milliseconds>(endPack - startPack);
#if DEBUG_ORTHO_PATCH_PACKING
    std::cout << "Patch Packing time: " << deltaPackms.count() << " ms\n";
#endif
    decimateTexture.convert(mesh);
    curCC = packedCCList;

    std::cout << "(orthoAtlas) Processing time: " << deltaCreatems.count() + deltaPackms.count() << " ms\n";
    std::cout << "(orthoAtlas) Output # of charts: " << curCC.size() << ", " << mesh.pointCount() << " verts\n";

    return true;
}

//============================================================================

bool
TextureParametrization::generate_UVAtlas(const TriangleMesh<MeshType>& decimate,
                                         TriangleMesh<MeshType>& decimateTexture,
                                         const VMCEncoderParameters& params, std::string keepFilesPathPrefix) {
  TriangleMesh<float> mesh;
  mesh.convert(decimate);

  // Remove unwanted mesh components
  mesh.displacements().clear();
  mesh.colours().clear();
  mesh.texCoords().clear();
  mesh.texCoordTriangles().clear();
  mesh.normals().clear();
  mesh.normalTriangles().clear();

  std::cout << mesh.pointCount() << " vertices, " << mesh.triangleCount()
            << " faces\n";

  if ((mesh.pointCount() == 0) || (mesh.triangleCount() == 0)) {
    std::cerr << "ERROR: Invalid mesh\n";
    return 1;
  }

  // Prepare mesh for processing
  const float           epsilon = 0.F;
  std::vector<uint32_t> adjacency(3 * mesh.triangleCount());
  auto                  hr = DirectX::GenerateAdjacencyAndPointReps(
    reinterpret_cast<uint32_t*>(mesh.triangles().data()),
    mesh.triangleCount(),
    reinterpret_cast<DirectX::XMFLOAT3*>(mesh.points().data()),
    mesh.pointCount(),
    epsilon,
    nullptr,
    adjacency.data());
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed generating adjacency (" << hr << ")\n";
    return 1;
  }

  // Validation
  std::wstring msgs;
  DirectX::Validate(reinterpret_cast<uint32_t*>(mesh.triangles().data()),
                    mesh.triangleCount(),
                    mesh.pointCount(),
                    adjacency.data(),
                    DirectX::VALIDATE_BACKFACING | DirectX::VALIDATE_BOWTIES,
                    &msgs);
  if (!msgs.empty()) {
    std::cerr << "WARNING: \n";
    std::wcerr << msgs;
  }

  // Clean
  std::vector<uint32_t> dups;
  bool                  breakBowties = true;
  hr = DirectX::Clean(reinterpret_cast<uint32_t*>(mesh.triangles().data()),
                      mesh.triangleCount(),
                      mesh.pointCount(),
                      adjacency.data(),
                      nullptr,
                      dups,
                      breakBowties);
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed mesh clean " << hr << '\n';
    return 1;
  }

  if (!dups.empty()) {
    std::cout << " [" << dups.size() << " vertex dups]\n";

    mesh.reservePoints(mesh.pointCount() + dups.size());
    for (auto dupIdx : dups) { mesh.addPoint(mesh.point(dupIdx)); }
  }

  // Perform UVAtlas isocharting
  std::cout << "Computing isochart atlas on mesh...\n";

  std::vector<DirectX::UVAtlasVertex> vb;
  std::vector<uint8_t>                ib;
  float                               outStretch = 0.F;
  size_t                              outCharts  = 0;
  std::vector<uint32_t>               facePartitioning;
  std::vector<uint32_t>               vertexRemapArray;
  const auto                          start = std::chrono::steady_clock::now();
  hr =
    UVAtlasCreate(reinterpret_cast<DirectX::XMFLOAT3*>(mesh.points().data()),
                  mesh.pointCount(),
                  reinterpret_cast<uint32_t*>(mesh.triangles().data()),
                  DXGI_FORMAT_R32_UINT,
                  mesh.triangleCount(),
                  params.maxCharts,
                  params.maxStretch,
                  params.width,
                  params.height,
                  params.gutter,
                  adjacency.data(),
                  nullptr,
                  nullptr,
                  UVAtlasCallback,
                  DirectX::UVATLAS_DEFAULT_CALLBACK_FREQUENCY,
                  params.uvOptions,
                  vb,
                  ib,
                  &facePartitioning,
                  &vertexRemapArray,
                  &outStretch,
                  &outCharts);
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed creating isocharts " << hr << '\n';
    return 1;
  }

  namespace chrono = std::chrono;
  auto end         = chrono::steady_clock::now();
  auto deltams     = chrono::duration_cast<chrono::milliseconds>(end - start);
  std::cout << "(UVAtlas) Processing time: " << deltams.count() << " ms\n";
  std::cout << "(UVAtlas) Output # of charts: " << outCharts << ", resulting stretching "
            << outStretch << ", " << vb.size() << " verts\n";

  assert(ib.size() == 3 * mesh.triangles().size() * sizeof(uint32_t));
  memcpy(mesh.triangles().data(), ib.data(), ib.size());

  assert(vertexRemapArray.size() == vb.size());
  std::vector<Vec3<float>> pos(vertexRemapArray.size());
  hr = DirectX::UVAtlasApplyRemap(
    reinterpret_cast<DirectX::XMFLOAT3*>(mesh.points().data()),
    sizeof(DirectX::XMFLOAT3),
    mesh.pointCount(),
    vertexRemapArray.size(),
    vertexRemapArray.data(),
    reinterpret_cast<DirectX::XMFLOAT3*>(pos.data()));
  if (FAILED(hr)) {
    std::cerr << "ERROR: Failed applying atlas vertex remap (" << hr << ")\n";
    return 1;
  }
  //  std::swap(mesh.points(), pos); // float to double conversion issues
  mesh.points().clear();
  mesh.points().resize(pos.size());
  for (size_t i = 0; i < pos.size(); i++) { mesh.point(i) = pos[i]; }

  msgs.clear();
  DirectX::Validate(reinterpret_cast<uint32_t*>(mesh.triangles().data()),
                    mesh.triangleCount(),
                    mesh.pointCount(),
                    adjacency.data(),
                    DirectX::VALIDATE_DEFAULT,
                    &msgs);
  if (!msgs.empty()) {
    std::cerr << "WARNING: \n";
    std::wcerr << msgs;
  }

  // Copy isochart UVs into mesh
  mesh.reserveTexCoords(vb.size());
  std::transform(vb.begin(),
                 vb.end(),
                 std::back_inserter(mesh.texCoords()),
                 [](DirectX::UVAtlasVertex& vtx) {
                   return Vec2<float>{vtx.uv.x, vtx.uv.y};
                 });

  mesh.texCoordTriangles() = mesh.triangles();

  decimateTexture.convert(mesh);

  return 0;
}

//============================================================================
bool
TextureParametrization::create_patches(TriangleMesh<MeshType>& mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& finalCCList,
    std::vector<int32_t>& finalConnectedComponentsCategories,
    const VMCEncoderParameters& params, std::string keepFilesPathPrefix) {
    int orientationCount = params.use45DegreeProjection ? 18 : 6;
    Vec3<double>* orientations = params.use45DegreeProjection ? PROJDIRECTION18 : PROJDIRECTION6;
    //create connected components and project them into patches (local UV)
    std::vector<int32_t> connectedComponentsCategories;
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>> connectedComponents;
    const auto ccCount0 = ExtractConnectedComponentsByNormal(mesh,
        orientations, orientationCount,
        connectedComponentsCategories, &connectedComponents,
        params.useVertexCriteria,
        params.bUseSeedHistogram,
        params.strongGradientThreshold,
        params.maxCCAreaRatio,
        params.maxNumFaces,
        params.bFaceClusterMerge,
        params.lambdaRDMerge,
        params.check2DConnectivity,
        params.keepIntermediateFiles, keepFilesPathPrefix);
#if DEBUG_ORTHO_CREATE_PATCH
    std::cout << "Created " << connectedComponents.size() << " patches" << std::endl;
#endif

    //check for overlapping triangles, and break the connected component if necessary
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>> projectedCCList;
    std::vector<int32_t> projectedConnectedComponentsCategories;
    int numTotalProjectedTriangles = 0;

    int idxCC = 0;
    for (auto& ccComp : connectedComponents) {
        int32_t ccCategory = connectedComponentsCategories[idxCC++];
        //get triangle normal direction
        std::vector<Vec3<MeshType>> triangleNormalsCC;
        if ((params.adjustNormalDirection))
            ccComp->computeTriangleNormals(triangleNormalsCC);
        int numProjectedTriangles = 0;
        std::vector<bool> triStatus;
        triStatus.resize(ccComp->triangles().size(), false);
        while (ccComp->triangles().size() > numProjectedTriangles) {
            std::shared_ptr<TriangleMesh<MeshType>> projectedCC;
            projectedCC.reset(new TriangleMesh<MeshType>);
            projectedCCList.push_back(projectedCC);
            //determine the best orientation for the connected components
            if ((numProjectedTriangles != 0) && (params.adjustNormalDirection)) {
                std::vector<double> dotProdNormal;
                dotProdNormal.resize(orientationCount, 0.0);
                std::vector<bool> bAllowOrientation;
                bAllowOrientation.resize(orientationCount, true);
                int triNormalIdx = 0;
                for (auto tri : ccComp->triangles()) {
                    if (!triStatus[triNormalIdx])
                    {
                        for (int i = 0; i < orientationCount; i++) {
                            double dotProd = triangleNormalsCC[triNormalIdx] * orientations[i];
                            if (dotProd > 0.0)
                                dotProdNormal[i] += 1 / dotProd;
                            else
                                if (dotProd < 0.0)
                                    bAllowOrientation[i] = false;
                        }
                    }
                    triNormalIdx++;
                }
                //now check the best category
                double bestScore = std::numeric_limits<double>::max();
                for (size_t j = 0; j < orientationCount; ++j) {
                    if (bAllowOrientation[j] && (dotProdNormal[j] < bestScore)) {
                        bestScore = dotProdNormal[j];
                        ccCategory = j;
                    }
                }
            }
            projectedConnectedComponentsCategories.push_back(ccCategory);
            projectedCC->setMaterialLibrary(mesh.materialLibrary());
            auto& vertexList = projectedCC->points();
            auto& triList = projectedCC->triangles();
            auto& uvList = projectedCC->texCoords();
            auto& uvTriList = projectedCC->texCoordTriangles();
            std::vector<vmesh::Box2<MeshType>> triAABBList;
            triAABBList.clear();
            std::vector<int64_t> edgeList;
            edgeList.clear();
            int triIdx = 0;
            for (auto tri : ccComp->triangles()) {
                if (triStatus[triIdx])
                {
                    triIdx++;
                    continue;
                }
                //create 2D orthographic projection
                Vec2<MeshType> uv[3];
                Vec3<MeshType> vertex[3];
                bool vertexInList[3];
                int          vertexCCIndex[3];
                bool         edgeInList[3];
                vmesh::Box2<MeshType> triBB;
                for (int idx = 0; idx < 3; idx++) {
                    vertex[idx] = ccComp->point(tri[idx]);
                    uv[idx] = projectPoint(ccComp->point(tri[idx]), ccCategory);
                    triBB.enclose(uv[idx]);
                    // assuming vertex is not in the list
                    vertexInList[idx] = false;
                    vertexCCIndex[idx] = -1;
                }
#if DEBUG_ORTHO_CREATE_PATCH_VERBOSE
                std::cout << "tri[" << triIdx << "]:{(" << uv[0][0] << "," << uv[0][1] <<
                    "),(" << uv[1][0] << "," << uv[1][1] << "),(" << uv[2][0] << "," << uv[2][1] << ")}" << std::endl;
#endif
                bool canProject = true;
                //check if triangle can be projected, skip this check for the first triangle
                if (!triAABBList.empty()) {
                    //check if the vertices are in the list already
                    for (int idx = 0; idx < 3; idx++) {
                        auto vertPos = std::find(vertexList.begin(), vertexList.end(), vertex[idx]);
                        if (vertPos != vertexList.end())
                        {
                            vertexInList[idx] = true;
                            vertexCCIndex[idx] = vertPos - vertexList.begin();
                        }
                    }
                    //check if the edges are in the list already
                    for (int idx = 0; idx < 3; idx++) {
                        edgeInList[idx] = true;
                        if (vertexInList[0] && vertexInList[1]) {
                            if (std::find(edgeList.begin(), edgeList.end(), EdgeIndex(vertexCCIndex[idx], vertexCCIndex[(idx + 1) % 3])) != edgeList.end())
                            {
                                //edge has already been tested by another triangle
                                edgeInList[idx] = false;
                            }
                        }
                    }
                    //check if the bounding box overlap
                    for (int triIdx = 0; triIdx < triAABBList.size(); triIdx++) {
                        auto& projTriBB = triAABBList[triIdx];
                        if (projTriBB.intersects(triBB)) {
                            // check if the vertices are inside the triangle
                            for (int idx = 0; idx < 3 && canProject; idx++) {
                                if (!vertexInList[idx]) {
                                    if (isInsideTriangleUsingArea(uvList[uvTriList[triIdx][0]], uvList[uvTriList[triIdx][1]], uvList[uvTriList[triIdx][2]],
                                        uv[idx])) {
                                        canProject = false;
#if DEBUG_ORTHO_CREATE_PATCH_VERBOSE
                                        std::cout << "[!canProject] New UV(" << uv[idx][0] << "," << uv[idx][1] << ") is inside triangle[" << triIdx << "]" << std::endl;
#endif
                                    }
                                }
                            }
                            // check if the triangle surface will occlude any vertices
                            for (int idx = 0; idx < 3 && canProject; idx++) {
                                auto& projUV = uvList[uvTriList[triIdx][idx]];
                                if ((uv[0] != projUV) && (uv[1] != projUV) && (uv[2] != projUV))
                                {
                                    if (isInsideTriangleUsingArea(uv[0], uv[1], uv[2], projUV)) {
                                        canProject = false;
#if DEBUG_ORTHO_CREATE_PATCH_VERBOSE
                                        std::cout << "[!canProject] UV(" << projUV[0] << "," << projUV[1] << ") is inside new triangle" << std::endl;
#endif
                                    }
                                }
                            }
                            // check if the edges intersect
                            for (int idxProj = 0; idxProj < 3 && canProject; idxProj++) {
                                auto& uv1Idx = uvTriList[triIdx][idxProj];
                                auto& uv2Idx = uvTriList[triIdx][(idxProj + 1) % 3];
                                auto& uv1 = uvList[uv1Idx];
                                auto& uv2 = uvList[uv2Idx];
                                for (int idx = 0; idx < 3 && canProject; idx++) {
                                    if (edgeInList[idx]) {
                                        bool testEdge = true;
                                        if (vertexInList[idx]) {
                                            if ((uv1Idx == vertexCCIndex[idx]) || (uv2Idx == vertexCCIndex[idx]))
                                                testEdge = false;
                                        }
                                        if (vertexInList[(idx + 1) % 3]) {
                                            if ((uv1Idx == vertexCCIndex[(idx + 1) % 3]) || (uv2Idx == vertexCCIndex[(idx + 1) % 3]))
                                                testEdge = false;
                                        }
                                        if (testEdge) {
                                            if (edgesCross(uv1, uv2, uv[idx], uv[(idx + 1) % 3])) {
                                                canProject = false;
#if DEBUG_ORTHO_CREATE_PATCH_VERBOSE
                                                std::cout << "[!canProject] edges [(" << uv1[0] << "," << uv1[1] << ")(" << uv2[0] << "," << uv2[1] <<
                                                    ")] and [(" << uv[idx][0] << "," << uv[idx][1] << ")(" << uv[(idx + 1) % 3][0] << "," << uv[(idx + 1) % 3][1] << ")] cross" << std::endl;
#endif                                            
                                            }
                                        }
                                    }
                                }
                            }

                        }
                    }
                }
                //add the triangle to the projected connected component
                if (canProject)
                {
                    triStatus[triIdx] = true;
                    numTotalProjectedTriangles++;
                    numProjectedTriangles++;
                    //inserting UV coordinates and triangle
                    for (int idx = 0; idx < 3; idx++) {
                        if (!vertexInList[idx]) {
                            //add uv coordinates and uv triangle
                            vertexCCIndex[idx] = uvList.size();
                            uvList.push_back(uv[idx]);
                            vertexList.push_back(vertex[idx]);
                        }
                    }
                    uvTriList.push_back(vmesh::Triangle(vertexCCIndex[0], vertexCCIndex[1], vertexCCIndex[2]));
                    triList.push_back(vmesh::Triangle(vertexCCIndex[0], vertexCCIndex[1], vertexCCIndex[2]));
                    //inserting the edge
                    for (int idx = 0; idx < 3; idx++) {
                        auto edgeIdx = EdgeIndex(vertexCCIndex[idx], vertexCCIndex[(idx + 1) % 3]);
                        //if (std::find(edgeList.begin(), edgeList.end(), edgeIdx) == edgeList.end())
                        edgeList.push_back(edgeIdx);
                    }
                    //insert the bounding box
                    triAABBList.push_back(triBB);
                }
                triIdx++;
            }
        }
    }
#if DEBUG_ORTHO_CREATE_PATCH
    std::cout << "Created " << projectedCCList.size() << " projected patches" << std::endl;
    {
        int ccIdx = 0;
        TriangleMesh<MeshType> meshDebug;
        double totalPerimeter = 0;
        double totalStretchL2 = 0;
        double minStretchL2 = std::numeric_limits<double>::max();
        double maxStretchL2 = std::numeric_limits<double>::min();
        for (auto cc : projectedCCList) {
            meshDebug.append(*cc);
#if DEBUG_ORTHO_CREATE_PATCH_VERBOSE
            if (params.keepIntermediateFiles) {
                auto prefixDEBUG = keepFilesPathPrefix + "_proj_CC#" + std::to_string(ccIdx);
                cc->save(prefixDEBUG + ".obj");
            }
            std::cout << "CC[" << ccIdx << "] -> perimeter(" << cc->perimeter() << "), stretchL2(" << cc->stretchL2(projectedConnectedComponentsCategories[ccIdx]) << ")" << std::endl;
#endif
            totalPerimeter += cc->perimeter();
            auto l2 = cc->stretchL2(projectedConnectedComponentsCategories[ccIdx]);
            totalStretchL2 += l2;
            if (l2 > maxStretchL2)
                maxStretchL2 = l2;
            if (l2 < minStretchL2)
                minStretchL2 = l2;
            ccIdx++;
        }
        std::cout << "(" << ccIdx << " CC after projection) TOTAL PERIMETER: " << totalPerimeter <<
            ", TOTAL STRETCH L2: " << totalStretchL2 <<
            ", AVG. STRETCH L2: " << totalStretchL2 / (double)ccIdx <<
            ", MIN. STRETCH L2: " << minStretchL2 <<
            ", MAX. STRETCH L2: " << maxStretchL2 << std::endl;
        if (params.keepIntermediateFiles) {
            meshDebug.texCoords().clear();
            meshDebug.texCoordTriangles().clear();
            meshDebug.reserveTexCoords(meshDebug.triangleCount() * 3);
            meshDebug.reserveTexCoordTriangles(meshDebug.triangleCount());
            int idxCC = 0;
            int idxTriangle = 0;
            int side = 100;
            for (auto cc : projectedCCList) {
                float x = ((projectedConnectedComponentsCategories[idxCC] % 9) + 0.5) / (9.0);
                float y = 1 - ((projectedConnectedComponentsCategories[idxCC] / 9) + 0.5) / (2.0);
                for (int idx = 0; idx < cc->triangles().size(); idx++) {
                    meshDebug.texCoords().push_back(Vec2<float>(x, y));
                    meshDebug.texCoords().push_back(Vec2<float>(x, y));
                    meshDebug.texCoords().push_back(Vec2<float>(x, y));
                    meshDebug.texCoordTriangles().push_back(vmesh::Triangle(3 * idxTriangle, 3 * idxTriangle + 1, 3 * idxTriangle + 2));
                    idxTriangle++;
                }
                idxCC++;
            }
            meshDebug.setMaterialLibrary(vmesh::basename(keepFilesPathPrefix + "_debug_categories.mtl"));
            meshDebug.save(keepFilesPathPrefix + "_debug_categories_after_projection.obj");
        }
    }
#endif

    if (params.check2DConnectivity) {
        int idxCC = 0;
        for (auto& patch : projectedCCList) {
            int32_t ccCategory = projectedConnectedComponentsCategories[idxCC++];
            std::vector<int> partition;
            int ccCount = ExtractConnectedComponentsByEdge(patch->texCoordTriangles(), patch->texCoordCount(), *patch, partition, !(params.iDeriveTextCoordFromPos == 3));
            for (int idxProjPatch = 0; idxProjPatch < ccCount; idxProjPatch++) {
                std::shared_ptr<TriangleMesh<MeshType>> finalCC;
                finalCC.reset(new TriangleMesh<MeshType>);;
                finalCC->setMaterialLibrary(mesh.materialLibrary());
                auto& vertexList = finalCC->points();
                auto& triList = finalCC->triangles();
                auto& uvList = finalCC->texCoords();
                auto& uvTriList = finalCC->texCoordTriangles();
                finalCCList.push_back(finalCC);
                finalConnectedComponentsCategories.push_back(ccCategory);
                for (int idxPartition = 0; idxPartition < partition.size(); idxPartition++) {
                    if (partition[idxPartition] == idxProjPatch) {
                        Vec2<MeshType> uv[3];
                        Vec3<MeshType> vertex[3];
                        for (int idx = 0; idx < 3; idx++) {
                            uv[idx] = patch->texCoord(patch->texCoordTriangle(idxPartition)[idx]);
                            vertex[idx] = patch->point(patch->triangle(idxPartition)[idx]);
                        }
                        //inserting UV coordinates and triangle
                        int uvIdx[3];
                        for (int idx = 0; idx < 3; idx++) {
                            auto uvPos = std::find(uvList.begin(), uvList.end(), uv[idx]);
                            if (uvPos == uvList.end()) {
                                //add uv coordinates and uv triangle
                                uvIdx[idx] = uvList.size();
                                uvList.push_back(uv[idx]);
                            }
                            else
                                uvIdx[idx] = uvPos - uvList.begin();
                        }
                        uvTriList.push_back(vmesh::Triangle(uvIdx[0], uvIdx[1], uvIdx[2]));
                        //inserting vertex and triangle
                        int vertIdx[3];
                        for (int idx = 0; idx < 3; idx++) {
                            auto vertPos = std::find(vertexList.begin(), vertexList.end(), vertex[idx]);
                            if (vertPos == vertexList.end()) {
                                //add uv coordinates and uv triangle
                                vertIdx[idx] = vertexList.size();
                                vertexList.push_back(vertex[idx]);
                            }
                            else
                                vertIdx[idx] = vertPos - vertexList.begin();
                        }
                        triList.push_back(vmesh::Triangle(vertIdx[0], vertIdx[1], vertIdx[2]));
                    }
                }
            }
        }
    }
    else {
        finalCCList = projectedCCList;
        finalConnectedComponentsCategories = projectedConnectedComponentsCategories;
    }
    //now adjust the UV coordinates to remove the bias
    for (int finalCCIdx = 0; finalCCIdx < finalCCList.size(); finalCCIdx++) {
        auto& finalCC = finalCCList[finalCCIdx];
        auto& ccCategory = finalConnectedComponentsCategories[finalCCIdx];
        auto& uvList = finalCC->texCoords();
        auto bbBox = finalCC->texCoordBoundingBox();
        for (auto& uv : uvList) {
            uv -= bbBox.min;
        }
        // now check the winding of the UV triangles and invert u (tangent) axis if necessary
        if ((ccCategory == 0) || (ccCategory == 4) || (ccCategory == 5) || (ccCategory == 6) || (ccCategory == 9) || (ccCategory == 11) || (ccCategory == 12) || (ccCategory == 14) || (ccCategory == 15)) {
            for (auto& uv : uvList) {
                uv[0] = (bbBox.max - bbBox.min)[0] - uv[0];
            }
        }
    }
#if DEBUG_ORTHO_CREATE_PATCH
    std::cout << "Total of " << finalCCList.size() << " final patches" << std::endl;
    {
        int ccIdx = 0;
        TriangleMesh<MeshType> meshDebug;
        double totalPerimeter = 0;
        double totalStretchL2 = 0;
        double minStretchL2 = std::numeric_limits<double>::max();
        double maxStretchL2 = std::numeric_limits<double>::min();
        for (auto cc : finalCCList) {
            meshDebug.append(*cc);
#if DEBUG_ORTHO_CREATE_PATCH_VERBOSE
            if (params.keepIntermediateFiles) {
                auto prefixDEBUG = keepFilesPathPrefix + "_final_CC#" + std::to_string(ccIdx);
                cc->save(prefixDEBUG + ".obj");
            }
            std::cout << "CC[" << ccIdx << "] -> perimeter(" << cc->perimeter() << "), stretchL2(" << cc->stretchL2(finalConnectedComponentsCategories[ccIdx]) << ")" << std::endl;
#endif
            totalPerimeter += cc->perimeter();
            auto l2 = cc->stretchL2(finalConnectedComponentsCategories[ccIdx]);
            totalStretchL2 += l2;
            if (l2 > maxStretchL2)
                maxStretchL2 = l2;
            if (l2 < minStretchL2)
                minStretchL2 = l2;
            ccIdx++;
        }
        std::cout << "( " << ccIdx << " CC ) TOTAL PERIMETER: " << totalPerimeter <<
            ", TOTAL STRETCH L2: " << totalStretchL2 <<
            ", AVG. STRETCH L2: " << totalStretchL2 / (double)ccIdx <<
            ", MIN. STRETCH L2: " << minStretchL2 <<
            ", MAX. STRETCH L2: " << maxStretchL2 << std::endl;
        if (params.keepIntermediateFiles) {
            meshDebug.texCoords().clear();
            meshDebug.texCoordTriangles().clear();
            meshDebug.reserveTexCoords(meshDebug.triangleCount() * 3);
            meshDebug.reserveTexCoordTriangles(meshDebug.triangleCount());
            int idxCC = 0;
            int idxTriangle = 0;
            int side = 100;
            for (auto cc : finalCCList) {
                float x = ((finalConnectedComponentsCategories[idxCC] % 9) + 0.5) / (9.0);
                float y = 1 - ((finalConnectedComponentsCategories[idxCC] / 9) + 0.5) / (2.0);
                for (int idx = 0; idx < cc->triangles().size(); idx++) {
                    meshDebug.texCoords().push_back(Vec2<float>(x, y));
                    meshDebug.texCoords().push_back(Vec2<float>(x, y));
                    meshDebug.texCoords().push_back(Vec2<float>(x, y));
                    meshDebug.texCoordTriangles().push_back(vmesh::Triangle(3 * idxTriangle, 3 * idxTriangle + 1, 3 * idxTriangle + 2));
                    idxTriangle++;
                }
                idxCC++;
            }
            meshDebug.setMaterialLibrary(vmesh::basename(keepFilesPathPrefix + "_debug_categories.mtl"));
            meshDebug.save(keepFilesPathPrefix + "_debug_categories_final.obj");
        }
    }
#endif
    return 0;
}

void TextureParametrization::patch_rasterization(vmesh::Plane<uint8_t>& occupancyCC,
    ConnectedComponent<MeshType>& packedCC,
    const VMCEncoderParameters& params) {
    auto bbCC = packedCC.texCoordBoundingBox();
    int widthOccCC = std::ceil((packedCC.getScale() * bbCC.max[0] + 2 * params.gutter) / (double)params.displacementVideoBlockSize);
    int heightOccCC = std::ceil((packedCC.getScale() * bbCC.max[1] + 2 * params.gutter) / (double)params.displacementVideoBlockSize);
    occupancyCC.resize(widthOccCC, heightOccCC);
    occupancyCC.fill(false);
    for (auto& tri : packedCC.texCoordTriangles()) {
        // get bounding box of projected triangle
        vmesh::Box2<MeshType> triBB(std::numeric_limits<MeshType>::max(), std::numeric_limits<MeshType>::lowest());
        vmesh::Vec2<MeshType> uv[3];
        vmesh::Vec2<MeshType> gutterUV(params.gutter);
        for (int idx = 0; idx < 3; idx++) {
            uv[idx] = packedCC.getScale() * packedCC.texCoord(tri[idx]) + gutterUV;
            triBB.enclose(uv[idx]);
        }
        // rasterize projected triangle to create occupancy map
        int h0 = std::max(0, (int)std::floor((triBB.min[1] - params.gutter) / params.displacementVideoBlockSize));
        int h1 = std::min(heightOccCC, (int)std::ceil((triBB.max[1] + params.gutter) / params.displacementVideoBlockSize));
        int w0 = std::max(0, (int)std::floor((triBB.min[0] - params.gutter) / params.displacementVideoBlockSize));
        int w1 = std::min(widthOccCC, (int)std::ceil((triBB.max[0] + params.gutter) / params.displacementVideoBlockSize));
        for (int h = h0; h < h1; h++) {
            for (int w = w0; w < w1; w++) {
                if (!occupancyCC.get(h, w)) {
                    bool emptyBlock = false;
                    for (int hh = -params.gutter; hh < (params.displacementVideoBlockSize + params.gutter) && !emptyBlock; hh++) {
                        for (int ww = -params.gutter; ww < (params.displacementVideoBlockSize + params.gutter) && !emptyBlock; ww++) {
                            if (isInsideTriangleUsingArea(uv[0], uv[1], uv[2], Vec2<MeshType>(w * params.displacementVideoBlockSize + ww, h * params.displacementVideoBlockSize + hh)))
                                emptyBlock = true;
                        }
                    }
                    occupancyCC.set(h, w, emptyBlock);
                }
            }
        }
    }
}

void TextureParametrization::patch_horizons(vmesh::Plane<uint8_t>& occupancyCC,
    std::vector<int>& topHorizon,
    std::vector<int>& bottomHorizon,
    std::vector<int>& rightHorizon,
    std::vector<int>& leftHorizon) {
    auto sizeU0 = occupancyCC.width();
    auto sizeV0 = occupancyCC.height();
    topHorizon.resize(sizeU0, 0);
    bottomHorizon.resize(sizeU0, 0);
    rightHorizon.resize(sizeV0, 0);
    leftHorizon.resize(sizeV0, 0);
    for (int j = 0; j < sizeU0; j++) {
        while (!occupancyCC.get(sizeV0 - 1 - topHorizon[j], j) && (topHorizon[j] < sizeV0 - 1)) topHorizon[j]++;
    }
    for (int j = 0; j < sizeU0; j++) {
        while (!occupancyCC.get(bottomHorizon[j], j) && (bottomHorizon[j] < sizeV0 - 1)) bottomHorizon[j]++;
    }
    for (int i = 0; i < sizeV0; i++) {
        while (!occupancyCC.get(i, sizeU0 - 1 - rightHorizon[i]) && (rightHorizon[i] < sizeU0 - 1)) rightHorizon[i]++;
    }
    for (int i = 0; i < sizeV0; i++) {
        while (!occupancyCC.get(i, leftHorizon[i]) && (leftHorizon[i] < sizeU0 - 1)) leftHorizon[i]++;
    }
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
    std::cout << "Top Horizon :[";
    for (int i = 0; i < sizeU0; i++) { std::cout << topHorizon[i] << ","; }
    std::cout << "]" << std::endl;
    std::cout << "Bottom Horizon :[";
    for (int i = 0; i < sizeU0; i++) { std::cout << bottomHorizon[i] << ","; }
    std::cout << "]" << std::endl;
    std::cout << "Right Horizon :[";
    for (int i = 0; i < sizeV0; i++) { std::cout << rightHorizon[i] << ","; }
    std::cout << "]" << std::endl;
    std::cout << "Left Horizon :[";
    for (int i = 0; i < sizeV0; i++) { std::cout << leftHorizon[i] << ","; }
    std::cout << "]" << std::endl;
#endif
}

bool TextureParametrization::pack_patch_flexible(
    vmesh::Plane<uint8_t>& occupancy,
    vmesh::Plane<uint8_t>& occupancyCC,
    ConnectedComponent<MeshType>& packedCC,
    const VMCEncoderParameters& params, std::string keepFilesPathPrefix, bool invertDirection) {
    int occupancySizeV = occupancy.height();
    int occupancySizeU = occupancy.width();
    int widthOccCC = occupancyCC.width();
    int heightOccCC = occupancyCC.height();
    //now place the patch in the empty area
    // U0,V0 search
    for (int vIdx = 0; vIdx < occupancySizeV; ++vIdx) {
        int v = invertDirection ? occupancySizeV - 1 - vIdx : vIdx;
        for (int u = 0; u < occupancySizeU; ++u) {
            // orientation search
            for (int orientation = 0; orientation < 4; orientation++) {
                bool fit = true;
                for (int vv = 0; vv < heightOccCC && fit; vv++) {
                    for (int uu = 0; uu < widthOccCC && fit; uu++) {
                        int newV, newU;
                        switch (orientation) {
                        case 0: // no rotation
                            newV = v + vv;
                            newU = u + uu;
                            break;
                        case 1: // 90 degree
                            newV = v + uu;
                            newU = u + (heightOccCC - 1 - vv);
                            break;
                        case 2: // 180 degrees
                            newV = v + (heightOccCC - 1 - vv);
                            newU = u + (widthOccCC - 1 - uu);
                            break;
                        case 3: // 270 degrees
                            newV = v + (widthOccCC - 1 - uu);
                            newU = u + vv;
                            break;
                        default: // no rotation
                            newV = v + vv;
                            newU = u + uu;
                            break;
                        }
                        if ((newV >= occupancySizeV) || (newV < 0))
                            fit = false;
                        else if ((newU >= occupancySizeU) || (newU < 0))
                            fit = false;
                        else if (occupancy.get(newV, newU) && occupancyCC.get(vv, uu))
                            fit = false;
                    }
                }
                if (fit)
                {
                    //update the global occupancy map
                    for (int vv = 0; vv < heightOccCC; vv++) {
                        for (int uu = 0; uu < widthOccCC; uu++) {
                            int newV, newU;
                            switch (orientation) {
                            case 0: // no rotation
                                newV = v + vv;
                                newU = u + uu;
                                break;
                            case 1: // 90 degree
                                newV = v + uu;
                                newU = u + (heightOccCC - 1 - vv);
                                break;
                            case 2: // 180 degrees
                                newV = v + (heightOccCC - 1 - vv);
                                newU = u + (widthOccCC - 1 - uu);
                                break;
                            case 3: // 270 degrees
                                newV = v + (widthOccCC - 1 - uu);
                                newU = u + vv;
                                break;
                            default: // no rotation
                                newV = v + vv;
                                newU = u + uu;
                                break;
                            }
                            if (!occupancy.get(newV, newU))
                                occupancy.set(newV, newU, occupancyCC.get(vv, uu));
                        }
                    }
                    //update the UV coordinates of the connected components
                    // scale
                    for (auto& uv : packedCC.texCoords()) {
                        uv *= packedCC.getScale();
                    }
                    // gutter
                    for (auto& uv : packedCC.texCoords()) {
                        uv += Vec2<MeshType>(params.gutter, params.gutter);
                    }
                    // rotation
                    for (auto& uv : packedCC.texCoords()) {
                        MeshType uu = uv[0];
                        MeshType vv = uv[1];
                        switch (orientation) {
                        case 0: // no rotation
                            break;
                        case 1: // 90 degree
                            uv[1] = uu;
                            uv[0] = (heightOccCC * params.displacementVideoBlockSize - vv);
                            break;
                        case 2: // 180 degrees
                            uv[1] = (heightOccCC * params.displacementVideoBlockSize - vv);
                            uv[0] = (widthOccCC * params.displacementVideoBlockSize - uu);
                            break;
                        case 3: // 270 degrees
                            uv[1] = (widthOccCC * params.displacementVideoBlockSize - uu);
                            uv[0] = vv;
                            break;
                        default: // no rotation
                            uv[1] = vv;
                            uv[0] = uu;
                            break;
                        }
                    }
                    packedCC.setOrientation(orientation);
                    // translation
                    for (auto& uv : packedCC.texCoords()) {
                        uv += Vec2<MeshType>(u * params.displacementVideoBlockSize, v * params.displacementVideoBlockSize);
                    }
                    packedCC.setU0(u);
                    packedCC.setV0(v);
                    packedCC.setSizeU(widthOccCC);
                    packedCC.setSizeV(heightOccCC);
#if DEBUG_ORTHO_PATCH_PACKING
                    std::cout << "CC[" << packedCC.getIdxPatch() <<
                        "] = (#tri: " << packedCC.triangles().size() <<
                        ", #vert: " << packedCC.points().size() <<
                        ", Area: " << packedCC.area() <<
                        ", Perimeter: " << packedCC.perimeter() <<
                        ", P: " << packedCC.getProjection() <<
                        ", S: " << packedCC.getScale() <<
                        ", O: " << packedCC.getOrientation() <<
                        ", U0: " << packedCC.getU0() <<
                        ", V0: " << packedCC.getV0() <<
                        ", SizeU: " << packedCC.getSizeU() <<
                        ", SizeV: " << packedCC.getSizeV() <<
                        ")" << std::endl;
                    if (params.keepIntermediateFiles) {
                        // dump of the current state of the occupancy
                        vmesh::Frame<uint8_t> occupancyVideo;
                        occupancyVideo.resize(occupancySizeU, occupancySizeV, vmesh::ColourSpace::BGR444p);
                        occupancyVideo.plane(0) = occupancy;
                        for (auto& pixel : occupancyVideo.plane(0).buffer()) {
                            pixel *= 255;
                        }
                        int deltaVV = std::max(1, (heightOccCC - 1));
                        for (int vv = 0; vv < heightOccCC; vv += deltaVV) {
                            for (int uu = 0; uu < widthOccCC; uu++) {
                                int newV, newU;
                                switch (orientation) {
                                case 0: // no rotation
                                    newV = v + vv;
                                    newU = u + uu;
                                    break;
                                case 1: // 90 degree
                                    newV = v + uu;
                                    newU = u + (heightOccCC - 1 - vv);
                                    break;
                                case 2: // 180 degrees
                                    newV = v + (heightOccCC - 1 - vv);
                                    newU = u + (widthOccCC - 1 - uu);
                                    break;
                                case 3: // 270 degrees
                                    newV = v + (widthOccCC - 1 - uu);
                                    newU = u + vv;
                                    break;
                                default: // no rotation
                                    newV = v + vv;
                                    newU = u + uu;
                                    break;
                                }
                                occupancyVideo.plane(1).set(newV, newU, 255);
                            }
                        }
                        int deltaUU = std::max(1, (widthOccCC - 1));
                        for (int vv = 0; vv < heightOccCC; vv++) {
                            for (int uu = 0; uu < widthOccCC; uu += deltaUU) {
                                int newV, newU;
                                switch (orientation) {
                                case 0: // no rotation
                                    newV = v + vv;
                                    newU = u + uu;
                                    break;
                                case 1: // 90 degree
                                    newV = v + uu;
                                    newU = u + (heightOccCC - 1 - vv);
                                    break;
                                case 2: // 180 degrees
                                    newV = v + (heightOccCC - 1 - vv);
                                    newU = u + (widthOccCC - 1 - uu);
                                    break;
                                case 3: // 270 degrees
                                    newV = v + (widthOccCC - 1 - uu);
                                    newU = u + vv;
                                    break;
                                default: // no rotation
                                    newV = v + vv;
                                    newU = u + uu;
                                    break;
                                }
                                occupancyVideo.plane(1).set(newV, newU, 255);
                            }
                        }
                        auto prefix = keepFilesPathPrefix + "_occupancy" + std::to_string(occupancySizeU) + "x" + std::to_string(occupancySizeV) + "_bgrp";
                        occupancyVideo.append(prefix + ".rgb");
                    }
#endif
                    return true;
                }
            }
        }
    }
    return false;
}

bool TextureParametrization::pack_patch_flexible_tetris(
    vmesh::Plane<uint8_t>& occupancy,
    vmesh::Plane<uint8_t>& occupancyCC,
    ConnectedComponent<MeshType>& packedCC,
    const VMCEncoderParameters& params,
    std::string keepFilesPathPrefix,
    std::vector<int>& horizon,
    std::vector<int>& topHorizon,
    std::vector<int>& bottomHorizon,
    std::vector<int>& rightHorizon,
    std::vector<int>& leftHorizon) {
    int occupancySizeV = occupancy.height();
    int occupancySizeU = occupancy.width();
    int widthOccCC = occupancyCC.width();
    int heightOccCC = occupancyCC.height();
    //now place the patch in the empty area
    // U0,V0 search
    bool foundPos = false;
    int bestU = -1;
    int bestV = -1;
    int bestOrientation = -1;
    int best_wasted_space = (std::numeric_limits<int>::max)();
    int minHorizon = std::numeric_limits<int32_t>::max();
    int maxHorizon = 0;
    for (auto& horVal : horizon) {
        if (horVal < minHorizon) minHorizon = horVal;
        if (horVal > maxHorizon) maxHorizon = horVal;
    }
    // orientation search
    for (int orientation = 0; orientation < 4; orientation++) {
        int minV = std::numeric_limits<int32_t>::max(); ;
        switch (orientation) {
        case 0: // no rotation
            for (int idx = 0; idx < widthOccCC; idx++) minV = std::max<int32_t>(0, std::min<int32_t>(minV, minHorizon - bottomHorizon[idx]));
            break;
        case 1: // 90 degree
            for (int idx = 0; idx < heightOccCC; idx++) minV = std::max<int32_t>(0, std::min<int32_t>(minV, minHorizon - leftHorizon[idx]));
            break;
        case 2: // 180 degrees
            for (int idx = 0; idx < widthOccCC; idx++) minV = std::max<int32_t>(0, std::min<int32_t>(minV, minHorizon - (heightOccCC - topHorizon[idx])));
            break;
        case 3: // 270 degrees
            for (int idx = 0; idx < heightOccCC; idx++) minV = std::max<int32_t>(0, std::min<int32_t>(minV, minHorizon - (widthOccCC - rightHorizon[idx])));
            break;
        default: // no rotation
            for (int idx = 0; idx < widthOccCC; idx++) minV = std::max<int32_t>(0, std::min<int32_t>(minV, minHorizon - bottomHorizon[idx]));
            break;
        }
        for (int v = minV; v < maxHorizon + 1; ++v) {
            for (int u = 0; u < occupancySizeU; ++u) {
                //check if the patch bounding box fit the canvas
                int maxV, maxU;
                switch (orientation) {
                case 0: // no rotation
                    maxV = v + heightOccCC - 1;
                    maxU = u + widthOccCC - 1;
                    break;
                case 1: // 90 degree
                    maxV = v + widthOccCC - 1;
                    maxU = u + heightOccCC - 1;
                    break;
                case 2: // 180 degrees
                    maxV = v + heightOccCC - 1;
                    maxU = u + widthOccCC - 1;
                    break;
                case 3: // 270 degrees
                    maxV = v + widthOccCC - 1;
                    maxU = u + heightOccCC - 1;
                    break;
                default: // no rotation
                    maxV = v + heightOccCC - 1;
                    maxU = u + widthOccCC - 1;
                    break;
                }
                if ((maxV >= occupancySizeV) || (maxU >= occupancySizeU))
                    continue;
                //check if the patch is above the horizon
                bool isAboveHorizon = true;
                switch (orientation) {
                case 0: // no rotation
                    for (int idx = 0; idx < widthOccCC && isAboveHorizon; idx++) {
                        if (v + bottomHorizon[idx] < horizon[u + idx]) { isAboveHorizon = false; }
                    }
                    break;
                case 1: // 90 degree
                    for (int idx = 0; idx < heightOccCC && isAboveHorizon; idx++) {
                        if (v + leftHorizon[heightOccCC - 1 - idx] < horizon[u + idx]) { isAboveHorizon = false; }
                    }
                    break;
                case 2: // 180 degrees
                    for (int idx = 0; idx < widthOccCC && isAboveHorizon; idx++) {
                        if (v + topHorizon[widthOccCC - 1 - idx] < horizon[u + idx]) { isAboveHorizon = false; }
                    }
                    break;
                case 3: // 270 degrees
                    for (int idx = 0; idx < heightOccCC && isAboveHorizon; idx++) {
                        if (v + rightHorizon[idx] < horizon[u + idx]) { isAboveHorizon = false; }
                    }
                    break;
                default: // no rotation
                    for (int idx = 0; idx < widthOccCC && isAboveHorizon; idx++) {
                        if (v + bottomHorizon[idx] < horizon[u + idx]) { isAboveHorizon = false; }
                    }
                    break;
                }                //check if the patch can fit on space above the horizon
                if (!isAboveHorizon)
                    continue;
                bool fit = true;
                for (int vv = 0; vv < heightOccCC && fit; vv++) {
                    for (int uu = 0; uu < widthOccCC && fit; uu++) {
                        int newV, newU;
                        switch (orientation) {
                        case 0: // no rotation
                            newV = v + vv;
                            newU = u + uu;
                            break;
                        case 1: // 90 degree
                            newV = v + uu;
                            newU = u + (heightOccCC - 1 - vv);
                            break;
                        case 2: // 180 degrees
                            newV = v + (heightOccCC - 1 - vv);
                            newU = u + (widthOccCC - 1 - uu);
                            break;
                        case 3: // 270 degrees
                            newV = v + (widthOccCC - 1 - uu);
                            newU = u + vv;
                            break;
                        default: // no rotation
                            newV = v + vv;
                            newU = u + uu;
                            break;
                        }
                        if ((newV >= occupancySizeV) || (newV < 0))
                            fit = false;
                        else if ((newU >= occupancySizeU) || (newU < 0))
                            fit = false;
                        else if (occupancy.get(newV, newU) && occupancyCC.get(vv, uu))
                            fit = false;
                    }
                }
                if (fit) {
                    //calculate the wasted space
                    int wasted_space = 0;
                    int extension_horizon = 0;
                    int maxDelta = 0;
                    int wasted_space_external = 0;
                    int wasted_space_internal = 0;
                    int lambda = 100;  //--> bias towards the upper part of the canvas
                    switch (orientation) {
                    case 0: // no rotation
                        for (int idx = 0; idx < widthOccCC; idx++) {
                            //check how much horizon is streched
                            int delta = (v + heightOccCC - 1 - topHorizon[idx]) - horizon[u + idx];
                            if (delta > 0) extension_horizon += delta;
                            if ((delta > 0) && ((delta + horizon[u + idx]) > maxDelta)) maxDelta = delta + horizon[u + idx];
                            //from the horizon to the patch
                            wasted_space_external += v + bottomHorizon[idx] - horizon[u + idx];
                            // calculating internal wasted space --> because of new block2patch
                            // restriction, this area only contains locations for the local patch
                            for (int idx2 = bottomHorizon[idx] + 1; idx2 < heightOccCC - topHorizon[idx]; idx2++) {
                                if (!occupancyCC.get(idx2, idx)) wasted_space_internal++;
                            }
                        }
                        wasted_space = int(lambda * maxDelta + extension_horizon + wasted_space_external + wasted_space_internal);
                        break;
                    case 1: // 90 degree
                        for (int idx = 0; idx < heightOccCC; idx++) {
                            //check how much horizon is streched
                            int delta = (v + widthOccCC - 1 - rightHorizon[heightOccCC - 1 - idx]) - horizon[u + idx];
                            if (delta > 0) extension_horizon += delta;
                            if ((delta > 0) && ((delta + horizon[u + idx]) > maxDelta)) maxDelta = delta + horizon[u + idx];
                            //from the horizon to the patch
                            wasted_space_external += v + leftHorizon[heightOccCC - 1 - idx] - horizon[u + idx];
                            // calculating internal wasted space
                            for (int idx2 = int(widthOccCC - 1 - rightHorizon[idx]); idx2 >= leftHorizon[idx]; idx2--) {
                                if (!occupancyCC.get(idx, idx2)) wasted_space_internal++;
                            }
                        }
                        wasted_space = int(lambda * maxDelta + extension_horizon + wasted_space_external + wasted_space_internal);
                        break;
                    case 2: // 180 degrees
                        for (int idx = 0; idx < widthOccCC; idx++) {
                            //check how much horizon is streched
                            int delta = (v + heightOccCC - 1 - bottomHorizon[widthOccCC - 1 - idx]) - horizon[u + idx];
                            if (delta > 0) extension_horizon += delta;
                            if ((delta > 0) && ((delta + horizon[u + idx]) > maxDelta)) maxDelta = delta + horizon[u + idx];
                            //from the horizon to the patch
                            wasted_space_external += v + topHorizon[widthOccCC - 1 - idx] - horizon[u + idx];
                            // calculating internal wasted space
                            for (int idx2 = int(heightOccCC - 1 - topHorizon[idx]); idx2 >= bottomHorizon[idx]; idx2--) {
                                if (!occupancyCC.get(idx2, idx)) wasted_space_internal++;
                            }
                        }
                        wasted_space = int(lambda * maxDelta + extension_horizon + wasted_space_external + wasted_space_internal);
                        break;
                    case 3: // 270 degrees
                        for (int idx = 0; idx < heightOccCC; idx++) {
                            //check how much horizon is streched
                            int delta = (v + widthOccCC - 1 - leftHorizon[idx]) - horizon[u + idx];
                            if (delta > 0) extension_horizon += delta;
                            if ((delta > 0) && ((delta + horizon[u + idx]) > maxDelta)) maxDelta = delta + horizon[u + idx];
                            //from the horizon to the patch
                            wasted_space_external += v + rightHorizon[idx] - horizon[u + idx];
                            // calculating internal wasted space
                            for (int idx2 = leftHorizon[idx] + 1; idx2 < widthOccCC - rightHorizon[idx]; idx2++) {
                                if (!occupancyCC.get(idx, idx2)) wasted_space_internal++;
                            }
                        }
                        wasted_space = int(lambda * maxDelta + extension_horizon + wasted_space_external + wasted_space_internal);
                        break;
                    default: // no rotation
                        for (int idx = 0; idx < widthOccCC; idx++) {
                            //check how much horizon is streched
                            int delta = (v + widthOccCC - 1 - topHorizon[idx]) - horizon[u + idx];
                            if (delta > 0) extension_horizon += delta;
                            if ((delta > 0) && ((delta + horizon[u + idx]) > maxDelta)) maxDelta = delta + horizon[u + idx];
                            //from the horizon to the patch
                            wasted_space_external += v + bottomHorizon[idx] - horizon[u + idx];
                            // calculating internal wasted space --> because of new block2patch
                            // restriction, this area only contains locations for the local patch
                            for (int idx2 = bottomHorizon[idx] + 1; idx2 < widthOccCC - topHorizon[idx]; idx2++) {
                                if (!occupancyCC.get(idx2, idx)) wasted_space_internal++;
                            }
                        }
                        wasted_space = int(lambda * maxDelta + extension_horizon + wasted_space_external + wasted_space_internal);
                        break;
                    }
                    //now save the position with less wasted space
                    if (wasted_space < best_wasted_space) {
                        best_wasted_space = wasted_space;
                        bestU = u;
                        bestV = v;
                        bestOrientation = orientation;
                        foundPos = true;
#if DEBUG_ORTHO_PATCH_PACKING
                        if (params.keepIntermediateFiles) {
                            // dump of the current state of the occupancy
                            vmesh::Frame<uint8_t> occupancyVideo;
                            occupancyVideo.resize(occupancySizeU, occupancySizeV, vmesh::ColourSpace::BGR444p);
                            occupancyVideo.plane(0) = occupancy;
                            for (auto& pixel : occupancyVideo.plane(0).buffer()) {
                                pixel *= 255;
                            }
                            //copy patch
                                    //update the global occupancy map
                            for (int vv = 0; vv < heightOccCC; vv++) {
                                for (int uu = 0; uu < widthOccCC; uu++) {
                                    int newV, newU;
                                    switch (bestOrientation) {
                                    case 0: // no rotation
                                        newV = bestV + vv;
                                        newU = bestU + uu;
                                        break;
                                    case 1: // 90 degree
                                        newV = bestV + uu;
                                        newU = bestU + (heightOccCC - 1 - vv);
                                        break;
                                    case 2: // 180 degrees
                                        newV = bestV + (heightOccCC - 1 - vv);
                                        newU = bestU + (widthOccCC - 1 - uu);
                                        break;
                                    case 3: // 270 degrees
                                        newV = bestV + (widthOccCC - 1 - uu);
                                        newU = bestU + vv;
                                        break;
                                    default: // no rotation
                                        newV = bestV + vv;
                                        newU = bestU + uu;
                                        break;
                                    }
                                    occupancyVideo.plane(1).set(newV, newU, 255 * occupancyCC.get(vv, uu));
                                }
                            }
                            int deltaVV = std::max(1, (heightOccCC - 1));
                            for (int vv = 0; vv < heightOccCC; vv += deltaVV) {
                                for (int uu = 0; uu < widthOccCC; uu++) {
                                    int newV, newU;
                                    switch (bestOrientation) {
                                    case 0: // no rotation
                                        newV = bestV + vv;
                                        newU = bestU + uu;
                                        break;
                                    case 1: // 90 degree
                                        newV = bestV + uu;
                                        newU = bestU + (heightOccCC - 1 - vv);
                                        break;
                                    case 2: // 180 degrees
                                        newV = bestV + (heightOccCC - 1 - vv);
                                        newU = bestU + (widthOccCC - 1 - uu);
                                        break;
                                    case 3: // 270 degrees
                                        newV = bestV + (widthOccCC - 1 - uu);
                                        newU = bestU + vv;
                                        break;
                                    default: // no rotation
                                        newV = bestV + vv;
                                        newU = bestU + uu;
                                        break;
                                    }
                                    occupancyVideo.plane(1).set(newV, newU, 255);
                                }
                            }
                            int deltaUU = std::max(1, (widthOccCC - 1));
                            for (int vv = 0; vv < heightOccCC; vv++) {
                                for (int uu = 0; uu < widthOccCC; uu += deltaUU) {
                                    int newV, newU;
                                    switch (bestOrientation) {
                                    case 0: // no rotation
                                        newV = bestV + vv;
                                        newU = bestU + uu;
                                        break;
                                    case 1: // 90 degree
                                        newV = bestV + uu;
                                        newU = bestU + (heightOccCC - 1 - vv);
                                        break;
                                    case 2: // 180 degrees
                                        newV = bestV + (heightOccCC - 1 - vv);
                                        newU = bestU + (widthOccCC - 1 - uu);
                                        break;
                                    case 3: // 270 degrees
                                        newV = bestV + (widthOccCC - 1 - uu);
                                        newU = bestU + vv;
                                        break;
                                    default: // no rotation
                                        newV = bestV + vv;
                                        newU = bestU + uu;
                                        break;
                                    }
                                    occupancyVideo.plane(1).set(newV, newU, 255);
                                }
                            }
                            for (int idx = 0; idx < occupancySizeU; idx++) {
                                occupancyVideo.plane(0).set(horizon[idx], idx, 0);
                                occupancyVideo.plane(1).set(horizon[idx], idx, 0);
                                occupancyVideo.plane(2).set(horizon[idx], idx, 255);
                            }
                            auto prefix = keepFilesPathPrefix + "_patch" + std::to_string(packedCC.getIdxPatch()) + "_occupancy" + std::to_string(occupancySizeU) + "x" + std::to_string(occupancySizeV) + "_bgrp";
                            occupancyVideo.append(prefix + ".rgb");
                        }
#endif
                    }

                }
            }
        }
    }
    if (foundPos)
    {
        //update the global occupancy map
        for (int vv = 0; vv < heightOccCC; vv++) {
            for (int uu = 0; uu < widthOccCC; uu++) {
                int newV, newU;
                switch (bestOrientation) {
                case 0: // no rotation
                    newV = bestV + vv;
                    newU = bestU + uu;
                    break;
                case 1: // 90 degree
                    newV = bestV + uu;
                    newU = bestU + (heightOccCC - 1 - vv);
                    break;
                case 2: // 180 degrees
                    newV = bestV + (heightOccCC - 1 - vv);
                    newU = bestU + (widthOccCC - 1 - uu);
                    break;
                case 3: // 270 degrees
                    newV = bestV + (widthOccCC - 1 - uu);
                    newU = bestU + vv;
                    break;
                default: // no rotation
                    newV = bestV + vv;
                    newU = bestU + uu;
                    break;
                }
                if (!occupancy.get(newV, newU))
                    occupancy.set(newV, newU, occupancyCC.get(vv, uu));
            }
        }
        //update the UV coordinates of the connected components
        // scale
        for (auto& uv : packedCC.texCoords()) {
            uv *= packedCC.getScale();
        }
        // gutter
        for (auto& uv : packedCC.texCoords()) {
            uv += Vec2<MeshType>(params.gutter, params.gutter);
        }
        // rotation
        for (auto& uv : packedCC.texCoords()) {
            MeshType uu = uv[0];
            MeshType vv = uv[1];
            switch (bestOrientation) {
            case 0: // no rotation
                break;
            case 1: // 90 degree
                uv[1] = uu;
                uv[0] = (heightOccCC * params.displacementVideoBlockSize - vv);
                break;
            case 2: // 180 degrees
                uv[1] = (heightOccCC * params.displacementVideoBlockSize - vv);
                uv[0] = (widthOccCC * params.displacementVideoBlockSize - uu);
                break;
            case 3: // 270 degrees
                uv[1] = (widthOccCC * params.displacementVideoBlockSize - uu);
                uv[0] = vv;
                break;
            default: // no rotation
                uv[1] = vv;
                uv[0] = uu;
                break;
            }
        }
        packedCC.setOrientation(bestOrientation);
        // translation
        for (auto& uv : packedCC.texCoords()) {
            uv += Vec2<MeshType>(bestU * params.displacementVideoBlockSize, bestV * params.displacementVideoBlockSize);
        }
        packedCC.setU0(bestU);
        packedCC.setV0(bestV);
        packedCC.setSizeU(widthOccCC);
        packedCC.setSizeV(heightOccCC);
#if DEBUG_ORTHO_PATCH_PACKING
        std::cout << "CC[" << packedCC.getIdxPatch() <<
            "] = (#tri: " << packedCC.triangles().size() <<
            ", #vert: " << packedCC.points().size() <<
            ", Area: " << packedCC.area() <<
            ", Perimeter: " << packedCC.perimeter() <<
            ", P: " << packedCC.getProjection() <<
            ", S: " << packedCC.getScale() <<
            ", O: " << packedCC.getOrientation() <<
            ", U0: " << packedCC.getU0() <<
            ", V0: " << packedCC.getV0() <<
            ", SizeU: " << packedCC.getSizeU() <<
            ", SizeV: " << packedCC.getSizeV() <<
            ")" << std::endl;
        if (params.keepIntermediateFiles) {
            // dump of the current state of the occupancy
            vmesh::Frame<uint8_t> occupancyVideo;
            occupancyVideo.resize(occupancySizeU, occupancySizeV, vmesh::ColourSpace::BGR444p);
            occupancyVideo.plane(0) = occupancy;
            for (auto& pixel : occupancyVideo.plane(0).buffer()) {
                pixel *= 255;
            }
            int deltaVV = std::max(1, (heightOccCC - 1));
            for (int vv = 0; vv < heightOccCC; vv += deltaVV) {
                for (int uu = 0; uu < widthOccCC; uu++) {
                    int newV, newU;
                    switch (bestOrientation) {
                    case 0: // no rotation
                        newV = bestV + vv;
                        newU = bestU + uu;
                        break;
                    case 1: // 90 degree
                        newV = bestV + uu;
                        newU = bestU + (heightOccCC - 1 - vv);
                        break;
                    case 2: // 180 degrees
                        newV = bestV + (heightOccCC - 1 - vv);
                        newU = bestU + (widthOccCC - 1 - uu);
                        break;
                    case 3: // 270 degrees
                        newV = bestV + (widthOccCC - 1 - uu);
                        newU = bestU + vv;
                        break;
                    default: // no rotation
                        newV = bestV + vv;
                        newU = bestU + uu;
                        break;
                    }
                    occupancyVideo.plane(1).set(newV, newU, 255);
                }
            }
            int deltaUU = std::max(1, (widthOccCC - 1));
            for (int vv = 0; vv < heightOccCC; vv++) {
                for (int uu = 0; uu < widthOccCC; uu += deltaUU) {
                    int newV, newU;
                    switch (bestOrientation) {
                    case 0: // no rotation
                        newV = bestV + vv;
                        newU = bestU + uu;
                        break;
                    case 1: // 90 degree
                        newV = bestV + uu;
                        newU = bestU + (heightOccCC - 1 - vv);
                        break;
                    case 2: // 180 degrees
                        newV = bestV + (heightOccCC - 1 - vv);
                        newU = bestU + (widthOccCC - 1 - uu);
                        break;
                    case 3: // 270 degrees
                        newV = bestV + (widthOccCC - 1 - uu);
                        newU = bestU + vv;
                        break;
                    default: // no rotation
                        newV = bestV + vv;
                        newU = bestU + uu;
                        break;
                    }
                    occupancyVideo.plane(1).set(newV, newU, 255);
                }
            }
            for (int idx = 0; idx < occupancySizeU; idx++) {
                occupancyVideo.plane(0).set(horizon[idx], idx, 0);
                occupancyVideo.plane(1).set(horizon[idx], idx, 0);
                occupancyVideo.plane(2).set(horizon[idx], idx, 255);
            }
            auto prefix = keepFilesPathPrefix + "_occupancy" + std::to_string(occupancySizeU) + "x" + std::to_string(occupancySizeV) + "_bgrp";
            occupancyVideo.append(prefix + ".rgb");
        }
#endif
        return true;
    }
    return false;
}

bool TextureParametrization::pack_patch_flexible_with_temporal_stability(
    vmesh::Plane<uint8_t>& occupancy,
    vmesh::Plane<uint8_t>& occupancyCC,
    ConnectedComponent<MeshType>& packedCC,
    ConnectedComponent<MeshType>& matchedCC,
    const VMCEncoderParameters& params, std::string keepFilesPathPrefix) {
    int occupancySizeV = occupancy.height();
    int occupancySizeU = occupancy.width();
    int widthOccCC = occupancyCC.width();
    int heightOccCC = occupancyCC.height();
    // now place the patch in the empty area, maintaining the patch orientation
    int orientation = matchedCC.getOrientation();
    // spiral search to find the closest available position
    int x = 0;
    int y = 0;
    int end = (std::max)(occupancySizeU, occupancySizeV) * (std::max)(occupancySizeU, occupancySizeV) * 4;
    for (int i = 0; i < end; ++i) {
        // Translate coordinates and mask them out.
        int u = x + matchedCC.getU0();
        int v = y + matchedCC.getV0();
        bool fit = true;
        for (int vv = 0; vv < heightOccCC && fit; vv++) {
            for (int uu = 0; uu < widthOccCC && fit; uu++) {
                int newV, newU;
                switch (orientation) {
                case 0: // no rotation
                    newV = v + vv;
                    newU = u + uu;
                    break;
                case 1: // 90 degree
                    newV = v + uu;
                    newU = u + (heightOccCC - 1 - vv);
                    break;
                case 2: // 180 degrees
                    newV = v + (heightOccCC - 1 - vv);
                    newU = u + (widthOccCC - 1 - uu);
                    break;
                case 3: // 270 degrees
                    newV = v + (widthOccCC - 1 - uu);
                    newU = u + vv;
                    break;
                default: // no rotation
                    newV = v + vv;
                    newU = u + uu;
                    break;
                }
                if ((newV >= occupancySizeV) || (newV < 0))
                    fit = false;
                else if ((newU >= occupancySizeU) || (newU < 0))
                    fit = false;
                else if (occupancy.get(newV, newU) && occupancyCC.get(vv, uu))
                    fit = false;
            }
        }
        if (fit)
        {
            //update the global occupancy map
            for (int vv = 0; vv < heightOccCC; vv++) {
                for (int uu = 0; uu < widthOccCC; uu++) {
                    int newV, newU;
                    switch (orientation) {
                    case 0: // no rotation
                        newV = v + vv;
                        newU = u + uu;
                        break;
                    case 1: // 90 degree
                        newV = v + uu;
                        newU = u + (heightOccCC - 1 - vv);
                        break;
                    case 2: // 180 degrees
                        newV = v + (heightOccCC - 1 - vv);
                        newU = u + (widthOccCC - 1 - uu);
                        break;
                    case 3: // 270 degrees
                        newV = v + (widthOccCC - 1 - uu);
                        newU = u + vv;
                        break;
                    default: // no rotation
                        newV = v + vv;
                        newU = u + uu;
                        break;
                    }
                    if (!occupancy.get(newV, newU))
                        occupancy.set(newV, newU, occupancyCC.get(vv, uu));
                }
            }
            //update the UV coordinates of the connected components
            // scale
            for (auto& uv : packedCC.texCoords()) {
                uv *= packedCC.getScale();
            }
            // gutter
            for (auto& uv : packedCC.texCoords()) {
                uv += Vec2<MeshType>(params.gutter, params.gutter);
            }
            // rotation
            for (auto& uv : packedCC.texCoords()) {
                MeshType uu = uv[0];
                MeshType vv = uv[1];
                switch (orientation) {
                case 0: // no rotation
                    break;
                case 1: // 90 degree
                    uv[1] = uu;
                    uv[0] = (heightOccCC * params.displacementVideoBlockSize - vv);
                    break;
                case 2: // 180 degrees
                    uv[1] = (heightOccCC * params.displacementVideoBlockSize - vv);
                    uv[0] = (widthOccCC * params.displacementVideoBlockSize - uu);
                    break;
                case 3: // 270 degrees
                    uv[1] = (widthOccCC * params.displacementVideoBlockSize - uu);
                    uv[0] = vv;
                    break;
                default: // no rotation
                    uv[1] = vv;
                    uv[0] = uu;
                    break;
                }
            }
            packedCC.setOrientation(orientation);
            // translation
            for (auto& uv : packedCC.texCoords()) {
                uv += Vec2<MeshType>(u * params.displacementVideoBlockSize, v * params.displacementVideoBlockSize);
            }
            packedCC.setU0(u);
            packedCC.setV0(v);
            packedCC.setSizeU(widthOccCC);
            packedCC.setSizeV(heightOccCC);
#if DEBUG_ORTHO_PATCH_PACKING            
            std::cout << "CC[" << packedCC.getIdxPatch() <<
                ",m:" << packedCC.getRefPatch() <<
                "] = (#tri: " << packedCC.triangles().size() <<
                ", #vert: " << packedCC.points().size() <<
                ", Area: " << packedCC.area() <<
                ", Perimeter: " << packedCC.perimeter() <<
                ", P: " << packedCC.getProjection() <<
                ", S: " << packedCC.getScale() <<
                ", O: " << packedCC.getOrientation() <<
                ", U0: " << packedCC.getU0() <<
                ", V0: " << packedCC.getV0() <<
                ", SizeU: " << packedCC.getSizeU() <<
                ", SizeV: " << packedCC.getSizeV() <<
                ")" << std::endl;
            if (params.keepIntermediateFiles) {
                // dump of the current state of the occupancy
                vmesh::Frame<uint8_t> occupancyVideo;
                occupancyVideo.resize(occupancySizeU, occupancySizeV, vmesh::ColourSpace::BGR444p);
                occupancyVideo.plane(0) = occupancy;
                for (auto& pixel : occupancyVideo.plane(0).buffer()) {
                    pixel *= 255;
                }
                int deltaVV = std::max(1, (heightOccCC - 1));
                for (int vv = 0; vv < heightOccCC; vv += deltaVV) {
                    for (int uu = 0; uu < widthOccCC; uu++) {
                        int newV, newU;
                        switch (orientation) {
                        case 0: // no rotation
                            newV = v + vv;
                            newU = u + uu;
                            break;
                        case 1: // 90 degree
                            newV = v + uu;
                            newU = u + (heightOccCC - 1 - vv);
                            break;
                        case 2: // 180 degrees
                            newV = v + (heightOccCC - 1 - vv);
                            newU = u + (widthOccCC - 1 - uu);
                            break;
                        case 3: // 270 degrees
                            newV = v + (widthOccCC - 1 - uu);
                            newU = u + vv;
                            break;
                        default: // no rotation
                            newV = v + vv;
                            newU = u + uu;
                            break;
                        }
                        occupancyVideo.plane(1).set(newV, newU, 255);
                    }
                }
                int deltaUU = std::max(1, (widthOccCC - 1));
                for (int vv = 0; vv < heightOccCC; vv++) {
                    for (int uu = 0; uu < widthOccCC; uu += deltaUU) {
                        int newV, newU;
                        switch (orientation) {
                        case 0: // no rotation
                            newV = v + vv;
                            newU = u + uu;
                            break;
                        case 1: // 90 degree
                            newV = v + uu;
                            newU = u + (heightOccCC - 1 - vv);
                            break;
                        case 2: // 180 degrees
                            newV = v + (heightOccCC - 1 - vv);
                            newU = u + (widthOccCC - 1 - uu);
                            break;
                        case 3: // 270 degrees
                            newV = v + (widthOccCC - 1 - uu);
                            newU = u + vv;
                            break;
                        default: // no rotation
                            newV = v + vv;
                            newU = u + uu;
                            break;
                        }
                        occupancyVideo.plane(1).set(newV, newU, 255);
                    }
                }
                auto prefix = keepFilesPathPrefix + "_occupancy" + std::to_string(occupancySizeU) + "x" + std::to_string(occupancySizeV) + "_bgrp";
                occupancyVideo.append(prefix + ".rgb");
            }
#endif
            return true;
        }
        else {
            if (abs(x) <= abs(y) && (x != y || x >= 0)) {
                x += ((y >= 0) ? 1 : -1);
            }
            else {
                y += ((x >= 0) ? -1 : 1);
            }
        }
    }
    return false;
}

bool
TextureParametrization::pack_patches_with_patch_scale_and_rotation(
    TriangleMesh<MeshType>& mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>& projectedConnectedComponentsCategories,
    std::vector<ConnectedComponent<MeshType>>& packedCCList,
    const VMCEncoderParameters& params,
    double scalingAdjustment, std::string keepFilesPathPrefix) {
    //pack patches into the atlas image (global UV)
    const double avgAreaNumVertRatio = mesh.area() / (double)mesh.pointCount();
    const auto bbMesh = mesh.boundingBox();
    double frameScaleProjection = (double)(params.height - 2 * params.gutter) / (double)(params.displacementVideoBlockSize * ceil((bbMesh.max - bbMesh.min).normInf() / params.displacementVideoBlockSize));
    frameScaleProjection *= scalingAdjustment; // leaving some room for adjustments
    vmesh::Plane<uint8_t> occupancy;
    int occupancySizeV = std::ceil((double)params.height / (double)params.displacementVideoBlockSize);
    int occupancySizeU = std::ceil((double)params.width / (double)params.displacementVideoBlockSize);
    occupancy.resize(occupancySizeU, occupancySizeV);
    occupancy.fill(false);
    int ccIdx = 0;
    // load UV coordinates
    for (int idx = 0; idx < projectedCCList.size(); idx++) {
        auto& ccMesh = projectedCCList[idx];
        vmesh::ConnectedComponent<MeshType> cc;
        //copy mesh
        cc.triangles() = ccMesh->triangles();
        cc.points() = ccMesh->points();
        cc.texCoords() = ccMesh->texCoords();
        cc.texCoordTriangles() = ccMesh->texCoordTriangles();
        // set projection and initial scale
        cc.setProjection(projectedConnectedComponentsCategories[idx]);
        cc.setFrameScale(frameScaleProjection);
        cc.setScale(frameScaleProjection);
        packedCCList.push_back(cc);
    }
    //sort list to start packing from largest cc to smallest
    std::sort(packedCCList.begin(), packedCCList.end(), [&](ConnectedComponent<MeshType> a, ConnectedComponent<MeshType> b) {
        return (a.area() > b.area());
        });
    // obtain the scaling of each connected component 
    for (auto& packedCC : packedCCList) {
        if (params.bPatchScaling) {
            double ccAreaNumVerRatio = packedCC.area() / (double)packedCC.pointCount();
            if (ccAreaNumVerRatio < (avgAreaNumVertRatio * 0.5)) {
                packedCC.setScale(packedCC.getScale() * 2.0);
            }
            else {
                packedCC.setScale(packedCC.getScale() * 1.0);
            }
        }
        else {
            packedCC.setScale(packedCC.getScale() * 1.0);
        }
    }
    // pack loop
    for (int idxPack = 0; idxPack < packedCCList.size(); idxPack++) {
        auto& packedCC = packedCCList[idxPack];
        // set index and reference
        packedCC.setIdxPatch(idxPack);
        packedCC.setRefPatch(-1);
        //rasterize the patch
        vmesh::Plane<uint8_t> occupancyCC;
        patch_rasterization(occupancyCC, packedCC, params);
        //now place the patch in the empty area
        bool invertDirection = (packedCC.area() < (packedCCList[0].area() * 0.01)) && params.packSmallPatchesOnTop;
        if (!pack_patch_flexible(occupancy, occupancyCC, packedCC, params, keepFilesPathPrefix, invertDirection))
        {
            // do the cc again and reduce it's size
            packedCC.setScale(packedCC.getScale() * params.packingScaling);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
            std::cout << "Reducing patch scale by " << (1 - params.packingScaling) * 100 << "% (" << packedCC.getScale() <<
                ")" << std::endl;
#endif
            if ((packedCC.getScale() / frameScaleProjection) < 0.5) {
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
                std::cout << "Connected Component using small scale (" << packedCC.getScale() <<
                    "), better to repack all connected components with a new adjusted scale (current adjustment: " << scalingAdjustment <<
                    ")" << std::endl;
#endif
                packedCCList.clear();
                return false;
            }
            idxPack--;//re-do the last element
        }
    }
    // add patches to mesh
    mesh.clear();
    ccIdx = 0;
    for (auto& cc : packedCCList) {
        mesh.append(cc);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
        if (params.keepIntermediateFiles) {
            auto prefix = keepFilesPathPrefix + "_final_packed_CC#" + std::to_string(ccIdx);
            cc.save(prefix + ".obj");
        }
        ccIdx++;
#endif
    }
    //now turn the coordinates into the [0,1] range
    for (auto& uv : mesh.texCoords()) {
        uv[0] /= (occupancySizeU * params.displacementVideoBlockSize);
        uv[1] /= (occupancySizeV * params.displacementVideoBlockSize);
    }
#if DEBUG_ORTHO_PATCH_PACKING
    if (params.keepIntermediateFiles) {
        mesh.save(keepFilesPathPrefix + "_orthoAtlas.obj");
    }
#endif
    return true;
}

bool
TextureParametrization::pack_patches_with_patch_scale_and_rotation_and_temporal_stabilization(TriangleMesh<MeshType>& mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>& projectedConnectedComponentsCategories,
    std::vector<ConnectedComponent<MeshType>>& packedCCList,
    std::vector<ConnectedComponent<MeshType>>& previousFramePackedCCList,
    const VMCEncoderParameters& params,
    double scalingAdjustment, std::string keepFilesPathPrefix) {
    //pack patches into the atlas image (global UV)
    const double avgAreaNumVertRatio = mesh.area() / (double)mesh.pointCount();
    const auto bbMesh = mesh.boundingBox();
    double frameScaleProjection = (double)(params.height - 2 * params.gutter) / (double)(params.displacementVideoBlockSize * ceil((bbMesh.max - bbMesh.min).normInf() / params.displacementVideoBlockSize));
    frameScaleProjection *= scalingAdjustment; // leaving some room for adjustments
    vmesh::Plane<uint8_t> occupancy;
    int occupancySizeV = std::ceil((double)params.height / (double)params.displacementVideoBlockSize);
    int occupancySizeU = std::ceil((double)params.width / (double)params.displacementVideoBlockSize);
    occupancy.resize(occupancySizeU, occupancySizeV);
    occupancy.fill(false);
    int ccIdx = 0;
    //lists with the matched and unmatched patches
    std::vector<int> isMatched;
    isMatched.resize(projectedCCList.size(), -1);
    //find matches for the connected components of previous frames
    float  thresholdIOU = 0.3F; // volumes have to have at least 30% overlap
    for (int prevIdx = 0; prevIdx < previousFramePackedCCList.size(); prevIdx++) {
        auto& previousCC = previousFramePackedCCList[prevIdx];
        float maxIou = 0.0F;
        int   bestIdx = -1;
        auto  prevBB = previousCC.boundingBox();
        for (int idxCur = 0; idxCur < projectedCCList.size(); idxCur++) {
            auto& curCC = projectedCCList[idxCur];
            if ((isMatched[idxCur] >= 0) ||
                (previousCC.getProjection() != projectedConnectedComponentsCategories[idxCur]))
                continue;
            auto curBB = curCC->boundingBox();
            auto intersect = curBB & prevBB;
            double intersectVolume = intersect.volume();
            double unionVolume = curBB.volume() + prevBB.volume() - intersectVolume;
            float iou = intersectVolume / unionVolume;
            if (iou > maxIou) {
                bestIdx = idxCur;
                maxIou = iou;
            }
        }
        if (maxIou > thresholdIOU) {
            isMatched[bestIdx] = prevIdx;
        }
    }

    // packing matched patches
    std::vector<vmesh::ConnectedComponent<MeshType>> matchedPackedCCList;
    // load and scale UV coordinates
    for (int idx = 0; idx < isMatched.size(); idx++) {
        if (isMatched[idx] != -1) {
            vmesh::ConnectedComponent<MeshType> cc;
            //load mesh
            cc.triangles() = projectedCCList[idx]->triangles();
            cc.points() = projectedCCList[idx]->points();
            cc.texCoords() = projectedCCList[idx]->texCoords();
            cc.texCoordTriangles() = projectedCCList[idx]->texCoordTriangles();
            //set projection and initial scale
            cc.setProjection(projectedConnectedComponentsCategories[idx]);
            cc.setFrameScale(frameScaleProjection);
            cc.setScale(frameScaleProjection);
            //set reference
            cc.setIdxPatch(idx);
            cc.setRefPatch(isMatched[idx]);
            matchedPackedCCList.push_back(cc);
        }
    }
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
    std::cout << "Found " << matchedPackedCCList.size() << " matched patches" << std::endl;
#endif
    //sort list to have similar order as the reference
    std::sort(matchedPackedCCList.begin(), matchedPackedCCList.end(), [&](ConnectedComponent<MeshType> a, ConnectedComponent<MeshType> b) {
        return (a.getRefPatch() > b.getRefPatch());
        });
    // obtain the scaling of each connected component 
    for (auto& packedCC : matchedPackedCCList) {
        if (params.bPatchScaling) {
            double ccAreaNumVerRatio = packedCC.area() / (double)packedCC.pointCount();
            if (ccAreaNumVerRatio < (avgAreaNumVertRatio * 0.5)) {
                packedCC.setScale(packedCC.getScale() * 2.0);
            }
            else {
                packedCC.setScale(packedCC.getScale() * 1.0);
            }
        }
        else {
            packedCC.setScale(packedCC.getScale() * 1.0);
        }
    }
    // pack loop for matched patches
    while (matchedPackedCCList.size() > 0) {
        vmesh::ConnectedComponent<MeshType> packedCC = matchedPackedCCList.back();
        int isMatchedPos = packedCC.getIdxPatch();
        packedCC.setIdxPatch(ccIdx);
        int matchRefId = packedCC.getRefPatch();
        auto& matchedCC = previousFramePackedCCList[matchRefId];
        matchedPackedCCList.pop_back();
        //rasterize the patch
        vmesh::Plane<uint8_t> occupancyCC;
        patch_rasterization(occupancyCC, packedCC, params);
        //now place the patch in the empty area
        if (!pack_patch_flexible_with_temporal_stability(occupancy, occupancyCC, packedCC, matchedCC, params, keepFilesPathPrefix))
        {
            // do the cc again and reduce it's size
            packedCC.setScale(packedCC.getScale() * params.packingScaling);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
            std::cout << "Reducing patch scale by " << (1 - params.packingScaling) * 100 << "% (" << packedCC.getScale() <<
                ")" << std::endl;
#endif
            if ((packedCC.getScale() / frameScaleProjection) < 0.5) {
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
                std::cout << "Connected Component using small scale (" << packedCC.getScale() <<
                    "), will try to pack without temporal stability " << std::endl;
#endif
                isMatched[isMatchedPos] = -1;
            }
            else {
                packedCC.setIdxPatch(isMatchedPos);
                matchedPackedCCList.push_back(packedCC);
            }
        }
        else {
            packedCCList.push_back(packedCC);
            ccIdx++;
        }
    }

    //packing unmatched patches
    std::vector<vmesh::ConnectedComponent<MeshType>> unmatchedPackedCCList;
    // load  UV coordinates
    for (int idx = 0; idx < isMatched.size(); idx++) {
        if (isMatched[idx] == -1) {
            vmesh::ConnectedComponent<MeshType> cc;
            //load mesh
            cc.triangles() = projectedCCList[idx]->triangles();
            cc.points() = projectedCCList[idx]->points();
            cc.texCoords() = projectedCCList[idx]->texCoords();
            cc.texCoordTriangles() = projectedCCList[idx]->texCoordTriangles();
            //set projection and initial scale
            cc.setProjection(projectedConnectedComponentsCategories[idx]);
            cc.setFrameScale(frameScaleProjection);
            cc.setScale(frameScaleProjection);
            //set reference
            cc.setIdxPatch(idx);
            cc.setRefPatch(-1);
            unmatchedPackedCCList.push_back(cc);
        }
    }
    //sort list to start packing from largest cc to smallest
    std::sort(unmatchedPackedCCList.begin(), unmatchedPackedCCList.end(), [&](ConnectedComponent<MeshType> a, ConnectedComponent<MeshType> b) {
        return (a.area() > b.area());
        });
    // obtain the scaling of each connected component 
    for (auto& packedCC : unmatchedPackedCCList) {
        if (params.bPatchScaling) {
            double ccAreaNumVerRatio = packedCC.area() / (double)packedCC.pointCount();
            if (ccAreaNumVerRatio < (avgAreaNumVertRatio * 0.5)) {
                packedCC.setScale(packedCC.getScale() * 2.0);
            }
            else
                packedCC.setScale(packedCC.getScale() * 1.0);
        }
        else {
            packedCC.setScale(packedCC.getScale() * 1.0);
        }
    }
    // pack loop for unmatched patches
    for (int idxPack = 0; idxPack < unmatchedPackedCCList.size(); idxPack++) {
        auto& packedCC = unmatchedPackedCCList[idxPack];
        // set index and reference
        packedCC.setIdxPatch(ccIdx);
        packedCC.setRefPatch(-1);
        //rasterize the patch
        vmesh::Plane<uint8_t> occupancyCC;
        patch_rasterization(occupancyCC, packedCC, params);
        //now place the patch in the empty area
        bool invertDirection = (packedCC.area() < (packedCCList[0].area() * 0.01)) && params.packSmallPatchesOnTop;
        if (!pack_patch_flexible(occupancy, occupancyCC, packedCC, params, keepFilesPathPrefix, invertDirection))
        {
            // do the cc again and reduce it's size
            packedCC.setScale(packedCC.getScale() * params.packingScaling);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
            std::cout << "Reducing patch scale by " << (1 - params.packingScaling) * 100 << "% (" << packedCC.getScale() <<
                ")" << std::endl;
#endif
            if ((packedCC.getScale() / frameScaleProjection) < 0.5) {
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
                std::cout << "Connected Component using small scale (" << packedCC.getScale() <<
                    "), better to repack all connected components with a new adjusted scale (current adjustment: " << scalingAdjustment <<
                    ")" << std::endl;
#endif
                packedCCList.clear();
                return false;
            }
            idxPack--;//re-do the last element
        }
        else {
            packedCCList.push_back(packedCC);
            ccIdx++;
        }
    }

    // add patches to mesh
    mesh.clear();
    ccIdx = 0;
    for (auto& cc : packedCCList) {
        mesh.append(cc);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
        if (params.keepIntermediateFiles) {
            auto prefix = keepFilesPathPrefix + "_final_packed_CC#" + std::to_string(ccIdx);
            cc.save(prefix + ".obj");
        }
        ccIdx++;
#endif
    }
    //now turn the coordinates into the [0,1] range
    for (auto& uv : mesh.texCoords()) {
        uv[0] /= (occupancySizeU * params.displacementVideoBlockSize);
        uv[1] /= (occupancySizeV * params.displacementVideoBlockSize);
    }
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
    if (params.keepIntermediateFiles) {
        mesh.save(keepFilesPathPrefix + "_orthoAtlas.obj");
    }
#endif
    return true;
}

bool
TextureParametrization::tetris_packing_patches_with_patch_scale_and_rotation(
    TriangleMesh<MeshType>& mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>& projectedConnectedComponentsCategories,
    std::vector<ConnectedComponent<MeshType>>& packedCCList,
    const VMCEncoderParameters& params,
    double scalingAdjustment, std::string keepFilesPathPrefix) {
    //pack patches into the atlas image (global UV)
    const double avgAreaNumVertRatio = mesh.area() / (double)mesh.pointCount();
    const auto bbMesh = mesh.boundingBox();
    double frameScaleProjection = (double)(params.height - 2 * params.gutter) / (double)(params.displacementVideoBlockSize * ceil((bbMesh.max - bbMesh.min).normInf() / params.displacementVideoBlockSize));
    frameScaleProjection *= scalingAdjustment; // leaving some room for adjustments
    vmesh::Plane<uint8_t> occupancy;
    int occupancySizeV = std::ceil((double)params.height / (double)params.displacementVideoBlockSize);
    int occupancySizeU = std::ceil((double)params.width / (double)params.displacementVideoBlockSize);
    occupancy.resize(occupancySizeU, occupancySizeV);
    occupancy.fill(false);
    std::vector<int> horizon;
    horizon.resize(occupancySizeU, 0);
#if DEBUG_ORTHO_PATCH_PACKING
    std::cout << "Horizon :[";
    for (int i = 0; i < occupancySizeU; i++) { std::cout << horizon[i] << ","; }
    std::cout << "]" << std::endl;
#endif
    int ccIdx = 0;
    // load UV coordinates
    double totalArea = 0;
    for (int idx = 0; idx < projectedCCList.size(); idx++) {
        auto& ccMesh = projectedCCList[idx];
        vmesh::ConnectedComponent<MeshType> cc;
        //copy mesh
        cc.triangles() = ccMesh->triangles();
        cc.points() = ccMesh->points();
        cc.texCoords() = ccMesh->texCoords();
        cc.texCoordTriangles() = ccMesh->texCoordTriangles();
        // set projection and initial scale
        cc.setProjection(projectedConnectedComponentsCategories[idx]);
        cc.setFrameScale(frameScaleProjection);
        cc.setScale(frameScaleProjection);
        packedCCList.push_back(cc);
        auto ccBB = cc.texCoordBoundingBox();
        int widthOccCC = std::ceil((cc.getScale() * ccBB.max[0] + 2 * params.gutter) / (double)params.displacementVideoBlockSize);
        int heightOccCC = std::ceil((cc.getScale() * ccBB.max[1] + 2 * params.gutter) / (double)params.displacementVideoBlockSize);
        totalArea += (widthOccCC) * (heightOccCC);
    }
    if (totalArea > (1.5 * (occupancySizeV * occupancySizeU)))
    {
#if DEBUG_ORTHO_PATCH_PACKING
        std::cout << "Canvas area (" << occupancySizeV * occupancySizeU <<
            "), is not big enough to contain all patches at current scale ( " << scalingAdjustment <<
            "), the total area needed is " << totalArea << std::endl;
#endif
        packedCCList.clear();
        return false;
    }
    //sort list to start packing from largest cc to smallest
    std::sort(packedCCList.begin(), packedCCList.end(), [&](ConnectedComponent<MeshType> a, ConnectedComponent<MeshType> b) {
        return (a.area() > b.area()); // packing from largest to smallest patch
        });
    // obtain the scaling of each connected component 
    for (auto& packedCC : packedCCList) {
        if (params.bPatchScaling) {
            double ccAreaNumVerRatio = packedCC.area() / (double)packedCC.pointCount();
            if (ccAreaNumVerRatio < (avgAreaNumVertRatio * 0.5)) {
                packedCC.setScale(packedCC.getScale() * 2.0);
            }
            else {
                packedCC.setScale(packedCC.getScale() * 1.0);
            }
        }
        else {
            packedCC.setScale(packedCC.getScale() * 1.0);
        }
    }
    // pack loop
    for (int idxPack = 0; idxPack < packedCCList.size(); idxPack++) {
        auto& packedCC = packedCCList[idxPack];
        // set index and reference
        packedCC.setIdxPatch(idxPack);
        packedCC.setRefPatch(-1);
        //rasterize the patch
        vmesh::Plane<uint8_t> occupancyCC;
        patch_rasterization(occupancyCC, packedCC, params);
        std::vector<int> topHorizon;
        std::vector<int> bottomHorizon;
        std::vector<int> rightHorizon;
        std::vector<int> leftHorizon;
        patch_horizons(occupancyCC, topHorizon, bottomHorizon, rightHorizon, leftHorizon);
        //now place the patch in the empty area
        bool invertDirection = (packedCC.area() < (packedCCList[0].area() * 0.01)) && params.packSmallPatchesOnTop;
        if (!pack_patch_flexible_tetris(occupancy, occupancyCC, packedCC, params, keepFilesPathPrefix, horizon, topHorizon, bottomHorizon, rightHorizon, leftHorizon))
        {
            // do the cc again and reduce it's size
            packedCC.setScale(packedCC.getScale() * params.packingScaling);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
            std::cout << "Reducing patch scale by " << (1 - params.packingScaling) * 100 << "% (" << packedCC.getScale() <<
                ")" << std::endl;
#endif
            if ((packedCC.getScale() / frameScaleProjection) < 0.25) {
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
                std::cout << "Connected Component using small scale (" << packedCC.getScale() <<
                    "), better to repack all connected components with a new adjusted scale (current adjustment: " << scalingAdjustment <<
                    ")" << std::endl;
#endif
                packedCCList.clear();
                return false;
            }
            idxPack--;//re-do the last element
        }
        else {
            //update the horizon
            int bestU = packedCC.getU0();
            int sizeU = packedCC.getSizeU();
            int bestV = packedCC.getV0();
            int sizeV = packedCC.getSizeV();
            int bestOrientation = packedCC.getOrientation();
            int    newVal;
            switch (packedCC.getOrientation()) {
            case 0: // no rotation
                for (int idx = 0; idx < sizeU; idx++) {
                    newVal = int(bestV + sizeV - 1 - topHorizon[idx]);
                    if (newVal > horizon[bestU + idx]) horizon[bestU + idx] = newVal;
                }
                break;
            case 1: // 90 degree
                for (int idx = 0; idx < sizeV; idx++) {
                    newVal = int(bestV + sizeU - 1 - rightHorizon[sizeV - 1 - idx]);
                    if (newVal > horizon[bestU + idx]) horizon[bestU + idx] = newVal;
                }
                break;
            case 2: // 180 degrees
                for (int idx = 0; idx < sizeU; idx++) {
                    newVal = int(bestV + sizeV - 1 - bottomHorizon[sizeU - 1 - idx]);
                    if (newVal > horizon[bestU + idx]) horizon[bestU + idx] = newVal;
                }
                break;
            case 3: // 270 degrees
                for (int idx = 0; idx < sizeV; idx++) {
                    newVal = int(bestV + sizeU - 1 - leftHorizon[idx]);
                    if (newVal > horizon[bestU + idx]) horizon[bestU + idx] = newVal;
                }
                break;
            default: // no rotation
                for (int idx = 0; idx < sizeU; idx++) {
                    newVal = int(bestV + sizeV - 1 - topHorizon[idx]);
                    if (newVal > horizon[bestU + idx]) horizon[bestU + idx] = newVal;
                }
                break;
            }                //check if the patch can fit on space above the horizon
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
            std::cout << "Horizon :[";
            for (int i = 0; i < occupancySizeU; i++) { std::cout << horizon[i] << ","; }
            std::cout << "]" << std::endl;
#endif
        }
    }
    // add patches to mesh
    mesh.clear();
    ccIdx = 0;
    for (auto& cc : packedCCList) {
        mesh.append(cc);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
        if (params.keepIntermediateFiles) {
            auto prefix = keepFilesPathPrefix + "_final_packed_CC#" + std::to_string(ccIdx);
            cc.save(prefix + ".obj");
        }
        ccIdx++;
#endif
    }
    //now turn the coordinates into the [0,1] range
    for (auto& uv : mesh.texCoords()) {
        uv[0] /= (occupancySizeU * params.displacementVideoBlockSize);
        uv[1] /= (occupancySizeV * params.displacementVideoBlockSize);
    }
#if DEBUG_ORTHO_PATCH_PACKING
    if (params.keepIntermediateFiles) {
        mesh.save(keepFilesPathPrefix + "_orthoAtlas.obj");
    }
#endif
    return true;
}

bool
TextureParametrization::projection_packing_patches_with_patch_scale_and_rotation(
    TriangleMesh<MeshType>& mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>& projectedConnectedComponentsCategories,
    std::vector<ConnectedComponent<MeshType>>& packedCCList,
    const VMCEncoderParameters& params,
    double scalingAdjustment, std::string keepFilesPathPrefix) {
    //pack patches into the atlas image (global UV)
    const double avgAreaNumVertRatio = mesh.area() / (double)mesh.pointCount();
    const auto bbMesh = mesh.boundingBox();
    double frameScaleProjection = (double)(params.height - 2 * params.gutter) / (double)(params.displacementVideoBlockSize * ceil((bbMesh.max - bbMesh.min).normInf() / params.displacementVideoBlockSize));
    frameScaleProjection *= scalingAdjustment; // leaving some room for adjustments
    vmesh::Plane<uint8_t> occupancy;
    int occupancySizeV = std::ceil((double)params.height / (double)params.displacementVideoBlockSize);
    int occupancySizeU = std::ceil((double)params.width / (double)params.displacementVideoBlockSize);
    occupancy.resize(occupancySizeU, occupancySizeV);
    occupancy.fill(false);

    // check the number of categories that we need to pack
    auto categories = projectedConnectedComponentsCategories;
    std::sort(categories.begin(), categories.end());
    auto last = std::unique(categories.begin(), categories.end());
    categories.erase(last, categories.end());
    int numCategories = categories.size();

    //now allocate structures for each category
    std::vector <vmesh::Plane<uint8_t>> occupancyPerCategory;
    std::vector<std::vector<ConnectedComponent<MeshType>>> packedCCListPerCategory;
    std::vector<vmesh::Box2<MeshType>> bbPerCategory;
    std::vector<int> numOccupiedPos;
    occupancyPerCategory.resize(numCategories);
    packedCCListPerCategory.resize(numCategories);
    numOccupiedPos.resize(numCategories, 0);
    bbPerCategory.resize(numCategories);
    for (int i = 0; i < numCategories; i++) {
        occupancyPerCategory[i].resize(occupancySizeU, occupancySizeV);
        occupancyPerCategory[i].fill(false);
        bbPerCategory[i].min = std::numeric_limits<MeshType>::max();
        bbPerCategory[i].max = std::numeric_limits<MeshType>::min();
        // load connected component
        for (int idx = 0; idx < projectedCCList.size(); idx++) {
            if (projectedConnectedComponentsCategories[idx] == categories[i]) {
                auto& ccMesh = projectedCCList[idx];
                vmesh::ConnectedComponent<MeshType> cc;
                //copy mesh
                cc.triangles() = ccMesh->triangles();
                cc.points() = ccMesh->points();
                cc.texCoords() = ccMesh->texCoords();
                cc.texCoordTriangles() = ccMesh->texCoordTriangles();
                // set projection and initial scale
                cc.setProjection(projectedConnectedComponentsCategories[idx]);
                cc.setFrameScale(frameScaleProjection);
                cc.setScale(frameScaleProjection);
                packedCCListPerCategory[i].push_back(cc);
            }
        }
        //sort list to start packing from smallest to largest area 
        std::sort(packedCCListPerCategory[i].begin(), packedCCListPerCategory[i].end(), [&](ConnectedComponent<MeshType> a, ConnectedComponent<MeshType> b) {
            return (a.area() > b.area());
            });
        // pack loop keeping the projection position
        for (int idx = 0; idx < packedCCListPerCategory[i].size(); idx++) {
            vmesh::ConnectedComponent<MeshType>& packedCC = packedCCListPerCategory[i][idx];
            packedCC.setIdxPatch(idx);
            packedCC.setRefPatch(categories[i]);
            vmesh::ConnectedComponent<MeshType> matchedCC;
            matchedCC.setOrientation(0);
            int tangentAxis, biTangentAxis, projAxis;
            int tangentAxisDir, biTangentAxisDir, projAxisDir;
            getProjectedAxis(categories[i], tangentAxis, biTangentAxis, projAxis, tangentAxisDir, biTangentAxisDir, projAxisDir);
            auto bb = packedCC.boundingBox();
            matchedCC.setU0((packedCC.getScale() * bb.min[tangentAxis] + params.gutter) / (double)params.displacementVideoBlockSize);
            matchedCC.setV0((packedCC.getScale() * bb.min[biTangentAxis] + params.gutter) / (double)params.displacementVideoBlockSize);
            //rasterize the patch
            vmesh::Plane<uint8_t> occupancyCC;
            patch_rasterization(occupancyCC, packedCC, params);
            //now place the patch in the empty area
            if (!pack_patch_flexible_with_temporal_stability(occupancyPerCategory[i], occupancyCC, packedCC, matchedCC, params, keepFilesPathPrefix))
            {
                if (!pack_patch_flexible(occupancyPerCategory[i], occupancyCC, packedCC, params, keepFilesPathPrefix, false)) {
                    // do the cc again and reduce it's size
                    packedCC.setScale(packedCC.getScale() * params.packingScaling);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
                    std::cout << "Reducing patch scale by " << (1 - params.packingScaling) * 100 << "% (" << packedCC.getScale() <<
                        ")" << std::endl;
#endif
                    if ((packedCC.getScale() / frameScaleProjection) < 0.5) {
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
                        std::cout << "Connected Component using small scale (" << packedCC.getScale() <<
                            "), better to repack all connected components with a new adjusted scale (current adjustment: " << scalingAdjustment <<
                            ")" << std::endl;
#endif
                        packedCCList.clear();
                        return false;
                    }
                    else
                        idx--;
                }
                else {
                    bbPerCategory[i].enclose(Vec2<MeshType>(packedCC.getU0(), packedCC.getV0()));
                    bbPerCategory[i].enclose(Vec2<MeshType>(packedCC.getU0() + packedCC.getSizeU(), packedCC.getV0() + packedCC.getSizeV()));
                }
            }
            else {
                bbPerCategory[i].enclose(Vec2<MeshType>(packedCC.getU0(), packedCC.getV0()));
                bbPerCategory[i].enclose(Vec2<MeshType>(packedCC.getU0() + packedCC.getSizeU(), packedCC.getV0() + packedCC.getSizeV()));
            }

        }
        //calculate the occupied area
        for (int h = 0; h < occupancySizeV; h++) {
            for (int w = 0; w < occupancySizeU; w++) {
                if (occupancyPerCategory[i].get(w, h))
                    numOccupiedPos[i]++;
            }
        }
    }
    // now merge all occupancy maps into a single atlas
    // place the categories with largest areas first
    std::vector<int> catOrder;
    catOrder.resize(numCategories);
    for (int i = 0; i < numCategories; i++) catOrder[i] = i;
    std::sort(catOrder.begin(), catOrder.end(), [&](int a, int b) {
        return (numOccupiedPos[a] > numOccupiedPos[b]);
        });
    // find the occupancy map bounding box
    for (int i = 0; i < numCategories; i++) {
        int idx = catOrder[i];
        vmesh::ConnectedComponent<MeshType> packedCC;
        packedCC.setIdxPatch(idx);
        //create the occupancy for the group of patches
        vmesh::Plane<uint8_t> occupancyCC;
        int widthOccCC = bbPerCategory[idx].max[0] - bbPerCategory[idx].min[0];
        int heightOccCC = bbPerCategory[idx].max[1] - bbPerCategory[idx].min[1];
        occupancyCC.resize(widthOccCC, heightOccCC);
        occupancyCC.fill(false);
        for (int h = bbPerCategory[idx].min[1]; h < bbPerCategory[idx].max[1]; h++) {
            for (int w = bbPerCategory[idx].min[0]; w < bbPerCategory[idx].max[0]; w++) {
                occupancyCC.set(h - bbPerCategory[idx].min[1], w - bbPerCategory[idx].min[0], occupancyPerCategory[idx].get(h, w));
            }
        }
        //now place the patch in the empty area
        if (!pack_patch_flexible(occupancy, occupancyCC, packedCC, params, keepFilesPathPrefix, false)) {
            // do the packing again with reduce size
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
            std::cout << "Combined Connected Component using small scale (" << packedCCListPerCategory[idx][0].getScale() <<
                "), better to repack all connected components with a new adjusted scale (current adjustment: " << scalingAdjustment <<
                ")" << std::endl;
#endif
            packedCCList.clear();
            return false;
        }
        else {
            // update all the positions with the new orientation (U0,V0)
            for (auto& cc : packedCCListPerCategory[idx]) {
                int newU0 = cc.getU0();
                int newV0 = cc.getV0();
                // remove the occupancy map corner
                for (auto& uv : cc.texCoords()) {
                    uv -= Vec2<MeshType>(bbPerCategory[idx].min[0] * params.displacementVideoBlockSize, bbPerCategory[idx].min[1] * params.displacementVideoBlockSize);
                }
                newU0 -= bbPerCategory[idx].min[0];
                newV0 -= bbPerCategory[idx].min[1];
                // now apply the latest packing results
                // rotation
                for (auto& uv : cc.texCoords()) {
                    MeshType uu = uv[0];
                    MeshType vv = uv[1];
                    switch (packedCC.getOrientation()) {
                    case 0: // no rotation
                        break;
                    case 1: // 90 degree
                        uv[1] = uu;
                        uv[0] = (heightOccCC * params.displacementVideoBlockSize - vv);
                        break;
                    case 2: // 180 degrees
                        uv[1] = (heightOccCC * params.displacementVideoBlockSize - vv);
                        uv[0] = (widthOccCC * params.displacementVideoBlockSize - uu);
                        break;
                    case 3: // 270 degrees
                        uv[1] = (widthOccCC * params.displacementVideoBlockSize - uu);
                        uv[0] = vv;
                        break;
                    default: // no rotation
                        break;
                    }
                }
                MeshType uu0 = newU0;
                MeshType vv0 = newV0;
                switch (packedCC.getOrientation()) {
                case 0: // no rotation
                    break;
                case 1: // 90 degree
                    newV0 = uu0;
                    newU0 = (heightOccCC - (cc.getSizeV() + vv0));
                    break;
                case 2: // 180 degrees
                    newV0 = (heightOccCC - (cc.getSizeV() + vv0));
                    newU0 = (widthOccCC - (cc.getSizeU() + uu0));
                    break;
                case 3: // 270 degrees
                    newV0 = (widthOccCC - (cc.getSizeU() + uu0));
                    newU0 = vv0;
                    break;
                default: // no rotation
                    break;
                }
                cc.setOrientation(packedCC.getOrientation());
                // translation
                for (auto& uv : cc.texCoords()) {
                    uv += Vec2<MeshType>(packedCC.getU0() * params.displacementVideoBlockSize, packedCC.getV0() * params.displacementVideoBlockSize);
                }
                newU0 += packedCC.getU0();
                cc.setU0(newU0);
                newV0 += packedCC.getV0();
                cc.setV0(newV0);
                cc.setIdxPatch(packedCCList.size());
                packedCCList.push_back(cc);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
                std::cout << "added CC[" << cc.getIdxPatch() <<
                    "] = (#tri: " << cc.triangles().size() <<
                    ", #vert: " << cc.points().size() <<
                    ", P: " << cc.getProjection() <<
                    ", S: " << cc.getScale() <<
                    ", O: " << cc.getOrientation() <<
                    ", U0: " << cc.getU0() <<
                    ", V0: " << cc.getV0() <<
                    ", SizeU: " << cc.getSizeU() <<
                    ", SizeV: " << cc.getSizeV() <<
                    ")" << std::endl;
#endif                
            }
        }
    }

    // add patches to mesh
    mesh.clear();
    int ccIdx = 0;
    for (auto& cc : packedCCList) {
        mesh.append(cc);
#if DEBUG_ORTHO_PATCH_PACKING_VERBOSE
        auto prefix = keepFilesPathPrefix + "_final_packed_connected_component_#" + std::to_string(ccIdx);
        cc.save(prefix + ".obj");
        std::cout << "FINAL CC[" << ccIdx <<
            "] -> perimeter(" << cc.perimeter() <<
            "), stretchL2(" << cc.stretchL2(cc.getProjection()) << ")" << std::endl;
#endif
        ccIdx++;
    }
    //now turn the coordinates into the [0,1] range
    for (auto& uv : mesh.texCoords()) {
        uv[0] /= (occupancySizeU * params.displacementVideoBlockSize);
        uv[1] /= (occupancySizeV * params.displacementVideoBlockSize);
    }
#if DEBUG_ORTHO_PATCH_PACKING
    if (params.keepIntermediateFiles) {
        mesh.save(keepFilesPathPrefix + "_hierarchical_packing_orthoAtlas.obj");
    }
#endif
    return true;
}

//============================================================================

}  // namespace vmesh