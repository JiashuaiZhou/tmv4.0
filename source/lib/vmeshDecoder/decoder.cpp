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

#include "decoder.hpp"

#include <cstdio>
#include <sstream>

#include "motionContexts.hpp"
#include "displacementContexts.hpp"
#include "entropy.hpp"

#include "geometryDecoder.hpp"
#include "videoDecoder.hpp"
#include "colourConverter.hpp"

namespace vmesh {

bool
VMCDecoder::addNeighbor(int32_t vertex1,
                        int32_t vertex2,
                        int32_t maxNumNeighborsMotion) {
  bool duplicate      = false;
  auto vertex         = vertex1;
  auto vertexNeighbor = vertex2;
  auto predCount      = numNeighborsMotion[vertex];
  predCount           = std::min(predCount, maxNumNeighborsMotion);
  if (vertex > vertexNeighbor) {  // Check if vertexNeighbor is available
    for (int32_t n = 0; n < predCount; ++n) {
      if (vertexAdjTableMotion[vertex * maxNumNeighborsMotion + n]
          == vertexNeighbor) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      if (predCount == maxNumNeighborsMotion) {
        vertexAdjTableMotion[vertex * maxNumNeighborsMotion
                             + maxNumNeighborsMotion - 1] = vertexNeighbor;
      } else {
        vertexAdjTableMotion[vertex * maxNumNeighborsMotion + predCount] =
          vertexNeighbor;
        numNeighborsMotion[vertex] = predCount + 1;
      }
    }
  }
  duplicate      = false;
  vertex         = vertex2;
  vertexNeighbor = vertex1;
  predCount      = numNeighborsMotion[vertex];
  if (vertex > vertexNeighbor) {  // Check if vertexNeighbor is available
    for (int32_t n = 0; n < predCount; ++n) {
      if (vertexAdjTableMotion[vertex * maxNumNeighborsMotion + n]
          == vertexNeighbor) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      if (predCount == maxNumNeighborsMotion) {
        vertexAdjTableMotion[vertex * maxNumNeighborsMotion
                             + maxNumNeighborsMotion - 1] = vertexNeighbor;
      } else {
        vertexAdjTableMotion[vertex * maxNumNeighborsMotion + predCount] =
          vertexNeighbor;
        numNeighborsMotion[vertex] = predCount + 1;
      }
    }
  }
  return true;
}

bool
VMCDecoder::computeVertexAdjTableMotion(
  const std::vector<Vec3<int32_t>>& triangles,
  int32_t                           vertexCount,
  int32_t                           maxNumNeighborsMotion) {
  const auto triangleCount = int32_t(triangles.size());
  // const auto vertexCount = int32_t(reference.size());
  if (vertexAdjTableMotion.size() < vertexCount * maxNumNeighborsMotion) {
    vertexAdjTableMotion.resize(vertexCount * maxNumNeighborsMotion);
  }
  if (numNeighborsMotion.size() < vertexCount) {
    numNeighborsMotion.resize(vertexCount);
  }
  for (int32_t v = 0; v < vertexCount; ++v) { numNeighborsMotion[v] = 0; }

  for (int32_t triangleIndex = 0; triangleIndex < triangleCount;
       ++triangleIndex) {
    const auto& tri = triangles[triangleIndex];
    assert(tri.i() >= 0 && tri.i() < vertexCount);
    assert(tri.j() >= 0 && tri.j() < vertexCount);
    assert(tri.k() >= 0 && tri.k() < vertexCount);
    // Add Neighbors
    addNeighbor(tri.i(), tri.j(), maxNumNeighborsMotion);
    addNeighbor(tri.i(), tri.k(), maxNumNeighborsMotion);
    addNeighbor(tri.j(), tri.k(), maxNumNeighborsMotion);
  }
  return true;
}

//============================================================================

//============================================================================

bool
VMCDecoder::decompressMotion(
  const V3cBitstream&               syntax,
  const BaseMeshTileLayer&          bmtl,
  const std::vector<Vec3<int32_t>>& trianglesReference,
  const std::vector<Vec3<int32_t>>& referenceReference,
  const std::vector<Vec2<int32_t>>& baseIntegrateIndicesReference,
  const std::vector<Vec3<int32_t>>& trianglesBase,
  const std::vector<Vec3<int32_t>>& referenceBase,
  std::vector<Vec3<int32_t>>&       current,
  const VMCDecoderParameters& /*params*/) {
  printf("decompressMotion \n");
  fflush(stdout);
  // here we should decode all_skip_mode from bitstream before starting to decode the MVs
  bool                 all_skip_mode = true;
  int                  no_skip_num   = 0;
  std::vector<uint8_t> no_skip_vindices;
  uint8_t skip_mode_bits;  // 1 bit for all_skip_mode, 7 bits for no_skip_num

  int                        skip_mode_size = sizeof(skip_mode_bits);
  std::vector<Vec3<int32_t>> triangles;
  std::vector<Vec3<int32_t>> reference;
  std::vector<Vec2<int32_t>> baseIntegrateIndices;
  auto&                      bmth  = bmtl.getHeader();
  auto&                      bmtdu = bmtl.getDataUnit();
  if (!bmtdu.getMotionSkipFlag()) {  // skip_mode_bits == 255
    skip_mode_bits = 255;
    triangles      = trianglesBase;
    reference      = referenceBase;
  } else if (bmtdu.getMotionSkipAll()) {  // skip_mode_bits == 128
    skip_mode_bits       = 128;
    triangles            = trianglesReference;
    reference            = referenceReference;
    baseIntegrateIndices = baseIntegrateIndicesReference;
  } else {
    all_skip_mode        = false;
    no_skip_num          = bmtdu.getMotionSkipCount();
    skip_mode_bits       = bmtdu.getMotionSkipCount();
    triangles            = trianglesReference;
    reference            = referenceReference;
    baseIntegrateIndices = baseIntegrateIndicesReference;
  }
  if (!all_skip_mode) {
    no_skip_vindices.resize(no_skip_num);
    // const auto byteCount =
    //   no_skip_num * sizeof(decltype(no_skip_vindices)::value_type);
    // std::copy(bitstream.buffer.begin() + _byteCounter,
    //           bitstream.buffer.begin() + _byteCounter + byteCount,
    //           reinterpret_cast<uint8_t*>(no_skip_vindices.data()));
    // _byteCounter += byteCount;
    // skip_mode_size += byteCount;
    no_skip_vindices = bmtdu.getMotionSkipVextexIndices();
    for (int i = 0; i < no_skip_num; ++i) {
      std::cout << "[DEBUG][stat] non-skippable MV "
                << static_cast<int32_t>(no_skip_vindices[i])
                << " , index: " << i
                << ". Total non-skippable MVs: " << no_skip_num << "\n";
    }
  }
  printf("MotionSkipFlag  = %d  \n", bmtdu.getMotionSkipFlag());
  printf("MotionSkipAll   = %d  \n", bmtdu.getMotionSkipAll());
  printf("MotionSkipCount = %d  \n", bmtdu.getMotionSkipCount());
  printf("skip_mode_bits  = %u \n", skip_mode_bits);
  printf("skip_mode_size  = %d \n", skip_mode_size);

  VMCMotionACContext ctx;
  EntropyDecoder     arithmeticDecoder;
  // const auto* const  bufferPtr =
  //   reinterpret_cast<const char*>(bitstream.buffer.data() + _byteCounter);
  // _byteCounter += byteCount;
  // arithmeticDecoder.setBuffer(byteCount, bufferPtr);
  const auto& atlas    = syntax.getAtlas();
  const auto& asps     = atlas.getAtlasSequenceParameterSet(0);
  const auto& ext      = asps.getAspsVdmcExtension();
  const auto& baseMesh = syntax.getBaseMesh();
  const auto  bmfpsId  = bmth.getBaseMeshFrameParameterSetId();
  const auto& bmfps    = baseMesh.getBaseMeshFrameParameterSet(bmfpsId);
  const auto& buffer   = bmtdu.getData().vector();

  uint32_t byteCount = buffer.size();
  // bitstream.read(byteCount, _byteCounter);
  std::cout << "Motion byte count = " << byteCount << '\n';
  arithmeticDecoder.setBuffer(buffer.size(),
                              reinterpret_cast<const char*>(buffer.data()));
  arithmeticDecoder.start();
  const auto refPointCount = static_cast<int32_t>(reference.size());
  const auto motionCount   = refPointCount + no_skip_num;
  const auto pointCount =
    refPointCount + static_cast<int32_t>(baseIntegrateIndices.size());
  if (createVertexAdjTableMotion) {
    computeVertexAdjTableMotion(
      triangles, motionCount, ext.getMaxNumNeighborsMotion());
    createVertexAdjTableMotion = false;
  }
  std::vector<Vec3<int32_t>> motion(motionCount);
  current.resize(pointCount);
  std::cout << "[DEBUG][stat] total vertex num: " << pointCount
            << " duplicated vertex num: " << baseIntegrateIndices.size()
            << " non-skippable MV num: " << no_skip_num << std::endl;
  int32_t remainP = motionCount;
  int32_t vindexS = 0, vindexE = 0, vCount = 0;
  while (remainP) {
    vindexS              = vindexE;
    const auto predIndex = arithmeticDecoder.decode(ctx.ctxPred);
    vCount               = 0;
    while ((vCount < bmfps.getMotionGroupSize()) && (remainP)) {
      int vindex0 = vindexE;
      ++vindexE;
      --remainP;
      ++vCount;
      Vec3<int32_t> res{};
      for (int32_t k = 0; k < 3; ++k) {
        int32_t value = 0;
        if (arithmeticDecoder.decode(ctx.ctxCoeffGtN[0][k]) != 0) {
          const auto sign = arithmeticDecoder.decode(ctx.ctxSign[k]);
          ++value;
          if (arithmeticDecoder.decode(ctx.ctxCoeffGtN[1][k]) != 0) {
            value +=
              1 + arithmeticDecoder.decodeExpGolomb(0, ctx.ctxCoeffRemPrefix);
          }
          if (sign != 0) { value = -value; }
        }
        res[k] = value;
      }
      if (predIndex == 0) {
        motion[vindex0] = res;
      } else {
        int vindex = vindex0;
        if (vindex0 >= refPointCount) {
          auto idx          = no_skip_vindices[vindex0 - refPointCount];
          auto integrate_to = baseIntegrateIndices[idx][1];
          auto it           = std::lower_bound(
            baseIntegrateIndices.begin(),
            baseIntegrateIndices.end(),
            Vec2<int32_t>(integrate_to, 0),
            [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) {
              return a[0] < b[0];
            });
          auto shift = std::distance(baseIntegrateIndices.begin(), it);
          vindex     = static_cast<int32_t>(integrate_to - shift);
        }
        Vec3<int32_t> pred(0);
        int32_t       predCount = numNeighborsMotion[vindex];
        for (int32_t i = 0; i < predCount; ++i) {
          const auto vindex1 =
            vertexAdjTableMotion[vindex * ext.getMaxNumNeighborsMotion() + i];
          const auto& mv1 = motion[vindex1];
          for (int32_t k = 0; k < 3; ++k) { pred[k] += mv1[k]; }
        }
        if (predCount > 1) {
          const auto bias = predCount >> 1;
          for (int32_t k = 0; k < 3; ++k) {
            pred[k] = pred[k] >= 0 ? (pred[k] + bias) / predCount
                                   : -(-pred[k] + bias) / predCount;
          }
        }
        motion[vindex0] = res + pred;
      }
    }
  }
  for (int vindex = 0; vindex < pointCount; ++vindex) {
    auto index = vindex;
    auto it =
      std::lower_bound(baseIntegrateIndices.begin(),
                       baseIntegrateIndices.end(),
                       Vec2<int32_t>(index, 0),
                       [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) {
                         return a[0] < b[0];
                       });
    if (it != baseIntegrateIndices.end() && (*it)[0] == index) {
      // set integrated index
      index = (*it)[1];
      it =
        std::lower_bound(baseIntegrateIndices.begin(),
                         baseIntegrateIndices.end(),
                         Vec2<int32_t>(index, 0),
                         [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) {
                           return a[0] < b[0];
                         });
    }
    // move up index
    auto shift       = std::distance(baseIntegrateIndices.begin(), it);
    auto refindex    = index - shift;
    auto motionindex = refindex;
    auto it_no_skip =
      std::find_if(no_skip_vindices.begin(),
                   no_skip_vindices.end(),
                   [&](const decltype(no_skip_vindices)::value_type& a) {
                     return baseIntegrateIndices[a][0] == vindex;
                   });
    if (it_no_skip != no_skip_vindices.end()) {
      auto no_skip_index = std::distance(no_skip_vindices.begin(), it_no_skip);
      motionindex        = refPointCount + no_skip_index;
    }
    current[vindex] = reference[refindex] + motion[motionindex];
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressBaseMesh(const V3cBitstream&         syntax,
                               const BaseMeshTileLayer&    bmtl,
                               const AtlasTileLayerRbsp&   atl,
                               const VMCGroupOfFrames&     gof,
                               VMCFrameInfo&               frameInfo,
                               VMCFrame&                   frame,
                               TriangleMesh<MeshType>&     rec,
                               const VMCDecoderParameters& params) {
  printf("Decompress base mesh: Frame = %d \n", frameInfo.frameIndex);
  fflush(stdout);
  // if (!decodeFrameHeader(bitstream, frameInfo)) return false;
  auto& bmth       = bmtl.getHeader();
  auto& bmtdu      = bmtl.getDataUnit();
  auto& baseMesh   = syntax.getBaseMesh();
  auto  bmspsId    = bmth.getBaseMeshFrameParameterSetId();
  auto& bmsps      = baseMesh.getBaseMeshSequenceParameterSet(bmspsId);
  auto& base       = frame.base;
  auto& qpositions = frame.qpositions;
  frameInfo.type   = bmth.getBaseMeshType();
  auto& atlas      = syntax.getAtlas();
  auto& asps       = atlas.getAtlasSequenceParameterSet(0);
  auto& ext        = asps.getAspsVdmcExtension();
  printf("Scale position: Frame = %d \n", frameInfo.frameIndex);
  fflush(stdout);
  const auto bitDepthPosition = asps.getGeometry3dBitdepthMinus1() + 1;
  printf("BitDepthPosition = %u \n", bitDepthPosition);
  const auto scalePosition = ((1 << (bmsps.getQpPositionMinus1() + 1)) - 1.0)
                             / ((1 << (bitDepthPosition)) - 1.0);
  const auto iscalePosition = 1.0 / scalePosition;
  if (frameInfo.type == I_BASEMESH) {
    printf("Intra index = %d \n", frameInfo.frameIndex);

    // Decode base mesh
    printf("Decode base mesh \n");
    auto decoder = GeometryDecoder<MeshType>::create(
      GeometryCodecId(bmsps.getBaseMeshCodecId()));
    GeometryDecoderParameters decoderParams;
    decoder->decode(bmtdu.getData().vector(), decoderParams, base);
    printf("BaseMeshDeco: done \n");
    fflush(stdout);

    // orthoAtlas
    const auto& ath = atl.getHeader();
    if (ath.getType() == I_TILE) {
      const auto& atdu = atl.getDataUnit();
      const auto& mpdu =
        atdu.getPatchInformationData()[0]
          .getMeshPatchDataUnit();  // currently only contains one single patch
      const auto& afps = syntax.getAtlas().getAtlasFrameParameterSet(
        ath.getAtlasFrameParameterSetId());
      const auto& afpsVmcExt = afps.getAfpsVdmcExtension();
      const auto& asps       = syntax.getAtlas().getAtlasSequenceParameterSet(
        afps.getAtlasSequenceParameterSetId());
      const auto& aspsVmcExt = asps.getAspsVdmcExtension();
      // adjust the UV coordinates of the decoded mesh if indicated by SPS
      if (aspsVmcExt.getProjectionTextCoordEnableFlag()) {
        if (afpsVmcExt.getProjectionTextcoordPresentFlag(
              mpdu.getSubmeshId())) {
          // create the connected components
          int numCC = mpdu.getProjectionTextcoordSubpatchCountMinus1() + 1;
          std::vector<vmesh::ConnectedComponent<double>> connectedComponents;
          std::vector<vmesh::Box2<double>>               bbBoxes;
          std::vector<int>                               trianglePartition;
          connectedComponents.resize(numCC);
          bbBoxes.resize(numCC);
          trianglePartition.resize(base.triangleCount(), -1);
          switch (aspsVmcExt.getProjectionTextCoordMappingMethod()) {
          default:
          case 0:  // uv coordinate is being used for transmitting the triangle to patch mapping
          {
            for (int triIdx = 0; triIdx < base.triangleCount(); triIdx++) {
              auto tri    = base.triangle(triIdx);
              auto texTri = base.texCoordTriangle(triIdx);
              int  idxCC  = base.texCoord(texTri[0])[0];
              //now add the points
              trianglePartition[triIdx] = idxCC;
              for (int i = 0; i < 3; i++) {
                connectedComponents[idxCC].addPoint(base.point(tri[i])
                                                    * iscalePosition);
              }
              connectedComponents[idxCC].addTriangle(
                connectedComponents[idxCC].points().size() - 3,
                connectedComponents[idxCC].points().size() - 2,
                connectedComponents[idxCC].points().size() - 1);
            }
            // remove the UVs
            base.texCoords().clear();
            base.texCoordTriangles().clear();
            break;
          }
          case 1:  // face ID is being used for transmitting the triangle to patch mapping
          {
            for (int triIdx = 0; triIdx < base.triangleCount(); triIdx++) {
              auto tri   = base.triangle(triIdx);
              auto idxCC = base.faceId(triIdx);
              //now add the points
              trianglePartition[triIdx] = idxCC;
              for (int i = 0; i < 3; i++) {
                connectedComponents[idxCC].addPoint(base.point(tri[i])
                                                    * iscalePosition);
              }
              connectedComponents[idxCC].addTriangle(
                connectedComponents[idxCC].points().size() - 3,
                connectedComponents[idxCC].points().size() - 2,
                connectedComponents[idxCC].points().size() - 1);
            }
            break;
          }
          case 2:  // connected components is being used for implicit triangle to patch mapping
          {
            std::vector<int> partition;
            ExtractConnectedComponents(
              base.triangles(), base.pointCount(), base, partition);
            std::vector<vmesh::ConnectedComponent<double>>
              unsortedConnectedComponents;
            unsortedConnectedComponents.resize(connectedComponents.size());
            for (int triIdx = 0; triIdx < base.triangleCount(); triIdx++) {
              auto tri   = base.triangle(triIdx);
              int  idxCC = partition[triIdx];
              //now add the points
              trianglePartition[triIdx] = idxCC;
              for (int i = 0; i < 3; i++) {
                unsortedConnectedComponents[idxCC].addPoint(base.point(tri[i])
                                                            * iscalePosition);
              }
              unsortedConnectedComponents[idxCC].addTriangle(
                unsortedConnectedComponents[idxCC].points().size() - 3,
                unsortedConnectedComponents[idxCC].points().size() - 2,
                unsortedConnectedComponents[idxCC].points().size() - 1);
            }
            // index sorting
            std::vector<int> sortedIndex(numCC);
            for (int val = 0; val < numCC; val++) sortedIndex[val] = val;
            std::sort(
              sortedIndex.begin(), sortedIndex.end(), [&](int a, int b) {
                return (unsortedConnectedComponents[a].area()
                        > unsortedConnectedComponents[b].area());
              });
            for (int triIdx = 0; triIdx < base.triangleCount(); triIdx++) {
              int idxCC = partition[triIdx];
              //now add the points
              trianglePartition[triIdx] =
                std::find(sortedIndex.begin(), sortedIndex.end(), idxCC)
                - sortedIndex.begin();
            }
            for (int triIdx = 0; triIdx < base.triangleCount(); triIdx++) {
              auto tri   = base.triangle(triIdx);
              int  idxCC = trianglePartition[triIdx];
              //now add the points
              for (int i = 0; i < 3; i++) {
                connectedComponents[idxCC].addPoint(base.point(tri[i])
                                                    * iscalePosition);
              }
              connectedComponents[idxCC].addTriangle(
                connectedComponents[idxCC].points().size() - 3,
                connectedComponents[idxCC].points().size() - 2,
                connectedComponents[idxCC].points().size() - 1);
            }
            // now remove duplicate points for the base mesh to recover the correct normal
            std::vector<Vec3<MeshType>> pointsOutput;
            std::vector<Triangle>       trianglesOutput;
            std::vector<int32_t>        mapping;
            UnifyVertices(base.points(),
                          base.triangles(),
                          pointsOutput,
                          trianglesOutput,
                          mapping);
            swap(base.points(), pointsOutput);
            swap(base.triangles(), trianglesOutput);
            RemoveDegeneratedTriangles(base);
            break;
          }
          }
          // create the homography transform
          for (int idxCC = 0; idxCC < numCC; idxCC++) {
            connectedComponents[idxCC].setProjection(
              mpdu.getProjectionTextcoordProjectionId(idxCC));
            connectedComponents[idxCC].setOrientation(
              mpdu.getProjectionTextcoordOrientationId(idxCC));
            connectedComponents[idxCC].setU0(
              mpdu.getProjectionTextcoord2dPosX(idxCC));
            connectedComponents[idxCC].setV0(
              mpdu.getProjectionTextcoord2dPosY(idxCC));
            connectedComponents[idxCC].setSizeU(
              mpdu.getProjectionTextcoord2dSizeXMinus1(idxCC) + 1);
            connectedComponents[idxCC].setSizeV(
              mpdu.getProjectionTextcoord2dSizeYMinus1(idxCC) + 1);
            double patchScale = mpdu.getProjectionTextcoordFrameScale();
            if (mpdu.getProjectionTextcoordScalePresentFlag(idxCC)) {
              for (int i = 0;
                   i <= mpdu.getProjectionTextcoordSubpatchScale(idxCC);
                   i++)
                patchScale *= aspsVmcExt.getProjectionTextCoordScaleFactor();
            }
            connectedComponents[idxCC].setScale(patchScale);
            // calculate the bounding box
            bbBoxes[idxCC] = connectedComponents[idxCC].boundingBoxProjected(
              connectedComponents[idxCC].getProjection());
#ifdef DEBUG_ORTHO
            if (params.keepIntermediateFiles) {
              auto prefix = _keepFilesPathPrefix + "_transmitted_CC#"
                            + std::to_string(idxCC);
              connectedComponents[idxCC].save(prefix + "_DEC.obj");
            }
#endif
          }
          // create the (u,v) coordinate from (x,y,z) and the corresponding homography transform
          base.texCoordTriangles().resize(base.triangleCount());
          for (int triIdx = 0; triIdx < base.triangleCount(); triIdx++) {
            auto tri   = base.triangle(triIdx);
            int  idxCC = trianglePartition[triIdx];
            int  uvIdx[3];
            for (int i = 0; i < 3; i++) {
              auto newUV = connectedComponents[idxCC].convert(
                base.point(tri[i]) * iscalePosition,
                bbBoxes[idxCC],
                afpsVmcExt.getProjectionTextcoordWidth(mpdu.getSubmeshId()),
                afpsVmcExt.getProjectionTextcoordHeight(mpdu.getSubmeshId()),
                afpsVmcExt.getProjectionTextcoordGutter(mpdu.getSubmeshId()),
                1 << asps
                       .getLog2PatchPackingBlockSize());  //we are using the same block size as the geometry, should we change and use a specific one for the projection???
              //check if the newUV already exist
              auto pos = std::find(
                base.texCoords().begin(), base.texCoords().end(), newUV);
              if (pos == base.texCoords().end()) {
                uvIdx[i] = base.texCoords().size();
                base.addTexCoord(newUV);
              } else {
                uvIdx[i] = pos - base.texCoords().begin();
              }
            }
            //for degenerate triangles in 2D, since it can happen, but the saveOBJ function complains, we will create a new UV coordinate
            if (uvIdx[0] == uvIdx[1] && uvIdx[0] == uvIdx[2]) {
              uvIdx[1] = base.texCoords().size();
              base.addTexCoord(base.texCoords()[uvIdx[0]]);
              uvIdx[2] = base.texCoords().size();
              base.addTexCoord(base.texCoords()[uvIdx[0]]);
            } else if (uvIdx[0] == uvIdx[1]) {
              uvIdx[1] = base.texCoords().size();
              base.addTexCoord(base.texCoords()[uvIdx[0]]);
            } else if (uvIdx[0] == uvIdx[2]) {
              uvIdx[2] = base.texCoords().size();
              base.addTexCoord(base.texCoords()[uvIdx[0]]);
            } else if (uvIdx[1] == uvIdx[2]) {
              uvIdx[2] = base.texCoords().size();
              base.addTexCoord(base.texCoords()[uvIdx[1]]);
            }
            base.setTexCoordTriangle(
              triIdx, vmesh::Triangle(uvIdx[0], uvIdx[1], uvIdx[2]));
          }
        }
      }
    }

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      auto prefix = _keepFilesPathPrefix + "fr_"
                    + std::to_string(frameInfo.frameIndex) + "_base";
      base.save(prefix + "_dec.ply");
      save(prefix + ".drc", bmtdu.getData().vector());
    }
    qpositions.resize(base.pointCount());
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      qpositions[v] = base.point(v).round();
    }

    const auto scaleTexCoord =
      std::pow(2.0, bmsps.getQpTexCoordMinus1() + 1) - 1.0;
    const auto iscaleTexCoord = 1.0 / scaleTexCoord;
    for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
      base.texCoord(tc) *= iscaleTexCoord;
    }
    // Duplicated Vertex Reduction
    removeDuplicatedVertices(frame, umapping, frameInfo.type == I_BASEMESH);
    if (params.keepIntermediateFiles) {
      auto& baseClean = frame.baseClean;
      baseClean.save(_keepFilesPathPrefix + "fr_"
                     + std::to_string(frameInfo.frameIndex)
                     + "_baseClean.obj");
    }
    createVertexAdjTableMotion = true;
  } else if (frameInfo.type == P_BASEMESH) {
    // frameInfo.referenceFrameIndex =
    //   frameInfo.frameIndex - 1 - int32_t(bmth.getReferenceFrameIndex());
    const auto& refFrame = gof.frame(frameInfo.referenceFrameIndex);
    base                 = refFrame.base;
    decompressMotion(syntax,
                     bmtl,
                     refFrame.baseClean.triangles(),
                     refFrame.baseClean.points(),
                     refFrame.baseIntegrateIndices,
                     base.triangles(),
                     refFrame.qpositions,
                     qpositions,
                     params);
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      base.point(v) = qpositions[v];
    }
    // Duplicated Vertex Reduction
    removeDuplicatedVertices(frame, umapping, frameInfo.type == I_BASEMESH);
  } else {
    const auto& refFrame = gof.frame(frameInfo.referenceFrameIndex);
    base                 = refFrame.base;
    qpositions           = refFrame.qpositions;
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      base.point(v) = qpositions[v];
    }
    // Duplicated Vertex Reduction
    frame.baseClean            = refFrame.baseClean;
    frame.baseIntegrateIndices = refFrame.baseIntegrateIndices;
  }

  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.point(v) *= iscalePosition;
  }

  // Note JR: not referred to base mesh but to subdivision, should be moved outside of this function
  subdivideBaseMesh(frame,
                    rec,
                    ext.getSubdivisionMethod(),
                    ext.getSubdivisionIterationCount(),
                    ext.getInterpolateDisplacementNormals(),
                    ext.getAddReconstructedNormals());

  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressDisplacementsVideo(const V3cBitstream&         syntax,
                                         FrameSequence<uint16_t>&    dispVideo,
                                         const VMCDecoderParameters& params) {
  printf("Decompress displacements video \n");
  fflush(stdout);
  const auto& atlas        = syntax.getAtlas();
  auto&       displacement = atlas.getDisplacement();

  // Decode video
  auto& vps = syntax.getVps();
  auto& gi  = vps.getGeometryInformation(0);
  auto  decoder =
    VideoDecoder<uint16_t>::create(VideoCodecId(gi.getGeometryCodecId()));
  decoder->decode(displacement, dispVideo, 10);

  // Save intermediate files
  if (params.keepIntermediateFiles) {
    auto path = dispVideo.createName(_keepFilesPathPrefix + "disp_dec", 10);
    save(removeExtension(path) + ".bin", displacement);
    dispVideo.save(path);
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressDisplacementsAC(const V3cBitstream&   syntax,
                                      VMCGroupOfFrames&     gof,
                                      VMCGroupOfFramesInfo& gofInfo,
                                      int32_t               subBlockSize) {
  const auto&    displacement = syntax.getDisplacement();
  const auto&    buffer       = displacement.getData().vector();
  EntropyDecoder dispDecoder;
  dispDecoder.setBuffer(buffer.size(),
                        reinterpret_cast<const char*>(buffer.data()));
  dispDecoder.start();
  int32_t lodCount = int32_t(gof.frames[0].subdivInfoLevelOfDetails.size());
  std::vector<std::vector<VMCDispContext>> ctxDisp(2);
  ctxDisp[0].resize(lodCount);
  ctxDisp[1].resize(lodCount);
  std::vector<VMCDispContext> ctxNonzero(2);
  VMCDispContext              ctxBypass;

  for (int32_t frameIndex = 0; frameIndex < gofInfo.frameCount_;
       frameIndex++) {
    auto&       frame     = gof.frames[frameIndex];
    const auto& frameInfo = gofInfo.frameInfo(frameIndex);
    int32_t     ctxIdx;
    if (frameInfo.type == I_BASEMESH) {
      ctxIdx = 0;
    } else {
      ctxIdx = 1;
    }

    frame.disp.resize(frame.subdivInfoLevelOfDetails[lodCount - 1].pointCount);
    for (auto& disp : frame.disp) { disp = 0.0; }
    std::vector<int32_t> lod = {0};
    for (int32_t i = 0; i < lodCount; ++i) {
      lod.push_back(frame.subdivInfoLevelOfDetails[i].pointCount);
    }

    for (int32_t dim = 0; dim < 3; dim++) {
      int32_t lastPos  = -1;
      int32_t lastPosS = -1;
      int32_t lastPosT = -1;
      int32_t lastPosV = -1;
      auto    codedFlag =
        dispDecoder.decode(ctxNonzero[ctxIdx].ctxCoeffGtN[0][dim]);
      if (codedFlag == 0) {
        continue;
      } else {
        lastPos = dispDecoder.decodeExpGolomb(
          12, ctxNonzero[ctxIdx].ctxCoeffRemPrefix);
        for (int32_t i = 0; i < lodCount; ++i) {
          if (lod[i] <= lastPos && lastPos < lod[i + 1]) {
            lastPosS = i;
            break;
          }
        }
        lastPosT = (lastPos - lod[lastPosS]) / subBlockSize;
        lastPosV = (lastPos - lod[lastPosS]) % subBlockSize;
      }
      int32_t codedBlockFlag[10] = {0};
      for (int32_t level = lastPosS; level >= 0; --level) {
        if (level < lastPosS) {
          codedBlockFlag[level] =
            dispDecoder.decode(ctxDisp[ctxIdx][level].ctxCodedBlock[dim]);
          if (codedBlockFlag[level] == 0) { continue; }
        }
        int32_t codedSubBlockFlag[1000] = {0};
        auto    blockNum = (lod[level + 1] - lod[level]) / subBlockSize + 1;
        for (int32_t t = blockNum - 1; t >= 0; --t) {
          if ((level < lastPosS && codedBlockFlag[level] == 1)
              || (level == lastPosS && t < lastPosT)) {
            codedSubBlockFlag[t] =
              dispDecoder.decode(ctxDisp[ctxIdx][level].ctxCodedSubBlock[dim]);
            if (codedSubBlockFlag[t] == 0) { continue; }
          }
          auto subBlockSize_ = subBlockSize;
          if (t == blockNum - 1) {
            subBlockSize_ = (lod[level + 1] - lod[level]) % subBlockSize;
          }
          for (int32_t v = subBlockSize_ - 1; v >= 0; --v) {
            if (lastPosS == level) {
              if (lastPosT < t || (lastPosT == t && lastPosV < v)
                  || (t < lastPosT && codedSubBlockFlag[t] == 0)) {
                continue;
              }
            }
            int32_t value    = 0;
            int32_t coeffGt0 = 0;
            int32_t coeffGt1 = 0;
            int32_t coeffGt2 = 0;
            coeffGt0 =
              dispDecoder.decode(ctxDisp[ctxIdx][level].ctxCoeffGtN[0][dim]);
            if (coeffGt0) {
              const auto sign = dispDecoder.decode(ctxBypass.ctxStatic);
              ++value;
              coeffGt1 =
                dispDecoder.decode(ctxDisp[ctxIdx][level].ctxCoeffGtN[1][dim]);
              if (coeffGt1) {
                value += 1
                         + dispDecoder.decodeExpGolomb(
                           0, ctxBypass.ctxCoeffRemStatic);
              }
              if (sign) { value = -value; }
            }
            frame.disp[lod[level] + t * subBlockSize + v][dim] = value;
          }
        }
      }
    }
    if (ctxIdx) {
      const auto& frameRef = gof.frames[frameIndex - 1];
      for (int32_t vindex = 0; vindex < frame.disp.size(); vindex++) {
        frame.disp[vindex] += frameRef.disp[vindex];
      }
    }
  }  // frame loop
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompressTextureVideo(const V3cBitstream&         syntax,
                                   Sequence&                   reconsctruct,
                                   const VMCDecoderParameters& params) {
  printf("Decompress texture video\n");
  fflush(stdout);
  auto& vps = syntax.getVps();
  if (!vps.getAttributeVideoPresentFlag(0)) {
    for (auto& texture : reconsctruct.textures()) {
      texture.clear();
      texture.resize(1, 1, ColourSpace::BGR444p);
      texture.zero();
    }
  } else {
    auto& altas     = syntax.getAtlas();
    auto& attribute = altas.getAttribute();

    // Decode video
    printf("Decode video \n");
    fflush(stdout);
    FrameSequence<uint16_t> dec;
    auto&                   ai = vps.getAttributeInformation(0);
    auto                    decoder =
      VideoDecoder<uint16_t>::create(VideoCodecId(ai.getAttributeCodecId(0)));
    decoder->decode(attribute, dec, 10);

    if (dec.colourSpace() != ColourSpace::BGR444p
        && dec.colourSpace() != ColourSpace::GBR444p
        && dec.colourSpace() != ColourSpace::RGB444p) {
      // Convert video
      printf("Convert video \n");
      fflush(stdout);
#if USE_HDRTOOLS
      auto convert = ColourConverter<uint16_t>::create(1);
      convert->initialize(params.textureVideoHDRToolDecConfig);
#else
      auto convert = ColourConverter<uint16_t>::create(0);
      auto mode    = "YUV420p_BGR444p_10_8_"
                  + std::to_string(params.textureVideoUpsampleFilter) + "_"
                  + std::to_string(params.textureVideoFullRange);
      convert->initialize(mode);
#endif
      convert->convert(dec);
    }

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      FrameSequence<uint8_t> dec8(dec);
      auto path = dec8.createName(_keepFilesPathPrefix + "tex_dec", 8);
      dec8.save(path);
      save(removeExtension(path) + ".bin", attribute);
    }

    // Store result in 8 bits frames
    reconsctruct.textures() = dec;
    dec.clear();
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCDecoder::decompress(const V3cBitstream&         syntax,
                       VMCGroupOfFramesInfo&       gofInfoSrc,
                       Sequence&                   reconstruct,
                       const VMCDecoderParameters& params) {
  VMCGroupOfFrames     gof;
  VMCGroupOfFramesInfo gofInfo;
  gofInfo.index_           = gofInfoSrc.index_;
  gofInfo.startFrameIndex_ = gofInfoSrc.startFrameIndex_;
  printf("Decompress gop: \n");
  fflush(stdout);
  auto&                   vps        = syntax.getVps();
  const auto&             atlas      = syntax.getAtlas();
  const auto&             baseMesh   = syntax.getBaseMesh();
  const auto&             asps       = atlas.getAtlasSequenceParameterSet(0);
  const auto&             afps       = atlas.getAtlasFrameParameterSet(0);
  auto&                   ext        = asps.getAspsVdmcExtension();
  auto&                   gi         = vps.getGeometryInformation(0);
  const auto              frameCount = atlas.getFrameCount(asps, afps);
  FrameSequence<uint16_t> dispVideo;
  gofInfo.frameCount_ = frameCount;
  gofInfo.framesInfo_.resize(frameCount);
  gof.resize(frameCount);
  reconstruct.resize(frameCount);
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    auto& bmtl      = baseMesh.getBaseMeshTileLayer(frameIndex);
    auto& bmth      = bmtl.getHeader();
    auto& bmtdu     = bmtl.getDataUnit();
    auto& refList   = bmth.getRefListStruct();
    auto& frameInfo = gofInfo.framesInfo_[frameIndex];
    auto& frame     = gof.frame(frameIndex);
    auto& rec       = reconstruct.mesh(frameIndex);
    auto& atl       = atlas.getAtlasTileLayer(
      frameIndex);  // frameIndex and tileIndex are the same in this case, since we only have one tile per frame?
    frameInfo.frameIndex = frameIndex;
    frameInfo.type       = bmth.getBaseMeshType();
    if (bmth.getBaseMeshType() == P_BASEMESH) {
      if (refList.getStrafEntrySignFlag(0))
        frameInfo.referenceFrameIndex =
          frameInfo.frameIndex + refList.getAbsDeltaAfocSt(0);
      else
        frameInfo.referenceFrameIndex =
          frameInfo.frameIndex - refList.getAbsDeltaAfocSt(0);
    } else if (bmth.getBaseMeshType() == SKIP_BASEMESH) {
      frameInfo.referenceFrameIndex = frameInfo.frameIndex - 1;
    }
    printf("Frame %4d: type = %s ReferenceFrame = %4d \n",
           frameInfo.frameIndex,
           toString(frameInfo.type).c_str(),
           frameInfo.referenceFrameIndex);
    decompressBaseMesh(syntax, bmtl, atl, gof, frameInfo, frame, rec, params);
  }
  if (ext.getEncodeDisplacements() == 1) {
    decompressDisplacementsAC(syntax, gof, gofInfo, ext.getSubBlockSize());
  } else if (ext.getEncodeDisplacements() == 2) {
    decompressDisplacementsVideo(syntax, dispVideo, params);
  }
  if (ext.getEncodeDisplacements()) {
    for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
      auto& frame = gof.frame(frameIndex);
      auto& rec   = reconstruct.mesh(frameIndex);
      if (ext.getEncodeDisplacements() == 2) {
        reconstructDisplacementFromVideoFrame(
          dispVideo.frame(frameIndex),
          frame,
          rec,
          1 << asps.getLog2PatchPackingBlockSize(),
          gi.getGeometry2dBitdepthMinus1() + 1,
          ext.getDisplacement1D(),
          ext.getDisplacementReversePacking());
      }

      const auto bitDepthPosition = asps.getGeometry3dBitdepthMinus1() + 1;
      if (ext.getLodDisplacementQuantizationFlag()) {
        inverseQuantizeDisplacements(
          frame,
          bitDepthPosition,
          ext.getLiftingQuantizationParametersPerLevelOfDetails());
      } else {
        inverseQuantizeDisplacements(frame,
                                     bitDepthPosition,
                                     ext.getLiftingLevelOfDetailInverseScale(),
                                     ext.getLiftingQPs());
      }
      computeInverseLinearLifting(frame.disp,
                                  frame.subdivInfoLevelOfDetails,
                                  frame.subdivEdges,
                                  ext.getLiftingPredictionWeight(),
                                  ext.getLiftingUpdateWeight(),
                                  ext.getLiftingSkipUpdate());
      applyDisplacements(
        frame, bitDepthPosition, rec, ext.getDisplacementCoordinateSystem());
    }
  }

  // decompress texture
  if (!decompressTextureVideo(syntax, reconstruct, params)) { return false; }
  printf("Done \n");
  fflush(stdout);
  gofInfoSrc = gofInfo;

  // Quantize UV coordinate
  if (!params.dequantizeUV) {
    const auto bitDepthTexCoord = asps.getGeometry2dBitdepthMinus1() + 1;
    const auto scale            = (1 << bitDepthTexCoord) - 1.0;
    for (auto& rec : reconstruct.meshes()) {
      const auto texCoordCount = rec.texCoordCount();
      for (int32_t i = 0; i < texCoordCount; ++i) { rec.texCoord(i) *= scale; }
    }
  }
  // Reconstruct normals
  if (!params.reconstructNormals) {
    for (auto& rec : reconstruct.meshes()) { rec.resizeNormals(0); }
  }
  return true;
}

//============================================================================

}  // namespace vmesh
