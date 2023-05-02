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

#include "encoder.hpp"

#include <cmath>

#include <array>
#include <cstdio>
#include <sstream>
#include <unordered_map>

#include "motionContexts.hpp"
#include "entropy.hpp"
#include "vmc.hpp"
#include "metrics.hpp"
#include "geometryDecimate.hpp"
#include "textureParametrization.hpp"
#include "geometryParametrization.hpp"
#include "transferColor.hpp"
#include "util/checksum.hpp"
#include "util/misc.hpp"

#include "geometryEncoder.hpp"
#include "videoEncoder.hpp"
#include "colourConverter.hpp"

namespace vmesh {

//----------------------------------------------------------------------------

static bool
saveCache(const VMCEncoderParameters&   params,
          const TriangleMesh<MeshType>& mesh,
          const std::string             name,
          const int32_t                 gofIndex,
          const int32_t                 frameIndex) {
  if (params.cachingDirectory.empty()) return false;
  return mesh.save(params.cachingDirectory + separator() + name
                   + expandNum("_gof%02d", gofIndex)
                   + expandNum("_fr%04d.vmb", frameIndex));
}

//----------------------------------------------------------------------------

static bool
loadCache(const VMCEncoderParameters& params,
          TriangleMesh<MeshType>&     mesh,
          const std::string           name,
          const int32_t               gofIndex,
          const int32_t               frameIndex) {
  mesh.clear();
  if (params.cachingDirectory.empty()) return false;
  return mesh.load(params.cachingDirectory + separator() + name
                   + expandNum("_gof%02d", gofIndex)
                   + expandNum("_fr%04d.vmb", frameIndex));
}

//============================================================================

int32_t
initialize(V3cBitstream&               syntax,
           const VMCEncoderParameters& params,
           const VMCGroupOfFramesInfo& gofInfo) {
  const uint8_t aspsId  = 0;
  const uint8_t afpsId  = 0;
  const uint8_t bmspsId = 0;
  const uint8_t bmfpsId = 0;

  // Allocate VPS, Atlas and base mesh
  syntax.allocateAtlas(1);
  syntax.allocateBaseMesh(1);
  syntax.getActiveVpsId()   = 0;
  syntax.getAtlasIndex()    = 0;
  syntax.getBaseMeshIndex() = 0;

  // V3C parameter set
  auto& vps = syntax.addV3CParameterSet(0);
  vps.allocateAtlas(1);
  vps.getV3CParameterSetId()               = 0;
  vps.getAtlasId(0)                        = 0;
  vps.getFrameWidth(0)                     = params.textureWidth;
  vps.getFrameHeight(0)                    = params.textureHeight;
  vps.getMapCountMinus1(0)                 = 0;
  vps.getMultipleMapStreamsPresentFlag(0)  = 0;
  vps.getAuxiliaryVideoPresentFlag(0)      = false;
  vps.getOccupancyVideoPresentFlag(0)      = false;
  vps.getGeometryVideoPresentFlag(0)       = params.encodeDisplacementsVideo;
  vps.getAttributeVideoPresentFlag(0)      = params.encodeTextureVideo;
  vps.getMapAbsoluteCodingEnableFlag(0, 0) = true;
  vps.getMapPredictorIndexDiff(0, 0)       = false;

  // Geometry information
  auto& gi                         = vps.getGeometryInformation(0);
  gi.getGeometry2dBitdepthMinus1() = params.geometryVideoBitDepth - 1;
  gi.getGeometryCodecId()          = params.geometryVideoCodecId;

  // Attribute information
  auto& ai = vps.getAttributeInformation(0);
  ai.allocate(1);
  ai.getAttribute2dBitdepthMinus1(0) = params.textureVideoBitDepth - 1;
  ai.getAttributeCodecId(0)          = params.textureVideoCodecId;

  // Atlas
  auto& atlas = syntax.getAtlas();

  // Atlas sequence parameter set
  auto& asps                         = atlas.addAtlasSequenceParameterSet();
  asps.getFrameWidth()               = params.textureWidth;
  asps.getFrameHeight()              = params.textureHeight;
  asps.getGeometry3dBitdepthMinus1() = params.bitDepthPosition - 1;
  asps.getGeometry2dBitdepthMinus1() = params.bitDepthTexCoord - 1;
  asps.getLog2PatchPackingBlockSize() =
    log2(params.displacementVideoBlockSize);
  asps.getExtensionFlag()     = 1;
  asps.getVdmcExtensionFlag() = 1;

  // ASPS extension
  auto& ext                        = asps.getAspsVdmcExtension();
  ext.getAddReconstructedNormals() = params.addReconstructedNormals;
  ext.getSubdivisionMethod()       = params.intraGeoParams.subdivisionMethod;
  ext.getSubdivisionIterationCount() =
    uint8_t(params.liftingSubdivisionIterationCount);
  ext.getLiftingQPs(0) = uint8_t(params.liftingQP[0]);
  ext.getLiftingQPs(1) = uint8_t(params.liftingQP[1]);
  ext.getLiftingQPs(2) = uint8_t(params.liftingQP[2]);
  ext.getInterpolateDisplacementNormals() =
    params.interpolateDisplacementNormals;
  ext.getDisplacementReversePacking() = params.displacementReversePacking;

  // Atlas frame parameter set
  auto& afps                            = atlas.addAtlasFrameParameterSet();
  afps.getAtlasSequenceParameterSetId() = aspsId;
  afps.getAtlasFrameParameterSetId()    = afpsId;

  // Atlas frame tile information
  auto& afti                           = afps.getAtlasFrameTileInformation();
  afti.getSingleTileInAtlasFrameFlag() = true;
  afti.getSignalledTileIdFlag()        = false;

  // Atlas tile layer
  for (size_t i = 0; i < gofInfo.frameCount_; i++) {
    auto& atl                            = atlas.addAtlasTileLayer();
    auto& ath                            = atl.getHeader();
    atl.getAtlasFrmOrderCntVal()         = i;
    ath.getAtlasFrmOrderCntLsb()         = i;
    ath.getAtlasFrameParameterSetId()    = afpsId;
    ath.getNumRefIdxActiveOverrideFlag() = false;
    ath.getRefAtlasFrameListSpsFlag()    = false;
    ath.getRefAtlasFrameListIdx()        = 0;
    ath.getId()                          = 0;
    ath.getType()                        = SKIP_TILE;

    // init ref list;
    auto& refList              = ath.getRefListStruct();
    refList.getNumRefEntries() = 0;
    refList.allocate();
    for (size_t j = 0; j < refList.getNumRefEntries(); j++) {
      int32_t afocDiff                  = j == 0 ? -1 : 1;
      refList.getAbsDeltaAfocSt(j)      = std::abs(afocDiff);
      refList.getStrafEntrySignFlag(j)  = afocDiff < 0 ? false : true;
      refList.getStRefAtlasFrameFlag(j) = true;
    }
  }

  // Mesh sub bitstream
  auto& baseMesh = syntax.getBaseMesh();

  // Base mesh sequence parameter set
  auto& bmsps = baseMesh.addBaseMeshSequenceParameterSet(bmspsId);
  bmsps.getBaseMeshCodecId()  = params.meshCodecId;
  bmsps.getQpPositionMinus1() = params.qpPosition - 1;
  bmsps.getQpTexCoordMinus1() = params.qpTexCoord - 1;

  // Base mesh frame parameter set
  auto& bmfps = baseMesh.addBaseMeshFrameParameterSet(bmfpsId);
  bmfps.getBaseMeshSequenceParameterSetId() = bmspsId;
  bmfps.getBaseMeshFrameParameterSetId()    = bmfpsId;
  bmfps.getMotionGroupSize()                = uint8_t(params.motionGroupSize);

  // Base mesh frame parameter set
  for (size_t i = 0; i < gofInfo.frameCount_; i++) {
    auto& bmtl                            = baseMesh.addBaseMeshTileLayer();
    auto& bmth                            = bmtl.getHeader();
    auto& bmtdu                           = bmtl.getDataUnit();
    bmtl.getBaseMeshFrmOrderCntVal()      = i;
    bmth.getBaseMeshFrmOrderCntLsb()      = i;
    bmth.getBaseMeshFrameParameterSetId() = bmfpsId;
    bmth.getNumRefIdxActiveOverrideFlag() = false;
    bmth.getRefBaseMeshFrameListSpsFlag() = false;
    bmth.getRefBaseMeshFrameListIdx()     = 0;
    bmth.getBaseMeshId()                  = 0;
    // init ref list;
    auto& refList              = bmth.getRefListStruct();
    refList.getNumRefEntries() = 0;
    refList.allocate();
    for (size_t j = 0; j < refList.getNumRefEntries(); j++) {
      int32_t afocDiff                  = j == 0 ? -1 : 1;
      refList.getAbsDeltaAfocSt(j)      = std::abs(afocDiff);
      refList.getStrafEntrySignFlag(j)  = afocDiff < 0 ? false : true;
      refList.getStRefAtlasFrameFlag(j) = true;
    }
  }

  // Video streams
  if (params.encodeDisplacementsVideo)
    atlas.createVideoBitstream(vmesh::VIDEO_GEOMETRY);
  if (params.encodeTextureVideo)
    atlas.createVideoBitstream(vmesh::VIDEO_ATTRIBUTE);
  return 0;
}

//----------------------------------------------------------------------------

void
VMCEncoder::decimateInput(const TriangleMesh<MeshType>& input,
                          VMCFrame&                     frame,
                          TriangleMesh<MeshType>&       decimate,
                          const VMCEncoderParameters&   params) {
  std::cout << "Simplify  " << frame.frameIndex << "\n";
  const auto index  = frame.frameIndex;
  auto       prefix = _keepFilesPathPrefix + "fr_" + std::to_string(index);
  // Load cache files
  bool skipSimplify = false;
  if (params.cachingPoint >= CachingPoint::SIMPLIFY) {
    if (loadCache(params, frame.mapped, "mapped", _gofIndex, index)
        && loadCache(params, frame.reference, "reference", _gofIndex, index)
        && loadCache(params, decimate, "decimate", _gofIndex, index)) {
      skipSimplify = true;
    }
  }
  printf("skipSimplify = %d \n", skipSimplify);
  fflush(stdout);
  if (!skipSimplify) {
    // Save intermediate files
    if (params.keepIntermediateFiles) {
      input.save(prefix + "_simp_input.ply");
    }
    // Create mapped, reference and decimate
    GeometryDecimate geometryDecimate;
    if (geometryDecimate.generate(
          input, frame.reference, decimate, frame.mapped, params)) {
      std::cerr << "Error: can't simplify mesh !\n";
      exit(-1);
    }

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      frame.mapped.save(prefix + "_mapped.ply");
      frame.reference.save(prefix + "_reference.ply");
      decimate.save(prefix + "_decimate.ply");
    }

    // Save cache files
    if (params.cachingPoint >= CachingPoint::SIMPLIFY) {
      saveCache(params, frame.mapped, "mapped", _gofIndex, index);
      saveCache(params, frame.reference, "reference", _gofIndex, index);
      saveCache(params, decimate, "decimate", _gofIndex, index);
    }
  }
}
//============================================================================

void
VMCEncoder::removeDegeneratedTrianglesCrossProduct(
  TriangleMesh<MeshType>& mesh,
  const int32_t&          frameIndex) {
  TriangleMesh<MeshType> degenerateMesh;

  auto& degenerateTriangles         = degenerateMesh.triangles();
  auto& degenerateTexCoordTriangles = degenerateMesh.texCoordTriangles();
  auto& degenerateNormalTriangles   = degenerateMesh.normalTriangles();
  degenerateMesh.points().resize(mesh.pointCount());

  auto triangleCount = mesh.triangleCount();
  if (triangleCount <= 0) { return; }

  const auto hasTexCoords = mesh.texCoordTriangleCount() == triangleCount;
  const auto hasNormals   = mesh.normalTriangleCount() == triangleCount;
  std::vector<Triangle> triangles;
  triangles.reserve(triangleCount);

  int32_t removedDegeneratedTrianglesArea0 = 0;

  for (int32_t tindex = 0; tindex < triangleCount; tindex++) {
    const auto& tri   = mesh.triangle(tindex);
    const auto& coord = mesh.points();

    const auto i = tri[0];
    const auto j = tri[1];
    const auto k = tri[2];
    auto       area =
      computeTriangleArea(coord[tri[0]], coord[tri[1]], coord[tri[2]]);

    if (area > 0) {
      triangles.push_back(tri);
    } else {
      std::cout << "Triangle with area 0 is removed." << std::endl;
      removedDegeneratedTrianglesArea0++;
      degenerateTriangles.push_back(tri);
    }
  }

  std::cout << "frame : " << frameIndex
            << " , removedDegeneratedTrianglesArea0count = "
            << removedDegeneratedTrianglesArea0 << std::endl;

  std::swap(mesh.triangles(), triangles);

  //Change connectivity
  if (1) {
    triangles.clear();

    triangleCount                = mesh.triangleCount();
    auto degenerateTriangleCount = degenerateMesh.triangleCount();

    std::vector<int32_t> vertexID0s(degenerateTriangleCount),
      vertexID1s(degenerateTriangleCount), vertexID2s(degenerateTriangleCount);

    for (uint32_t degetindex = 0; degetindex < degenerateTriangleCount;
         degetindex++) {
      auto& degetri = degenerateTriangles[degetindex];

      const auto& coord = mesh.points();

      //Check middle point
      auto                doubleMax = std::numeric_limits<double>::max();
      auto                doubleMin = std::numeric_limits<double>::min();
      std::vector<double> minV      = {doubleMax, doubleMax, doubleMax};
      std::vector<double> maxV      = {doubleMin, doubleMin, doubleMin};

      std::vector<int> minID = {0, 0, 0}, maxID = {0, 0, 0};

      for (int vertexID = 0; vertexID < 3; vertexID++) {
        auto& p = coord[degetri[vertexID]];

        //xyz
        for (int i = 0; i < 3; i++) {
          if (p[i] < minV[i]) {
            minV[i]  = p[i];
            minID[i] = vertexID;
          }
          if (p[i] > maxV[i]) {
            maxV[i]  = p[i];
            maxID[i] = vertexID;
          }
        }
      }

      int middleID = 0;
      for (int vertexID = 0; vertexID < 3; vertexID++) {
        //xyz
        bool flag = false;
        for (int i = 0; i < 3; i++) {
          if (minID[i] == vertexID || maxID[i] == vertexID) { flag = 1; }
        }
        if (!flag) {
          middleID = vertexID;
          break;
        }
      }

      auto& vertexID0 = vertexID0s[degetindex];
      auto& vertexID1 = vertexID1s[degetindex];
      auto& vertexID2 = vertexID2s[degetindex];

      if (middleID == 0) {
        vertexID0 = degetri[0];
        vertexID1 = degetri[1];
        vertexID2 = degetri[2];
      } else if (middleID == 1) {
        vertexID0 = degetri[1];
        vertexID1 = degetri[2];
        vertexID2 = degetri[0];
      } else if (middleID == 2) {
        vertexID0 = degetri[2];
        vertexID1 = degetri[0];
        vertexID2 = degetri[1];
      }
    }

    for (int32_t tindex = 0; tindex < triangleCount; tindex++) {
      const auto& tri   = mesh.triangle(tindex);
      const auto& coord = mesh.points();

      bool      addedFlag = false;
      Vec3<int> newFace1, newFace2;

      for (uint32_t degetindex = 0;
           !addedFlag && (degetindex < degenerateTriangleCount);
           degetindex++) {
        auto& vertexID0 = vertexID0s[degetindex];
        auto& vertexID1 = vertexID1s[degetindex];
        auto& vertexID2 = vertexID2s[degetindex];

        std::vector<Vec3<int>> vIDs{{tri[0], tri[1], tri[2]},
                                    {tri[1], tri[2], tri[0]},
                                    {tri[2], tri[0], tri[1]}};
        for (auto& vID : vIDs) {
          auto& vID0 = vID[0];
          auto& vID1 = vID[1];
          auto& vID2 = vID[2];

          if ((vID1 == vertexID1 && vID2 == vertexID2)
              || (vID1 == vertexID2 && vID2 == vertexID1)) {
            newFace1 = {vID1, vertexID0, vID0};
            newFace2 = {vID0, vertexID0, vID2};

            triangles.push_back(newFace1);
            triangles.push_back(newFace2);
            addedFlag = true;
            break;
          }
        }
      }

      if (!addedFlag) triangles.push_back(tri);
    }
    std::swap(mesh.triangles(), triangles);
  }
}
//============================================================================

void
VMCEncoder::textureParametrization(VMCFrame&                   frame,
                                   TriangleMesh<MeshType>&     decimate,
                                   const VMCEncoderParameters& params) {
  auto prefix =
    _keepFilesPathPrefix + "fr_" + std::to_string(frame.frameIndex);
  std::cout << "Texture parametrization \n";

  // Load cache files
  bool skipUVAtlas = false;
  if (params.cachingPoint >= CachingPoint::UVATLAS) {
    if (loadCache(params,
                  frame.decimateTexture,
                  "decimateTexture",
                  _gofIndex,
                  frame.frameIndex)) {
      skipUVAtlas = true;
    }
  }

  printf("skipUVAtlas = %d \n", skipUVAtlas);
  fflush(stdout);
  if (!skipUVAtlas) {
    //Remove degenerate triangles
    removeDegeneratedTrianglesCrossProduct(decimate, frame.frameIndex);

    TextureParametrization textureParametrization;
    textureParametrization.generate(decimate, frame.decimateTexture, params);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      frame.decimateTexture.save(prefix + "_decimateTexture.ply");
    }

    // Save cache files
    if (params.cachingPoint >= CachingPoint::UVATLAS) {
      saveCache(params,
                frame.decimateTexture,
                "decimateTexture",
                _gofIndex,
                frame.frameIndex);
    }
  }
}

//----------------------------------------------------------------------------

void
VMCEncoder::geometryParametrization(VMCGroupOfFrames&             gof,
                                    VMCGroupOfFramesInfo&         gofInfo,
                                    VMCFrame&                     frame,
                                    const TriangleMesh<MeshType>& input,
                                    const VMCEncoderParameters&   params,
                                    int32_t& lastIntraFrameIndex) {
  const auto frameIndex = frame.frameIndex;
  auto prefix = _keepFilesPathPrefix + "fr_" + std::to_string(frameIndex);
  // Load cache files
  bool skipSubDiv = false;
  if (params.cachingPoint >= CachingPoint::SUBDIV) {
    if (loadCache(params, frame.base, "base", _gofIndex, frameIndex)
        && loadCache(params, frame.subdiv, "subdiv", _gofIndex, frameIndex)) {
      skipSubDiv = true;
    }
  }
  printf("skipSubDiv = %d \n", skipSubDiv);
  fflush(stdout);
  if (!skipSubDiv) {
    // Intra geometry parametrization
    std::cout << "Intra geometry parametrization\n";
    GeometryParametrization fitsubdivIntra;
    TriangleMesh<MeshType>  mtargetIntra, subdiv0Intra;
    TriangleMesh<MeshType>  baseIntra, subdivIntra, nsubdivIntra;
    TriangleMesh<MeshType>  baseInter, subdivInter, nsubdivInter;
    fitsubdivIntra.generate(frame.reference,        // target
                            frame.decimateTexture,  // source
                            frame.mapped,           // mapped
                            mtargetIntra,           // mtarget
                            subdiv0Intra,           // subdiv0
                            params.intraGeoParams,  // params
                            baseIntra,              // base
                            subdivIntra,            // deformed
                            nsubdivIntra);          // ndeformed

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      baseIntra.save(prefix + "_intra_base.ply");
      subdivIntra.save(prefix + "_intra_subdiv.ply");
      nsubdivIntra.save(prefix + "_intra_nsubdiv.ply");
    }
    bool  chooseIntra = true;
    auto& frameInfo   = gofInfo.frameInfo(frameIndex);
    if (params.subdivInter && frameIndex > 0
        && ((!params.subdivInterWithMapping)
            || frameInfo.referenceFrameIndex != -1)) {
      // Inter geometry parametrization
      std::cout << "Inter geometry parametrization: withMapping = "
                << params.subdivInterWithMapping << "\n";
      GeometryParametrization fitsubdivInter;
      if (params.subdivInterWithMapping) {
        // With mapping
        printf("ref frame= %d \n", frameInfo.referenceFrameIndex);
        fflush(stdout);
        frameInfo.referenceFrameIndex =
          std::max(frameInfo.referenceFrameIndex, lastIntraFrameIndex);
        auto&                  ref = gof.frame(frameInfo.referenceFrameIndex);
        TriangleMesh<MeshType> mappedInter;
        TriangleMesh<MeshType> subdiv0Inter;
        fitsubdivInter.generate(ref.reference,          // target
                                ref.decimateTexture,    // source
                                ref.mapped,             // mapped
                                input,                  // mtarget
                                subdiv0Inter,           // subdiv0
                                params.interGeoParams,  // params
                                baseInter,              // base
                                subdivInter,            // deformed
                                nsubdivInter);          // ndeformed
      } else {
        // Without mapping
        // subdiv0 : ld frame - 1 subdiv
        // souce   : ld frame - 1 base
        // target  : ld frame - 1 reference
        auto&                  ref = gof.frame(frameIndex - 1);
        TriangleMesh<MeshType> mappedInter;
        TriangleMesh<MeshType> mtargetInter;
        fitsubdivInter.generate(frame.reference,        // target
                                ref.base,               // source
                                mappedInter,            // mapped
                                mtargetInter,           // mtarget
                                ref.subdiv,             // subdiv0
                                params.interGeoParams,  // params
                                baseInter,              // base
                                subdivInter,            // deformed
                                nsubdivInter);          // ndeformed
      }

      // Save intermediate files
      if (params.keepIntermediateFiles) {
        baseInter.save(prefix + "_inter_base.ply");
        subdivInter.save(prefix + "_inter_subdiv.ply");
        nsubdivInter.save(prefix + "_inter_nsubdiv.ply");
      }

      // ComputeMetric/Decision
      VMCMetrics           metricsIntra;
      VMCMetrics           metricsInter;
      VMCMetricsParameters metricParams;
      metricParams.computePcc = true;
      metricParams.qp         = params.bitDepthPosition;
      metricParams.qt         = params.bitDepthTexCoord;
      for (int c = 0; c < 3; c++) {
        metricParams.minPosition[c] = params.minPosition[c];
        metricParams.maxPosition[c] = params.maxPosition[c];
      }
      metricParams.normalCalcModificationEnable =
        params.normalCalcModificationEnable;
      metricsIntra.compute(input, subdivIntra, metricParams);
      metricsInter.compute(input, subdivInter, metricParams);
      auto metIntra = metricsIntra.getPccResults();
      auto metInter = metricsInter.getPccResults();
      int  pos      = 1;
      chooseIntra =
        metIntra[pos] - metInter[pos] > params.maxAllowedD2PSNRLoss;
      printf("Frame %3d: choose %s: met %8.4f - %8.4f = %8.4f > %8.4f \n",
             frameIndex,
             chooseIntra ? "Intra" : "Inter",
             metIntra[pos],
             metInter[pos],
             metIntra[pos] - metInter[pos],
             params.maxAllowedD2PSNRLoss);
    }
    if (chooseIntra) {
      frame.base                    = baseIntra;
      frame.subdiv                  = subdivIntra;
      frameInfo.type                = I_BASEMESH;
      frameInfo.referenceFrameIndex = -1;
      lastIntraFrameIndex           = frameIndex;
    } else {
      frame.base                    = baseInter;
      frame.subdiv                  = subdivInter;
      frameInfo.type                = P_BASEMESH;
      frameInfo.referenceFrameIndex = frameInfo.frameIndex - 1;
      printf("Update referenceFrameIndex = %d \n",
             frameInfo.referenceFrameIndex);
    }

    // Save cache files
    if (params.cachingPoint >= CachingPoint::SUBDIV) {
      saveCache(params, frame.base, "base", _gofIndex, frameIndex);
      saveCache(params, frame.subdiv, "subdiv", _gofIndex, frameIndex);
    }
  }
}

//============================================================================

void
VMCEncoder::unifyVertices(const VMCGroupOfFramesInfo& gofInfo,
                          VMCGroupOfFrames&           gof,
                          const VMCEncoderParameters& params) {
  const auto                        startFrame = gofInfo.startFrameIndex_;
  const auto                        frameCount = gofInfo.frameCount_;
  const auto                        lastFrame  = startFrame + frameCount - 1;
  std::vector<std::vector<int32_t>> umappings(frameCount);
  for (int f = startFrame; f <= lastFrame; ++f) {
    const auto findex = f - startFrame;
    auto&      frame  = gof.frames[findex];
    frame.base.resizeNormals(0);
    frame.base.resizeNormalTriangles(0);
    frame.subdiv.resizeNormals(0);
    frame.subdiv.resizeNormalTriangles(0);
    frame.reference.clear();
    frame.mapped.clear();
    frame.decimateTexture.clear();
    if (params.unifyVertices && params.liftingSubdivisionIterationCount == 0) {
      auto& base   = frame.base;
      auto& subdiv = frame.subdiv;
      assert(base.pointCount() == subdiv.pointCount());
      assert(base.triangleCount() == subdiv.triangleCount());
      assert(base.texCoordCount() == subdiv.texCoordCount());
      assert(base.texCoordTriangleCount() == subdiv.texCoordTriangleCount());
      std::vector<Vec3<MeshType>> upoints;
      std::vector<Triangle>       utriangles;
      const auto                  pointCount0 = base.pointCount();
      const auto&                 frameInfo   = gofInfo.frameInfo(findex);
      auto&                       umapping    = umappings[findex];
      if (frameInfo.type == I_BASEMESH) {
        printf("Frame %2d: INTRA \n", findex);
        UnifyVertices(
          base.points(), base.triangles(), upoints, utriangles, umapping);
        std::swap(base.points(), upoints);
        std::swap(base.triangles(), utriangles);
        RemoveDegeneratedTriangles(base);
        const auto pointCount1 = base.pointCount();
        assert(pointCount1 <= pointCount0);
        upoints.resize(pointCount1);
        for (int32_t vindex = 0; vindex < pointCount0; ++vindex) {
          upoints[umapping[vindex]] = subdiv.point(vindex);
        }
        std::swap(subdiv.points(), upoints);
        subdiv.triangles()         = base.triangles();
        subdiv.texCoordTriangles() = base.texCoordTriangles();
      } else {
        printf("Frame %2d: INTER: Index = %2d ref = %2d Previous = %2d \n",
               findex,
               frameInfo.frameIndex,
               frameInfo.referenceFrameIndex,
               frameInfo.previousFrameIndex);
        umapping             = umappings[frameInfo.previousFrameIndex];
        const auto& refFrame = gof.frames[frameInfo.previousFrameIndex];
        upoints              = base.points();
        base                 = refFrame.base;
        for (int32_t vindex = 0; vindex < pointCount0; ++vindex) {
          base.setPoint(umapping[vindex], upoints[vindex]);
        }
        upoints = subdiv.points();
        subdiv  = refFrame.subdiv;
        for (int32_t vindex = 0; vindex < pointCount0; ++vindex) {
          subdiv.setPoint(umapping[vindex], upoints[vindex]);
        }
      }
      // Save intermediate files
      if (params.keepIntermediateFiles) {
        const auto prefix =
          _keepFilesPathPrefix + "fr_" + std::to_string(findex);
        frame.base.save(prefix + "_base_uni.ply");
        frame.subdiv.save(prefix + "_subdiv_uni.ply");
      }
    }
  }
}

//============================================================================

bool
VMCEncoder::compressDisplacementsVideo(FrameSequence<uint16_t>&    dispVideo,
                                       V3cBitstream&               syntax,
                                       const VMCEncoderParameters& params) {
  //Encode
  VideoEncoderParameters videoEncoderParams;
  videoEncoderParams.encoderConfig_    = params.geometryVideoEncoderConfig;
  videoEncoderParams.inputBitDepth_    = params.geometryVideoBitDepth;
  videoEncoderParams.internalBitDepth_ = params.geometryVideoBitDepth;
  videoEncoderParams.outputBitDepth_   = params.geometryVideoBitDepth;
  videoEncoderParams.qp_               = 8;
  FrameSequence<uint16_t> rec;
  printf("geometryVideoCodecId = %d \n", (int)params.geometryVideoCodecId);
  fflush(stdout);
  auto& atlas        = syntax.getAtlas();
  auto& displacement = atlas.getDisplacement();
  auto& vps          = syntax.getVps();
  auto& gi           = vps.getGeometryInformation(0);
  auto  encoder =
    VideoEncoder<uint16_t>::create(VideoCodecId(gi.getGeometryCodecId()));
  encoder->encode(dispVideo, videoEncoderParams, displacement, rec);

  // Save intermediate files
  if (params.keepIntermediateFiles) {
    auto prefix = _keepFilesPathPrefix + "disp";
    dispVideo.save(dispVideo.createName(prefix + "_enc", 10));
    rec.save(rec.createName(prefix + "_rec", 10));
    save(prefix + ".h265", displacement);
  }
  dispVideo                     = rec;
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::compressTextureVideo(Sequence&                   reconstruct,
                                 V3cBitstream&               syntax,
                                 const VMCEncoderParameters& params) {
  printf("Compress texture video \n");
  fflush(stdout);
  if (!params.encodeTextureVideo) {
    for (auto& texture : reconstruct.textures()) {
      texture.clear();
      texture.resize(1, 1, ColourSpace::BGR444p);
      texture.zero();
    }
  } else {
    const auto width      = params.textureWidth;
    const auto height     = params.textureHeight;
    const auto frameCount = reconstruct.frameCount();
    // Save source video
    if (params.keepIntermediateFiles) {
      FrameSequence<uint8_t> bgrSrc(
        width, height, ColourSpace::BGR444p, frameCount);
      for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex)
        bgrSrc[frameIndex] = reconstruct.texture(frameIndex);
      bgrSrc.save(bgrSrc.createName(_keepFilesPathPrefix + "tex_src", 8));
    }

    // convert BGR444 8bits to BGR444 10bits
    FrameSequence<uint16_t> src(reconstruct.textures());
    printf("reconstruct size = %d %d \n",
           reconstruct.textures().width(),
           reconstruct.textures().height());
    printf("src size = %d %d \n", src.width(), src.height());
    if (!params.textureBGR444) {
      // convert BGR444 to YUV420
#if USE_HDRTOOLS
      auto convert = ColourConverter<uint16_t>::create(1);
      convert->initialize(params.textureVideoHDRToolEncConfig);
#else
      auto convert = ColourConverter<uint16_t>::create(0);
      auto mode    = "BGR444p_YUV420p_8_10_"
                  + std::to_string(params.textureVideoDownsampleFilter) + "_"
                  + std::to_string(params.textureVideoFullRange);
      convert->initialize(mode);
#endif
      convert->convert(src);
    } else {
      src.resize(width, height, ColourSpace::BGR444p, frameCount);
    }

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      src.save(src.createName(_keepFilesPathPrefix + "tex_enc", 10));
    }

    //Encode
    VideoEncoderParameters videoEncoderParams;
    videoEncoderParams.encoderConfig_    = params.textureVideoEncoderConfig;
    videoEncoderParams.inputBitDepth_    = params.textureVideoBitDepth;
    videoEncoderParams.internalBitDepth_ = params.textureVideoBitDepth;
    videoEncoderParams.outputBitDepth_   = params.textureVideoBitDepth;
    videoEncoderParams.qp_               = params.textureVideoQP;
    FrameSequence<uint16_t> rec;
    auto&                   atlas     = syntax.getAtlas();
    auto&                   attribute = atlas.getAttribute();
    auto&                   vps       = syntax.getVps();
    auto&                   ai        = vps.getAttributeInformation(0);
    auto                    encoder =
      VideoEncoder<uint16_t>::create(VideoCodecId(ai.getAttributeCodecId(0)));
    encoder->encode(src, videoEncoderParams, attribute, rec);
    src.clear();
    printf("encode texture video done \n");
    fflush(stdout);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      rec.save(rec.createName(_keepFilesPathPrefix + "tex_rec", 10));
      save(_keepFilesPathPrefix + ".h265", attribute);
    }

    // Convert Rec YUV420 to BGR444
    if (!params.textureBGR444) {
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
      convert->convert(rec);
    }

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      FrameSequence<uint8_t> rec8(rec);
      rec8.save(rec8.createName(_keepFilesPathPrefix + "tex_rec", 8));
    }

    // convert BGR444 10bits to BGR444 8bits
    reconstruct.textures() = rec;
    rec.clear();
    printf("compressTextureVideo done \n");
    fflush(stdout);
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::computeDracoMapping(TriangleMesh<MeshType>      base,
                                std::vector<int32_t>&       mapping,
                                const int32_t               frameIndex,
                                const VMCEncoderParameters& params) const {
  // Save intermediate files
  if (params.keepIntermediateFiles) {
    base.save(_keepFilesPathPrefix + "fr_" + std::to_string(frameIndex)
              + "_mapping_src.ply");
  }
  // Scale
  const auto scalePosition = 1 << (18 - params.bitDepthPosition);
  const auto scaleTexCoord = 1 << 18;
  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.setPoint(v, Round(base.point(v) * scalePosition));
  }
  for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
    base.setTexCoord(tc, Round(base.texCoord(tc) * scaleTexCoord));
  }
  // Save intermediate files
  if (params.keepIntermediateFiles) {
    base.save(_keepFilesPathPrefix + "fr_" + std::to_string(frameIndex)
              + "_mapping_scale.ply");
  }

  // Encode
  GeometryEncoderParameters encoderParams;
  encoderParams.dracoUsePosition_  = params.dracoUsePosition;
  encoderParams.dracoUseUV_        = params.dracoUseUV;
  encoderParams.dracoMeshLossless_ = params.dracoMeshLossless;
  TriangleMesh<MeshType> rec;
  std::vector<uint8_t>   geometryBitstream;
  auto encoder = GeometryEncoder<MeshType>::create(params.meshCodecId);
  printf("DracoMapping: use_position = %d use_uv = %d mesh_lossless = %d \n",
         encoderParams.dracoUsePosition_,
         encoderParams.dracoUseUV_,
         encoderParams.dracoMeshLossless_);
  encoder->encode(base, encoderParams, geometryBitstream, rec);

  // Save intermediate files
  if (params.keepIntermediateFiles) {
    auto prefix =
      _keepFilesPathPrefix + "fr_" + std::to_string(frameIndex) + "_mapping";
    base.save(prefix + "_enc.ply");
    rec.save(prefix + "_rec.ply");
    save(prefix + ".drc", geometryBitstream);
  }

  // Geometry parametrisation base
  auto fsubdiv0 = base;
  fsubdiv0.subdivideMidPoint(params.liftingSubdivisionIterationCount);
  std::vector<int32_t> texCoordToPoint0;
  fsubdiv0.computeTexCoordToPointMapping(texCoordToPoint0);
  struct ArrayHasher {
    std::size_t operator()(const std::array<double, 5>& a) const {
      std::size_t h = 0;
      for (auto e : a) {
        h ^= std::hash<double>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
      }
      return h;
    }
  };
  std::map<std::array<double, 5>, int32_t> map0;
  const auto texCoordCount0 = fsubdiv0.texCoordCount();
  for (int32_t v = 0; v < texCoordCount0; ++v) {
    const auto& point0    = fsubdiv0.point(std::max(0, texCoordToPoint0[v]));
    const auto& texCoord0 = fsubdiv0.texCoord(v);
    const std::array<double, 5> vertex0{
      point0[0], point0[1], point0[2], texCoord0[0], texCoord0[1]};
    map0[vertex0] = texCoordToPoint0[v];
  }

  // Geometry parametrisation rec
  auto fsubdiv1 = rec;
  fsubdiv1.subdivideMidPoint(params.liftingSubdivisionIterationCount);
  const auto pointCount1 = fsubdiv1.pointCount();
  mapping.resize(pointCount1, -1);
  std::vector<bool> tags(pointCount1, false);
  for (int32_t t = 0, tcount = fsubdiv1.triangleCount(); t < tcount; ++t) {
    const auto& tri   = fsubdiv1.triangle(t);
    const auto& triUV = fsubdiv1.texCoordTriangle(t);
    for (int32_t k = 0; k < 3; ++k) {
      const auto indexPos = tri[k];
      if (tags[indexPos]) { continue; }
      tags[indexPos]                            = true;
      const auto                  indexTexCoord = triUV[k];
      const auto&                 point1        = fsubdiv1.point(indexPos);
      const auto&                 texCoord1 = fsubdiv1.texCoord(indexTexCoord);
      const std::array<double, 5> vertex1   = {
        point1[0], point1[1], point1[2], texCoord1[0], texCoord1[1]};
      const auto it = map0.find(vertex1);
      if (it != map0.end()) {
        mapping[indexPos] = map0[vertex1];
      } else {
        assert(0);
      }
    }
  }
  return true;
}

//----------------------------------------------------------------------------

BaseMeshType
VMCEncoder::chooseSkipOrInter(const std::vector<Vec3<int32_t>>& current,
                              const std::vector<Vec3<int32_t>>& reference) {
  const auto         pointCount = int32_t(current.size());
  VMCMotionACContext ctx;
  float              cost_inter = 0.0;
  float              cost_skip  = 0.0;
  float              lamda      = 6.25 * pointCount;
  for (int vindex = 0; vindex < pointCount; ++vindex) {
    const auto motion = current[vindex] - reference[vindex];
    for (int32_t k = 0; k < 3; ++k) {
      cost_skip += motion[k] > 0 ? float(motion[k]) : float(-motion[k]);
    }
    const auto res0  = motion;
    const auto bits0 = ctx.estimateBits(res0, 0);
    cost_inter += float(bits0);
  }
  cost_skip *= lamda;
  if (cost_skip > cost_inter) {
    return P_BASEMESH;
  } else {
    return SKIP_BASEMESH;
  }
}

//----------------------------------------------------------------------------

bool
VMCEncoder::compressMotion(
  const std::vector<Vec3<int32_t>>& triangles,
  const std::vector<Vec3<int32_t>>& reference,
  const std::vector<Vec2<int32_t>>& baseIntegrateIndices,
  const std::vector<Vec3<int32_t>>& current,
  BaseMeshTileLayer&                bmtl,
  const VMCEncoderParameters&       params) {
  const auto         pointCount  = int32_t(current.size());
  const auto         maxAcBufLen = pointCount * 3 * 4 + 1024;
  VMCMotionACContext ctx;
  EntropyEncoder     arithmeticEncoder;
  arithmeticEncoder.setBuffer(maxAcBufLen, nullptr);
  arithmeticEncoder.start();
  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(triangles, pointCount, vertexToTriangle);
  std::vector<int8_t>        available(pointCount, 0);
  std::vector<int8_t>        vtags(pointCount);
  std::vector<int32_t>       vadj;
  std::vector<int32_t>       tadj;
  std::vector<Vec3<int32_t>> motion;
  std::vector<Vec3<int32_t>> motion_no_skip;
  motion.reserve(pointCount);
  std::vector<uint8_t> no_skip_vindices;
  std::vector<int32_t> no_skip_refvindices;
  for (int vindex = 0; vindex < pointCount; ++vindex) {
    auto it =
      std::lower_bound(baseIntegrateIndices.begin(),
                       baseIntegrateIndices.end(),
                       Vec2<int32_t>(vindex, 0),
                       [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) {
                         return a[0] < b[0];
                       });
    if (it != baseIntegrateIndices.end() && (*it)[0] == vindex) {
      auto integrate_from = (*it)[0];
      auto integrate_to   = (*it)[1];
      if (current[integrate_from] == current[integrate_to]) {
        // skip (same motion vector)
      } else {
        auto no_skip_vindex = static_cast<int32_t>(
          std::distance(baseIntegrateIndices.begin(), it));
        if (no_skip_vindex > 255) {
          std::cout << "[DEBUG][error] no_skip_vindex: " << no_skip_vindex
                    << std::endl;
          return 1;
        }
        no_skip_vindices.push_back(
          static_cast<decltype(no_skip_vindices)::value_type>(no_skip_vindex));
        it =
          std::lower_bound(baseIntegrateIndices.begin(),
                           baseIntegrateIndices.end(),
                           Vec2<int32_t>(integrate_to, 0),
                           [](const Vec2<int32_t>& a, const Vec2<int32_t>& b) {
                             return a[0] < b[0];
                           });
        auto shift     = std::distance(baseIntegrateIndices.begin(), it);
        auto refvindex = static_cast<int32_t>(integrate_to - shift);
        no_skip_refvindices.push_back(refvindex);
        motion_no_skip.push_back(current[vindex] - reference[refvindex]);
        std::cout << "[DEBUG][stat] MV of " << integrate_from << " and "
                  << integrate_to
                  << " are not same: " << *motion_no_skip.rbegin() << " vs "
                  << (current[integrate_to] - reference[refvindex])
                  << ". Total MVs: "
                  << static_cast<int32_t>(baseIntegrateIndices.size())
                  << ". vertex index: " << no_skip_vindex << "\n";
      }
    } else {
      auto shift     = std::distance(baseIntegrateIndices.begin(), it);
      auto refvindex = vindex - shift;
      motion.push_back(current[vindex] - reference[refvindex]);
    }
  }
  std::cout << "[DEBUG][stat] total vertex num: " << pointCount
            << " duplicated vertex num: "
            << (current.size() - reference.size())
            << " non-skippable MV num: " << no_skip_vindices.size()
            << std::endl;
  bool all_skip_mode = no_skip_vindices.empty();
  auto no_skip_num   = no_skip_vindices.size();
  // here we should add all_skip_mode into bitstream before starting to encode the MVs
  uint8_t skip_mode_bits;  // 1 bit for all_skip_mode, 7 bits for no_skip_num
  if (no_skip_num > 127) {
    std::cout << "[DEBUG][error] no_skip_num: " << no_skip_num << std::endl;
    return 1;
  };
  auto& bmth  = bmtl.getHeader();
  auto& bmtdu = bmtl.getDataUnit();
  if (baseIntegrateIndices.empty()) {
    skip_mode_bits             = 255;
    bmtdu.getMotionSkipFlag()  = false;
    bmtdu.getMotionSkipCount() = 0;
  } else {
    bmtdu.getMotionSkipFlag() = true;
    if (all_skip_mode) {
      skip_mode_bits             = 128;
      bmtdu.getMotionSkipAll()   = true;
      bmtdu.getMotionSkipCount() = 0;
    } else {
      skip_mode_bits             = static_cast<uint8_t>(no_skip_num);
      bmtdu.getMotionSkipFlag()  = true;
      bmtdu.getMotionSkipAll()   = false;
      bmtdu.getMotionSkipCount() = no_skip_num;
    }
  }
  // bitstream.write(skip_mode_bits);
  if (!all_skip_mode) {
    bmtdu.getMotionSkipVextexIndices() = no_skip_vindices;
    motion.insert(motion.end(), motion_no_skip.begin(), motion_no_skip.end());
  }
  printf("MotionSkipFlag  = %d \n", bmtdu.getMotionSkipFlag());
  printf("MotionSkipAll   = %d \n", bmtdu.getMotionSkipAll());
  printf("MotionSkipCount = %d \n", bmtdu.getMotionSkipCount());
  printf("skip_mode_bits  = %u \n", skip_mode_bits);
  const auto motionCount   = static_cast<int32_t>(motion.size());
  const auto refPointCount = static_cast<int32_t>(reference.size());
  printf("pointCount = %d motionCount = %d \n", pointCount, motionCount);
  fflush(stdout);
  int32_t remainP = motionCount;
  int32_t vindexS = 0, vindexE = 0, vCount = 0;
  while (remainP) {
    vindexS = vindexE;
    std::vector<Vec3<int32_t>> resV0, resV1;
    resV0.resize(0);
    resV1.resize(0);
    auto bits0 = ctx.estimatePred(0);
    auto bits1 = ctx.estimatePred(1);
    vCount     = 0;
    while ((vCount < params.motionGroupSize) && (remainP)) {
      int32_t vindex0 = vindexE;
      ++vindexE;
      --remainP;
      int vindex = vindex0;
      if (vindex0 >= refPointCount) {
        vindex = no_skip_refvindices[vindex0 - refPointCount];
      }
      ComputeAdjacentVertices(
        vindex, triangles, vertexToTriangle, vtags, vadj);
      Vec3<int32_t> pred(0);
      int32_t       predCount = 0;
      for (int vindex1 : vadj) {
        if (available[vindex1] != 0) {
          const auto& mv1 = motion[vindex1];
          for (int32_t k = 0; k < 3; ++k) { pred[k] += mv1[k]; }
          ++predCount;
        }
      }
      if (predCount > 1) {
        const auto bias = predCount >> 1;
        for (int32_t k = 0; k < 3; ++k) {
          pred[k] = pred[k] >= 0 ? (pred[k] + bias) / predCount
                                 : -(-pred[k] + bias) / predCount;
        }
      }
      available[vindex0] = 1;
      const auto res0    = motion[vindex0];
      const auto res1    = motion[vindex0] - pred;
      ++vCount;
      resV0.push_back(res0);
      resV1.push_back(res1);
      bits0 += ctx.estimateRes(res0);
      bits1 += ctx.estimateRes(res1);
    }
    std::vector<Vec3<int32_t>> resV;
    Vec3<int32_t>              res{};
    if (bits0 <= bits1) {
      resV = resV0;
      arithmeticEncoder.encode(0, ctx.ctxPred);
    } else {
      resV = resV1;
      arithmeticEncoder.encode(1, ctx.ctxPred);
    }
    vCount = -1;
    for (int vindex0 = vindexS; vindex0 < vindexE; ++vindex0) {
      ++vCount;
      res = resV[vCount];
      for (int32_t k = 0; k < 3; ++k) {
        auto value = res[k];
        arithmeticEncoder.encode(static_cast<int>(value != 0),
                                 ctx.ctxCoeffGtN[0][k]);
        if (value == 0) { continue; }
        arithmeticEncoder.encode(static_cast<int>(value < 0), ctx.ctxSign[k]);
        value = std::abs(value) - 1;
        arithmeticEncoder.encode(static_cast<int>(value != 0),
                                 ctx.ctxCoeffGtN[1][k]);
        if (value == 0) { continue; }
        assert(value > 0);
        arithmeticEncoder.encodeExpGolomb(
          --value, 0, ctx.ctxCoeffRemPrefix[k], ctx.ctxCoeffRemSuffix[k]);
      }
    }
  }
  const auto length = arithmeticEncoder.stop();
  std::cout << "Motion byte count = " << length << '\n';
  assert(length <= std::numeric_limits<uint32_t>::max());
  bmtdu.getData().resize(length);
  std::copy(arithmeticEncoder.buffer(),
            arithmeticEncoder.buffer() + length,
            bmtdu.getData().vector().begin());
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::compressBaseMesh(const VMCGroupOfFrames&     gof,
                             const VMCFrameInfo&         frameInfo,
                             VMCFrame&                   frame,
                             TriangleMesh<MeshType>&     rec,
                             BaseMeshTileLayer&          bmtl,
                             const VMCEncoderParameters& params) {
  // if (!encodeFrameHeader(frameInfo, bitstream)) { return false; }
  const auto scalePosition =
    ((1 << params.qpPosition) - 1.0) / ((1 << params.bitDepthPosition) - 1.0);
  const auto scaleTexCoord  = std::pow(2.0, params.qpTexCoord) - 1.0;
  const auto iscalePosition = 1.0 / scalePosition;
  const auto iscaleTexCoord = 1.0 / scaleTexCoord;
  auto&      base           = frame.base;
  auto&      subdiv         = frame.subdiv;
  auto&      bmth           = bmtl.getHeader();
  auto&      bmtdu          = bmtl.getDataUnit();
  if (frameInfo.type == I_BASEMESH) bmth.getBaseMeshType()    = frameInfo.type;

  // Save intermediate files
  std::string prefix = "";
  if (params.keepIntermediateFiles) {
    prefix =
      _keepFilesPathPrefix + "fr_" + std::to_string(frameInfo.frameIndex);
    base.save(prefix + "_base_compress.ply");
    subdiv.save(prefix + "_subdiv_compress.ply");
  }
  if (frameInfo.type == I_BASEMESH) {
    const auto texCoordBBox = base.texCoordBoundingBox();
    const auto delta        = texCoordBBox.max - texCoordBBox.min;
    const auto d            = std::max(delta[0], delta[1]);
    if (d > 2.0) {  // make sure texCoords are between 0.0 and 1.0
      const auto scale = 1.0 / (std::pow(2.0, params.bitDepthTexCoord) - 1.0);
      for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount;
           ++tc) {
        base.setTexCoord(tc, base.texCoord(tc) * scale);
      }
    }
    computeDracoMapping(base, frame.mapping, frameInfo.frameIndex, params);
    if (params.keepIntermediateFiles) {
      base.save(prefix + "_post_mapping_base.ply");
    }
    // quantize base mesh
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      base.setPoint(v, Round(base.point(v) * scalePosition));
    }
    for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
      base.setTexCoord(tc, Round(base.texCoord(tc) * scaleTexCoord));
    }

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      base.save(prefix + "_post_mapping_base_quant.ply");
    }

    // Encode
    GeometryEncoderParameters encoderParams;
    encoderParams.qp_                = params.qpPosition;
    encoderParams.qt_                = params.qpTexCoord;
    encoderParams.dracoUsePosition_  = params.dracoUsePosition;
    encoderParams.dracoUseUV_        = params.dracoUseUV;
    encoderParams.dracoMeshLossless_ = params.dracoMeshLossless;
    TriangleMesh<MeshType> rec;
    std::vector<uint8_t>   geometryBitstream;
    auto encoder = GeometryEncoder<MeshType>::create(GeometryCodecId::DRACO);
    printf("BaseMeshEnco: use_position = %d use_uv = %d mesh_lossless = %d \n",
           encoderParams.dracoUsePosition_,
           encoderParams.dracoUseUV_,
           encoderParams.dracoMeshLossless_);
    encoder->encode(base, encoderParams, bmtdu.getData().vector(), rec);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      base.save(prefix + "_base_enc.ply");
      rec.save(prefix + "_base_rec.ply");
      save(prefix + "_base.drc", bmtdu.getData().vector());
    }

    // Round positions
    base             = rec;
    auto& qpositions = frame.qpositions;
    qpositions.resize(base.pointCount());
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      qpositions[v] = base.point(v).round();
    }
    for (int32_t tc = 0, tccount = base.texCoordCount(); tc < tccount; ++tc) {
      base.setTexCoord(tc, base.texCoord(tc) * iscaleTexCoord);
    }
  } else {
    // quantize base mesh
    const auto& refFrame   = gof.frame(frameInfo.referenceFrameIndex);
    auto&       mapping    = frame.mapping;
    auto&       qpositions = frame.qpositions;
    mapping                = refFrame.mapping;
    const auto pointCountRecBaseMesh = refFrame.base.pointCount();
    qpositions.resize(pointCountRecBaseMesh);
    for (int32_t v = 0; v < pointCountRecBaseMesh; ++v) {
      assert(mapping[v] >= 0 && mapping[v] < base.pointCount());
      qpositions[v] = (base.point(mapping[v]) * scalePosition).round();
    }
    base = refFrame.base;
    for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
      base.point(v) = qpositions[v];
    }

    // decide to use P_BASEMESH or SKIP_BASEMESH
    auto        frameInfoInterOrSkip = frameInfo;
    frameInfoInterOrSkip.type =
      chooseSkipOrInter(qpositions, refFrame.qpositions);
    bmth.getBaseMeshType() = frameInfoInterOrSkip.type;

    if (frameInfoInterOrSkip.type == P_BASEMESH) {
      printf("Inter: index = %d ref = %d \n",
             frameInfo.frameIndex,
             frameInfo.referenceFrameIndex);
      fflush(stdout);

      if (params.motionWithoutDuplicatedVertices) {
        compressMotion(refFrame.baseClean.triangles(),
                       refFrame.baseClean.points(),
                       refFrame.baseIntegrateIndices,
                       qpositions,
                       bmtl,
                       params);
      } else {
        compressMotion(
          base.triangles(), refFrame.qpositions, {}, qpositions, bmtl, params);
      }

      // bmth.getReferenceFrameIndex() =
      //   frameInfo.frameIndex - frameInfo.referenceFrameIndex - 1;
      auto& refList = bmth.getRefListStruct();
      refList.allocate(1);
      refList.getAbsDeltaAfocSt(0) =
        frameInfo.frameIndex - frameInfo.referenceFrameIndex;
      refList.getStrafEntrySignFlag(0)  = false;
      refList.getStRefAtlasFrameFlag(0) = true;
    } else {
      printf("Skip: index = %d ref = %d \n",
             frameInfo.frameIndex,
             frameInfo.referenceFrameIndex);
      fflush(stdout);

      // copy and quantize base mesh
      qpositions = refFrame.qpositions;
      base                   = refFrame.base;
      for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
        base.point(v) = qpositions[v];
      }

      // bmth.getReferenceFrameIndex() =
      //   frameInfo.frameIndex - frameInfo.referenceFrameIndex - 1;
      // no reference frame index is written to bitstream
    }
  }
  // Duplicated Vertex Reduction
  if (params.motionWithoutDuplicatedVertices) {
    removeDuplicatedVertices(frame);
    if (params.keepIntermediateFiles) {
      frame.baseClean.save(prefix + "_baseClean.obj");
    }
  }
  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.setPoint(v, base.point(v) * iscalePosition);
  }
  subdivideBaseMesh(frame,
                    rec,
                    params.intraGeoParams.subdivisionMethod,
                    params.liftingSubdivisionIterationCount,
                    params.interpolateDisplacementNormals,
                    params.addReconstructedNormals);

  if (!params.dracoMeshLossless) {
    auto rsubdiv = rec;
    std::swap(rsubdiv, subdiv);
    const auto& mapping = frame.mapping;
    for (int32_t v = 0, vcount = subdiv.pointCount(); v < vcount; ++v) {
      const auto vindex = mapping[v];
      assert(vindex >= 0 && vindex < vcount);
      subdiv.setPoint(v, rsubdiv.point(vindex));
    }
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::computeDisplacements(VMCFrame&                     frame,
                                 const TriangleMesh<MeshType>& rec,
                                 const VMCEncoderParameters&   params) {
  const auto& subdiv = frame.subdiv;
  auto&       disp   = frame.disp;
  disp.resize(rec.pointCount());
  for (int32_t v = 0, vcount = rec.pointCount(); v < vcount; ++v) {
    if (params.displacementCoordinateSystem
        == DisplacementCoordinateSystem::LOCAL) {
      const auto     n = rec.normal(v);
      Vec3<MeshType> t{};
      Vec3<MeshType> b{};
      computeLocalCoordinatesSystem(n, t, b);
      const auto& pt0   = rec.point(v);
      const auto& pt1   = subdiv.point(v);
      const auto  delta = pt1 - pt0;
      disp[v]           = Vec3<MeshType>(delta * n, delta * t, delta * b);
    } else {
      disp[v] = subdiv.point(v) - rec.point(v);
    }
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::quantizeDisplacements(VMCFrame&                   frame,
                                  const VMCEncoderParameters& params) {
  const auto& infoLevelOfDetails = frame.subdivInfoLevelOfDetails;
  const auto  lodCount           = int32_t(infoLevelOfDetails.size());
  assert(lodCount > 0);
  const auto dispDimensions = params.applyOneDimensionalDisplacement ? 1 : 3;
  std::vector<double> scale(dispDimensions);
  std::vector<double> lodScale(dispDimensions);
  for (int32_t k = 0; k < dispDimensions; ++k) {
    const auto qp = params.liftingQP[k];
    scale[k] =
      qp >= 0 ? pow(2.0, 16 - params.bitDepthPosition + (4 - qp) / 6.0) : 0.0;
    lodScale[k] = 1.0 / params.liftingLevelOfDetailInverseScale[k];
  }
  auto& disp = frame.disp;
  for (int32_t it = 0, vcount0 = 0; it < lodCount; ++it) {
    const auto vcount1 = infoLevelOfDetails[it].pointCount;
    for (int32_t v = vcount0; v < vcount1; ++v) {
      auto& d = disp[v];
      for (int32_t k = 0; k < dispDimensions; ++k) {
        d[k] = d[k] >= 0.0
                 ? std::floor(d[k] * scale[k] + params.liftingBias[k])
                 : -std::floor(-d[k] * scale[k] + params.liftingBias[k]);
      }
    }
    vcount0 = vcount1;
    for (int32_t k = 0; k < dispDimensions; ++k) { scale[k] *= lodScale[k]; }
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::computeDisplacementVideoFrame(
  const VMCFrame& frame,
  Frame<uint16_t>&
    dispVideoFrame,  // , ColourSpace::YUV400p, ColourSpace::YUV444p
  const VMCEncoderParameters& params) {
  const auto pixelsPerBlock =
    params.displacementVideoBlockSize * params.displacementVideoBlockSize;
  const auto  shift      = uint16_t((1 << params.geometryVideoBitDepth) >> 1);
  const auto& disp       = frame.disp;
  const auto  planeCount = dispVideoFrame.planeCount();
  for (int32_t p = 0; p < planeCount; ++p) {
    dispVideoFrame.plane(p).fill(shift);
  }
  int32_t start;
  if (params.displacementUse420 && (!params.applyOneDimensionalDisplacement)) {
    start = dispVideoFrame.width() * dispVideoFrame.height() / 3 - 1;
  }
  else {
    start = dispVideoFrame.width() * dispVideoFrame.height() - 1;
  }
  for (int32_t v = 0, vcount = int32_t(disp.size()); v < vcount; ++v) {
    // to do: optimize power of 2
    const auto& d          = disp[v];
    const auto  v0         = params.displacementReversePacking ? start - v : v;
    const auto  blockIndex = v0 / pixelsPerBlock;
    const auto  indexWithinBlock = v0 % pixelsPerBlock;
    const auto  x0 = (blockIndex % params.geometryVideoWidthInBlocks)
                    * params.displacementVideoBlockSize;
    const auto y0 = (blockIndex / params.geometryVideoWidthInBlocks)
                    * params.displacementVideoBlockSize;
    const int32_t dispDim = params.applyOneDimensionalDisplacement ? 1 : 3;
    int32_t x = 0;
    int32_t y = 0;
    computeMorton2D(indexWithinBlock, x, y);
    assert(x < params.displacementVideoBlockSize);
    assert(y < params.displacementVideoBlockSize);

    const auto x1 = x0 + x;
    const auto y1 = y0 + y;
    for (int32_t p = 0; p < dispDim; ++p) {
      const auto dshift = int32_t(shift + d[p]);
      assert(dshift >= 0 && dshift < (1 << params.geometryVideoBitDepth));
      if (params.displacementUse420) {
        dispVideoFrame.plane(0).set(p * dispVideoFrame.height() / 3 + y1, x1, uint16_t(dshift));
      }
      else {
        dispVideoFrame.plane(p).set(y1, x1, uint16_t(dshift));
      }
    }
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::compress(const VMCGroupOfFramesInfo& gofInfoSrc,
                     const Sequence&             source,
                     V3cBitstream&               syntax,
                     Sequence&                   reconstruct,
                     const VMCEncoderParameters& params) {
  VMCGroupOfFrames gof;
  auto             gofInfo = gofInfoSrc;
  gofInfo.trace();
  const int32_t frameCount          = gofInfo.frameCount_;
  int32_t       lastIntraFrameIndex = 0;
  initialize(syntax, params, gofInfo);

  gof.resize(source.frameCount());
  reconstruct.resize(source.frameCount());
  printf("Compress: frameCount = %d \n", frameCount);
  fflush(stdout);
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    auto& frame = gof.frame(frameIndex);
    if (params.baseIsSrc && params.subdivIsBase) {
      if (params.baseIsSrc) { frame.base = source.mesh(frameIndex); }
      if (params.subdivIsBase) {
        auto& frameInfo = gofInfo.frameInfo(frameIndex);
        frame.subdiv    = frame.base;
        if (params.subdivInter && frameInfo.referenceFrameIndex != -1) {
          frameInfo.referenceFrameIndex = frameInfo.frameIndex - 1;
        }
      }
    } else {
      TriangleMesh<MeshType> decimate;
      decimateInput(source.mesh(frameIndex), frame, decimate, params);
      textureParametrization(frame, decimate, params);
      decimate.clear();
      geometryParametrization(gof,
                              gofInfo,
                              frame,
                              source.mesh(frameIndex),
                              params,
                              lastIntraFrameIndex);
      if (params.subdivIsBase) { frame.subdiv = frame.base; }
    }
  }

  // Compress
  printf("Compress \n");
  fflush(stdout);
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    const auto& frame     = gof.frame(frameIndex);
    const auto& frameInfo = gofInfo.frameInfo(frameIndex);
    printf(
      "Frame %2d: type = %s base = %6d points subdiv = %6d points Ref = %d \n",
      frameIndex,
      toString(frameInfo.type).c_str(),
      frame.base.pointCount(),
      frame.subdiv.pointCount(),
      frameInfo.referenceFrameIndex);
    // Save intermediate files
    if (params.keepIntermediateFiles) {
      auto prefix = _keepFilesPathPrefix + "fr_" + std::to_string(frameIndex);
      frame.base.save(prefix + "_base_org.ply");
      frame.subdiv.save(prefix + "_subdiv_org.ply");
    }
  }

  // Unify vertices
  printf("UnifyVertices \n");
  fflush(stdout);
  unifyVertices(gofInfo, gof, params);

  // Compress geometry
  // Bitstream     bitstreamGeo;
  const int32_t pixelsPerBlock =
    params.displacementVideoBlockSize * params.displacementVideoBlockSize;
  const auto widthDispVideo =
    params.geometryVideoWidthInBlocks * params.displacementVideoBlockSize;
  auto       heightDispVideo      = 0;
  auto colourSpaceDispVideo = ColourSpace::YUV420p;
  if (!params.displacementUse420) {
    colourSpaceDispVideo = params.applyOneDimensionalDisplacement ? ColourSpace::YUV400p : ColourSpace::YUV444p;
  }
  FrameSequence<uint16_t> dispVideo;
  dispVideo.resize(0, 0, colourSpaceDispVideo, frameCount);

  printf("frameCount = %zu \n", frameCount);
  auto& baseMesh = syntax.getBaseMesh();
  // auto& msdu = atlas.getMeshSequenceDataUnit(0);
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    const auto& frameInfo      = gofInfo.frameInfo(frameIndex);
    auto&       frame          = gof.frame(frameIndex);
    auto&       dispVideoFrame = dispVideo.frame(frameIndex);
    auto&       rec            = reconstruct.mesh(frameIndex);
    auto&       bmtl           = baseMesh.getBaseMeshTileLayer(frameIndex);
    printf("Frame %4d: type = %s ReferenceFrame = %4d \n",
           frameInfo.frameIndex,
           toString(frameInfo.type).c_str(),
           frameInfo.referenceFrameIndex);
    compressBaseMesh(gof, frameInfo, frame, rec, bmtl, params);
    const auto vertexCount = frame.subdiv.pointCount();
    if (params.encodeDisplacementsVideo) {
      const auto blockCount =
        (vertexCount + pixelsPerBlock - 1) / pixelsPerBlock;
      const auto geometryVideoHeightInBlocks =
        (blockCount + params.geometryVideoWidthInBlocks - 1)
        / params.geometryVideoWidthInBlocks;
      const auto width =
        params.geometryVideoWidthInBlocks * params.displacementVideoBlockSize;
      auto height =
        geometryVideoHeightInBlocks * params.displacementVideoBlockSize;
      if (params.displacementUse420 && (!params.applyOneDimensionalDisplacement)) {
        height *= 3;
      }
      dispVideoFrame.resize(width, height, colourSpaceDispVideo);
      computeDisplacements(frame, rec, params);
      computeForwardLinearLifting(frame.disp,
                                  frame.subdivInfoLevelOfDetails,
                                  frame.subdivEdges,
                                  params.liftingPredictionWeight,
                                  params.liftingUpdateWeight,
                                  params.liftingSkipUpdate);
      quantizeDisplacements(frame, params);
      computeDisplacementVideoFrame(frame, dispVideoFrame, params);
    }
  }
  // resize all the frame to the same resolution
  if (params.encodeDisplacementsVideo) {
    const auto padding = uint16_t((1 << params.geometryVideoBitDepth) >> 1);
    dispVideo.standardizeFrameSizes(true, padding);
    auto& atlas              = syntax.getAtlas();
    auto& asps               = atlas.getAtlasSequenceParameterSet(0);
    auto& ext                = asps.getAspsVdmcExtension();
    ext.getWidthDispVideo()  = uint32_t(dispVideo.width());
    ext.getHeightDispVideo() = uint32_t(dispVideo.height());
  }
  // Encode displacements video
  if (params.encodeDisplacementsVideo
      && (!compressDisplacementsVideo(dispVideo, syntax, params))) {
    return false;
  }

  // Reconstruct
  printf("Reconstruct \n");
  reconstruct.textures().resize(
    params.encodeTextureVideo ? params.textureWidth : 1,
    params.encodeTextureVideo ? params.textureHeight : 1,
    ColourSpace::BGR444p,
    frameCount);

  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    auto& frame = gof.frame(frameIndex);
    printf("Reconstruct frame %2d / %d \n", frameIndex, frameCount);
    fflush(stdout);
    if (params.encodeDisplacementsVideo) {
      reconstructDisplacementFromVideoFrame(dispVideo.frame(frameIndex),
                                            frame,
                                            reconstruct.mesh(frameIndex),
                                            params.displacementVideoBlockSize,
                                            params.geometryVideoBitDepth,
                                            params.applyOneDimensionalDisplacement,
                                            params.displacementReversePacking);
      inverseQuantizeDisplacements(frame,
                                   params.bitDepthPosition,
                                   params.liftingLevelOfDetailInverseScale,
                                   params.liftingQP);
      computeInverseLinearLifting(frame.disp,
                                  frame.subdivInfoLevelOfDetails,
                                  frame.subdivEdges,
                                  params.liftingPredictionWeight,
                                  params.liftingUpdateWeight,
                                  params.liftingSkipUpdate);
      applyDisplacements(frame,
                         params.bitDepthPosition,
                         reconstruct.mesh(frameIndex),
                         params.displacementCoordinateSystem);
    }
    tic("ColorTransfer");
    if (params.encodeTextureVideo && params.textureTransferEnable) {
      TransferColor transferColor;
      auto& refMesh = source.mesh(frameIndex);
      auto& refTexture = source.texture(frameIndex);
      auto& targetMesh = reconstruct.mesh(frameIndex);
      auto& targetTexture = reconstruct.texture(frameIndex);
      Frame<uint8_t> pastTexture;
      bool usePastTexture = false;
      if (params.textureTransferCopyBackground && frameIndex > 0 && gofInfo.frameInfo(frameIndex).type == P_BASEMESH) {
          //occupancy map will be the same as the reference, so use the background from reference as well
          pastTexture = reconstruct.texture(gofInfo.frameInfo(frameIndex).referenceFrameIndex);
          usePastTexture = true;
      }
      if (!transferColor.transfer(refMesh,
                                  refTexture,
                                  targetMesh,
                                  targetTexture,
                                  usePastTexture,
                                  pastTexture,
                                  params)) {
        return false;
      }
    } else {
      reconstruct.texture(frameIndex) = source.texture(frameIndex);
    }
    toc("ColorTransfer");
  }
  // compress texture
  if (!compressTextureVideo(reconstruct, syntax, params)) { return false; }

  // Quantize UV coordinate
  if (!params.dequantizeUV) {
    const auto scale = (1 << params.bitDepthTexCoord) - 1.0;
    for (auto& rec : reconstruct.meshes()) {
      const auto texCoordCount = rec.texCoordCount();
      for (int32_t i = 0; i < texCoordCount; ++i) {
        rec.setTexCoord(i, rec.texCoord(i) * scale);
      }
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
