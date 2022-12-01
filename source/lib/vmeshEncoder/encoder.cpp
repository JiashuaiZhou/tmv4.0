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

#include "util/bitstream.hpp"
#include "contexts.hpp"
#include "entropy.hpp"
#include "util/kdtree.hpp"
#include "vmc.hpp"
#include "metrics.hpp"
#include "geometryDecimate.hpp"
#include "textureParametrization.hpp"
#include "geometryParametrization.hpp"
#include "util/checksum.hpp"

#include "virtualGeometryEncoder.hpp"
#include "virtualVideoEncoder.hpp"
#include "virtualColourConverter.hpp"

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
VMCEncoder::removeDegeneratedTrianglesCrossProduct(TriangleMesh<MeshType>& mesh,
                                                   const int32_t& frameIndex) {

    TriangleMesh<MeshType> degenerateMesh;

    auto& degenerateTriangles = degenerateMesh.triangles();
    auto& degenerateTexCoordTriangles = degenerateMesh.texCoordTriangles();
    auto& degenerateNormalTriangles = degenerateMesh.normalTriangles();
    degenerateMesh.points().resize(mesh.pointCount());

    auto triangleCount = mesh.triangleCount();
    if (triangleCount <= 0) { return; }

    const auto hasTexCoords = mesh.texCoordTriangleCount() == triangleCount;
    const auto hasNormals = mesh.normalTriangleCount() == triangleCount;
    std::vector<Triangle> triangles;
    triangles.reserve(triangleCount);

    int32_t removedDegeneratedTrianglesArea0 = 0;

    for (int32_t tindex = 0; tindex < triangleCount; tindex++) {
        const auto& tri = mesh.triangle(tindex);
        const auto& coord = mesh.points();

        const auto i = tri[0];
        const auto j = tri[1];
        const auto k = tri[2];
        auto area = computeTriangleArea(coord[tri[0]], coord[tri[1]], coord[tri[2]]);

        if (area > 0) {
            triangles.push_back(tri);
        }
        else {
            std::cout << "Triangle with area 0 is removed." << std::endl;
            removedDegeneratedTrianglesArea0++;
            degenerateTriangles.push_back(tri);
        }
    }

    std::cout << "frame : " << frameIndex << " , removedDegeneratedTrianglesArea0count = " << removedDegeneratedTrianglesArea0 << std::endl;

    std::swap(mesh.triangles(), triangles);

    //Change connectivity
    if (1) {

        triangles.clear();

        triangleCount = mesh.triangleCount();
        auto degenerateTriangleCount = degenerateMesh.triangleCount();

        std::vector<int32_t> vertexID0s(degenerateTriangleCount), vertexID1s(degenerateTriangleCount), vertexID2s(degenerateTriangleCount);

        for (uint32_t degetindex = 0; degetindex < degenerateTriangleCount; degetindex++) {
            auto& degetri = degenerateTriangles[degetindex];

            const auto& coord = mesh.points();

            //Check middle point
            auto doubleMax = std::numeric_limits<double>::max();
            auto doubleMin = std::numeric_limits<double>::min();
            std::vector<double> minV = { doubleMax ,doubleMax ,doubleMax };
            std::vector<double> maxV = { doubleMin ,doubleMin ,doubleMin };

            std::vector<int> minID = { 0,0,0 }, maxID = { 0,0,0 };

            for (int vertexID = 0; vertexID < 3; vertexID++) {

                auto& p = coord[degetri[vertexID]];

                //xyz
                for (int i = 0; i < 3; i++) {
                    if (p[i] < minV[i])
                    {
                        minV[i] = p[i];
                        minID[i] = vertexID;
                    }
                    if (p[i] > maxV[i])
                    {
                        maxV[i] = p[i];
                        maxID[i] = vertexID;
                    }

                }
            }

            int middleID = 0;
            for (int vertexID = 0; vertexID < 3; vertexID++) {
                //xyz
                bool flag = false;
                for (int i = 0; i < 3; i++) {
                    if (minID[i] == vertexID || maxID[i] == vertexID) {
                        flag = 1;
                    }
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
            }
            else if (middleID == 1) {
                vertexID0 = degetri[1];
                vertexID1 = degetri[2];
                vertexID2 = degetri[0];
            }
            else if (middleID == 2) {
                vertexID0 = degetri[2];
                vertexID1 = degetri[0];
                vertexID2 = degetri[1];
            }
        }

        for (int32_t tindex = 0; tindex < triangleCount; tindex++) {
            const auto& tri = mesh.triangle(tindex);
            const auto& coord = mesh.points();

            bool addedFlag = false;
            Vec3<int> newFace1, newFace2;

            for (uint32_t degetindex = 0; !addedFlag && (degetindex < degenerateTriangleCount); degetindex++) {
                auto& vertexID0 = vertexID0s[degetindex];
                auto& vertexID1 = vertexID1s[degetindex];
                auto& vertexID2 = vertexID2s[degetindex];

                std::vector<Vec3<int>> vIDs{ { tri[0], tri[1], tri[2] }, { tri[1], tri[2], tri[0] }, { tri[2], tri[0], tri[1] } };
                for (auto& vID : vIDs) {
                    auto& vID0 = vID[0];
                    auto& vID1 = vID[1];
                    auto& vID2 = vID[2];

                    if ((vID1 == vertexID1 && vID2 == vertexID2) || (vID1 == vertexID2 && vID2 == vertexID1)) {
                        newFace1 = { vID1, vertexID0, vID0 };
                        newFace2 = { vID0, vertexID0, vID2 };

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
VMCEncoder::textureParametrization(VMCFrame&                     frame,
                                   TriangleMesh<MeshType>&       decimate,
                                   const VMCEncoderParameters&   params) {
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
      frameInfo.type                = FrameType::INTRA;
      frameInfo.referenceFrameIndex = -1;
      lastIntraFrameIndex           = frameIndex;
    } else {
      frame.base                    = baseInter;
      frame.subdiv                  = subdivInter;
      frameInfo.type                = FrameType::SKIP;
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
      if (frameInfo.type == FrameType::INTRA) {        
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
                                       Bitstream&                  bitstream,
                                       const VMCEncoderParameters& params) {
  //Encode
  VideoEncoderParameters videoEncoderParams;
  videoEncoderParams.encoderConfig_    = params.geometryVideoEncoderConfig;
  videoEncoderParams.inputBitDepth_    = params.geometryVideoBitDepth;
  videoEncoderParams.internalBitDepth_ = params.geometryVideoBitDepth;
  videoEncoderParams.outputBitDepth_   = params.geometryVideoBitDepth;
  videoEncoderParams.qp_               = 8;
  FrameSequence<uint16_t> rec;
  std::vector<uint8_t>    videoBitstream;
  auto                    encoder =
    VirtualVideoEncoder<uint16_t>::create(params.geometryVideoCodecId);
  encoder->encode(dispVideo, videoEncoderParams, videoBitstream, rec);

  // Save intermediate files
  if (params.keepIntermediateFiles) {
    auto prefix = _keepFilesPathPrefix + "disp";
    dispVideo.save(dispVideo.createName(prefix + "_enc", 10));
    rec.save(rec.createName(prefix + "_rec", 10));
    save(prefix + ".h265", videoBitstream);
  }
  dispVideo = rec;

  bitstream.write((uint32_t)videoBitstream.size());
  bitstream.append(videoBitstream);
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::compressTextureVideo(Sequence&                   reconstruct,
                                 Bitstream&                  bitstream,
                                 const VMCEncoderParameters& params) const {
  if ( !params.encodeTextureVideo  ) {
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
    // convert BGR444 to YUV420
#if USE_HDRTOOLS
    auto convert = VirtualColourConverter<uint16_t>::create(1);
    convert->initialize(params.textureVideoHDRToolEncConfig);
#else
    auto convert = VirtualColourConverter<uint16_t>::create(0);
    auto mode    = "BGR444p_YUV420p_8_10_"
                + std::to_string(params.textureVideoDownsampleFilter) + "_"
                + std::to_string(params.textureVideoFullRange);
    convert->initialize(mode);
#endif
    convert->convert(src);

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
    std::vector<uint8_t>    videoBitstream;
    auto                    encoder =
      VirtualVideoEncoder<uint16_t>::create(params.textureVideoCodecId);
    encoder->encode(src, videoEncoderParams, videoBitstream, rec);
    bitstream.write((uint32_t)videoBitstream.size());
    bitstream.append(videoBitstream);
    src.clear();
    printf("encode texture video done \n");
    fflush(stdout);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      rec.save(rec.createName(_keepFilesPathPrefix + "tex_rec", 10));
      save(_keepFilesPathPrefix + ".h265", videoBitstream);
    }

    // Convert Rec YUV420 to BGR444
#if USE_HDRTOOLS
    convert->initialize(params.textureVideoHDRToolDecConfig);
#else
    mode = "YUV420p_BGR444p_10_8_"
           + std::to_string(params.textureVideoUpsampleFilter) + "_"
           + std::to_string(params.textureVideoFullRange);
    convert->initialize(mode);
#endif
    convert->convert(rec);

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
  GeometryEncoderParameters dracoParams;
  dracoParams.cl_ = 10;
  TriangleMesh<MeshType> rec;
  std::vector<uint8_t>   geometryBitstream;
  auto                   encoder =
    VirtualGeometryEncoder<MeshType>::create(GeometryCodecId::DRACO);
  encoder->encode(base, dracoParams, geometryBitstream, rec);

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

bool
VMCEncoder::compressMotion(const std::vector<Vec3<int32_t>>& triangles,
                           const std::vector<Vec3<int32_t>>& current,
                           const std::vector<Vec3<int32_t>>& reference,
                           Bitstream&                        bitstream,
                           const VMCEncoderParameters& /*params*/) {
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
  std::vector<Vec3<int32_t>> motion(pointCount);
  for (int vindex = 0; vindex < pointCount; ++vindex) {
    motion[vindex] = current[vindex] - reference[vindex];
  }
  for (int vindex0 = 0; vindex0 < pointCount; ++vindex0) {
    ComputeAdjacentVertices(vindex0, triangles, vertexToTriangle, vtags, vadj);
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
    available[vindex0]  = 1;
    const auto    res0  = motion[vindex0];
    const auto    res1  = motion[vindex0] - pred;
    const auto    bits0 = ctx.estimateBits(res0, 0);
    const auto    bits1 = ctx.estimateBits(res1, 1);
    Vec3<int32_t> res{};
    if (bits0 <= bits1) {
      res = res0;
      arithmeticEncoder.encode(0, ctx.ctxPred);
    } else {
      res = res1;
      arithmeticEncoder.encode(1, ctx.ctxPred);
    }
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

  const auto length = arithmeticEncoder.stop();
  std::cout << "Motion byte count = " << length << '\n';
  assert(length <= std::numeric_limits<uint32_t>::max());
  const auto byteCount = uint32_t(length);
  bitstream.write(byteCount);
  const auto offset = bitstream.size();
  bitstream.resize(offset + byteCount);
  std::copy(arithmeticEncoder.buffer(),
            arithmeticEncoder.buffer() + byteCount,
            bitstream.buffer.begin() + offset);
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::compressBaseMesh(const VMCGroupOfFrames&     gof,
                             const VMCFrameInfo&         frameInfo,
                             VMCFrame&                   frame,
                             TriangleMesh<MeshType>&     rec,
                             Bitstream&                  bitstream,
                             const VMCEncoderParameters& params) {
  if (!encodeFrameHeader(frameInfo, bitstream)) { return false; }
  const auto scalePosition =
    ((1 << params.qpPosition) - 1.0) / ((1 << params.bitDepthPosition) - 1.0);
  const auto scaleTexCoord  = std::pow(2.0, params.qpTexCoord) - 1.0;
  const auto iscalePosition = 1.0 / scalePosition;
  const auto iscaleTexCoord = 1.0 / scaleTexCoord;
  auto&      base           = frame.base;
  auto&      subdiv         = frame.subdiv;
  // Save intermediate files
  std::string prefix = "";
  if (params.keepIntermediateFiles) {
    prefix =
      _keepFilesPathPrefix + "fr_" + std::to_string(frameInfo.frameIndex);
    base.save(prefix + "_base_compress.ply");
    subdiv.save(prefix + "_subdiv_compress.ply");
  }
  printf("Frame %4d: type = %s \n",
         frameInfo.frameIndex,
         frameInfo.type == FrameType::INTRA ? "Intra" : "Inter");
  if (frameInfo.type == FrameType::INTRA) {
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
    GeometryEncoderParameters dracoParams;
    dracoParams.qp_ = params.qpPosition;
    dracoParams.qt_ = params.qpTexCoord;
    TriangleMesh<MeshType> rec;
    std::vector<uint8_t>   geometryBitstream;
    auto                   encoder =
      VirtualGeometryEncoder<MeshType>::create(GeometryCodecId::DRACO);
    encoder->encode(base, dracoParams, geometryBitstream, rec);

    // Save intermediate files
    if (params.keepIntermediateFiles) {
      base.save(prefix + "_base_enc.ply");
      rec.save(prefix + "_base_rec.ply");
      save(prefix + "_base.drc", geometryBitstream);
    }

    // Store bitstream
    auto bitstreamByteCount0 = bitstream.size();
    bitstream.write((uint32_t)geometryBitstream.size());
    bitstream.append(geometryBitstream);
    _stats.baseMeshByteCount += bitstream.size() - bitstreamByteCount0;

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
    printf("Inter: index = %d ref = %d \n",
           frameInfo.frameIndex,
           frameInfo.referenceFrameIndex);
    fflush(stdout);

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
    auto bitstreamByteCount0 = bitstream.size();
    compressMotion(
      base.triangles(), qpositions, refFrame.qpositions, bitstream, params);
    _stats.motionByteCount += bitstream.size() - bitstreamByteCount0;
  }
  for (int32_t v = 0, vcount = base.pointCount(); v < vcount; ++v) {
    base.setPoint(v, base.point(v) * iscalePosition);
  }
  subdivideBaseMesh(frame,
                    rec,
                    params.intraGeoParams.subdivisionMethod,
                    params.liftingSubdivisionIterationCount);

  auto rsubdiv = rec;
  std::swap(rsubdiv, subdiv);
  const auto& mapping = frame.mapping;
  for (int32_t v = 0, vcount = subdiv.pointCount(); v < vcount; ++v) {
    const auto vindex = mapping[v];
    assert(vindex >= 0 && vindex < vcount);
    subdiv.setPoint(v, rsubdiv.point(vindex));
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
        d[k] =
          d[k] >= 0.0
            ? std::floor(d[k] * scale[k] + params.liftingBias[k])
            : -std::floor(-d[k] * scale[k]
                          + params.liftingBias[k]);
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
  const VMCFrame&             frame,
  Frame<uint16_t>&            dispVideoFrame,  // , ColourSpace::YUV400p, ColourSpace::YUV444p
  const VMCEncoderParameters& params) {
  const auto pixelsPerBlock =
    params.geometryVideoBlockSize * params.geometryVideoBlockSize;
  const auto  shift = uint16_t((1 << params.geometryVideoBitDepth) >> 1);
  const auto& disp  = frame.disp;
  const auto  planeCount = dispVideoFrame.planeCount();
  for (int32_t p = 0; p < planeCount; ++p) {
    dispVideoFrame.plane(p).fill(shift);
  }
  for (int32_t v = 0, vcount = int32_t(disp.size()); v < vcount; ++v) {
    const auto& d          = disp[v];
    const auto  blockIndex = v / pixelsPerBlock;  // to do: optimize power of 2
    const auto  indexWithinBlock =
      v % pixelsPerBlock;  // to do: optimize power of 2
    const auto x0 =
      (blockIndex % params.geometryVideoWidthInBlocks)
      * params.geometryVideoBlockSize;  // to do: optimize power of 2
    const auto y0 =
      (blockIndex / params.geometryVideoWidthInBlocks)
      * params.geometryVideoBlockSize;  // to do: optimize power of 2
    int32_t x = 0;
    int32_t y = 0;
    computeMorton2D(indexWithinBlock, x, y);
    assert(x < params.geometryVideoBlockSize);
    assert(y < params.geometryVideoBlockSize);

    const auto x1 = x0 + x;
    const auto y1 = y0 + y;
    for (int32_t p = 0; p < planeCount; ++p) {
      const auto dshift = int32_t(shift + d[p]);
      assert(dshift >= 0 && dshift < (1 << params.geometryVideoBitDepth));
      dispVideoFrame.plane(p).set(y1, x1, uint16_t(dshift));
    }
  }
  return true;
}

//============================================================================

static Vec3<MeshType>
computeNearestPointColour(
  const Vec3<MeshType>&                      point0,
  const Frame<uint8_t>&                      targetTexture,
  const TriangleMesh<MeshType>&              targetMesh,
  const KdTree<MeshType>&                    kdtree,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangleTarget,
  double&                                    minDist2) {
  const auto* const neighboursTarget = vertexToTriangleTarget.neighbours();
  const auto        nnCount          = 1;
  int32_t           index            = 0;
  MeshType          sqrDist          = NAN;
  nanoflann::KNNResultSet<MeshType, int32_t> resultSet(nnCount);
  resultSet.init(&index, &sqrDist);
  kdtree.query(point0.data(), resultSet);
  auto ruv         = targetMesh.texCoord(index);
  minDist2         = std::numeric_limits<MeshType>::max();
  const auto start = vertexToTriangleTarget.neighboursStartIndex(index);
  const auto end   = vertexToTriangleTarget.neighboursEndIndex(index);
  for (int j = start; j < end; ++j) {
    const auto tindex = neighboursTarget[j];
    assert(tindex < targetMesh.triangleCount());
    const auto&    tri = targetMesh.triangle(tindex);
    const auto&    pt0 = targetMesh.point(tri[0]);
    const auto&    pt1 = targetMesh.point(tri[1]);
    const auto&    pt2 = targetMesh.point(tri[2]);
    Vec3<MeshType> bcoord{};
    const auto cpoint = ClosestPointInTriangle(point0, pt0, pt1, pt2, &bcoord);
    //    assert(bcoord[0] >= 0.0 && bcoord[1] >= 0.0 && bcoord[2] >= 0.0);
    assert(fabs(1.0 - bcoord[0] - bcoord[1] - bcoord[2]) < 0.000001);
    const auto d2 = (cpoint - point0).norm2();
    if (d2 < minDist2) {
      minDist2          = d2;
      const auto& triUV = targetMesh.texCoordTriangle(tindex);
      const auto& uv0   = targetMesh.texCoord(triUV[0]);
      const auto& uv1   = targetMesh.texCoord(triUV[1]);
      const auto& uv2   = targetMesh.texCoord(triUV[2]);
      ruv               = bcoord[0] * uv0 + bcoord[1] * uv1 + bcoord[2] * uv2;
    }
  }
  return targetTexture.bilinear(ruv[1], ruv[0]);
}

//----------------------------------------------------------------------------

bool
VMCEncoder::transferTexture(const TriangleMesh<MeshType>& input,
                            const Frame<uint8_t>&         inputTexture,
                            TriangleMesh<MeshType>&       rec,
                            Frame<uint8_t>&               outputTexture,
                            const VMCEncoderParameters&   params) {
  auto targetMesh = input;
  // normalize texcoords so they are between 0.0 and 1.0
  const auto tcCount = targetMesh.texCoordCount();
  const auto uvScale = 1.0 / ((1 << params.bitDepthTexCoord) - 1);
  for (int32_t uvIndex = 0; uvIndex < tcCount; ++uvIndex) {
    targetMesh.setTexCoord(uvIndex, targetMesh.texCoord(uvIndex) * uvScale);
  }
  targetMesh.subdivideMidPoint(
    params.textureTransferSamplingSubdivisionIterationCount);
  if (params.invertOrientation) { rec.invertOrientation(); }
  bool ret =
    transferTexture(targetMesh, rec, inputTexture, outputTexture, params);
  if (params.invertOrientation) { rec.invertOrientation(); }
  return ret;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::transferTexture(const TriangleMesh<MeshType>& targetMesh,
                            const TriangleMesh<MeshType>& sourceMesh,
                            const Frame<uint8_t>&         targetTexture,
                            Frame<uint8_t>&               outputTexture,
                            const VMCEncoderParameters&   params) {
  if ((targetMesh.pointCount() == 0) || (sourceMesh.pointCount() == 0)
      || targetMesh.triangleCount() != targetMesh.texCoordTriangleCount()
      || sourceMesh.triangleCount() != sourceMesh.texCoordTriangleCount()
      || outputTexture.width() <= 0 || outputTexture.height() <= 0) {
    return false;
  }

  StaticAdjacencyInformation<int32_t> vertexToTriangleTarget;
  ComputeVertexToTriangle(
    targetMesh.triangles(), targetMesh.pointCount(), vertexToTriangleTarget);
  KdTree<MeshType> kdtree(3, targetMesh.points(), 10);  // dim, cloud, max leaf
  const auto       oWidth         = outputTexture.width();
  const auto       oHeight        = outputTexture.height();
  const auto       oWidthMinus1   = oWidth - 1;
  const auto       oHeightMinus1  = oHeight - 1;
  auto&            oB             = outputTexture.plane(0);
  auto&            oG             = outputTexture.plane(1);
  auto&            oR             = outputTexture.plane(2);
  const auto       sTriangleCount = sourceMesh.triangleCount();
  Plane<int32_t>   triangleMap;
  triangleMap.resize(oWidth, oHeight);
  Plane<uint8_t> occupancy;
  occupancy.resize(oWidth, oHeight);
  occupancy.fill(uint8_t(0));

  for (int32_t t = 0; t < sTriangleCount; ++t) {
    const auto&        triUV  = sourceMesh.texCoordTriangle(t);
    const Vec2<double> uv[3]  = {sourceMesh.texCoord(triUV[0]),
                                 sourceMesh.texCoord(triUV[1]),
                                 sourceMesh.texCoord(triUV[2])};
    const auto&        triPos = sourceMesh.triangle(t);
    const Vec3<double> pos[3] = {sourceMesh.point(triPos[0]),
                                 sourceMesh.point(triPos[1]),
                                 sourceMesh.point(triPos[2])};
    const auto         area   = (uv[1] - uv[0]) ^ (uv[2] - uv[0]);
    if (area <= 0.0) { continue; }
    assert(area > 0.0);
    const auto iarea = area > 0.0 ? 1.0 / area : 1.0;
    auto       i0    = oHeightMinus1;
    auto       i1    = 0;
    auto       j0    = oWidthMinus1;
    auto       j1    = 0;
    for (const auto& k : uv) {
      const auto i = int32_t(std::floor(k[1] * oHeightMinus1));
      const auto j = int32_t(std::floor(k[0] * oWidthMinus1));
      i0           = std::min(i0, i);
      j0           = std::min(j0, j);
      i1           = std::max(i1, i);
      j1           = std::max(j1, j);
    }
    i0 = std::max(i0, 0);
    i1 = std::min(i1, oHeightMinus1);
    j0 = std::max(j0, 0);
    j1 = std::min(j1, oWidthMinus1);
    for (int32_t i = i0; i <= i1; ++i) {
      const auto y = double(i) / oHeightMinus1;
      for (int32_t j = j0; j <= j1; ++j) {
        const auto         x = double(j) / oWidthMinus1;
        const Vec2<double> uvP(x, y);
        auto               w0 = ((uv[2] - uv[1]) ^ (uvP - uv[1])) * iarea;
        auto               w1 = ((uv[0] - uv[2]) ^ (uvP - uv[2])) * iarea;
        auto               w2 = ((uv[1] - uv[0]) ^ (uvP - uv[0])) * iarea;
        if (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) {
          const auto point0   = w0 * pos[0] + w1 * pos[1] + w2 * pos[2];
          double     minDist2 = NAN;
          const auto bgr      = computeNearestPointColour(point0,
                                                     targetTexture,
                                                     targetMesh,
                                                     kdtree,
                                                     vertexToTriangleTarget,
                                                     minDist2);

          const auto ii = oHeightMinus1 - i;
          oB.set(ii, j, uint8_t(std::round(bgr[0])));
          oG.set(ii, j, uint8_t(std::round(bgr[1])));
          oR.set(ii, j, uint8_t(std::round(bgr[2])));
          occupancy.set(ii, j, uint8_t(255));
          triangleMap.set(ii, j, t);
        }
      }
    }
  }
  const int32_t shift[4][2] = {{-1, -0}, {0, -1}, {1, 0}, {0, 1}};
  for (int32_t it = 0;
       it < params.textureTransferPaddingBoundaryIterationCount;
       ++it) {
    const auto checkValue = uint8_t(255 - it);
    for (int32_t i = 0; i < oHeight; ++i) {
      for (int32_t j = 0; j < oWidth; ++j) {
        if (occupancy(i, j) != 0U) { continue; }
        double       minTriangleDist2 = std::numeric_limits<double>::max();
        Vec3<double> bgr(0.0);
        int32_t      count = 0;
        for (const auto* k : shift) {
          const auto i1 = i + k[0];
          const auto j1 = j + k[1];
          if (i1 < 0 || j1 < 0 || i1 >= oHeight || j1 >= oWidth
              || occupancy(i1, j1) != checkValue) {
            continue;
          }

          const auto y = double(oHeightMinus1 - i) / oHeightMinus1;
          const auto x = double(j) / oWidthMinus1;

          const Vec2<double> uvP(x, y);

          const auto         t     = triangleMap(i1, j1);
          const auto&        triUV = sourceMesh.texCoordTriangle(t);
          const Vec2<double> uv[3] = {sourceMesh.texCoord(triUV[0]),
                                      sourceMesh.texCoord(triUV[1]),
                                      sourceMesh.texCoord(triUV[2])};

          const auto&        triPos = sourceMesh.triangle(t);
          const Vec3<double> pos[3] = {sourceMesh.point(triPos[0]),
                                       sourceMesh.point(triPos[1]),
                                       sourceMesh.point(triPos[2])};
          const auto         area   = (uv[1] - uv[0]) ^ (uv[2] - uv[0]);
          assert(area > 0.0);
          const auto iarea    = area > 0.0 ? 1.0 / area : 1.0;
          const auto w0       = ((uv[2] - uv[1]) ^ (uvP - uv[1])) * iarea;
          const auto w1       = ((uv[0] - uv[2]) ^ (uvP - uv[2])) * iarea;
          const auto w2       = ((uv[1] - uv[0]) ^ (uvP - uv[0])) * iarea;
          const auto point0   = w0 * pos[0] + w1 * pos[1] + w2 * pos[2];
          double     minDist2 = NAN;
          bgr += computeNearestPointColour(point0,
                                           targetTexture,
                                           targetMesh,
                                           kdtree,
                                           vertexToTriangleTarget,
                                           minDist2);
          ++count;
          if (minDist2 < minTriangleDist2) {
            minTriangleDist2 = minDist2;
            triangleMap.set(i, j, t);
          }
        }
        if (count != 0) {
          bgr /= count;
          oB.set(i, j, uint8_t(std::round(bgr[0])));
          oG.set(i, j, uint8_t(std::round(bgr[1])));
          oR.set(i, j, uint8_t(std::round(bgr[2])));
          occupancy.set(i, j, uint8_t(checkValue - 1));
        }
      }
    }
  }  
  if (params.textureTransferPaddingDilateIterationCount != 0) {
    Frame<uint8_t> tmpTexture;
    Plane<uint8_t> tmpOccupancy;
    for (int32_t it = 0;
         it < params.textureTransferPaddingDilateIterationCount;
         ++it) {
      DilatePadding(outputTexture, occupancy, tmpTexture, tmpOccupancy);
      DilatePadding(tmpTexture, tmpOccupancy, outputTexture, occupancy);
    }
  }
  fflush(stdout);
  if (params.textureTransferPaddingMethod == PaddingMethod::PUSH_PULL) {
    PullPushPadding(outputTexture, occupancy);
  } else if (params.textureTransferPaddingMethod
             == PaddingMethod::SPARSE_LINEAR) {
    PullPushPadding(outputTexture, occupancy);
    SparseLinearPadding(outputTexture,
                        occupancy,
                        params.textureTransferPaddingSparseLinearThreshold);
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::encodeSequenceHeader(const VMCGroupOfFrames&     gof,
                                 FrameSequence<uint16_t>&    dispVideo,
                                 Bitstream&                  bitstream,
                                 const VMCEncoderParameters& params) const {
  if (dispVideo.width() < 0 || dispVideo.width() > 16384
      || dispVideo.height() < 0 || dispVideo.height() > 16384
      || (dispVideo.frameCount() != 0
          && dispVideo.frameCount() != gof.frameCount())
      || gof.frameCount() < 0 || gof.frameCount() > 65535
      || params.textureWidth < 0 || params.textureWidth > 16384
      || params.textureHeight < 0 || params.textureHeight > 16384
      || params.bitDepthPosition < 0 || params.bitDepthPosition > 16
      || params.bitDepthTexCoord < 0 || params.bitDepthTexCoord > 16) {
    return false;
  }
  const auto     frameCount             = uint16_t(gof.frameCount());
  const uint16_t widthDispVideo         = uint32_t(dispVideo.width());
  const uint16_t heightDispVideo        = uint32_t(dispVideo.height());
  const uint16_t widthTexVideo          = uint32_t( params.textureWidth );
  const uint16_t heightTexVideo         = uint32_t( params.textureHeight );
  const uint8_t  geometryVideoBlockSize = params.geometryVideoBlockSize;
  const auto     bitDepth               = uint8_t((params.bitDepthPosition - 1)
                                + ((params.bitDepthTexCoord - 1) << 4));
  const uint8_t subdivInfo =
    uint8_t(params.intraGeoParams.subdivisionMethod)
    + ((params.liftingSubdivisionIterationCount) << 4);
  const auto qpBaseMesh =
    uint8_t((params.qpPosition - 1) + ((params.qpTexCoord - 1) << 4));
  const uint8_t liftingQPs[3] = {
    uint8_t(params.liftingQP[0]),
    uint8_t(params.liftingQP[1]),
    uint8_t(params.liftingQP[2])};
  const uint8_t bitField =
    static_cast<int>(params.encodeDisplacementsVideo)
    | (static_cast<int>(params.encodeTextureVideo) << 1)
    | (static_cast<int>(params.applyOneDimensionalDisplacement) << 2);
  bitstream.write(frameCount);
  bitstream.write(bitField);
  bitstream.write(bitDepth);
  bitstream.write(subdivInfo);
#if defined(CODE_CODEC_ID)
  bitstream.write(uint8_t(params.meshCodecId));
#endif
  bitstream.write(qpBaseMesh);
  if (params.encodeDisplacementsVideo) {
#if defined(CODE_CODEC_ID)
    bitstream.write(uint8_t(params.geometryVideoCodecId));
#endif
    bitstream.write(widthDispVideo);
    bitstream.write(heightDispVideo);
    bitstream.write(geometryVideoBlockSize);
    bitstream.write(liftingQPs[0]);
    bitstream.write(liftingQPs[1]);
    bitstream.write(liftingQPs[2]);
  }
  if (params.encodeTextureVideo) {
#if defined(CODE_CODEC_ID)
    bitstream.write(uint8_t(params.textureVideoCodecId));
#endif
    bitstream.write(widthTexVideo);
    bitstream.write(heightTexVideo);
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::encodeFrameHeader(const VMCFrameInfo& frameInfo,
                              Bitstream&          bitstream) {
  const auto frameType = uint8_t(frameInfo.type);
  assert(frameInfo.patchCount >= 0 && frameInfo.patchCount <= 256);
  const auto patchCountMinusOne = uint8_t(frameInfo.patchCount - 1);
  bitstream.write(frameType);
  bitstream.write(patchCountMinusOne);
  if (frameInfo.type != FrameType::INTRA) {
    uint8_t referenceFrameIndex =
      frameInfo.frameIndex - frameInfo.referenceFrameIndex - 1;
    assert(frameInfo.frameIndex > frameInfo.referenceFrameIndex);
    assert(frameInfo.frameIndex - frameInfo.referenceFrameIndex - 1 < 256);
    bitstream.write(referenceFrameIndex);
  }
  return true;
}

//----------------------------------------------------------------------------

bool
VMCEncoder::compress(const VMCGroupOfFramesInfo& gofInfoSrc,
                     const Sequence&             source,
                     Bitstream&                  bitstream,
                     Sequence&                   reconstruct,
                     const VMCEncoderParameters& params) {
  VMCGroupOfFrames gof;
  auto             gofInfo = gofInfoSrc;
  gofInfo.trace();
  const int32_t frameCount             = gofInfo.frameCount_;
  int32_t       lastIntraFrameIndex    = 0;
  _stats.reset();
  _stats.totalByteCount = bitstream.size();
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
    printf("Frame %2d: base = %6d points subdiv = %6d points Ref = %d \n",
           frameIndex,
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
  Bitstream     bitstreamGeo;
  const int32_t pixelsPerBlock =
    params.geometryVideoBlockSize * params.geometryVideoBlockSize;
  const auto widthDispVideo =
    params.geometryVideoWidthInBlocks * params.geometryVideoBlockSize;
  auto                    heightDispVideo = 0;
  const auto colourSpaceDispVideo = params.applyOneDimensionalDisplacement
                                      ? ColourSpace::YUV400p
                                      : ColourSpace::YUV444p;
  FrameSequence<uint16_t> dispVideo;
  dispVideo.resize(0, 0, colourSpaceDispVideo, frameCount);
  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    const auto& frameInfo      = gofInfo.frameInfo(frameIndex);
    auto&       frame          = gof.frame(frameIndex);
    auto&       dispVideoFrame = dispVideo.frame(frameIndex);
    auto&       rec            = reconstruct.mesh(frameIndex);
    compressBaseMesh(gof, frameInfo, frame, rec, bitstreamGeo, params);
    const auto vertexCount = frame.subdiv.pointCount();
    _stats.baseMeshVertexCount += frame.base.pointCount();
    _stats.vertexCount += rec.pointCount();
    _stats.faceCount += rec.triangleCount();
    if (params.encodeDisplacementsVideo) {
      const auto blockCount =
        (vertexCount + pixelsPerBlock - 1) / pixelsPerBlock;
      const auto geometryVideoHeightInBlocks =
        (blockCount + params.geometryVideoWidthInBlocks - 1)
        / params.geometryVideoWidthInBlocks;
      heightDispVideo =
        std::max(heightDispVideo,
                 geometryVideoHeightInBlocks * params.geometryVideoBlockSize);
      dispVideoFrame.resize(
        widthDispVideo, heightDispVideo, colourSpaceDispVideo);
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
    dispVideo.resize(
      widthDispVideo, heightDispVideo, colourSpaceDispVideo, frameCount);
  } else {
    dispVideo.resize(0, 0, ColourSpace::YUV444p, 0);
  }

  // write sequence header
  if (!encodeSequenceHeader(gof, dispVideo, bitstream, params)) {
    return false;
  }
  bitstream.append(bitstreamGeo.buffer);

  // Encode displacements video
  _stats.frameCount             = frameCount;
  _stats.displacementsByteCount = bitstream.size();
  if (params.encodeDisplacementsVideo
      && (!compressDisplacementsVideo(dispVideo, bitstream, params))) {
    return false;
  }
  _stats.displacementsByteCount =
    bitstream.size() - _stats.displacementsByteCount;

  // Reconstruct
  printf("Reconstruct \n");
  reconstruct.textures().resize(
    params.encodeTextureVideo ? params.textureWidth : 1,
    params.encodeTextureVideo ? params.textureHeight : 1,
    ColourSpace::BGR444p,
    frameCount);

  for (int32_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    auto& frame = gof.frame(frameIndex);
    printf("Reconstruct frame %2d / %d \n",frameIndex,frameCount);
    fflush(stdout);
    if (params.encodeDisplacementsVideo) {
      reconstructDisplacementFromVideoFrame(dispVideo.frame(frameIndex),
                                            frame,
                                            reconstruct.mesh(frameIndex),
                                            params.geometryVideoBlockSize,
                                            params.geometryVideoBitDepth);
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
                         reconstruct.mesh(frameIndex),
                         params.displacementCoordinateSystem);
    }
    if (params.encodeTextureVideo
        && !transferTexture(source.mesh(frameIndex),
                            source.texture(frameIndex),
                            reconstruct.mesh(frameIndex),
                            reconstruct.texture(frameIndex),
                            params)) {
      return false;
    }
  }

  // compress texture  
  printf("Compress texture video \n");
  fflush(stdout);
  _stats.textureByteCount = bitstream.size();
  if ( !compressTextureVideo(reconstruct, bitstream, params)) {
    return false;
  }
  printf("Compress texture video done \n");
  fflush(stdout);
  _stats.textureByteCount = bitstream.size() - _stats.textureByteCount;
  _stats.totalByteCount   = bitstream.size() - _stats.totalByteCount;

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
  printf("Compress done \n");
  fflush(stdout);
  return true;
}

//============================================================================

}  // namespace vmesh
