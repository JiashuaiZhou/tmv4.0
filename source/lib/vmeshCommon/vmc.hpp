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

#pragma once

#include <cstdint>
#include <string>

#include "util/image.hpp"
#include "util/mesh.hpp"
#include "vmcStats.hpp"

namespace vmesh {

//============================================================================

enum class FrameType {
  INTER = 0,
  INTRA = 1,
  SKIP  = 2
};

//============================================================================

enum class DisplacementCoordinateSystem {
  CANNONICAL = 0,
  LOCAL      = 1,
};

//============================================================================

enum class PaddingMethod {
  NONE          = 0,
  PUSH_PULL     = 1,
  SPARSE_LINEAR = 2
};

//============================================================================

enum class CachingPoint {
  NONE     = 0,
  SIMPLIFY = 1,
  UVATLAS  = 2,
  SUBDIV   = 3
};

//============================================================================

enum class SmoothingMethod {
  NONE              = 0,
  VERTEX_CONSTRAINT = 1
};

//============================================================================

struct VMCSequenceParameterSet {
  int32_t widthDispVideo                      = 0;
  int32_t heightDispVideo                     = 0;
  int32_t widthTexVideo                       = 0;
  int32_t heightTexVideo                      = 0;
  int32_t frameCount                          = 0;
  int32_t geometryVideoBlockSize              = 0;
  int32_t geometryVideoBitDepth               = 10;
  int32_t textureVideoBitDepth                = 10;
  int32_t bitDepthPosition                    = 0;
  int32_t bitDepthTexCoord                    = 0;
  int32_t qpPosition                          = 0;
  int32_t qpTexCoord                          = 0;
  int32_t subdivisionIterationCount           = 0;
  int32_t liftingQP[3]                        = {0, 0, 0};
  uint8_t motionGroupSize                     = 16;
  double  liftingLevelOfDetailInverseScale[3] = {2.0, 2.0, 2.0};
  double  liftingUpdateWeight                 = 0.125;
  double  liftingPredictionWeight             = 0.5;
  bool    liftingSkipUpdate                   = false;
  bool    encodeDisplacementsVideo            = true;
  bool    encodeTextureVideo                  = true;
  bool    interpolateDisplacementNormals      = false;
  bool    displacementReversePacking          = true;

  SubdivisionMethod subdivisionMethod = SubdivisionMethod::MID_POINT;
  DisplacementCoordinateSystem displacementCoordinateSystem =
    DisplacementCoordinateSystem::LOCAL;

  GeometryCodecId meshCodecId          = GeometryCodecId::DRACO;
  VideoCodecId    geometryVideoCodecId = VideoCodecId::HM;
  VideoCodecId    textureVideoCodecId  = VideoCodecId::HM;
  bool            dracoUsePosition     = false;
  bool            dracoUseUV           = false;
  bool            dracoMeshLossless    = false;
};

//============================================================================

struct VMCFrameInfo {
  int32_t   frameIndex          = -1;
  int32_t   referenceFrameIndex = -1;
  int32_t   previousFrameIndex  = -1;
  int32_t   patchCount          = 1;
  FrameType type                = FrameType::INTRA;
};

//============================================================================

struct VMCGroupOfFramesInfo {
  void resize(int32_t frameCount) { framesInfo_.resize(frameCount); }

  const VMCFrameInfo& frameInfo(int32_t frameIndex) const {
    assert(frameIndex >= 0 && frameIndex < frameCount_);
    return framesInfo_[frameIndex];
  }

  VMCFrameInfo& frameInfo(int32_t frameIndex) {
    assert(frameIndex >= 0 && frameIndex < frameCount_);
    return framesInfo_[frameIndex];
  }

  VMCFrameInfo& operator[](int index) { return framesInfo_[index]; }

  void trace() const {
    printf("  - Gof %2d: frameCount = %d startFrame = %d \n",
           index_,
           frameCount_,
           startFrameIndex_);
    for (const auto& frameInfo : framesInfo_) {
      printf("    - frameIndex = %3d refIndex = %3d type = %s \n",
             frameInfo.frameIndex,
             frameInfo.referenceFrameIndex,
             frameInfo.type == FrameType::INTRA   ? "Intra"
             : frameInfo.type == FrameType::INTER ? "Inter"
                                                  : "Skip");
    }
  }
  int32_t                   startFrameIndex_ = -1;
  int32_t                   frameCount_      = -1;
  int32_t                   index_           = 0;
  std::vector<VMCFrameInfo> framesInfo_;
};

//============================================================================

struct VMCFrame {
  int32_t                           frameIndex;
  TriangleMesh<MeshType>            base;
  TriangleMesh<MeshType>            reference;
  TriangleMesh<MeshType>            mapped;
  TriangleMesh<MeshType>            decimateTexture;
  TriangleMesh<MeshType>            subdiv;
  std::vector<int32_t>              mapping;
  std::vector<Vec3<int32_t>>        qpositions;
  std::vector<Vec3<MeshType>>       disp;
  std::vector<int64_t>              subdivEdges;
  std::vector<SubdivisionLevelInfo> subdivInfoLevelOfDetails;
  TriangleMesh<int32_t>             baseClean;
  std::vector<Vec2<int32_t>>        baseIntegrateIndices;
};

//============================================================================

struct VMCGroupOfFrames {
  void resize(int32_t frameCount) {
    frames.resize(frameCount);
    for (int32_t i = 0; i < frameCount; i++) { frames[i].frameIndex = i; }
  }

  const VMCFrame& frame(int32_t frameIndex) const {
    assert(frameIndex >= 0 && frameIndex < frameCount());
    return frames[frameIndex];
  }

  VMCFrame& frame(int32_t frameIndex) {
    assert(frameIndex >= 0 && frameIndex < frameCount());
    return frames[frameIndex];
  }

  int32_t frameCount() const { return int32_t(frames.size()); }

  VMCFrame& operator[](int index) { return frames[index]; }

  typename std::vector<VMCFrame>::iterator begin() { return frames.begin(); }
  typename std::vector<VMCFrame>::iterator end() { return frames.end(); }

  std::vector<VMCFrame> frames;
};

//============================================================================

class Sequence {
public:
  Sequence() {}
  Sequence(const Sequence&) = default;

  Sequence& operator=(const Sequence&) = default;

  ~Sequence() = default;

  MeshSequence<MeshType>&       meshes() { return _meshes; }
  FrameSequence<uint8_t>&       textures() { return _textures; }
  const MeshSequence<MeshType>& meshes() const { return _meshes; }
  const FrameSequence<uint8_t>& textures() const { return _textures; }

  const auto& mesh(int32_t frameIndex) const { return _meshes[frameIndex]; }
  auto&       mesh(int32_t frameIndex) { return _meshes[frameIndex]; }

  const auto& texture(int32_t frameIndex) const {
    return _textures[frameIndex];
  }
  auto& texture(int32_t frameIndex) { return _textures[frameIndex]; }

  void resize(int32_t frameCount) {
    _meshes.resize(frameCount);
    _textures.resize(0, 0, ColourSpace::BGR444p, frameCount);
  }
  const int32_t frameCount() const { return _meshes.frameCount(); }

  bool load(const std::string& meshPath,
            const std::string& texturePath,
            const int32_t      frameStart,
            const int32_t      frameCount) {
    if (!_textures.load(texturePath, frameStart, frameCount)
        || !_meshes.load(meshPath, frameStart, frameCount))
      return false;
    return true;
  }

  bool save(const std::string& meshPath,
            const std::string& texturePath,
            const std::string& materialLibPath,
            const int32_t      frameStart) {
    vmesh::Material<double> material;
    if (meshPath.empty()) { return true; }
    if ((!materialLibPath.empty()) && extension(meshPath) == "obj") {
      for (int f = 0; f < _meshes.frameCount(); ++f) {
        const auto n      = frameStart + f;
        auto       strTex = vmesh::expandNum(texturePath, n);
        auto       strMat = vmesh::expandNum(materialLibPath, n);
        material.texture  = vmesh::basename(strTex);
        _meshes[f].setMaterialLibrary(vmesh::basename(strMat));
        if (!material.save(strMat)) return false;
      }
    } else {
      for (int f = 0; f < _meshes.frameCount(); ++f) {
        const auto n      = frameStart + f;
        auto       strTex = vmesh::expandNum(texturePath, n);
        _meshes[f].setMaterialLibrary(vmesh::basename(strTex));
      }
    }
    if (!_meshes.save(meshPath, frameStart)
        || !_textures.save(texturePath, frameStart))
      return false;
    return true;
  }

private:
  MeshSequence<MeshType> _meshes;
  FrameSequence<uint8_t> _textures;
};

static int32_t
reconstructDisplacementFromVideoFrame(
  const Frame<uint16_t>&        dispVideoFrame,
  VMCFrame&                     frame,
  const TriangleMesh<MeshType>& rec,
  const int32_t                 geometryVideoBlockSize,
  const int32_t                 geometryVideoBitDepth,
  const int32_t                 displacementReversePacking) {
  printf("Reconstruct displacements from video frame \n");
  fflush(stdout);
  const auto geometryVideoWidthInBlocks =
    dispVideoFrame.width() / geometryVideoBlockSize;
  const auto pixelsPerBlock = geometryVideoBlockSize * geometryVideoBlockSize;
  const auto shift          = uint16_t((1 << geometryVideoBitDepth) >> 1);
  const auto planeCount     = dispVideoFrame.planeCount();
  const auto pointCount     = rec.pointCount();
  auto&      disp           = frame.disp;
  const int32_t start = dispVideoFrame.width() * dispVideoFrame.height() - 1;
  disp.assign(pointCount, Vec3<double>(0));
  for (int32_t v = 0; v < pointCount; ++v) {
    // to do: optimize power of 2
    const auto v0               = displacementReversePacking ? start - v : v;
    const auto blockIndex       = v0 / pixelsPerBlock;
    const auto indexWithinBlock = v0 % pixelsPerBlock;
    const auto x0 =
      (blockIndex % geometryVideoWidthInBlocks) * geometryVideoBlockSize;
    const auto y0 =
      (blockIndex / geometryVideoWidthInBlocks) * geometryVideoBlockSize;
    int32_t x = 0;
    int32_t y = 0;
    computeMorton2D(indexWithinBlock, x, y);
    assert(x < geometryVideoBlockSize);
    assert(y < geometryVideoBlockSize);
    const auto x1 = x0 + x;
    const auto y1 = y0 + y;
    auto&      d  = disp[v];
    for (int32_t p = 0; p < planeCount; ++p) {
      const auto& plane = dispVideoFrame.plane(p);
      d[p]              = double(plane.get(y1, x1)) - shift;
    }
  }
  return 0;
}

//----------------------------------------------------------------------------

static int32_t
subdivideBaseMesh(VMCFrame&               frame,
                  TriangleMesh<MeshType>& rec,
                  const SubdivisionMethod subdivisionMethod,
                  const int32_t           subdivisionIterationCount,
                  const int32_t           interpolateDisplacementNormals,
                  const bool              meshLossless) {
  printf("Subdivide base mesh \n");
  fflush(stdout);
  auto& infoLevelOfDetails = frame.subdivInfoLevelOfDetails;
  auto& subdivEdges        = frame.subdivEdges;
  rec                      = frame.base;
  if (interpolateDisplacementNormals && (!meshLossless)) {
    rec.computeNormals();
  }
  if (subdivisionMethod == SubdivisionMethod::MID_POINT) {
    rec.subdivideMidPoint(
      subdivisionIterationCount, &infoLevelOfDetails, &subdivEdges);
  } else {
    return -1;
  }
  if (!meshLossless) rec.resizeNormals(rec.pointCount());
  if (interpolateDisplacementNormals) {
    interpolateSubdivision(
      rec.normals(), infoLevelOfDetails, subdivEdges, 0.5, 0.5, true);
  } else {
    if (!meshLossless) rec.computeNormals();
  }
  return 0;
}

//----------------------------------------------------------------------------

static int32_t
applyDisplacements(
  VMCFrame&                           frame,
  TriangleMesh<MeshType>&             rec,
  const DisplacementCoordinateSystem& displacementCoordinateSystem) {
  printf("apply displacements \n");
  fflush(stdout);
  const auto& disp = frame.disp;
  for (int32_t v = 0, vcount = rec.pointCount(); v < vcount; ++v) {
    const auto& d = disp[v];
    if (displacementCoordinateSystem == DisplacementCoordinateSystem::LOCAL) {
      const auto     n = rec.normal(v);
      Vec3<MeshType> t{};
      Vec3<MeshType> b{};
      computeLocalCoordinatesSystem(n, t, b);
      rec.point(v) += d[0] * n + d[1] * t + d[2] * b;
    } else {
      rec.point(v) += d;
    }
  }
  return 0;
}

//----------------------------------------------------------------------------

static int32_t
inverseQuantizeDisplacements(
  VMCFrame&     frame,
  const int32_t bitDepthPosition,
  const double (&liftingLevelOfDetailInverseScale)[3],
  const int32_t (&liftingQP)[3]) {
  printf("Inverse quantize displacements \n");
  fflush(stdout);
  const auto& infoLevelOfDetails = frame.subdivInfoLevelOfDetails;
  const auto  lodCount           = int32_t(infoLevelOfDetails.size());
  assert(lodCount > 0);
  double iscale[3];
  double ilodScale[3];
  for (int32_t k = 0; k < 3; ++k) {
    const auto qp = liftingQP[k];
    iscale[k] =
      qp >= 0 ? pow(0.5, 16 - bitDepthPosition + (4 - qp) / 6.0) : 0.0;
    ilodScale[k] = liftingLevelOfDetailInverseScale[k];
  }
  auto& disp = frame.disp;
  for (int32_t it = 0, vcount0 = 0; it < lodCount; ++it) {
    const auto vcount1 = infoLevelOfDetails[it].pointCount;
    for (int32_t v = vcount0; v < vcount1; ++v) {
      auto& d = disp[v];
      for (int32_t k = 0; k < 3; ++k) { d[k] *= iscale[k]; }
    }
    vcount0 = vcount1;
    for (int32_t k = 0; k < 3; ++k) { iscale[k] *= ilodScale[k]; }
  }
  return 0;
}

//============================================================================

static inline std::string
removeFileExtension(const std::string string) {
  size_t pos = string.find_last_of(".");
  return pos != std::string::npos ? string.substr(0, pos) : string;
}

//============================================================================

static int32_t
removeDuplicatedVertices(VMCFrame& frame) {
  auto& base                 = frame.base;
  auto& qpositions           = frame.qpositions;
  auto& baseClean            = frame.baseClean;
  auto& baseIntegrateIndices = frame.baseIntegrateIndices;
  baseClean.clear();
  std::vector<int32_t> umapping;
  UnifyVertices(qpositions,
                base.triangles(),
                baseClean.points(),
                baseClean.triangles(),
                umapping);
  // generate baseIntegrateIndices
  baseIntegrateIndices.clear();
  std::map<int32_t, int32_t> uniqueIndices;
  const auto                 pointCount0 = int32_t(umapping.size());
  for (int32_t vindex0 = 0; vindex0 < pointCount0; ++vindex0) {
    auto vindex1 = umapping[vindex0];
    auto it      = uniqueIndices.find(vindex1);
    if (it == uniqueIndices.end()) {
      uniqueIndices[vindex1] = vindex0;
    } else {
      baseIntegrateIndices.push_back({vindex0, it->second});
    }
  }
  // copy UV coords and UV indices as is
  baseClean.texCoords().resize(base.texCoordCount());
  std::copy(base.texCoords().begin(),
            base.texCoords().end(),
            baseClean.texCoords().begin());
  baseClean.texCoordTriangles().resize(base.texCoordTriangleCount());
  std::copy(base.texCoordTriangles().begin(),
            base.texCoordTriangles().end(),
            baseClean.texCoordTriangles().begin());
  return 0;
}

//============================================================================

typedef std::tuple<
  std::string,
  std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>,
  std::chrono::nanoseconds>
                           TicToc;
static std::vector<TicToc> g_ticTocList;

//============================================================================

static void
tic(const std::string& name) {
  auto it = std::find_if(
    g_ticTocList.begin(), g_ticTocList.end(), [&name](const TicToc& element) {
      return std::get<0>(element) == name;
    });

  auto start = std::chrono::steady_clock::now();
  if (it != g_ticTocList.end()) {
    std::get<1>(*it) = start;
  } else {
    g_ticTocList.push_back(
      std::make_tuple(name, start, (std::chrono::nanoseconds)0));
  }
}

//============================================================================

static void
toc(const std::string& name) {
  auto it = std::find_if(
    g_ticTocList.begin(), g_ticTocList.end(), [&name](const TicToc& element) {
      return std::get<0>(element) == name;
    });

  if (it != g_ticTocList.end()) {
    auto end      = std::chrono::steady_clock::now();
    auto duration = end - std::get<1>(*it);
    std::get<2>(*it) += duration;
    std::cout << "Duration " << std::left << std::setw(25) << name
              << ": time = " << std::right << std::setw(15)
              << std::chrono::duration<double>(duration).count()
              << " s Total = " << std::setw(15)
              << std::chrono::duration<double>(std::get<2>(*it)).count()
              << "\n";
  } else {
    std::cout << "Duration " << name << ": can't find clock\n";
  }
}

//============================================================================

static void
traceTime() {
  std::cout << "Duration: \n";
  for (auto& el : g_ticTocList) {
    // auto duration = std::get<2>(el) - std::get<1>(el);
    std::cout << "  " << std::left << std::setw(25) << std::get<0>(el)
              << ": time = " << std::right << std::setw(15)
              << std::chrono::duration<double>(std::get<2>(el)).count()
              << " s \n";
  }
}

//============================================================================

}  // namespace vmesh
