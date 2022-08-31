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

#include <cassert>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <map>
#include <vector>
#include <tinyply.h>

#include "util/box.hpp"
#include "util/misc.hpp"
#include "util/sparsematrix.hpp"
#include "util/triangle.hpp"

namespace vmesh {

//============================================================================

enum GeometryCodecId {
#if defined(USE_DRACO_GEOMETRY_CODEC)
  DRACO = 0,
#endif
  UNKNOWN_GEOMETRY_CODEC = 255
};

//============================================================================

static std::istream&
operator>>(std::istream& in, GeometryCodecId& val) {
  std::string str;
  in >> str;
  val = GeometryCodecId::UNKNOWN_GEOMETRY_CODEC;
  printf("GeometryCodecId: str = %s \n", str.c_str());
#if defined(USE_DRACO_GEOMETRY_CODEC)
  printf("GeometryCodecId: here \n");
  if (str == "DRACO") { val = GeometryCodecId::DRACO; }
#endif
  if (val == GeometryCodecId::UNKNOWN_GEOMETRY_CODEC) {
    in.setstate(std::ios::failbit);
  }
  return in;
}

//============================================================================

enum class SubdivisionMethod {
  MID_POINT = 0,
};

//============================================================================

template<typename T>
class StaticAdjacencyInformation {
public:
  StaticAdjacencyInformation()                                  = default;
  StaticAdjacencyInformation(const StaticAdjacencyInformation&) = default;
  StaticAdjacencyInformation&
  operator=(const StaticAdjacencyInformation&) = default;
  ~StaticAdjacencyInformation()                = default;

  void clear() {
    _shift.resize(0);
    _neighbourCount.resize(0);
    _neighbours.resize(0);
  }

  int32_t size() const { return int32_t(_neighbourCount.size()); }
  void    reserve(const int32_t sz) {
    _shift.reserve(sz + 1);
    _neighbourCount.reserve(sz);
    _neighbours.reserve(6 * sz);
  }

  void resize(const int32_t sz) {
    _shift.resize(0);
    _neighbourCount.resize(0);
    _shift.resize(sz + 1, 0);
    _neighbourCount.resize(sz, 0);
  }

  void incrementNeighbourCount(const int32_t i) {
    assert(i < size());
    ++_shift[i + 1];
  }

  void incrementNeighbourCount(const int32_t i, const int32_t count) {
    assert(i < size());
    _shift[i + 1] += count;
  }

  void updateShift() {
    for (int32_t i = 1, count = int32_t(_shift.size()); i < count; ++i) {
      _shift[i] += _shift[i - 1];
    }
    _neighbours.resize(_shift.back());
  }

  int32_t maxNeighbourCount(const int32_t i) const {
    assert(i < size());
    assert(_shift[i + 1] >= _shift[i]);
    return _shift[i + 1] - _shift[i];
  }

  int32_t neighbourCount(const int32_t i) const {
    assert(i < size());
    return _neighbourCount[i];
  }

  void addNeighbour(const int32_t i, const T& neighbour) {
    assert(i >= 0 && i <= size());
    assert(neighbour >= 0);
    assert(_neighbourCount[i] < maxNeighbourCount(i));
    _neighbours[_shift[i] + _neighbourCount[i]++] = neighbour;
  }

  int32_t neighboursStartIndex(const int32_t i) const {
    assert(i < size());
    return _shift[i];
  }

  int32_t neighboursEndIndex(const int32_t i) const {
    assert(i < size());
    return _shift[i] + _neighbourCount[i];
  }

  const T* neighbours() const { return _neighbours.data(); }
  T*       neighbours() { return _neighbours.data(); }

private:
  std::vector<int32_t> _shift;
  std::vector<int32_t> _neighbourCount;
  std::vector<T>       _neighbours;
};

//============================================================================

struct SubdivisionLevelInfo {
  int32_t pointCount            = -1;
  int32_t texCoordCount         = -1;
  int32_t triangleCount         = -1;
  int32_t texCoordTriangleCount = -1;
};

//============================================================================

template<typename T>
struct Material {
  std::string  name             = "material0000";
  std::string  texture          = "texture.png";
  Vec3<double> ambiant          = Vec3<double>(1.0);
  Vec3<double> diffuse          = Vec3<double>(1.0);
  Vec3<double> specular         = Vec3<double>(1.0);
  double       specularExponent = 0.1;
  double       transparency     = 1.0;
  int32_t      illumination     = 2;
  bool         save(const std::string& fileName) {
    std::ofstream fout(fileName);
    if (fout.is_open()) {
      fout << "newmtl " << name << '\n';
      fout << "Ka " << ambiant << '\n';
      fout << "Kd " << diffuse << '\n';
      fout << "Ks " << specular << '\n';
      fout << "Tr " << transparency << '\n';
      fout << "illum " << illumination << '\n';
      fout << "Ns " << specularExponent << '\n';
      fout << "map_Kd " << texture << '\n';
      fout.close();
      return true;
    }
    return false;
  }
};

//============================================================================

template<typename T>
class TriangleMesh {
public:
  bool load(const std::string& fileName);
  bool save(const std::string& fileName, const T uvScale = T(1));

  TriangleMesh<T>& operator=(const TriangleMesh<T>&) = default;

  void scaleTextureCoordinates(int qt) {
    const auto scale = qt > 0 ? 1.0 / ((1 << qt) - 1) : 1.0;
    for (auto& v : _texCoord) { v *= scale; }
  }
  const std::string& materialLibrary() const { return _mtllib; }
  std::string&       materialLibrary() { return _mtllib; }
  void setMaterialLibrary(const std::string& mtllib) { _mtllib = mtllib; }

  int32_t displacementCount() const { return int32_t(_disp.size()); }
  int32_t pointCount() const { return int32_t(_coord.size()); }
  int32_t colourCount() const { return int32_t(_colour.size()); }
  int32_t triangleCount() const { return int32_t(_coordIndex.size()); }
  int32_t texCoordCount() const { return int32_t(_texCoord.size()); }

  int32_t texCoordTriangleCount() const {
    return int32_t(_texCoordIndex.size());
  }

  int32_t normalCount() const { return int32_t(_normal.size()); }
  int32_t normalTriangleCount() const { return int32_t(_normalIndex.size()); }

  const Vec3<T>& displacement(const int32_t dispIndex) const {
    assert(dispIndex >= 0 && dispIndex < displacementCount());
    return _disp[dispIndex];
  }

  Vec3<T>& displacement(const int32_t dispIndex) {
    assert(dispIndex >= 0 && dispIndex < displacementCount());
    return _disp[dispIndex];
  }

  const Vec3<T>& point(const int32_t pointIndex) const {
    assert(pointIndex >= 0 && pointIndex < pointCount());
    return _coord[pointIndex];
  }

  Vec3<T>& point(const int32_t pointIndex) {
    assert(pointIndex >= 0 && pointIndex < pointCount());
    return _coord[pointIndex];
  }

  const Vec3<T>& colour(const int32_t colourIndex) const {
    assert(colourIndex >= 0 && colourIndex < colourCount());
    return _colour[colourIndex];
  }

  Vec3<T>& colour(const int32_t colourIndex) {
    assert(colourIndex >= 0 && colourIndex < colourCount());
    return _colour[colourIndex];
  }

  const Vec2<T>& texCoord(const int32_t texCoordIndex) const {
    assert(texCoordIndex >= 0 && texCoordIndex < texCoordCount());
    return _texCoord[texCoordIndex];
  }

  Vec2<T>& texCoord(const int32_t texCoordIndex) {
    assert(texCoordIndex >= 0 && texCoordIndex < texCoordCount());
    return _texCoord[texCoordIndex];
  }

  const Vec3<T>& normal(const int32_t normalIndex) const {
    assert(normalIndex >= 0 && normalIndex < normalCount());
    return _normal[normalIndex];
  }

  Vec3<T>& normal(const int32_t normalIndex) {
    assert(normalIndex >= 0 && normalIndex < normalCount());
    return _normal[normalIndex];
  }

  T area() const {
    T area = T(0);
    for (const auto& tri : _coordIndex) {
      const auto i = tri[0];
      const auto j = tri[1];
      const auto k = tri[2];
      const auto a = _coord[i];
      const auto b = _coord[j];
      const auto c = _coord[k];
      area +=
        computeTriangleArea(_coord[tri[0]], _coord[tri[1]], _coord[tri[2]]);
    }
    return area;
  }

  void computeTriangleNormals(std::vector<Vec3<T>>& normals,
                              const bool            normalize = true) const {
    const auto triCount = triangleCount();
    normals.resize(triCount);
    for (int32_t t = 0; t < triCount; ++t) {
      const auto& tri = triangle(t);
      normals[t]      = computeTriangleNormal(
        _coord[tri[0]], _coord[tri[1]], _coord[tri[2]], normalize);
    }
  }

  void computeNormals(const bool normalize = true) {
    const auto ptCount = pointCount();
    _normal.resize(ptCount);
    std::fill(_normal.begin(), _normal.end(), T(0));
    for (const auto& tri : _coordIndex) {
      const auto i = tri[0];
      const auto j = tri[1];
      const auto k = tri[2];
      const auto n =
        computeTriangleNormal(_coord[i], _coord[j], _coord[k], false);
      _normal[i] += n;
      _normal[j] += n;
      _normal[k] += n;
    }
    if (normalize) {
      for (auto& n : _normal) { n.normalize(); }
    }
  }

  void computeUnitaryFaceNormals() {
    computeNormals();
    for (auto& n : _normal) { n.normalize(); }
  }

  Box3<T> dispBoundingBox() const {
    Box3<T> bbox;
    bbox.min = std::numeric_limits<T>::max();
    bbox.max = std::numeric_limits<T>::min();
    for (const auto& pt : _disp) {
      for (int32_t k = 0; k < 3; ++k) {
        bbox.min[k] = std::min(bbox.min[k], pt[k]);
        bbox.max[k] = std::max(bbox.max[k], pt[k]);
      }
    }
    return bbox;
  }

  Box3<T> boundingBox() const {
    Box3<T> bbox;
    bbox.min = std::numeric_limits<T>::max();
    bbox.max = std::numeric_limits<T>::min();
    for (const auto& pt : _coord) {
      for (int32_t k = 0; k < 3; ++k) {
        bbox.min[k] = std::min(bbox.min[k], pt[k]);
        bbox.max[k] = std::max(bbox.max[k], pt[k]);
      }
    }
    return bbox;
  }

  Box2<T> texCoordBoundingBox() const {
    Box2<T> bbox;
    bbox.min = std::numeric_limits<T>::max();
    bbox.max = std::numeric_limits<T>::min();
    for (const auto& pt : _texCoord) {
      for (int32_t k = 0; k < 2; ++k) {
        bbox.min[k] = std::min(bbox.min[k], pt[k]);
        bbox.max[k] = std::max(bbox.max[k], pt[k]);
      }
    }
    return bbox;
  }

  Box3<T> normalsBoundingBox() const {
    Box3<T> bbox;
    bbox.min = std::numeric_limits<T>::max();
    bbox.max = std::numeric_limits<T>::min();
    for (const auto& normal : _normal) {
      for (int32_t k = 0; k < 3; ++k) {
        bbox.min[k] = std::min(bbox.min[k], normal[k]);
        bbox.max[k] = std::max(bbox.max[k], normal[k]);
      }
    }
    return bbox;
  }

  void invertOrientation();

  const std::vector<Vec3<T>>&   displacements() const { return _disp; }
  const std::vector<Vec3<T>>&   points() const { return _coord; }
  const std::vector<Vec3<T>>&   colours() const { return _colour; }
  const std::vector<Vec3<int>>& triangles() const { return _coordIndex; }
  const std::vector<Vec2<T>>&   texCoords() const { return _texCoord; }
  const std::vector<Vec3<int>>& texCoordTriangles() const {
    return _texCoordIndex;
  }

  const std::vector<Vec3<T>>&   normals() const { return _normal; }
  const std::vector<Vec3<int>>& normalTriangles() const {
    return _normalIndex;
  }

  std::vector<Vec3<T>>&   displacements() { return _disp; }
  std::vector<Vec3<T>>&   points() { return _coord; }
  std::vector<Vec3<T>>&   colours() { return _colour; }
  std::vector<Vec3<int>>& triangles() { return _coordIndex; }
  std::vector<Vec2<T>>&   texCoords() { return _texCoord; }
  std::vector<Vec3<int>>& texCoordTriangles() { return _texCoordIndex; }
  std::vector<Vec3<T>>&   normals() { return _normal; }
  std::vector<Vec3<int>>& normalTriangles() { return _normalIndex; }

  Vec3<int> triangle(const int32_t triangleIndex) const {
    assert(triangleIndex >= 0 && triangleIndex < triangleCount());
    return _coordIndex[triangleIndex];
  }

  Vec3<int> texCoordTriangle(const int32_t texCoordTriangleIndex) const {
    assert(texCoordTriangleIndex >= 0
           && texCoordTriangleIndex < texCoordTriangleCount());
    return _texCoordIndex[texCoordTriangleIndex];
  }

  Vec3<int> normalTriangle(const int32_t normalTriangleIndex) const {
    assert(normalTriangleIndex >= 0
           && normalTriangleIndex < normalTriangleCount());
    return _normalIndex[normalTriangleIndex];
  }

  void setDisplacement(const int32_t dispIndex,
                       const T       dx,
                       const T       dy,
                       const T       dz) {
    assert(dispIndex >= 0 && dispIndex < displacementCount());
    auto& disp = _disp[dispIndex];
    disp[0]    = dx;
    disp[1]    = dy;
    disp[2]    = dz;
  }

  void setDisplacement(const int32_t dispIndex, const Vec3<T>& pt) {
    assert(dispIndex >= 0 && dispIndex < displacementCount());
    _disp[dispIndex] = pt;
  }

  void setPoint(const int32_t pointIndex, const T x, const T y, const T z) {
    assert(pointIndex >= 0 && pointIndex < pointCount());
    auto& pt = _coord[pointIndex];
    pt[0]    = x;
    pt[1]    = y;
    pt[2]    = z;
  }

  void setPoint(const int32_t pointIndex, const Vec3<T>& pt) {
    assert(pointIndex >= 0 && pointIndex < pointCount());
    _coord[pointIndex] = pt;
  }

  void setColour(const int32_t colourIndex, const T r, const T g, const T b) {
    assert(colourIndex >= 0 && colourIndex < colourCount());
    auto& rgb = _colour[colourIndex];
    rgb[0]    = r;
    rgb[1]    = g;
    rgb[2]    = b;
  }

  void setColour(const int32_t colourIndex, const Vec3<T>& rgb) {
    assert(colourIndex >= 0 && colourIndex < colourCount());
    _colour[colourIndex] = rgb;
  }

  void setTexCoord(const int32_t texCoordIndex, const T x, const T y) {
    assert(texCoordIndex >= 0 && texCoordIndex < texCoordCount());
    auto& pt = _texCoord[texCoordIndex];
    pt[0]    = x;
    pt[1]    = y;
  }

  void setTexCoord(const int32_t texCoordIndex, const Vec2<T>& texCoord) {
    assert(texCoordIndex >= 0 && texCoordIndex < texCoordCount());
    _texCoord[texCoordIndex] = texCoord;
  }

  void setNormal(const int32_t normalIndex, const T x, const T y, const T z) {
    assert(normalIndex >= 0 && normalIndex < normalCount());
    auto& normal = _normal[normalIndex];
    normal[0]    = x;
    normal[1]    = y;
    normal[2]    = z;
  }

  void setNormal(const int32_t normalIndex, const Vec3<T>& pt) {
    assert(normalIndex >= 0 && normalIndex < normalCount());
    _normal[normalIndex] = pt;
  }

  void addDisplacement(const Vec3<T>& disp) { _disp.push_back(disp); }
  void addDisplacement(const T dx, const T dy, const T dz) {
    _disp.push_back(Vec3<T>(dx, dy, dz));
  }

  void addPoint(const Vec3<T>& pt) { _coord.push_back(pt); }
  void addPoint(const T x, const T y, const T z) {
    _coord.push_back(Vec3<T>(x, y, z));
  }

  void addColour(const Vec3<T>& rgb) { _colour.push_back(rgb); }
  void addColour(const T r, const T g, const T b) {
    _colour.push_back(Vec3<T>(r, g, b));
  }

  void addTexCoord(const Vec2<T>& pt) { _texCoord.push_back(pt); }
  void addTexCoord(const T u, const T v) {
    _texCoord.push_back(Vec2<T>(u, v));
  }

  void addNormal(const Vec3<T>& pt) { _normal.push_back(pt); }
  void addNormal(const T x, const T y, const T z) {
    _normal.push_back(Vec3<T>(x, y, z));
  }

  void addTriangle(const Triangle& tri) { _coordIndex.push_back(tri); }
  void addTriangle(const int32_t i, const int32_t j, const int32_t k) {
    _coordIndex.emplace_back(i, j, k);
  }

  void addTexCoordTriangle(const Triangle& tri) {
    _texCoordIndex.push_back(tri);
  }

  void addTexCoordTriangle(const int32_t i, const int32_t j, const int32_t k) {
    _texCoordIndex.emplace_back(i, j, k);
  }

  void addNormalTriangle(const Triangle& tri) { _normalIndex.push_back(tri); }
  void addNormalTriangle(const int32_t i, const int32_t j, const int32_t k) {
    _normalIndex.emplace_back(i, j, k);
  }

  void setTriangle(const int32_t triangleIndex,
                   const int32_t i,
                   const int32_t j,
                   const int32_t k) {
    assert(triangleIndex >= 0 && triangleIndex < triangleCount());
    auto& tri = _coordIndex[triangleIndex];
    tri[0]    = i;
    tri[1]    = j;
    tri[2]    = k;
  }

  void setTriangle(const int32_t triangleIndex, const Triangle& tri) {
    assert(triangleIndex >= 0 && triangleIndex < triangleCount());
    _coordIndex[triangleIndex] = tri;
  }

  void setTexCoordTriangle(const int32_t triangleIndex,
                           const int32_t i,
                           const int32_t j,
                           const int32_t k) {
    assert(triangleIndex >= 0 && triangleIndex < texCoordTriangleCount());
    auto& tri = _texCoordIndex[triangleIndex];
    tri[0]    = i;
    tri[1]    = j;
    tri[2]    = k;
  }

  void setTexCoordTriangle(const int32_t triangleIndex, const Triangle& tri) {
    assert(triangleIndex >= 0 && triangleIndex < texCoordTriangleCount());
    _texCoordIndex[triangleIndex] = tri;
  }

  void setNormalTriangle(const int32_t triangleIndex,
                         const int32_t i,
                         const int32_t j,
                         const int32_t k) {
    assert(triangleIndex >= 0 && triangleIndex < normalTriangleCount());
    auto& tri = _normalIndex[triangleIndex];
    tri[0]    = i;
    tri[1]    = j;
    tri[2]    = k;
  }

  void setNormalTriangle(const int32_t triangleIndex, const Triangle& tri) {
    assert(triangleIndex >= 0 && triangleIndex < normalTriangleCount());
    _normalIndex[triangleIndex] = tri;
  }

  void resizeDisplacements(const int32_t ptCount) {
    _disp.resize(ptCount, 0.0);
  }

  void resizePoints(const int32_t ptCount) { _coord.resize(ptCount, 0.0); }

  void resizeColours(const int32_t colourCount) {
    _colour.resize(colourCount, 0.0);
  }

  void resizeTexCoords(const int32_t texCoordCount) {
    _texCoord.resize(texCoordCount, 0.0);
  }

  void resizeNormals(const int32_t normalCount) {
    _normal.resize(normalCount, 0.0);
  }

  void resizeTriangles(const int32_t triCount) {
    _coordIndex.resize(triCount, -1);
  }

  void resizeTexCoordTriangles(const int32_t texCoordTriangleCount) {
    _texCoordIndex.resize(texCoordTriangleCount, -1);
  }

  void resizeNormalTriangles(const int32_t normalTriangleCount) {
    _normalIndex.resize(normalTriangleCount, -1);
  }

  void reserveDisplacements(const int32_t ptCount) { _disp.reserve(ptCount); }
  void reservePoints(const int32_t ptCount) { _coord.reserve(ptCount); }
  void reserveColours(const int32_t colourCount) {
    _colour.reserve(colourCount);
  }
  void reserveTriangles(const int32_t triCount) {
    _coordIndex.reserve(triCount);
  }

  void reserveTexCoords(const int32_t texCoordCount) {
    _texCoord.reserve(texCoordCount);
  }

  void reserveTexCoordTriangles(const int32_t texCoordTriangleCount) {
    _texCoordIndex.reserve(texCoordTriangleCount);
  }

  void reserveNormals(const int32_t normalCount) {
    _normal.reserve(normalCount);
  }

  void reserveNormalTriangles(const int32_t normalTriangleCount) {
    _normalIndex.reserve(normalTriangleCount);
  }

  void clear() {
    _disp.clear();
    _coord.clear();
    _colour.clear();
    _coordIndex.clear();
    _texCoord.clear();
    _texCoordIndex.clear();
    _normal.clear();
    _normalIndex.clear();
  }

  void append(const TriangleMesh<double>& mesh);

  void subdivideMidPoint(
    int32_t                            iterationCount,
    std::vector<SubdivisionLevelInfo>* infoLevelOfDetails         = nullptr,
    std::vector<int64_t>*              coordEdges                 = nullptr,
    std::vector<int64_t>*              texCoordEdges              = nullptr,
    std::vector<int32_t>*              triangleToBaseMeshTriangle = nullptr);

  void
  computeTexCoordToPointMapping(std::vector<int32_t>& texCoordToPoint) const {
    const auto uvCount = texCoordCount();
    texCoordToPoint.resize(uvCount);
    std::fill(texCoordToPoint.begin(), texCoordToPoint.end(), -1);
    for (int32_t t = 0, tcount = triangleCount(); t < tcount; ++t) {
      const auto& tri   = triangle(t);
      const auto& triUV = texCoordTriangle(t);
      for (int32_t k = 0; k < 3; ++k) { texCoordToPoint[triUV[k]] = tri[k]; }
    }
  }
  template<typename D>
  void convert(const TriangleMesh<D>& src) {
    clear();

    _mtllib = src.materialLibrary();

    _coordIndex.resize(src.triangles().size());
    for (int i = 0; i < src.triangles().size(); i++) {
      _coordIndex[i] = src.triangle(i);
    }

    _texCoordIndex.resize(src.texCoordTriangles().size());
    for (int i = 0; i < src.texCoordTriangles().size(); i++) {
      _texCoordIndex[i] = src.texCoordTriangle(i);
    }

    _normalIndex.resize(src.normalTriangles().size());
    for (int i = 0; i < src.normalTriangles().size(); i++) {
      _texCoordIndex[i] = src.normalTriangle(i);
    }

    _disp.resize(src.displacements().size());
    for (int i = 0; i < src.displacements().size(); i++) {
      _disp[i] = (Vec3<T>)src.displacement(i);
    }

    _coord.resize(src.points().size());
    for (int i = 0; i < src.points().size(); i++) {
      _coord[i] = (Vec3<T>)src.point(i);
    }

    _colour.resize(src.colours().size());
    for (int i = 0; i < src.colours().size(); i++) {
      _colour[i] = (Vec3<T>)src.colour(i);
    }

    _texCoord.resize(src.texCoords().size());
    for (int i = 0; i < src.texCoords().size(); i++) {
      _texCoord[i] = (Vec2<T>)src.texCoord(i);
    }

    _normal.resize(src.normals().size());
    for (int i = 0; i < src.normals().size(); i++) {
      _normal[i] = (Vec3<T>)src.normal(i);
    }
  }

private:
  bool loadFromOBJ(const std::string& fileName);
  bool loadFromPLY(const std::string& fileName);
  bool saveToOBJ(const std::string& fileName, const T uvScale = T(1)) const;
  bool saveToPLY(const std::string& fileName,
                 const bool         binary,
                 const T            uvScale = T(1)) const;

  std::vector<Vec3<T>>   _disp;
  std::vector<Vec3<T>>   _coord;
  std::vector<Vec3<T>>   _colour;
  std::vector<Vec3<int>> _coordIndex;
  std::vector<Vec2<T>>   _texCoord;
  std::vector<Vec3<int>> _texCoordIndex;
  std::vector<Vec3<T>>   _normal;
  std::vector<Vec3<int>> _normalIndex;
  std::string            _mtllib;
};

//============================================================================

inline void
ComputeVertexToTriangle(
  const std::vector<Triangle>&         triangles,
  const int32_t                        vertexCount,
  StaticAdjacencyInformation<int32_t>& vertexToTriangle) {
  const auto triangleCount = int32_t(triangles.size());
  vertexToTriangle.resize(vertexCount);
  for (int32_t triangleIndex = 0; triangleIndex < triangleCount;
       ++triangleIndex) {
    const auto& tri = triangles[triangleIndex];
    assert(tri.i() >= 0 && tri.i() < vertexCount);
    assert(tri.j() >= 0 && tri.j() < vertexCount);
    assert(tri.k() >= 0 && tri.k() < vertexCount);
    vertexToTriangle.incrementNeighbourCount(tri.i());

    if (tri.j() != tri.i()) {  // check for degenerated case
      vertexToTriangle.incrementNeighbourCount(tri.j());
    }

    if (tri.k() != tri.i()
        && tri.k() != tri.j()) {  // check for degenerated case
      vertexToTriangle.incrementNeighbourCount(tri.k());
    }
  }
  vertexToTriangle.updateShift();

  for (int32_t triangleIndex = 0; triangleIndex < triangleCount;
       ++triangleIndex) {
    const auto& tri = triangles[triangleIndex];
    vertexToTriangle.addNeighbour(tri.i(), triangleIndex);
    if (tri.j() != tri.i()) {  // check for degenerated case
      vertexToTriangle.addNeighbour(tri.j(), triangleIndex);
    }

    if (tri.k() != tri.i()
        && tri.k() != tri.j()) {  // check for degenerated case
      vertexToTriangle.addNeighbour(tri.k(), triangleIndex);
    }
  }
}

//----------------------------------------------------------------------------

inline void
TagAdjacentTriangles(
  int32_t                                    vindex,
  const int8_t                               tag,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<int8_t>&                       ttags) {
  assert(vindex < vertexToTriangle.size());
  const auto& tadj  = vertexToTriangle.neighbours();
  const auto  start = vertexToTriangle.neighboursStartIndex(vindex);
  const auto  end   = vertexToTriangle.neighboursEndIndex(vindex);
  for (int i = start; i < end; ++i) { ttags[tadj[i]] = tag; }
}

//----------------------------------------------------------------------------

inline void
IncrementTagAdjacentTriangles(
  int32_t                                    vindex,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<int8_t>&                       ttags) {
  assert(vindex < vertexToTriangle.size());
  const auto& tadj  = vertexToTriangle.neighbours();
  const auto  start = vertexToTriangle.neighboursStartIndex(vindex);
  const auto  end   = vertexToTriangle.neighboursEndIndex(vindex);
  for (int i = start; i < end; ++i) { ++ttags[tadj[i]]; }
}

//----------------------------------------------------------------------------

inline int32_t
ComputeAdjacentVertexCount(
  int32_t                                    vindex,
  const std::vector<Triangle>&               triangles,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<int8_t>&                       vtags) {
  int32_t vcount = 0;
  assert(vindex < vertexToTriangle.size());
  const auto& tadj  = vertexToTriangle.neighbours();
  const auto  start = vertexToTriangle.neighboursStartIndex(vindex);
  const auto  end   = vertexToTriangle.neighboursEndIndex(vindex);
  for (int i = start; i < end; ++i) {
    const auto& triangle = triangles[tadj[i]];
    vtags[triangle[0]]   = int8_t(1);
    vtags[triangle[1]]   = int8_t(1);
    vtags[triangle[2]]   = int8_t(1);
  }
  vtags[vindex] = 0;
  for (int i = start; i < end; ++i) {
    const auto& triangle = triangles[tadj[i]];
    for (int j = 0; j < 3; ++j) {
      const auto index = triangle[j];
      if (vtags[index] != 0) {
        ++vcount;
        vtags[index] = int8_t(0);
      }
    }
  }
  return vcount;
}

//----------------------------------------------------------------------------

inline void
ComputeAdjacentVertices(
  int32_t                                    vindex,
  const std::vector<Triangle>&               triangles,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<int8_t>&                       vtags,
  std::vector<int32_t>&                      vadj) {
  vadj.resize(0);
  assert(vindex < vertexToTriangle.size());
  const auto& tadj  = vertexToTriangle.neighbours();
  const auto  start = vertexToTriangle.neighboursStartIndex(vindex);
  const auto  end   = vertexToTriangle.neighboursEndIndex(vindex);
  for (int i = start; i < end; ++i) {
    const auto& triangle = triangles[tadj[i]];
    vtags[triangle[0]]   = int8_t(1);
    vtags[triangle[1]]   = int8_t(1);
    vtags[triangle[2]]   = int8_t(1);
  }
  vtags[vindex] = 0;
  for (int i = start; i < end; ++i) {
    const auto& triangle = triangles[tadj[i]];
    for (int j = 0; j < 3; ++j) {
      const auto index = triangle[j];
      if (vtags[index] != 0) {
        vadj.push_back(index);
        vtags[index] = int8_t(0);
      }
    }
  }
}

//----------------------------------------------------------------------------

inline void
ComputeAdjacentTriangles(
  const Triangle&                            triangle,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<int8_t>&                       ttags,
  std::vector<int32_t>&                      adj) {
  adj.resize(0);
  TagAdjacentTriangles(triangle[0], 0, vertexToTriangle, ttags);
  TagAdjacentTriangles(triangle[1], 0, vertexToTriangle, ttags);
  TagAdjacentTriangles(triangle[2], 0, vertexToTriangle, ttags);
  const auto& tadj = vertexToTriangle.neighbours();
  for (int32_t k = 0; k < 3; ++k) {
    const auto vindex = triangle[k];
    assert(vindex < vertexToTriangle.size());
    const auto start = vertexToTriangle.neighboursStartIndex(vindex);
    const auto end   = vertexToTriangle.neighboursEndIndex(vindex);
    for (int i = start; i < end; ++i) {
      const auto tindex = tadj[i];
      if (ttags[tindex] == 0) {
        ttags[tindex] = int8_t(1);
        adj.push_back(tindex);
      }
    }
  }
}

//----------------------------------------------------------------------------

inline void
ComputeEdgeAdjacentTriangles(
  int32_t                                    vindex0,
  int32_t                                    vindex1,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<int8_t>&                       ttags,
  std::vector<int32_t>&                      tadj) {
  tadj.resize(0);
  TagAdjacentTriangles(vindex0, 0, vertexToTriangle, ttags);
  TagAdjacentTriangles(vindex1, 1, vertexToTriangle, ttags);
  const auto  start = vertexToTriangle.neighboursStartIndex(vindex0);
  const auto  end   = vertexToTriangle.neighboursEndIndex(vindex0);
  const auto& tadj0 = vertexToTriangle.neighbours();
  for (int i = start; i < end; ++i) {
    const auto tindex = tadj0[i];
    if (ttags[tindex] != 0) { tadj.push_back(tindex); }
  }
}

//----------------------------------------------------------------------------

inline int32_t
ComputeEdgeAdjacentTriangleCount(
  int32_t                                    vindex0,
  int32_t                                    vindex1,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<int8_t>&                       ttags) {
  TagAdjacentTriangles(vindex0, 0, vertexToTriangle, ttags);
  TagAdjacentTriangles(vindex1, 1, vertexToTriangle, ttags);
  const auto  start = vertexToTriangle.neighboursStartIndex(vindex0);
  const auto  end   = vertexToTriangle.neighboursEndIndex(vindex0);
  const auto& tadj0 = vertexToTriangle.neighbours();
  int32_t     count = 0;
  for (int i = start; i < end; ++i) {
    const auto tindex = tadj0[i];
    count += static_cast<int>(ttags[tindex] != 0);
  }
  return count;
}

//----------------------------------------------------------------------------

inline void
ComputeBoundaryVertices(
  const std::vector<Triangle>&               triangles,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<int8_t>&                       vtags,
  std::vector<int8_t>&                       ttags,
  std::vector<int8_t>&                       isBoundaryVertex) {
  const auto vertexCount = int32_t(vertexToTriangle.size());
  if (vertexCount == 0) { return; }
  isBoundaryVertex.resize(vertexCount);
  std::fill(isBoundaryVertex.begin(), isBoundaryVertex.end(), int8_t(0));
  std::vector<int32_t> vadj;
  for (int32_t vindex0 = 0; vindex0 < vertexCount; ++vindex0) {
    ComputeAdjacentVertices(vindex0, triangles, vertexToTriangle, vtags, vadj);
    for (int vindex1 : vadj) {
      if (vindex1 <= vindex0) { continue; }
      if (ComputeEdgeAdjacentTriangleCount(
            vindex0, vindex1, vertexToTriangle, ttags)
          == 1) {
        isBoundaryVertex[vindex0] = int8_t(1);
        isBoundaryVertex[vindex1] = int8_t(1);
      }
    }
  }
}

//----------------------------------------------------------------------------

inline int64_t
EdgeIndex(int32_t i0, int32_t i1) {
  return (int64_t(std::min(i0, i1)) << 32) + std::max(i0, i1);
}

template<typename T1, typename T2>
void
SubdivideMidPoint(std::vector<T1>&                     points,
                  std::vector<Triangle>&               triangles,
                  StaticAdjacencyInformation<int32_t>& vertexToTriangle,
                  StaticAdjacencyInformation<int32_t>& vertexToEdge,
                  std::vector<int8_t>&                 vtags,
                  std::vector<int64_t>&                edges,
                  std::vector<int32_t>* triangleToBaseMeshTriangle) {
  const auto vertexCount   = int32_t(points.size());
  const auto triangleCount = int32_t(triangles.size());
  ComputeVertexToTriangle(triangles, vertexCount, vertexToTriangle);

  vtags.resize(vertexCount);
  vertexToEdge.resize(vertexCount);
  int32_t edgeCount = 0;
  for (int32_t vindex0 = 0; vindex0 < vertexCount; ++vindex0) {
    const auto ncount =
      ComputeAdjacentVertexCount(vindex0, triangles, vertexToTriangle, vtags);
    vertexToEdge.incrementNeighbourCount(vindex0, ncount);
    edgeCount += ncount;
  }
  assert(!(edgeCount & 1));
  edgeCount /= 2;
  vertexToEdge.updateShift();
  // create vertices
  points.resize(vertexCount + edgeCount);
  edges.resize(vertexCount + edgeCount);

  std::vector<int32_t> vadj;
  int32_t              vertexCounter = vertexCount;
  const auto           r             = T2(1) / T2(2);
  for (int32_t vindex0 = 0; vindex0 < vertexCount; ++vindex0) {
    ComputeAdjacentVertices(vindex0, triangles, vertexToTriangle, vtags, vadj);
    for (const auto vindex1 : vadj) {
      if (vindex1 <= vindex0) { continue; }
      vertexToEdge.addNeighbour(vindex0, vertexCounter);
      vertexToEdge.addNeighbour(vindex1, vertexCounter);
      edges[vertexCounter]  = (int64_t(vindex0) << 32) + vindex1;
      points[vertexCounter] = (points[vindex0] + points[vindex1]) * r;
      ++vertexCounter;
    }
  }

  // create triangles
  triangles.resize(4 * triangleCount);
  if (triangleToBaseMeshTriangle) {
    triangleToBaseMeshTriangle->resize(4 * triangleCount);
  }
  int32_t triangleCounter = triangleCount;
  for (int32_t t = 0; t < triangleCount; ++t) {
    const auto tri = triangles[t];
    Triangle   tri0;
    for (int32_t k = 0; k < 3; ++k) {
      const auto  vindex0          = tri[k];
      const auto  vindex1          = tri[(k + 1) % 3];
      const auto  currentEdgeIndex = EdgeIndex(vindex0, vindex1);
      const auto& edgeIndexes      = vertexToEdge.neighbours();
      const auto  start = vertexToEdge.neighboursStartIndex(vindex0);
      const auto  end   = vertexToEdge.neighboursEndIndex(vindex0);
      for (int32_t n = start; n < end; ++n) {
        const auto eindex = edgeIndexes[n];
        if (edges[eindex] == currentEdgeIndex) {
          tri0[k] = eindex;
          break;
        }
      }
    }
    triangles[t] = tri0;
    if (triangleToBaseMeshTriangle) {
      const auto tindexBaseMesh = (*triangleToBaseMeshTriangle)[t];
      (*triangleToBaseMeshTriangle)[triangleCounter]     = tindexBaseMesh;
      (*triangleToBaseMeshTriangle)[triangleCounter + 1] = tindexBaseMesh;
      (*triangleToBaseMeshTriangle)[triangleCounter + 2] = tindexBaseMesh;
    }
    triangles[triangleCounter++] = Triangle(tri[0], tri0[0], tri0[2]);
    triangles[triangleCounter++] = Triangle(tri0[0], tri[1], tri0[1]);
    triangles[triangleCounter++] = Triangle(tri0[2], tri0[1], tri[2]);
  }
}

//============================================================================

template<typename T>
bool
RemoveDuplicatedTriangles(
  const std::vector<Triangle>&               trianglesInput,
  const std::vector<Vec3<T>>&                triangleNormals,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  std::vector<Triangle>&                     trianglesOutput,
  const bool removeDegeneratedTriangles = true) {
  struct TriangleInfo {
    double  score;
    int32_t outIdx;
  };

  // Output index for the next unique triangle
  int32_t nextOutIdx = 0;
  trianglesOutput.resize(trianglesInput.size());

  const auto triangleCount = int32_t(trianglesInput.size());
  std::map<Triangle, TriangleInfo> uniqueTriangles;
  const auto*                      neighbours = vertexToTriangle.neighbours();
  for (int32_t t = 0; t < triangleCount; ++t) {
    const auto tri   = orderComponents(trianglesInput[t]);
    double     score = 0.0;
    for (int32_t k = 0; k < 3; ++k) {
      const auto vindex = tri[k];
      const auto start  = vertexToTriangle.neighboursStartIndex(vindex);
      const auto end    = vertexToTriangle.neighboursEndIndex(vindex);
      for (int32_t n = start; n < end; ++n) {
        score += triangleNormals[t] * triangleNormals[neighbours[n]];
      }
    }

    auto it = uniqueTriangles.find(tri);
    if (it == uniqueTriangles.end()) {
      if (!removeDegeneratedTriangles || !isDegenerate(tri)) {
        trianglesOutput[nextOutIdx] = trianglesInput[t];
        uniqueTriangles[tri]        = {score, nextOutIdx++};
      }
    } else if (score > it->second.score) {
      it->second.score                   = score;
      trianglesOutput[it->second.outIdx] = trianglesInput[t];
    }
  }

  trianglesOutput.resize(uniqueTriangles.size());
  return true;
}

//----------------------------------------------------------------------------

template<typename T>
void
RemoveDegeneratedTriangles(TriangleMesh<T>& mesh) {
  const auto triangleCount = mesh.triangleCount();
  if (triangleCount <= 0) { return; }

  const auto hasTexCoords = mesh.texCoordTriangleCount() == triangleCount;
  const auto hasNormals   = mesh.normalTriangleCount() == triangleCount;
  std::vector<Triangle> triangles;
  std::vector<Triangle> texCoordTriangles;
  std::vector<Triangle> normalTriangles;
  triangles.reserve(triangleCount);

  if (hasTexCoords) { texCoordTriangles.reserve(triangleCount); }

  if (hasNormals) { normalTriangles.reserve(triangleCount); }

  for (int32_t tindex = 0; tindex < triangleCount; tindex++) {
    const auto& tri = mesh.triangle(tindex);
    if (tri[0] != tri[1] && tri[0] != tri[2] && tri[1] != tri[2]) {
      triangles.push_back(tri);
      if (hasTexCoords) {
        texCoordTriangles.push_back(mesh.texCoordTriangle(tindex));
      }
      if (hasNormals) {
        normalTriangles.push_back(mesh.normalTriangle(tindex));
      }
    }
  }

  std::swap(mesh.triangles(), triangles);
  if (hasTexCoords) { std::swap(mesh.texCoordTriangles(), texCoordTriangles); }

  if (hasNormals) { std::swap(mesh.normalTriangles(), normalTriangles); }
}

//----------------------------------------------------------------------------

template<typename T>
int32_t
UnifyVertices(const std::vector<Vec3<T>>&  pointsInput,
              const std::vector<Triangle>& trianglesInput,
              std::vector<Vec3<T>>&        pointsOutput,
              std::vector<Triangle>&       trianglesOutput,
              std::vector<int32_t>&        mapping) {
  const auto pointCount0    = int32_t(pointsInput.size());
  const auto triangleCount0 = int32_t(trianglesInput.size());
  pointsOutput.resize(0);
  pointsOutput.reserve(pointsInput.size());
  trianglesOutput.resize(0);
  trianglesOutput.reserve(trianglesInput.size());
  std::map<Vec3<T>, int32_t> uniquePoints;
  mapping.resize(pointCount0);

  int32_t pointCounter = 0;
  for (int32_t vindex = 0; vindex < pointCount0; ++vindex) {
    const auto& pt = pointsInput[vindex];
    const auto  it = uniquePoints.find(pt);
    if (it == uniquePoints.end()) {
      pointsOutput.push_back(pt);
      uniquePoints[pt] = pointCounter;
      mapping[vindex]  = pointCounter;
      ++pointCounter;
    } else {
      mapping[vindex] = it->second;
    }
  }
  assert(pointCounter <= pointCount0);

  trianglesOutput.resize(triangleCount0);
  for (int32_t tindex = 0; tindex < triangleCount0; ++tindex) {
    const auto& tri0 = trianglesInput[tindex];
    auto&       tri1 = trianglesOutput[tindex];
    for (int32_t k = 0; k < 3; ++k) {
      tri1[k] = mapping[tri0[k]];
      assert(tri1[k] >= 0 && tri1[k] < pointCounter);
    }
  }

  return pointCounter;
}

//----------------------------------------------------------------------------

template<typename T>
int32_t
ExtractConnectedComponents(const std::vector<Triangle>& triangles,
                           const int32_t                vertexCount,
                           const TriangleMesh<T>&       mesh,
                           std::vector<int32_t>&        partition,
                           std::vector<std::shared_ptr<TriangleMesh<T>>>*
                             connectedComponents = nullptr) {
  if (connectedComponents) { connectedComponents->clear(); }

  if (triangles.empty()) { return 0; }

  const auto pointCount    = mesh.pointCount();
  const auto texCoordCount = mesh.texCoordCount();
  const auto normalCount   = mesh.normalCount();
  const auto triangleCount = int32_t(triangles.size());
  assert(mesh.triangleCount() == 0 || mesh.triangleCount() == triangleCount);
  assert(mesh.texCoordTriangleCount() == 0
         || mesh.texCoordTriangleCount() == triangleCount);
  assert(mesh.normalTriangleCount() == 0
         || mesh.normalTriangleCount() == triangleCount);
  partition.resize(0);
  partition.resize(triangleCount, -1);

  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(triangles, vertexCount, vertexToTriangle);

  std::vector<int32_t> posMapping;
  std::vector<int32_t> texCoordMapping;
  std::vector<int32_t> normalMapping;
  std::vector<int32_t> triangleList;
  if (connectedComponents) {
    if (mesh.triangleCount()) { posMapping.resize(pointCount, -1); }
    if (mesh.texCoordTriangleCount()) {
      texCoordMapping.resize(texCoordCount, -1);
    }
    if (mesh.normalTriangleCount()) { normalMapping.resize(normalCount, -1); }
  }

  const auto*          neighbours = vertexToTriangle.neighbours();
  std::vector<int32_t> lifo;
  lifo.reserve(triangleCount);
  int32_t ccCount = 0;
  for (int32_t triangleIndex = 0; triangleIndex < triangleCount;
       ++triangleIndex) {
    if (partition[triangleIndex] == -1) {
      const auto ccIndex       = ccCount++;
      partition[triangleIndex] = ccIndex;
      lifo.push_back(triangleIndex);

      std::shared_ptr<TriangleMesh<T>> ccMesh;
      if (connectedComponents) {
        ccMesh.reset(new TriangleMesh<T>);
        connectedComponents->push_back(ccMesh);
        ccMesh->setMaterialLibrary(mesh.materialLibrary());
        triangleList.resize(0);
      }

      int32_t ccPosCount      = 0;
      int32_t ccTexCoordCount = 0;
      int32_t ccNormalCount   = 0;
      while (!lifo.empty()) {
        const auto tIndex = lifo.back();
        lifo.pop_back();

        if (connectedComponents) {
          triangleList.push_back(tIndex);

          if (mesh.triangleCount()) {
            const auto& tri = mesh.triangle(tIndex);
            for (int32_t k = 0; k < 3; ++k) {
              const auto v = tri[k];
              if (posMapping[v] == -1) {
                posMapping[v] = ccPosCount++;
                ccMesh->addPoint(mesh.point(v));
              }
            }
            ccMesh->addTriangle(
              posMapping[tri[0]], posMapping[tri[1]], posMapping[tri[2]]);
          }

          if (mesh.texCoordTriangleCount()) {
            const auto& texCoordTri = mesh.texCoordTriangle(tIndex);
            for (int32_t k = 0; k < 3; ++k) {
              const auto v = texCoordTri[k];
              if (texCoordMapping[v] == -1) {
                texCoordMapping[v] = ccTexCoordCount++;
                ccMesh->addTexCoord(mesh.texCoord(v));
              }
            }
            ccMesh->addTexCoordTriangle(texCoordMapping[texCoordTri[0]],
                                        texCoordMapping[texCoordTri[1]],
                                        texCoordMapping[texCoordTri[2]]);
          }

          if (mesh.normalTriangleCount()) {
            const auto& normalTri = mesh.normalTriangle(tIndex);
            for (int32_t k = 0; k < 3; ++k) {
              const auto v = normalTri[k];
              if (normalMapping[v] == -1) {
                normalMapping[v] = ccNormalCount++;
                ccMesh->addNormal(mesh.normal(v));
              }
            }

            ccMesh->addNormalTriangle(normalMapping[normalTri[0]],
                                      normalMapping[normalTri[1]],
                                      normalMapping[normalTri[2]]);
          }
        }

        const auto& tri = triangles[tIndex];
        for (int32_t k = 0; k < 3; ++k) {
          const auto v     = tri[k];
          const auto start = vertexToTriangle.neighboursStartIndex(v);
          const auto end   = vertexToTriangle.neighboursEndIndex(v);
          for (int32_t n = start; n < end; ++n) {
            const auto nIndex = neighbours[n];
            if (partition[nIndex] == -1) {
              partition[nIndex] = ccIndex;
              lifo.push_back(nIndex);
            }
          }
        }
      }

      if (connectedComponents) {
        for (const auto tIndex : triangleList) {
          if (mesh.triangleCount()) {
            const auto& tri = mesh.triangle(tIndex);
            for (int32_t k = 0; k < 3; ++k) { posMapping[tri[k]] = -1; }
          }

          if (mesh.texCoordTriangleCount()) {
            const auto& texCoordTri = mesh.texCoordTriangle(tIndex);
            for (int32_t k = 0; k < 3; ++k) {
              texCoordMapping[texCoordTri[k]] = -1;
            }
          }

          if (mesh.normalTriangleCount()) {
            const auto& normalTri = mesh.normalTriangle(tIndex);
            for (int32_t k = 0; k < 3; ++k) {
              normalMapping[normalTri[k]] = -1;
            }
          }
        }
      }
    }
  }

  return ccCount;
}

//----------------------------------------------------------------------------

template<typename T>
void
SmoothWithVertexConstraints(
  TriangleMesh<T>&                           mesh,
  const StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  const std::vector<int8_t>&                 isBoundaryVertex,
  std::vector<int8_t>&                       vtags,
  std::vector<int8_t>&                       ttags,
  const T                                    smoothingCoefficient,
  const T                                    maxError          = 0.001,
  const int32_t                              maxIterationCount = -1) {
  const auto pointCount    = mesh.pointCount();
  const auto triangleCount = mesh.triangleCount();
  if (!pointCount || !triangleCount || vertexToTriangle.size() != pointCount
      || vtags.size() != size_t(pointCount)
      || ttags.size() != size_t(triangleCount)) {
    return;
  }

  std::vector<int32_t> vadj;
  std::vector<int32_t> tadj;
  const auto           n = 2 * pointCount;
  SparseMatrix<T>      A(n, pointCount);

  for (int vindex0 = 0; vindex0 < pointCount; ++vindex0) {
    ComputeAdjacentVertices(
      vindex0, mesh.triangles(), vertexToTriangle, vtags, vadj);
    const auto neighbourCount = int32_t(vadj.size());

    if (isBoundaryVertex[vindex0] && neighbourCount) {
      int32_t boundaryNeighbourCount = 0;
      int32_t bneighbours[3];
      for (int i = 0; i < neighbourCount && boundaryNeighbourCount < 3; ++i) {
        const auto vindex1 = vadj[i];
        if (isBoundaryVertex[vindex1]) {
          bneighbours[boundaryNeighbourCount++] = vindex1;
        }
      }
      if (boundaryNeighbourCount == 2) {
        A.addElementInOrder(vindex0, bneighbours[0], -smoothingCoefficient);
        A.addElementInOrder(vindex0, bneighbours[1], -smoothingCoefficient);
        A.addElementInOrder(vindex0, vindex0, 2 * smoothingCoefficient);
      } else {
        A.addElementInOrder(vindex0, vindex0, T(0));
      }
    } else {
      for (int i = 0; i < neighbourCount; ++i) {
        A.addElementInOrder(vindex0, vadj[i], -smoothingCoefficient);
      }
      A.addElementInOrder(
        vindex0, vindex0, neighbourCount * smoothingCoefficient);
    }
  }
  for (int vindex0 = 0; vindex0 < pointCount; ++vindex0) {
    A.addElementInOrder(
      pointCount + vindex0, vindex0, /*weights ? weights[vindex0] :*/ T(1));
  }

  A.updatePointers();
  const auto At = A.transpose();
  const auto itCount =
    maxIterationCount == -1 ? pointCount * 2 : maxIterationCount;
  VecN<double> b(n);
  VecN<double> x(pointCount);
  VecN<double> y(n);
  VecN<double> p(pointCount);
  VecN<double> r(pointCount);
  VecN<double> q(pointCount);
  b = T(0);
  for (int32_t dim = 0; dim < 3; ++dim) {
    x = T(0);
    for (int32_t i = 0; i < pointCount; ++i) {
      b[pointCount + i] =
        /*weights ? weights[i] * mesh.point(i)[dim] :*/ mesh.point(i)[dim];
    }
    y     = b - A * x;
    r     = At * y;
    p     = r;
    T rtr = T(0);
    for (int32_t i = 0; i < pointCount; ++i) { rtr += r[i] * r[i]; }
    T       error = std::sqrt(rtr / pointCount);
    int32_t it    = 0;
    for (; it < itCount && error > maxError; ++it) {
      y       = A * p;
      q       = At * y;
      T pAtAp = T(0);
      for (int32_t i = 0; i < pointCount; ++i) { pAtAp += p[i] * q[i]; }
      const auto alpha = rtr / pAtAp;
      for (int32_t i = 0; i < pointCount; ++i) {
        x[i] += alpha * p[i];
        r[i] -= alpha * q[i];
      }
      T r1tr1 = T(0);
      for (int32_t i = 0; i < pointCount; ++i) { r1tr1 += r[i] * r[i]; }
      const T betha = r1tr1 / rtr;
      for (int32_t i = 0; i < pointCount; ++i) { p[i] = r[i] + betha * p[i]; }
      rtr   = r1tr1;
      error = std::sqrt(rtr / pointCount);
    }
    for (int32_t i = 0; i < pointCount; ++i) {
      Vec3<T>& point = mesh.point(i);
      point[dim]     = x[i];
    }
  }
}

//----------------------------------------------------------------------------

template<typename T>
void
SmoothWithVertexConstraints(TriangleMesh<T>& mesh,
                            const T          smoothingCoefficient,
                            const T          maxError          = 0.001,
                            const int32_t    maxIterationCount = -1) {
  const auto pointCount    = mesh.pointCount();
  const auto triangleCount = mesh.triangleCount();
  if (!pointCount || !triangleCount) { return; }
  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  ComputeVertexToTriangle(
    mesh.triangles(), mesh.pointCount(), vertexToTriangle);

  const auto&         triangles = mesh.triangles();
  std::vector<int8_t> vtags(pointCount);
  std::vector<int8_t> ttags(triangleCount);
  std::vector<int8_t> isBoundaryVertex;
  ComputeBoundaryVertices(
    triangles, vertexToTriangle, vtags, ttags, isBoundaryVertex);

  std::vector<int32_t> vadj;
  std::vector<int32_t> tadj;
  const auto           n = 2 * pointCount;
  SparseMatrix<T>      A(n, pointCount);

  for (int vindex0 = 0; vindex0 < pointCount; ++vindex0) {
    ComputeAdjacentVertices(vindex0, triangles, vertexToTriangle, vtags, vadj);
    const auto neighbourCount = int32_t(vadj.size());

    if (isBoundaryVertex[vindex0] && neighbourCount) {
      int32_t boundaryNeighbourCount = 0;
      int32_t bneighbours[3];
      for (int i = 0; i < neighbourCount && boundaryNeighbourCount < 3; ++i) {
        const auto vindex1 = vadj[i];
        if (isBoundaryVertex[vindex1]) {
          bneighbours[boundaryNeighbourCount++] = vindex1;
        }
      }
      if (boundaryNeighbourCount == 2) {
        A.addElementInOrder(vindex0, bneighbours[0], -smoothingCoefficient);
        A.addElementInOrder(vindex0, bneighbours[1], -smoothingCoefficient);
        A.addElementInOrder(vindex0, vindex0, 2 * smoothingCoefficient);
      } else {
        A.addElementInOrder(vindex0, vindex0, T(0));
      }
    } else {
      for (int i = 0; i < neighbourCount; ++i) {
        A.addElementInOrder(vindex0, vadj[i], -smoothingCoefficient);
      }
      A.addElementInOrder(
        vindex0, vindex0, neighbourCount * smoothingCoefficient);
    }
  }
  for (int vindex0 = 0; vindex0 < pointCount; ++vindex0) {
    A.addElementInOrder(pointCount + vindex0, vindex0, T(1));
  }

  A.updatePointers();
  const auto At = A.transpose();
  const auto itCount =
    maxIterationCount == -1 ? pointCount * 2 : maxIterationCount;
  VecN<double> b(n);
  VecN<double> x(pointCount);
  VecN<double> y(n);
  VecN<double> p(pointCount);
  VecN<double> r(pointCount);
  VecN<double> q(pointCount);
  b = T(0);
  for (int32_t dim = 0; dim < 3; ++dim) {
    x = T(0);
    for (int32_t i = 0; i < pointCount; ++i) {
      b[pointCount + i] = mesh.point(i)[dim];
    }
    y     = b - A * x;
    r     = At * y;
    p     = r;
    T rtr = T(0);
    for (int32_t i = 0; i < pointCount; ++i) { rtr += r[i] * r[i]; }
    T       error = std::sqrt(rtr / pointCount);
    int32_t it    = 0;
    for (; it < itCount && error > maxError; ++it) {
      y       = A * p;
      q       = At * y;
      T pAtAp = T(0);
      for (int32_t i = 0; i < pointCount; ++i) { pAtAp += p[i] * q[i]; }
      const auto alpha = rtr / pAtAp;
      for (int32_t i = 0; i < pointCount; ++i) {
        x[i] += alpha * p[i];
        r[i] -= alpha * q[i];
      }
      T r1tr1 = T(0);
      for (int32_t i = 0; i < pointCount; ++i) { r1tr1 += r[i] * r[i]; }
      const T betha = r1tr1 / rtr;
      for (int32_t i = 0; i < pointCount; ++i) { p[i] = r[i] + betha * p[i]; }
      rtr   = r1tr1;
      error = std::sqrt(rtr / pointCount);
    }
    for (int32_t i = 0; i < pointCount; ++i) {
      Vec3<T>& point = mesh.point(i);
      point[dim]     = x[i];
    }
  }
}

//----------------------------------------------------------------------------

template<typename T>
int32_t
ComputeMidPointSubdivisionMatrix(
  const int32_t                        vertexCount,
  std::vector<Triangle>&               triangles,
  StaticAdjacencyInformation<int32_t>& vertexToTriangle,
  StaticAdjacencyInformation<int32_t>& vertexToEdge,
  std::vector<int8_t>&                 vtags,
  SparseMatrix<T>&                     M) {
  struct Edge {
    int64_t eindex;
    int32_t vindex;
  };

  const auto triangleCount = int32_t(triangles.size());
  ComputeVertexToTriangle(triangles, vertexCount, vertexToTriangle);

  vtags.resize(vertexCount);
  vertexToEdge.resize(vertexCount);
  int32_t edgeCount = 0;
  for (int32_t vindex0 = 0; vindex0 < vertexCount; ++vindex0) {
    const auto ncount =
      ComputeAdjacentVertexCount(vindex0, triangles, vertexToTriangle, vtags);
    vertexToEdge.incrementNeighbourCount(vindex0, ncount);
    edgeCount += ncount;
  }
  assert(!(edgeCount & 1));
  edgeCount /= 2;
  vertexToEdge.updateShift();
  const auto outputVertexCount = vertexCount + edgeCount;
  M.initialize(outputVertexCount, vertexCount);

  for (int32_t vindex0 = 0; vindex0 < vertexCount; ++vindex0) {
    M.addElementInOrder(vindex0, vindex0, T(1));
  }

  std::vector<Edge>    edges(edgeCount);
  std::vector<int32_t> vadj;
  int32_t              edgeCounter   = 0;
  int32_t              vertexCounter = vertexCount;
  const auto           r2            = T(1) / 2;
  for (int32_t vindex0 = 0; vindex0 < vertexCount; ++vindex0) {
    ComputeAdjacentVertices(vindex0, triangles, vertexToTriangle, vtags, vadj);
    for (const auto vindex1 : vadj) {
      if (vindex1 <= vindex0) { continue; }
      vertexToEdge.addNeighbour(vindex0, edgeCounter);
      vertexToEdge.addNeighbour(vindex1, edgeCounter);

      Edge& edge  = edges[edgeCounter++];
      edge.vindex = vertexCounter;
      edge.eindex = (int64_t(vindex0) << 32) + vindex1;

      M.addElementInOrder(vertexCounter, vindex0, r2);
      M.addElementInOrder(vertexCounter, vindex1, r2);
      ++vertexCounter;
    }
  }

  // create triangles
  triangles.resize(4 * triangleCount);
  int32_t triangleCounter = triangleCount;
  for (int32_t t = 0; t < triangleCount; ++t) {
    const auto tri = triangles[t];
    Triangle   tri0;
    for (int32_t k = 0; k < 3; ++k) {
      const auto  vindex0          = tri[k];
      const auto  vindex1          = tri[(k + 1) % 3];
      const auto  currentEdgeIndex = EdgeIndex(vindex0, vindex1);
      const auto& edgeIndexes      = vertexToEdge.neighbours();
      const auto  start = vertexToEdge.neighboursStartIndex(vindex0);
      const auto  end   = vertexToEdge.neighboursEndIndex(vindex0);
      for (int32_t n = start; n < end; ++n) {
        const auto eindex = edgeIndexes[n];
        if (edges[eindex].eindex == currentEdgeIndex) {
          tri0[k] = edges[eindex].vindex;
          break;
        }
      }
    }
    triangles[t]                 = tri0;
    triangles[triangleCounter++] = Triangle(tri[0], tri0[0], tri0[2]);
    triangles[triangleCounter++] = Triangle(tri0[0], tri[1], tri0[1]);
    triangles[triangleCounter++] = Triangle(tri0[2], tri0[1], tri[2]);
  }
  M.updatePointers();
  assert(outputVertexCount == vertexCounter);
  return vertexCounter;
}

//----------------------------------------------------------------------------

template<typename T>
void
FitMidPointSubdivision(const TriangleMesh<T>& referenceMesh,
                       TriangleMesh<T>&       baseMesh,
                       const int32_t          subdivIterationCount = 1,
                       const T                maxError             = 0.0001,
                       const int32_t          maxIterationCount    = -1) {
  const auto vCount =
    baseMesh.pointCount() * std::pow(4, subdivIterationCount - 1);
  const auto tCount =
    baseMesh.triangleCount() * std::pow(4, subdivIterationCount - 1);
  std::vector<Triangle> triangles;
  triangles.reserve(4 * tCount);
  triangles.resize(baseMesh.triangleCount());
  std::copy(baseMesh.triangles().begin(),
            baseMesh.triangles().end(),
            triangles.begin());

  StaticAdjacencyInformation<int32_t> vertexToTriangle;
  StaticAdjacencyInformation<int32_t> vertexToEdge;
  std::vector<int8_t>                 vtags;

  vertexToTriangle.reserve(vCount);
  vertexToEdge.reserve(vCount);
  vtags.reserve(vCount);

  std::vector<SparseMatrix<T>> M(subdivIterationCount);
  std::vector<SparseMatrix<T>> Mt(subdivIterationCount);
  int32_t                      vertexCount1 = baseMesh.pointCount();
  for (int32_t s = 0; s < subdivIterationCount; ++s) {
    vertexCount1 = ComputeMidPointSubdivisionMatrix(
      vertexCount1, triangles, vertexToTriangle, vertexToEdge, vtags, M[s]);
    Mt[s] = M[s].transpose();
  }

  const auto vertexCount0 = baseMesh.pointCount();
  const auto n            = M[subdivIterationCount - 1].rowCount();
  const auto itCount =
    maxIterationCount == -1 ? 2 * vertexCount0 : maxIterationCount;
  VecN<T> b(n) /*, q(vertexCount0), r(vertexCount0)*/;

  std::vector<VecN<T>> R(subdivIterationCount + 1);
  std::vector<VecN<T>> P(subdivIterationCount + 1);
  std::vector<VecN<T>> Q(subdivIterationCount + 1);
  std::vector<VecN<T>> X(subdivIterationCount + 1);
  auto&                x = X[0];
  x.resize(vertexCount0);

  b = T(0);
  for (int32_t dim = 0; dim < 3; ++dim) {
    for (int32_t v = 0; v < vertexCount0; ++v) {
      x[v] = baseMesh.point(v)[dim];
    }
    for (int32_t v = 0; v < n; ++v) { b[v] = referenceMesh.point(v)[dim]; }
    for (int32_t s = 0; s < subdivIterationCount; ++s) {
      X[s + 1] = M[s] * X[s];
    }
    R[subdivIterationCount] = b - X[subdivIterationCount];  // y = b - A x
    for (int32_t s = subdivIterationCount - 1; s >= 0; --s) {
      R[s] = Mt[s] * R[s + 1];
    }
    auto& r = R[0];  // r = At * y
    auto& p = P[0];
    p       = r;
    T rtr   = T(0);
    for (int32_t i = 0; i < vertexCount0; ++i) { rtr += r[i] * r[i]; }
    T error = std::sqrt(rtr / vertexCount0);
    //    std::cout << "0 -> " << error << '\n';

    int32_t it = 0;
    for (; it < itCount && error > maxError; ++it) {
      for (int32_t s = 0; s < subdivIterationCount; ++s) {
        P[s + 1] = M[s] * P[s];
      }

      Q[subdivIterationCount] = P[subdivIterationCount];  // y = A p
      for (int32_t s = subdivIterationCount - 1; s >= 0; --s) {
        Q[s] = Mt[s] * Q[s + 1];
      }
      auto& q = Q[0];  // q = At y

      T pAtAp = T(0);
      for (int32_t i = 0; i < vertexCount0; ++i) { pAtAp += p[i] * q[i]; }
      const auto alpha = rtr / pAtAp;
      for (int32_t i = 0; i < vertexCount0; ++i) {
        x[i] += alpha * p[i];
        r[i] -= alpha * q[i];
      }
      T r1tr1 = T(0);
      for (int32_t i = 0; i < vertexCount0; ++i) { r1tr1 += r[i] * r[i]; }
      const T betha = r1tr1 / rtr;
      for (int32_t i = 0; i < vertexCount0; ++i) {
        p[i] = r[i] + betha * p[i];
      }
      rtr   = r1tr1;
      error = std::sqrt(rtr / vertexCount0);
      //      std::cout << it << " -> "<< error << '\n';
    }
    for (int32_t i = 0; i < vertexCount0; ++i) {
      Vec3<T>& point = baseMesh.point(i);
      point[dim]     = x[i];
    }
  }
}
template<class T1, class T2>
void
computeForwardLinearLifting(
  std::vector<T1>&                                signal,
  const std::vector<vmesh::SubdivisionLevelInfo>& infoLevelOfDetails,
  const std::vector<int64_t>&                     edges,
  const T2                                        predWeight,
  const T2                                        updateWeight,
  const bool                                      skipUpdate) {
  const auto lodCount = int32_t(infoLevelOfDetails.size());
  assert(lodCount > 0);
  const auto rfmtCount = lodCount - 1;
  for (int32_t it = rfmtCount - 1; it >= 0; --it) {
    const auto vcount0 = infoLevelOfDetails[it].pointCount;
    const auto vcount1 = infoLevelOfDetails[it + 1].pointCount;
    assert(vcount0 < vcount1 && vcount1 <= int32_t(signal.size()));
    // predict
    for (int32_t v = vcount0; v < vcount1; ++v) {
      const auto edge = edges[v];
      const auto v1   = int32_t(edge & 0xFFFFFFFF);
      const auto v2   = int32_t((edge >> 32) & 0xFFFFFFFF);
      assert(v1 >= 0 && v1 <= vcount0);
      assert(v2 >= 0 && v2 <= vcount0);
      signal[v] -= predWeight * (signal[v1] + signal[v2]);
    }
    // update
    for (int32_t v = vcount0; !skipUpdate && v < vcount1; ++v) {
      const auto edge = edges[v];
      const auto v1   = int32_t(edge & 0xFFFFFFFF);
      const auto v2   = int32_t((edge >> 32) & 0xFFFFFFFF);
      assert(v1 >= 0 && v1 <= vcount0);
      assert(v2 >= 0 && v2 <= vcount0);
      const auto d = updateWeight * signal[v];
      signal[v1] += d;
      signal[v2] += d;
    }
  }
}

//----------------------------------------------------------------------------

template<class T1, class T2>
void
computeInverseLinearLifting(
  std::vector<T1>&                                signal,
  const std::vector<vmesh::SubdivisionLevelInfo>& infoLevelOfDetails,
  const std::vector<int64_t>&                     edges,
  const T2                                        predWeight,
  const T2                                        updateWeight,
  const bool                                      skipUpdate) {
  const auto lodCount = int32_t(infoLevelOfDetails.size());
  assert(lodCount > 0);
  const auto rfmtCount = lodCount - 1;
  for (int32_t it = 0; it < rfmtCount; ++it) {
    const auto vcount0 = infoLevelOfDetails[it].pointCount;
    const auto vcount1 = infoLevelOfDetails[it + 1].pointCount;
    assert(vcount0 < vcount1 && vcount1 <= int32_t(signal.size()));

    // update
    for (int32_t v = vcount0; !skipUpdate && v < vcount1; ++v) {
      const auto edge = edges[v];
      const auto v1   = int32_t(edge & 0xFFFFFFFF);
      const auto v2   = int32_t((edge >> 32) & 0xFFFFFFFF);
      assert(v1 >= 0 && v1 <= vcount0);
      assert(v2 >= 0 && v2 <= vcount0);
      const auto d = updateWeight * signal[v];
      signal[v1] -= d;
      signal[v2] -= d;
    }

    // predict
    for (int32_t v = vcount0; v < vcount1; ++v) {
      const auto edge = edges[v];
      const auto v1   = int32_t(edge & 0xFFFFFFFF);
      const auto v2   = int32_t((edge >> 32) & 0xFFFFFFFF);
      assert(v1 >= 0 && v1 <= vcount0);
      assert(v2 >= 0 && v2 <= vcount0);
      signal[v] += predWeight * (signal[v1] + signal[v2]);
    }
  }
}

//----------------------------------------------------------------------------

template<class T1, class T2>
void
interpolateSubdivision(
  std::vector<T1>&                                signal,
  const std::vector<vmesh::SubdivisionLevelInfo>& infoLevelOfDetails,
  const std::vector<int64_t>&                     edges,
  const T2                                        weight1,
  const T2                                        weight2,
  const bool                                      normalize = false) {
  const auto lodCount = int32_t(infoLevelOfDetails.size());
  assert(lodCount >= 0);
  const auto rfmtCount = lodCount - 1;
  for (int32_t it = 0; it < rfmtCount; ++it) {
    const auto vcount0 = infoLevelOfDetails[it].pointCount;
    const auto vcount1 = infoLevelOfDetails[it + 1].pointCount;
    assert(vcount0 < vcount1 && vcount1 <= int32_t(signal.size()));
    for (int32_t v = vcount0; v < vcount1; ++v) {
      const auto edge = edges[v];
      const auto v1   = int32_t(edge & 0xFFFFFFFF);
      const auto v2   = int32_t((edge >> 32) & 0xFFFFFFFF);
      assert(v1 >= 0 && v1 <= vcount0);
      assert(v2 >= 0 && v2 <= vcount0);
      auto& vec = signal[v];
      vec       = weight1 * signal[v1] + weight2 * signal[v2];
      if (normalize) { vec.normalize(); }
    }
  }
}

//============================================================================

}  // namespace vmesh
