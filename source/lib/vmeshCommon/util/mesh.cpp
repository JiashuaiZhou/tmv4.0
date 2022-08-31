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

#include "util/mesh.hpp"

namespace vmesh {

//============================================================================

//============================================================================

template<typename T>
void
convertFromPlyData(std::shared_ptr<tinyply::PlyData>& src,
                   std::vector<T>&                    dst) {
#define CAST_UINT8(ptr) \
  reinterpret_cast<uint8_t*>(reinterpret_cast<void*>(ptr.data()))
  if (src) {
    dst.resize(src->count);
    std::memcpy(CAST_UINT8(dst), src->buffer.get(), src->buffer.size_bytes());
  }
}

//============================================================================

//============================================================================

// template<typename T>
// bool
// TriangleMesh<T>::load(const std::string& fileName, int f) {
//   std::string name = expandNum(fileName, f);
//   return load(name);
// }

inline bool
isObj(std::string fileName) {
  if (fileName.size() < 5) return false;
  std::string ext = fileName.substr(fileName.size() - 3, 3);
  std::for_each(ext.begin(), ext.end(), [](char& c) { c = ::tolower(c); });
  return ext == "obj";
}

inline bool
isPly(std::string fileName) {
  if (fileName.size() < 5) return false;
  std::string ext = fileName.substr(fileName.size() - 3, 3);
  std::for_each(ext.begin(), ext.end(), [](char& c) { c = ::tolower(c); });
  return ext == "ply";
}

template<typename T>
bool
TriangleMesh<T>::load(const std::string& fileName) {
  if (isObj(fileName)) return loadFromOBJ(fileName);
  if (isPly(fileName)) return loadFromPLY(fileName);
  printf("Can't read extension type: %s \n", fileName.c_str());
  return false;
}

template<typename T>
bool
TriangleMesh<T>::save(const std::string& fileName, const T uvScale) {
  if (isObj(fileName)) return saveToOBJ(fileName, uvScale);
  if (isPly(fileName)) return saveToPLY(fileName, uvScale);
  printf("Can't read extension type: %s \n", fileName.c_str());
  return false;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::loadFromOBJ(const std::string& fileName) {
  std::ifstream fin(fileName);
  if (fin.is_open()) {
    std::string              line;
    std::vector<std::string> tokens;
    clear();
    while (getline(fin, line)) {
      size_t prev = 0;
      size_t pos  = 0;
      tokens.resize(0);
      while ((pos = line.find_first_of(" ,;:/", prev)) != std::string::npos) {
        if (pos > prev) { tokens.push_back(line.substr(prev, pos - prev)); }
        prev = pos + 1;
      }
      if (prev < line.length()) {
        tokens.push_back(line.substr(prev, std::string::npos));
      }
      if (tokens[0] == "v" && tokens.size() >= 4) {
        const Vec3<T> pt(stod(tokens[1]), stod(tokens[2]), stod(tokens[3]));
        _coord.push_back(pt);
      }
      if (tokens[0] == "vt" && tokens.size() >= 3) {
        const Vec2<T> uv(stod(tokens[1]), stod(tokens[2]));
        _texCoord.push_back(uv);
      }
      if (tokens[0] == "vn" && tokens.size() >= 4) {
        const Vec3<T> normal(
          stod(tokens[1]), stod(tokens[2]), stod(tokens[3]));
        _normal.push_back(normal);
      } else if (tokens[0] == "f") {
        if (tokens.size() >= 10) {
          _coordIndex.emplace_back(
            stoi(tokens[1]) - 1, stoi(tokens[4]) - 1, stoi(tokens[7]) - 1);
          _texCoordIndex.emplace_back(
            stoi(tokens[2]) - 1, stoi(tokens[5]) - 1, stoi(tokens[8]) - 1);
          _normalIndex.emplace_back(
            stoi(tokens[3]) - 1, stoi(tokens[6]) - 1, stoi(tokens[9]) - 1);
        } else if (tokens.size() == 7) {
          _coordIndex.emplace_back(
            stoi(tokens[1]) - 1, stoi(tokens[3]) - 1, stoi(tokens[5]) - 1);
          _texCoordIndex.emplace_back(
            stoi(tokens[2]) - 1, stoi(tokens[4]) - 1, stoi(tokens[6]) - 1);
        } else if (tokens.size() == 4) {
          _coordIndex.emplace_back(
            stoi(tokens[1]) - 1, stoi(tokens[2]) - 1, stoi(tokens[3]) - 1);
        }
      } else if (tokens[0] == "mtllib" && tokens.size() >= 2) {
        _mtllib = tokens[1];
      }
    }
    fin.close();
    return true;
  }
  printf("Error loading file: %s \n", fileName.c_str());

  return false;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::saveToOBJ(const std::string& fileName,
                           const T            uvScale) const {
  std::ofstream fout(fileName);
  if (fout.is_open()) {
    const int32_t ptCount    = pointCount();
    const int32_t tcCount    = texCoordCount();
    const int32_t nCount     = normalCount();
    const int32_t cCount     = colourCount();
    const int32_t triCount   = triangleCount();
    const int32_t tcTriCount = texCoordTriangleCount();
    const int32_t nTriCount  = normalTriangleCount();

    assert(nTriCount == 0 || nTriCount == triCount);
    assert(tcTriCount == 0 || tcTriCount == triCount);
    fout << "####\n";
    fout << "# Coord:        " << ptCount << '\n';
    fout << "# Colour:       " << cCount << '\n';
    fout << "# Normals:      " << nCount << '\n';
    fout << "# TexCoord:     " << tcCount << '\n';
    fout << "# Triangles:    " << triCount << '\n';
    fout << "# TexTriangles: " << tcTriCount << '\n';
    fout << "# NrmTriangles: " << nTriCount << '\n';
    fout << "####\n";
    if (!_mtllib.empty()) { fout << "mtllib " << _mtllib << '\n'; }
    const auto hasColours =
      !_colour.empty() && _colour.size() == _coord.size();
    const auto hasDisplacements =
      !_disp.empty() && _disp.size() == _coord.size();
    for (int32_t pointIndex = 0; pointIndex < ptCount; ++pointIndex) {
      const auto& pt = point(pointIndex);
      fout << "v " << T(pt.x()) << ' ' << T(pt.y()) << ' ' << T(pt.z());
      if (hasColours) {
        const auto& c = colour(pointIndex);
        fout << ' ' << T(c.x()) << ' ' << T(c.y()) << ' ' << T(c.z());
      }
      if (hasDisplacements) {
        const auto& d = displacement(pointIndex);
        fout << ' ' << T(d.x()) << ' ' << T(d.y()) << ' ' << T(d.z());
      }
      fout << '\n';
    }

    for (int32_t uvIndex = 0; uvIndex < tcCount; ++uvIndex) {
      const auto& uv = texCoord(uvIndex) * uvScale;
      fout << "vt " << T(uv.x()) << ' ' << T(uv.y()) << '\n';
    }

    for (int32_t nIndex = 0; nIndex < nCount; ++nIndex) {
      const auto& n = normal(nIndex);
      fout << "vn " << T(n.x()) << ' ' << T(n.y()) << ' ' << T(n.z()) << '\n';
    }

    if (tcTriCount == 0 && nTriCount == 0) {
      for (int32_t triangleIndex = 0; triangleIndex < triCount;
           ++triangleIndex) {
        const auto& tri = triangle(triangleIndex);
        const auto  i   = (tri.x() + 1);
        const auto  j   = (tri.y() + 1);
        const auto  k   = (tri.z() + 1);

        assert(i != j && i != k && j != k);
        fout << "f " << i << ' ' << j << ' ' << k << '\n';
      }
    } else if (nTriCount && tcTriCount == 0) {
      for (int32_t triangleIndex = 0; triangleIndex < triCount;
           ++triangleIndex) {
        const auto& tri = triangle(triangleIndex);
        const auto  i0  = (tri.x() + 1);
        const auto  j0  = (tri.y() + 1);
        const auto  k0  = (tri.z() + 1);

        const auto& ntri = normalTriangle(triangleIndex);
        const auto  i1   = (ntri.x() + 1);
        const auto  j1   = (ntri.y() + 1);
        const auto  k1   = (ntri.z() + 1);

        assert(i0 != j0 && i0 != k0 && j0 != k0);
        assert(i1 != j1 && i1 != k1 && j1 != k1);
        fout << "f " << i0 << "//" << i1 << ' ' << j0 << "//" << j1 << ' '
             << k0 << "//" << k1 << '\n';
      }
    } else if (tcTriCount && nTriCount == 0) {
      for (int32_t triangleIndex = 0; triangleIndex < triCount;
           ++triangleIndex) {
        const auto& tri = triangle(triangleIndex);
        const auto  i0  = (tri.x() + 1);
        const auto  j0  = (tri.y() + 1);
        const auto  k0  = (tri.z() + 1);

        const auto& uvtri = texCoordTriangle(triangleIndex);
        const auto  i1    = (uvtri.x() + 1);
        const auto  j1    = (uvtri.y() + 1);
        const auto  k1    = (uvtri.z() + 1);

        assert(i0 != j0 && i0 != k0 && j0 != k0);
        assert(i1 != j1 && i1 != k1 && j1 != k1);
        fout << "f " << i0 << '/' << i1 << ' ' << j0 << '/' << j1 << ' ' << k0
             << '/' << k1 << '\n';
      }
    } else {
      for (int32_t triangleIndex = 0; triangleIndex < triCount;
           ++triangleIndex) {
        const auto& tri = triangle(triangleIndex);
        const auto  i0  = (tri.x() + 1);
        const auto  j0  = (tri.y() + 1);
        const auto  k0  = (tri.z() + 1);

        const auto& uvtri = texCoordTriangle(triangleIndex);
        const auto  i1    = (uvtri.x() + 1);
        const auto  j1    = (uvtri.y() + 1);
        const auto  k1    = (uvtri.z() + 1);

        const auto& ntri = normalTriangle(triangleIndex);
        const auto  i2   = (ntri.x() + 1);
        const auto  j2   = (ntri.y() + 1);
        const auto  k2   = (ntri.z() + 1);

        assert(i0 != j0 && i0 != k0 && j0 != k0);
        assert(i1 != j1 && i1 != k1 && j1 != k1);
        assert(i2 != j2 && i2 != k2 && j2 != k2);
        fout << "f " << i0 << '/' << i1 << '/' << i2 << ' ' << j0 << '/' << j1
             << '/' << j2 << ' ' << k0 << '/' << k1 << '/' << k2 << '\n';
      }
    }
    fout.close();
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::saveToPLY(const std::string& fileName,
                           const bool         binary,
                           const T            uvScale) const {
  std::filebuf fb;
  fb.open(fileName, binary ? std::ios::out | std::ios::binary : std::ios::out);
  std::ostream outstream(&fb);
  if (outstream.fail()) {
    throw std::runtime_error("failed to open " + fileName);
    return false;
  }
  tinyply::Type type;
  if (std::is_same<T, double>::value) type = tinyply::Type::FLOAT64;
  else if (std::is_same<T, float>::value) type = tinyply::Type::FLOAT32;
  else if (std::is_same<T, unsigned char>::value) tinyply::Type::UINT8;
  else {
    throw std::runtime_error("saveToPLY: type not supported");
    exit(-1);
  }

#define CAST_UINT8(ptr) \
  reinterpret_cast<const uint8_t*>(reinterpret_cast<const void*>(ptr.data()))
  tinyply::PlyFile ply;
  ply.get_comments().push_back("generated by mpeg vmesh");
  ply.get_comments().push_back("TextureFile " + _mtllib);
  ply.add_properties_to_element("vertex",
                                {"x", "y", "z"},
                                type,
                                _coord.size(),
                                CAST_UINT8(_coord),
                                tinyply::Type::INVALID,
                                0);
  if (!_colour.empty() && _colour.size() == _coord.size())
    ply.add_properties_to_element("vertex",
                                  {"red", "green", "blue"},
                                  type,
                                  _colour.size(),
                                  CAST_UINT8(_colour),
                                  tinyply::Type::INVALID,
                                  0);
  if (!_normal.empty() && _normal.size() == _coord.size())
    ply.add_properties_to_element("vertex",
                                  {"nx", "ny", "nz"},
                                  type,
                                  _normal.size(),
                                  CAST_UINT8(_normal),
                                  tinyply::Type::INVALID,
                                  0);
  if (!_disp.empty() && _disp.size() == _coord.size())
    ply.add_properties_to_element("vertex",
                                  {"dx", "dy", "dz"},
                                  type,
                                  _disp.size(),
                                  CAST_UINT8(_disp),
                                  tinyply::Type::INVALID,
                                  0);
  if (!_texCoord.empty() && _texCoord.size() == _coord.size()) {
    if (uvScale != T(1)) {
      auto texCoord = _texCoord;
      for (auto& value : texCoord) value *= uvScale;
      ply.add_properties_to_element("vertex",
                                    {"texture_u", "texture_v"},
                                    type,
                                    texCoord.size(),
                                    CAST_UINT8(texCoord),
                                    tinyply::Type::INVALID,
                                    0);
    } else {
      ply.add_properties_to_element("vertex",
                                    {"texture_u", "texture_v"},
                                    type,
                                    _texCoord.size(),
                                    CAST_UINT8(_texCoord),
                                    tinyply::Type::INVALID,
                                    0);
    }
  }
  ply.add_properties_to_element("face",
                                {"vertex_indices"},
                                tinyply::Type::INT32,
                                _coordIndex.size(),
                                CAST_UINT8(_coordIndex),
                                tinyply::Type::UINT8,
                                3);
  if (_texCoordIndex.size())
    ply.add_properties_to_element("face",
                                  {"texcoord"},
                                  tinyply::Type::INT32,
                                  _texCoordIndex.size(),
                                  CAST_UINT8(_texCoordIndex),
                                  tinyply::Type::UINT8,
                                  3);
  if (_normalIndex.size())
    ply.add_properties_to_element("face",
                                  {"normals"},
                                  tinyply::Type::INT32,
                                  _normalIndex.size(),
                                  CAST_UINT8(_normalIndex),
                                  tinyply::Type::UINT8,
                                  3);

  ply.write(outstream, binary);
  return true;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::loadFromPLY(const std::string& filename) {
  std::unique_ptr<std::istream> file;
  file.reset(new std::ifstream(filename, std::ios::binary));
  if (!file || file->fail()) {
    printf("failed to open: %s \n", filename);
    return false;
  }
  tinyply::PlyFile ply;
  ply.parse_header(*file);
  std::shared_ptr<tinyply::PlyData> coords, normals, colours, texcoords,
    displacements, triangles, texTriangles, normalTriangles;
  try {
    coords = ply.request_properties_from_element("vertex", {"x", "y", "z"});
  } catch (const std::exception& e) {}
  try {
    normals =
      ply.request_properties_from_element("vertex", {"nx", "ny", "nz"});
  } catch (const std::exception& e) {}
  try {
    colours =
      ply.request_properties_from_element("vertex", {"red", "green", "blue"});
  } catch (const std::exception& e) {}
  try {
    colours = ply.request_properties_from_element("vertex", {"r", "g", "b"});
  } catch (const std::exception& e) {}
  try {
    texcoords = ply.request_properties_from_element(
      "vertex", {"texture_u", "texture_v"});
  } catch (const std::exception& e) {}
  try {
    displacements =
      ply.request_properties_from_element("vertex", {"dx", "dy", "dz"});
  } catch (const std::exception& e) {}
  try {
    triangles =
      ply.request_properties_from_element("face", {"vertex_indices"}, 3);
  } catch (const std::exception& e) {}
  try {
    texTriangles =
      ply.request_properties_from_element("face", {"texcoord"}, 3);
  } catch (const std::exception& e) {}
  try {
    normalTriangles =
      ply.request_properties_from_element("face", {"normals"}, 3);
  } catch (const std::exception& e) {}

  ply.read(*file);

  convertFromPlyData(coords, _coord);
  convertFromPlyData(normals, _normal);
  convertFromPlyData(colours, _colour);
  convertFromPlyData(texcoords, _texCoord);
  convertFromPlyData(displacements, _disp);
  convertFromPlyData(triangles, _coordIndex);
  convertFromPlyData(texTriangles, _texCoordIndex);
  convertFromPlyData(normalTriangles, _normalIndex);
  return true;
}

//----------------------------------------------------------------------------

template<typename T>
void
TriangleMesh<T>::invertOrientation() {
  const auto hasTriangles         = triangleCount() > 0;
  const auto hasTexCoordTriangles = texCoordTriangleCount() > 0;
  const auto hasNormalTriangles   = normalTriangleCount() > 0;
  for (int32_t tindex = 0, tcount = triangleCount(); tindex < tcount;
       ++tindex) {
    if (hasTriangles) {
      assert(tindex < triangleCount());
      const auto& tri = triangle(tindex);
      setTriangle(tindex, tri[1], tri[0], tri[2]);
    }

    if (hasTexCoordTriangles) {
      assert(tindex < texCoordTriangleCount());
      const auto& tri = texCoordTriangle(tindex);
      setTexCoordTriangle(tindex, tri[1], tri[0], tri[2]);
    }

    if (hasNormalTriangles) {
      assert(tindex < normalTriangleCount());
      const auto& tri = normalTriangle(tindex);
      setNormalTriangle(tindex, tri[1], tri[0], tri[2]);
    }
  }
}

//----------------------------------------------------------------------------

template<typename T>
void
TriangleMesh<T>::append(const TriangleMesh<double>& mesh) {
  const auto offsetPositions         = pointCount();
  const auto offsetTexCoord          = texCoordCount();
  const auto offsetNormal            = normalCount();
  const auto offsetTriangles         = triangleCount();
  const auto offsetTexCoordTriangles = texCoordTriangleCount();
  const auto offsetNormalTriangles   = normalTriangleCount();

  const auto posCount         = mesh.pointCount();
  const auto tcCount          = mesh.texCoordCount();
  const auto nCount           = mesh.normalCount();
  const auto triCount         = mesh.triangleCount();
  const auto texCoordTriCount = mesh.texCoordTriangleCount();
  const auto normalTriCount   = mesh.normalTriangleCount();

  resizePoints(offsetPositions + posCount);
  for (int v = 0; v < posCount; ++v) {
    setPoint(offsetPositions + v, mesh.point(v));
  }

  resizeTexCoords(offsetTexCoord + tcCount);
  for (int v = 0; v < tcCount; ++v) {
    setTexCoord(offsetTexCoord + v, mesh.texCoord(v));
  }

  resizeNormals(offsetNormal + nCount);
  for (int v = 0; v < nCount; ++v) {
    setNormal(offsetNormal + v, mesh.normal(v));
  }

  resizeTriangles(offsetTriangles + triCount);
  for (int t = 0; t < triCount; ++t) {
    const auto tri = mesh.triangle(t) + offsetPositions;
    setTriangle(offsetTriangles + t, tri);
  }

  resizeTexCoordTriangles(offsetTexCoordTriangles + texCoordTriCount);
  for (int t = 0; t < texCoordTriCount; ++t) {
    const auto tri = mesh.texCoordTriangle(t) + offsetTexCoord;
    setTexCoordTriangle(offsetTexCoordTriangles + t, tri);
  }

  resizeNormalTriangles(offsetNormalTriangles + normalTriCount);
  for (int t = 0; t < normalTriCount; ++t) {
    const auto tri = mesh.normalTriangle(t) + offsetNormal;
    setNormalTriangle(offsetNormalTriangles + t, tri);
  }
}

//----------------------------------------------------------------------------

template<typename T>
void
TriangleMesh<T>::subdivideMidPoint(
  int32_t                            iterationCount,
  std::vector<SubdivisionLevelInfo>* infoLevelOfDetails,
  std::vector<int64_t>*              coordEdges,
  std::vector<int64_t>*              texCoordEdges,
  std::vector<int32_t>*              triangleToBaseMeshTriangle) {
  if (triangleToBaseMeshTriangle != nullptr) {
    auto&      triToBaseMeshTri = *triangleToBaseMeshTriangle;
    const auto tCount0          = this->triangleCount();
    triToBaseMeshTri.resize(tCount0);
    for (int32_t t = 0; t < tCount0; ++t) { triToBaseMeshTri[t] = t; }
  }

  if (infoLevelOfDetails != nullptr) {
    infoLevelOfDetails->resize(iterationCount + 1);
    auto& infoLevelOfDetail                 = (*infoLevelOfDetails)[0];
    infoLevelOfDetail.pointCount            = pointCount();
    infoLevelOfDetail.triangleCount         = triangleCount();
    infoLevelOfDetail.texCoordCount         = texCoordCount();
    infoLevelOfDetail.texCoordTriangleCount = texCoordTriangleCount();
  }

  if (iterationCount <= 0) { return; }

  const auto tCount = triangleCount() * std::pow(4, iterationCount - 1);
  if (texCoordTriangleCount() && texCoordCount()) {
    const auto tcCount = texCoordCount() * std::pow(4, iterationCount - 1);
    reserveTexCoordTriangles(4 * tCount);
    reserveTexCoords(4 * tcCount);
    StaticAdjacencyInformation<int32_t> vertexToTriangleUV;
    StaticAdjacencyInformation<int32_t> vertexToEdgeUV;
    std::vector<int8_t>                 vtagsUV;
    vertexToTriangleUV.reserve(tcCount);
    vertexToEdgeUV.reserve(tcCount);
    vtagsUV.reserve(tcCount);
    if (texCoordEdges == nullptr) {
      std::vector<int64_t> edges(4 * tcCount, int64_t(-1));
      for (int32_t it = 0; it < iterationCount; ++it) {
        SubdivideMidPoint<Vec2<T>, T>(texCoords(),
                                      texCoordTriangles(),
                                      vertexToTriangleUV,
                                      vertexToEdgeUV,
                                      vtagsUV,
                                      edges,
                                      nullptr);
        if (infoLevelOfDetails != nullptr) {
          auto& infoLevelOfDetail         = (*infoLevelOfDetails)[it + 1];
          infoLevelOfDetail.texCoordCount = texCoordCount();
          infoLevelOfDetail.texCoordTriangleCount = texCoordTriangleCount();
        }
      }
    } else {
      texCoordEdges->resize(4 * tcCount, int64_t(-1));
      for (int32_t it = 0; it < iterationCount; ++it) {
        SubdivideMidPoint<Vec2<T>, T>(texCoords(),
                                      texCoordTriangles(),
                                      vertexToTriangleUV,
                                      vertexToEdgeUV,
                                      vtagsUV,
                                      *texCoordEdges,
                                      nullptr);
        if (infoLevelOfDetails != nullptr) {
          auto& infoLevelOfDetail         = (*infoLevelOfDetails)[it + 1];
          infoLevelOfDetail.texCoordCount = texCoordCount();
          infoLevelOfDetail.texCoordTriangleCount = texCoordTriangleCount();
        }
      }
    }
  }
  if (triangleCount() && pointCount()) {
    const auto vCount = pointCount() * std::pow(4, iterationCount - 1);
    reservePoints(4 * vCount);
    reserveTriangles(4 * tCount);
    StaticAdjacencyInformation<int32_t> vertexToTriangle;
    StaticAdjacencyInformation<int32_t> vertexToEdge;
    std::vector<int8_t>                 vtags;
    vertexToTriangle.reserve(vCount);
    vertexToEdge.reserve(vCount);
    vtags.reserve(vCount);
    if (coordEdges == nullptr) {
      std::vector<int64_t> edges(4 * vCount, int64_t(-1));
      for (int32_t it = 0; it < iterationCount; ++it) {
        SubdivideMidPoint<Vec3<T>, T>(points(),
                                      triangles(),
                                      vertexToTriangle,
                                      vertexToEdge,
                                      vtags,
                                      edges,
                                      triangleToBaseMeshTriangle);
        if (infoLevelOfDetails != nullptr) {
          auto& infoLevelOfDetail         = (*infoLevelOfDetails)[it + 1];
          infoLevelOfDetail.pointCount    = pointCount();
          infoLevelOfDetail.triangleCount = triangleCount();
        }
      }
    } else {
      coordEdges->resize(4 * vCount, int64_t(-1));
      for (int32_t it = 0; it < iterationCount; ++it) {
        SubdivideMidPoint<Vec3<T>, T>(points(),
                                      triangles(),
                                      vertexToTriangle,
                                      vertexToEdge,
                                      vtags,
                                      *coordEdges,
                                      triangleToBaseMeshTriangle);
        if (infoLevelOfDetails != nullptr) {
          auto& infoLevelOfDetail         = (*infoLevelOfDetails)[it + 1];
          infoLevelOfDetail.pointCount    = pointCount();
          infoLevelOfDetail.triangleCount = triangleCount();
        }
      }
    }
  }
}

//============================================================================

template class TriangleMesh<double>;

//============================================================================

}  // namespace vmesh
