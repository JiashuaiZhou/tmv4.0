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
#include "checksum.hpp"
#include <tinyply.h>

namespace vmesh {

//============================================================================

//============================================================================

template<typename T, typename D>
void
templateConvert(std::shared_ptr<tinyply::PlyData> src, std::vector<T>& dst) {
  const size_t   numBytes = src->buffer.size_bytes();
  std::vector<D> data;
  dst.resize(src->count);
  const size_t length = dst[0].size();
  data.resize(src->count * length);
  std::memcpy(data.data(), src->buffer.get(), numBytes);
  for (size_t i = 0, idx = 0; i < src->count; i++)
    for (size_t j = 0; j < length; j++, idx++) dst[i][j] = data[idx];
}

template<typename T>
void
set(std::shared_ptr<tinyply::PlyData> src,
    std::vector<T>&                   dst,
    std::string                       name) {
  if (src) {
    switch (src->t) {
    case tinyply::Type::INT8: templateConvert<T, int8_t>(src, dst); break;
    case tinyply::Type::UINT8: templateConvert<T, uint8_t>(src, dst); break;
    case tinyply::Type::INT16: templateConvert<T, int16_t>(src, dst); break;
    case tinyply::Type::UINT16: templateConvert<T, uint16_t>(src, dst); break;
    case tinyply::Type::INT32: templateConvert<T, int32_t>(src, dst); break;
    case tinyply::Type::UINT32: templateConvert<T, uint32_t>(src, dst); break;
    case tinyply::Type::FLOAT32: templateConvert<T, float>(src, dst); break;
    case tinyply::Type::FLOAT64: templateConvert<T, double>(src, dst); break;
    default:
      printf("ERROR: PLY type not supported: %s \n", name.c_str());
      fflush(stdout);
      exit(-1);
      break;
    }
  }
}

//============================================================================

template<typename T>
bool
TriangleMesh<T>::load(const std::string& fileName, const int32_t frameIndex) {
  return load(expandNum(fileName, frameIndex));
}

//============================================================================

template<typename T>
bool
TriangleMesh<T>::load(const std::string& fileName) {
  auto ext = extension(fileName);
  if (ext == "obj") return loadFromOBJ(fileName);
  if (ext == "ply") return loadFromPLY(fileName);
  if (ext == "vmb") return loadFromVMB(fileName);
  printf("Can't read extension type: %s \n", fileName.c_str());
  return false;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::save(const std::string& fileName, const bool binary) const {
  printf("Save %s \n", fileName.c_str());
  auto ext = extension(fileName);
  if (ext == "obj") return saveToOBJ(fileName);
  if (ext == "ply") return saveToPLY(fileName, binary);
  if (ext == "vmb") return saveToVMB(fileName);
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
    // print("Load: " + fileName);
    return true;
  }
  printf("Error loading file: %s \n", fileName.c_str());

  return false;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::saveToOBJ(const std::string& fileName) const {
  std::ofstream fout(fileName);
  if (fout.is_open()) {
    const int32_t ptCount    = pointCount();
    const int32_t tcCount    = texCoordCount();
    const int32_t nCount     = normalCount();
    const int32_t triCount   = triangleCount();
    const int32_t tcTriCount = texCoordTriangleCount();
    const int32_t nTriCount  = normalTriangleCount();

    assert(nTriCount == 0 || nTriCount == triCount);
    assert(tcTriCount == 0 || tcTriCount == triCount);
    print("Save: " + fileName);
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
      const auto& uv = texCoord(uvIndex);
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
                           const bool         binary) const {
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
  else if (std::is_same<T, unsigned char>::value) type = tinyply::Type::UINT8;
  else {
    throw std::runtime_error("saveToPLY: type not supported");
    exit(-1);
  }

#define CAST_UINT8(ptr) \
  reinterpret_cast<const uint8_t*>(reinterpret_cast<const void*>(ptr.data()))
  tinyply::PlyFile ply;
  auto&            str = ply.get_comments();
  str.push_back("generated by mpeg-vmesh-tm + tinyply");
  str.push_back("TextureFile " + _mtllib);
  str.push_back("###");
  str.push_back("# Coord:        " + std::to_string(pointCount()));
  str.push_back("# Colour:       " + std::to_string(colourCount()));
  str.push_back("# Normals:      " + std::to_string(normalCount()));
  str.push_back("# TexCoord:     " + std::to_string(texCoordCount()));
  str.push_back("# Triangles:    " + std::to_string(triangleCount()));
  str.push_back("# TexTriangles: " + std::to_string(texCoordTriangleCount()));
  str.push_back("###");

  // Vertices
  ply.add_properties_to_element("vertex",
                                {"x", "y", "z"},
                                type,
                                _coord.size(),
                                CAST_UINT8(_coord),
                                tinyply::Type::INVALID,
                                0);

  if ((!_colour.empty()) && _colour.size() == _coord.size())
    ply.add_properties_to_element("vertex",
                                  {"red", "green", "blue"},
                                  type,
                                  _colour.size(),
                                  CAST_UINT8(_colour),
                                  tinyply::Type::INVALID,
                                  0);

  if ((!_normal.empty()) && _normal.size() == _coord.size())
    ply.add_properties_to_element("vertex",
                                  {"nx", "ny", "nz"},
                                  type,
                                  _normal.size(),
                                  CAST_UINT8(_normal),
                                  tinyply::Type::INVALID,
                                  0);

  // Faces
  ply.add_properties_to_element("face",
                                {"vertex_indices"},
                                tinyply::Type::INT32,
                                _coordIndex.size(),
                                CAST_UINT8(_coordIndex),
                                tinyply::Type::UINT8,
                                3);

  // Face texture coordinate
  std::vector<Vec3<Vec2<T>>> uvCoords;
  if (!_texCoordIndex.empty()) {
    const size_t triCount = triangleCount();
    uvCoords.resize(triCount);
    for (size_t i = 0; i < triCount; i++)
      for (size_t j = 0; j < 3; j++)
        uvCoords[i][j] = _texCoord[_texCoordIndex[i][j]];
    ply.add_properties_to_element("face",
                                  {"texcoord"},
                                  type,
                                  triCount,
                                  CAST_UINT8(uvCoords),
                                  tinyply::Type::UINT8,
                                  6);
  }
  ply.write(outstream, binary);
  return true;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::loadFromPLY(const std::string& fileName) {
  std::unique_ptr<std::istream> file;
  file.reset(new std::ifstream(fileName, std::ios::binary));
  if (!file || file->fail()) {
    printf("failed to open: %s \n", fileName.c_str());
    return false;
  }
  tinyply::PlyFile ply;
  ply.parse_header(*file);

  std::shared_ptr<tinyply::PlyData> coords, normals, colours, texcoords,
    triangles, texTriangles;
  try {
    coords = ply.request_properties_from_element("vertex", {"x", "y", "z"});
  } catch (const std::exception&) {}
  try {
    normals =
      ply.request_properties_from_element("vertex", {"nx", "ny", "nz"});
  } catch (const std::exception&) {}
  try {
    colours =
      ply.request_properties_from_element("vertex", {"red", "green", "blue"});
  } catch (const std::exception&) {}
  try {
    colours = ply.request_properties_from_element("vertex", {"r", "g", "b"});
  } catch (const std::exception&) {}
  try {
    texcoords = ply.request_properties_from_element(
      "vertex", {"texture_u", "texture_v"});
  } catch (const std::exception&) {}
  try {
    triangles =
      ply.request_properties_from_element("face", {"vertex_indices"}, 3);
  } catch (const std::exception&) {}
  try {
    texTriangles =
      ply.request_properties_from_element("face", {"texcoord"}, 6);
  } catch (const std::exception&) {}

  ply.read(*file);
  set(coords, _coord, "vertices");
  set(normals, _normal, "normals");
  set(colours, _colour, "colors");
  set(texcoords, _texCoord, "uvcoords");
  set(triangles, _coordIndex, "triangles");
  if (texTriangles) {
    const auto                 nbBytes  = texTriangles->buffer.size_bytes();
    const auto                 triCount = texTriangles->count;
    std::vector<Vec3<Vec2<T>>> uvCoords;
    uvCoords.resize(triCount);
    switch (texTriangles->t) {
    case tinyply::Type::FLOAT32: {
      std::vector<float> data;
      data.resize(texTriangles->count * 6);
      std::memcpy(data.data(), texTriangles->buffer.get(), nbBytes);
      for (size_t i = 0, idx = 0; i < triCount; i++)
        for (size_t j = 0; j < 3; j++)
          for (size_t k = 0; k < 2; k++, idx++) uvCoords[i][j][k] = data[idx];
      break;
    }
    case tinyply::Type::FLOAT64: {
      std::vector<double> data;
      data.resize(texTriangles->count * 6);
      std::memcpy(data.data(), texTriangles->buffer.get(), nbBytes);
      for (size_t i = 0, idx = 0; i < triCount; i++)
        for (size_t j = 0; j < 3; j++)
          for (size_t k = 0; k < 2; k++, idx++) uvCoords[i][j][k] = data[idx];
      break;
    }
    default:
      printf("ERROR: PLY only supports texcoord type: double or float \n");
      fflush(stdout);
      exit(-1);
      break;
    }
    _texCoordIndex.resize(triCount);
    _texCoord.clear();
    for (size_t i = 0; i < triCount; i++) {
      for (size_t j = 0; j < 3; j++) {
        size_t index = 0;
        auto   it =
          std::find(_texCoord.begin(), _texCoord.end(), uvCoords[i][j]);
        if (it == _texCoord.end()) {
          index = _texCoord.size();
          _texCoord.push_back(uvCoords[i][j]);
        } else {
          index = std::distance(_texCoord.begin(), it);
        }
        _texCoordIndex[i][j] = index;
      }
    }
  }
  return true;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::saveToVMB(const std::string& fileName) const {
  // Compute checksum
  Checksum checksum;
  auto     strChecksum = checksum.getChecksum(*this);
  // print("Save: " + fileName);
  // std::cout << "# Checksum:     " << strChecksum << "\n";

  // Get data type
  std::string type = "float";
  if (std::is_same<T, double>::value) type = "double";
  else if (std::is_same<T, float>::value) type = "float";
  else if (std::is_same<T, unsigned char>::value) type = "uint8";
  else {
    throw std::runtime_error("saveToPLY: type not supported");
    exit(-1);
  }

  // Write header
  std::ofstream fout(fileName, std::ofstream::out);
  if (!fout.is_open()) { return false; }
  fout << "#VMB" << std::endl;
  fout << "Type:     " << type << "\n";
  fout << "Coord:    " << pointCount() << "\n";
  fout << "Colour:   " << colourCount() << "\n";
  fout << "Normals:  " << normalCount() << "\n";
  fout << "TexCoord: " << texCoordCount() << "\n";
  fout << "Disp:     " << displacementCount() << "\n";
  fout << "Faces:    " << triangleCount() << "\n";
  fout << "TexFaces: " << texCoordTriangleCount() << "\n";
  fout << "NrmFaces: " << normalTriangleCount() << "\n";
  fout << "Mtllib:   " << _mtllib << "\n";
  fout << "Checksum: " << strChecksum << "\n";
  fout << "end_header \n";
  fout.clear();
  fout.close();

  // Write
  fout.open(fileName,
            std::ofstream::binary | std::ofstream::out | std::ofstream::app);

  // Vertex coordinates
  if (!_coord.empty())
    fout.write(reinterpret_cast<const char*>(_coord.data()),
               _coord.size() * sizeof(T) * 3);
  if (!_colour.empty())
    fout.write(reinterpret_cast<const char*>(_colour.data()),
               _colour.size() * sizeof(T) * 3);
  if (!_normal.empty())
    fout.write(reinterpret_cast<const char*>(_normal.data()),
               _normal.size() * sizeof(T) * 3);
  if (!_texCoord.empty())
    fout.write(reinterpret_cast<const char*>(_texCoord.data()),
               _texCoord.size() * sizeof(T) * 2);
  if (!_disp.empty())
    fout.write(reinterpret_cast<const char*>(_disp.data()),
               _disp.size() * sizeof(T) * 3);

  // Face indices
  if (!_coordIndex.empty())
    fout.write(reinterpret_cast<const char*>(_coordIndex.data()),
               _coordIndex.size() * sizeof(int) * 3);
  if (!_normalIndex.empty())
    fout.write(reinterpret_cast<const char*>(_normalIndex.data()),
               _normalIndex.size() * sizeof(int) * 3);
  if (!_texCoordIndex.empty())
    fout.write(reinterpret_cast<const char*>(_texCoordIndex.data()),
               _texCoordIndex.size() * sizeof(int) * 3);

  fout.close();
  return true;
}

//----------------------------------------------------------------------------

template<typename T>
bool
TriangleMesh<T>::loadFromVMB(const std::string& fileName) {
  std::ifstream ifs(fileName, std::ifstream::in);
  if (!ifs.is_open()) { return false; }
  std::string line;
  getline(ifs, line);
  if (line.rfind("#VMB", 0) != 0) {
    printf("ERROR: Read VMB %s can't read file format\n", fileName.c_str());
    exit(-1);
  }

  // Get data type
  std::string type = "float";
  if (std::is_same<T, double>::value) type = "double";
  else if (std::is_same<T, float>::value) type = "float";
  else if (std::is_same<T, unsigned char>::value) type = "uint8";
  else {
    throw std::runtime_error("saveToPLY: type not supported");
    exit(-1);
  }

  // Read header
  std::string readChecksum;
  for (; getline(ifs, line);) {
    if (line.rfind("end_header", 0) == 0) break;
    else if (line.rfind("#", 0) == 0) continue;
    else {
      std::stringstream ss(line);
      std::string       name, value;
      ss >> name >> value;
      if (name == "Coord:") _coord.resize(std::stoi(value));
      else if (name == "Colour:") _colour.resize(std::stoi(value));
      else if (name == "Normals:") _normal.resize(std::stoi(value));
      else if (name == "TexCoord:") _texCoord.resize(std::stoi(value));
      else if (name == "Disp:") _disp.resize(std::stoi(value));
      else if (name == "Faces:") _coordIndex.resize(std::stoi(value));
      else if (name == "TexFaces:") _texCoordIndex.resize(std::stoi(value));
      else if (name == "NrmTFaces:") _normalIndex.resize(std::stoi(value));
      else if (name == "Checksum:") readChecksum = value;
      else if (name == "type:") {
        if (type != value) {
          printf("ERROR: Read VMB %s data type not correct\n",
                 fileName.c_str());
          exit(-1);
        }
      }
    }
  }

  // Read data
  const auto headerCount = ifs.tellg();
  ifs.close();
  ifs.open(fileName, std::ifstream::binary | std::ifstream::in);
  ifs.seekg(headerCount);

  // Vertex coordinates
  if (!_coord.empty())
    ifs.read(reinterpret_cast<char*>(_coord.data()),
             _coord.size() * sizeof(T) * 3);
  if (!_colour.empty())
    ifs.read(reinterpret_cast<char*>(_colour.data()),
             _colour.size() * sizeof(T) * 3);
  if (!_normal.empty())
    ifs.read(reinterpret_cast<char*>(_normal.data()),
             _normal.size() * sizeof(T) * 3);
  if (!_texCoord.empty())
    ifs.read(reinterpret_cast<char*>(_texCoord.data()),
             _texCoord.size() * sizeof(T) * 2);
  if (!_disp.empty())
    ifs.read(reinterpret_cast<char*>(_disp.data()),
             _disp.size() * sizeof(T) * 3);

  // Face indices
  if (!_coordIndex.empty())
    ifs.read(reinterpret_cast<char*>(_coordIndex.data()),
             _coordIndex.size() * sizeof(int) * 3);
  if (!_normalIndex.empty())
    ifs.read(reinterpret_cast<char*>(_normalIndex.data()),
             _normalIndex.size() * sizeof(int) * 3);
  if (!_texCoordIndex.empty())
    ifs.read(reinterpret_cast<char*>(_texCoordIndex.data()),
             _texCoordIndex.size() * sizeof(int) * 3);
  ifs.close();

  // print("Load: " + fileName);

  // Compare checksums
  if (!readChecksum.empty()) {
    Checksum checksum;
    auto     computeChecksum = checksum.getChecksum(*this);
    if (computeChecksum != readChecksum) {
      printf("Error: Read/compute checksum are not the same: %s %s \n",
             readChecksum.c_str(),
             computeChecksum.c_str());
      fflush(stdout);
      exit(-1);
    } else {
      printf("Checksums match: %s \n", readChecksum.c_str());
    }
  }
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
TriangleMesh<T>::append(const TriangleMesh<T>& mesh) {
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

template class TriangleMesh<float>;
template class TriangleMesh<double>;

//============================================================================

}  // namespace vmesh
