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
// Author: jean-eudes.marvie@interdigital.com
// Author: loadObj/saveObj methods based on original code from Owlii
// *****************************************************************

// remove warning when using sprintf on MSVC
#ifdef _MSC_VER
#  define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <time.h>
#include <cmath>
// ply loader
// #define TINYPLY_IMPLEMENTATION
// will use the one from VMESH or ebEncode or ebDecode
#include "tinyply.h"
// mathematics
#include <glm/vec3.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include "ebIO.h"

using namespace eb;

///////////////////////////
// Private methods

bool IO::loadModel( std::string filename, Model& output ) {
  bool success = true;

  // sanity check
  if ( filename.size() < 5 ) {
    std::cout << "Error, invalid mesh file name " << filename << std::endl;
    return false;
  }

  // get extension
  std::string ext = filename.substr( filename.size() - 3, 3 );
  std::for_each( ext.begin(), ext.end(), []( char& c ) { c = ::tolower( c ); } );

  // do the job
  if ( ext == "ply" ) {
    std::cout << "Loading file: " << filename << std::endl;
    auto t1 = clock();
    success = IO::loadPly( filename, output );  // TODO handle read error
    if ( success ) {
      auto t2 = clock();
      std::cout << "Time on loading: " << ( (float)( t2 - t1 ) ) / CLOCKS_PER_SEC << " sec." << std::endl;
    }
  } else if ( ext == "obj" ) {
    std::cout << "Loading file: " << filename << std::endl;
    auto t1 = clock();
    success = IO::loadObj( filename, output );  // TODO handle read error
    if ( success ) {
      auto t2 = clock();
      std::cout << "Time on loading: " << ( (float)( t2 - t1 ) ) / CLOCKS_PER_SEC << " sec." << std::endl;
    }
  } else {
    std::cout << "Error, invalid mesh file extension (not in obj, ply)" << std::endl;
    return false;
  }

  if ( success ) {
    // print stats
    std::cout << "Input model: " << filename << std::endl;
    std::cout << "  Vertices: " << output.vertices.size() / 3 << std::endl;
    std::cout << "  UVs: " << output.uvcoords.size() / 2 << std::endl;
    std::cout << "  Colors: " << output.colors.size() / 3 << std::endl;
    std::cout << "  Normals: " << output.normals.size() / 3 << std::endl;
    std::cout << "  Triangles: " << output.triangles.size() / 3 << std::endl;
    std::cout << "  Trianglesuv: " << output.trianglesuv.size() / 3 << std::endl;
  }

  return success;
}

bool IO::saveModel( std::string filename, const Model& input ) {
  // sanity check
  if ( filename.size() < 5 ) {
    std::cout << "Error, invalid mesh file name " << filename << std::endl;
    return false;
  }

  // check output file extension
  std::string out_ext = filename.substr( filename.size() - 3, 3 );
  std::for_each( out_ext.begin(), out_ext.end(), []( char& c ) { c = ::tolower( c ); } );

  // write output
  if ( out_ext == "ply" ) {
    std::cout << "Saving file: " << filename << std::endl;
    auto t1  = clock();
    auto err = IO::savePly( filename, input );
    if ( !err ) {
      auto t2 = clock();
      std::cout << "Time on saving: " << ( (float)( t2 - t1 ) ) / CLOCKS_PER_SEC << " sec." << std::endl;
    }
    return err;
  } else if ( out_ext == "obj" ) {
    std::cout << "Saving file: " << filename << std::endl;
    auto t1  = clock();
    auto err = IO::saveObj( filename, input );
    if ( !err ) {
      auto t2 = clock();
      std::cout << "Time on saving: " << ( (float)( t2 - t1 ) ) / CLOCKS_PER_SEC << " sec." << std::endl;
    }
    return err;
  } else {
    std::cout << "Error: invalid mesh file extension (not in obj, ply)" << std::endl;
    return false;
  }

  // success
  return true;
}

bool IO::loadObj( std::string filename, Model& output ) {
    std::ifstream fin;
    // use a big 4MB buffer to accelerate reads
    char* buf = new char[4 * 1024 * 1024 + 1];
    fin.rdbuf()->pubsetbuf(buf, 4 * 1024 * 1024 + 1);
    fin.open(filename.c_str(), std::ios::in);
    if (!fin) {
        std::cerr << "Error: can't open file " << filename << std::endl;
        delete[] buf;
        return false;
    }
    int         temp_index;
    float       temp_pos, temp_uv, temp_normal, temp_col;
    std::string temp_flag, temp_str;
    std::string line;
    std::getline(fin, line);
    bool hasNormalIndices = false; // if file contains a normal index table
    while (fin) {
        std::istringstream in(line);
        temp_flag = "";
        in >> temp_flag;  // temp_flag: the first word in the line
        if (temp_flag.compare(std::string("mtllib")) == 0) {
            output.header = std::string(line.c_str());
        }
        else if (temp_flag.compare(std::string("v")) == 0) {
            // parse the position
            for (int i = 0; i < 3; i++) {
                in >> temp_pos;
                output.vertices.push_back(temp_pos);
            }
            // parse the color if any (re map 0.0-1.0 to 0-255 internal color format)
            while (in >> temp_col) { output.colors.push_back(std::roundf(temp_col * 255)); }
        }
        else if (temp_flag.compare(std::string("vn")) == 0) {
            for (int i = 0; i < 3; i++) {
                in >> temp_normal;
                output.normals.push_back(temp_normal);
            }
        }
        else if (temp_flag.compare(std::string("vt")) == 0) {
            for (int i = 0; i < 2; i++) {
                in >> temp_uv;
                output.uvcoords.push_back(temp_uv);
            }
        }
        else if (temp_flag.compare(std::string("f")) == 0) {
            // TODO parsing of normals indexes and reindex
            for (int i = 0; i < 3; i++) {
                in >> temp_str;
                const auto found = temp_str.find_first_of("/");
                if (found != std::string::npos) {
                    const auto foundLast = temp_str.find_last_of("/");
                    temp_index = atoi(temp_str.substr(0, found).c_str()) - 1;
                    // found exists so foundLast necessarly exists beeing at least equal to found
                    if (foundLast != found) {
                        hasNormalIndices = true;
                        if (foundLast - found > 1) { // vidx/uvidx/nrmidx
                            int uv_index = atoi(temp_str.substr(found + 1, foundLast - found).c_str()) - 1;
                            output.trianglesuv.push_back(uv_index);
                        }
                        // else vidx//nrmidx or 
                    }
                    else { // "vidx/uvidx"
                        int uv_index = atoi(temp_str.substr(found + 1, temp_str.size() - found).c_str()) - 1;
                        output.trianglesuv.push_back(uv_index);
                    }
                }
                else // vidx alone, no '/' sumbol
                    temp_index = atoi(temp_str.c_str()) - 1;
                output.triangles.push_back(temp_index);
            }
        }
        std::getline(fin, line);
    }
    fin.close();
    delete[] buf;

    if (hasNormalIndices || (output.normals.size() != 0 && output.normals.size() != output.vertices.size())) {
        std::cout << "Warning: obj read, normals with separate index table are not yet supported. Skipping normals."
            << std::endl;
        output.normals.clear();
    }

    return true;
}

bool IO::saveObj( std::string filename, const Model& input ) {
  std::ofstream fout;
  // use a big 4MB buffer to accelerate writes
  char* buf = new char[4 * 1024 * 1024 + 1];
  fout.rdbuf()->pubsetbuf( buf, 4 * 1024 * 1024 + 1 );
  fout.open( filename.c_str(), std::ios::out );
  if ( !fout ) {
    std::cerr << "Error: can't open file " << filename << std::endl;
    delete[] buf;
    return false;
  }
  // this is mandatory to print floats with full precision
  fout.precision( std::numeric_limits<float>::max_digits10 );

  printf( "_saveObj %-40s: V = %zu Vc = %zu N = %zu UV = %zu F = %zu Fuv = %zu \n",
          filename.c_str(),
          input.vertices.size() / 3,
          input.colors.size() / 3,
          input.normals.size() / 3,
          input.uvcoords.size() / 2,
          input.triangles.size() / 3,
          input.trianglesuv.size() / 3 );
  fflush( stdout );

  fout << input.header << std::endl;
  for ( int i = 0; i < input.vertices.size() / 3; i++ ) {
    fout << "v " << input.vertices[i * 3 + 0] << " " << input.vertices[i * 3 + 1] << " " << input.vertices[i * 3 + 2];
    if ( input.colors.size() == input.vertices.size() ) {
      fout << " " << input.colors[i * 3 + 0] / 255 << " " << input.colors[i * 3 + 1] / 255 << " "
           << input.colors[i * 3 + 2] / 255 << std::endl;
    } else {
      fout << std::endl;
    }
  }
  for ( int i = 0; i < input.normals.size() / 3; i++ ) {
    fout << "vn " << input.normals[i * 3 + 0] << " " << input.normals[i * 3 + 1] << " " << input.normals[i * 3 + 2]
         << std::endl;
  }
  for ( int i = 0; i < input.uvcoords.size() / 2; i++ ) {
    fout << "vt " << input.uvcoords[i * 2 + 0] << " " << input.uvcoords[i * 2 + 1] << std::endl;
  }
  if ( input.hasUvCoords() ) { fout << "usemtl material0000" << std::endl; }
  for ( int i = 0; i < input.triangles.size() / 3; i++ ) {
    if ( input.trianglesuv.size() == input.triangles.size() ) {
      fout << "f " << input.triangles[i * 3 + 0] + 1 << "/" << input.trianglesuv[i * 3 + 0] + 1 << " "
           << input.triangles[i * 3 + 1] + 1 << "/" << input.trianglesuv[i * 3 + 1] + 1 << " "
           << input.triangles[i * 3 + 2] + 1 << "/" << input.trianglesuv[i * 3 + 2] + 1 << std::endl;
    } else {
      fout << "f " << input.triangles[i * 3 + 0] + 1 << " " << input.triangles[i * 3 + 1] + 1 << " "
           << input.triangles[i * 3 + 2] + 1 << std::endl;
    }
  }
  fout.close();
  delete[] buf;
  return true;
}
template<typename T, typename D>
void templateConvert( std::shared_ptr<tinyply::PlyData> src,
                      const uint8_t                     numSrc,
                      std::vector<T>&                   dst,
                      const uint8_t                     numDst ) {
  const size_t   numBytes = src->buffer.size_bytes();
  std::vector<D> data;
  data.resize( src->count * numSrc );
  std::memcpy( data.data(), src->buffer.get(), numBytes );
  if ( numSrc == numDst ) {
    dst.assign( data.begin(), data.end() );
  } else {
    dst.resize( src->count * numDst );
    for ( size_t i = 0; i < src->count; i++ )
      for ( size_t c = 0; c < numDst; c++ ) dst[i * numDst + c] = (T)data[i * numSrc + c];
  }
}

template<typename T>
void set( std::shared_ptr<tinyply::PlyData> src,
          const uint8_t                     numSrc,
          std::vector<T>&                   dst,
          const uint8_t                     numDst,
          std::string                       name ) {
  if ( src ) {
    switch ( src->t ) {
    case tinyply::Type::INT8: templateConvert<T, int8_t>( src, numSrc, dst, numDst ); break;
    case tinyply::Type::UINT8: templateConvert<T, uint8_t>( src, numSrc, dst, numDst ); break;
    case tinyply::Type::INT16: templateConvert<T, int16_t>( src, numSrc, dst, numDst ); break;
    case tinyply::Type::UINT16: templateConvert<T, uint16_t>( src, numSrc, dst, numDst ); break;
    case tinyply::Type::INT32: templateConvert<T, int32_t>( src, numSrc, dst, numDst ); break;
    case tinyply::Type::UINT32: templateConvert<T, uint32_t>( src, numSrc, dst, numDst ); break;
    case tinyply::Type::FLOAT32: templateConvert<T, float>( src, numSrc, dst, numDst ); break;
    case tinyply::Type::FLOAT64: templateConvert<T, double>( src, numSrc, dst, numDst ); break;
    default:
      printf( "ERROR: PLY type not supported: %s \n", name.c_str() );
      fflush( stdout );
      exit( -1 );
      break;
    }
  }
}

bool IO::loadPly( std::string filename, Model& output ) {
  std::unique_ptr<std::istream> file_stream;
  file_stream.reset( new std::ifstream( filename.c_str(), std::ios::binary ) );
  tinyply::PlyFile file;
  file.parse_header( *file_stream );
  std::shared_ptr<tinyply::PlyData> _vertices, _normals, _colors, _colorsRGBA, _texcoords, _faces, _tripstrip, _uvfaces,
    _nrmfaces;

  // The header information can be used to programmatically extract properties on elements
  // known to exist in the header prior to reading the data. For brevity of this sample, properties
  // like vertex position are hard-coded:
  try {
    _vertices = file.request_properties_from_element( "vertex", { "x", "y", "z" } );
  } catch ( const std::exception& e ) { std::cerr << "skipping: " << e.what() << std::endl; }
  try {
    _normals = file.request_properties_from_element( "vertex", { "nx", "ny", "nz" } );
  } catch ( const std::exception& e ) { std::cerr << "skipping: " << e.what() << std::endl; }

  try {
    _colors = file.request_properties_from_element( "vertex", { "red", "green", "blue" } );
  } catch ( const std::exception& ) {}
  try {
    _colors = file.request_properties_from_element( "vertex", { "r", "g", "b" } );
  } catch ( const std::exception& ) {}
  try {
    _colorsRGBA = file.request_properties_from_element( "vertex", { "red", "green", "blue", "alpha" } );
  } catch ( const std::exception& ) {}

  try {
    _colorsRGBA = file.request_properties_from_element( "vertex", { "r", "g", "b", "a" } );
  } catch ( const std::exception& ) {}

  try {
    _texcoords = file.request_properties_from_element( "vertex", { "texture_u", "texture_v" } );
  } catch ( const std::exception& ) {}

  // Providing a list size hint (the last argument) is a 2x performance improvement. If you have
  // arbitrary ply files, it is best to leave this 0.
  try {
    _faces = file.request_properties_from_element( "face", { "vertex_indices" }, 3 );
  } catch ( const std::exception& e ) { std::cerr << "skipping: " << e.what() << std::endl; }

  try {
    _uvfaces = file.request_properties_from_element( "face", { "texcoord" }, 6 );
  } catch ( const std::exception& e ) { std::cerr << "skipping: " << e.what() << std::endl; }

  // // Tristrips must always be read with a 0 list size hint (unless you know exactly how many elements
  // // are specifically in the file, which is unlikely);
  // try {
  //   _tripstrip = file.request_properties_from_element( "tristrips", {"vertex_indices"}, 0 );
  // } catch ( const std::exception& e ) { std::cerr << "skipping " << e.what() << std::endl; }

  file.read( *file_stream );

  // now feed the data to the frame structure
  set( _vertices, 3, output.vertices, 3, "vertices" );
  set( _texcoords, 2, output.uvcoords, 2, "uvcoords" );
  set( _normals, 3, output.normals, 3, "normals" );
  set( _colors, 3, output.colors, 3, "colors" );
  set( _colorsRGBA, 4, output.colors, 3, "colorsRGBA" );
  set( _faces, 3, output.triangles, 3, "triangles" );
  if ( _uvfaces ) {
    const auto triCount = _uvfaces->count;
    output.trianglesuv.resize( triCount * 3 );    
    set( _uvfaces, 6, output.uvcoords, 6, "uvfaces" );    
    for ( size_t i = 0; i < triCount * 3; i++ ) output.trianglesuv[i] = i;
  }
  return true;
}

bool IO::savePly( std::string filename, const Model& input ) {
  std::ofstream fout;
  // use a big 4MB buffer to accelerate writes
  char* buf = new char[4 * 1024 * 1024 + 1];
  fout.rdbuf()->pubsetbuf( buf, 4 * 1024 * 1024 + 1 );
  fout.open( filename.c_str(), std::ios::out );
  if ( !fout ) {
    std::cerr << "Error: can't open file " << filename << std::endl;
    delete[] buf;
    return false;
  }
  // this is mandatory to print floats with full precision
  fout.precision( std::numeric_limits<float>::max_digits10 );

  fout << "ply" << std::endl;
  fout << "format ascii 1.0" << std::endl;
  fout << "comment Generated by InterDigital model processor" << std::endl;
  fout << "element vertex " << input.vertices.size() / 3 << std::endl;
  fout << "property float x" << std::endl;
  fout << "property float y" << std::endl;
  fout << "property float z" << std::endl;
  // normals
  if ( input.normals.size() == input.vertices.size() ) {
    fout << "property float nx" << std::endl;
    fout << "property float ny" << std::endl;
    fout << "property float nz" << std::endl;
  }
  // colors
  if ( input.colors.size() == input.vertices.size() ) {
    fout << "property uchar red" << std::endl;
    fout << "property uchar green" << std::endl;
    fout << "property uchar blue" << std::endl;
  }

  if ( !input.triangles.empty() ) {
    fout << "element face " << input.triangles.size() / 3 << std::endl;
    fout << "property list uchar int vertex_indices" << std::endl;
  }
  fout << "end_header" << std::endl;

  // comments
  for ( int i = 0; i < input.comments.size(); i++ ) { fout << input.comments[i] << std::endl; }

  // vertices and colors
  for ( int i = 0; i < input.vertices.size() / 3; i++ ) {
    fout << input.vertices[i * 3 + 0] << " " << input.vertices[i * 3 + 1] << " " << input.vertices[i * 3 + 2] << " ";
    if ( input.normals.size() == input.vertices.size() ) {
      fout << input.normals[i * 3 + 0] << " " << input.normals[i * 3 + 1] << " " << input.normals[i * 3 + 2] << " ";
    }
    if ( input.colors.size() == input.vertices.size() ) {
      // do not cast as char otherwise characters are printed instead if int8 values
      fout << (unsigned short)( std::roundf( input.colors[i * 3 + 0] ) ) << " "
           << (unsigned short)( std::roundf( input.colors[i * 3 + 1] ) ) << " "
           << (unsigned short)( std::roundf( input.colors[i * 3 + 2] ) );
    }
    fout << std::endl;
  }
  // topology
  for ( int i = 0; i < input.triangles.size() / 3; i++ ) {
    fout << "3 " << input.triangles[i * 3 + 0] << " " << input.triangles[i * 3 + 1] << " " << input.triangles[i * 3 + 2]
         << std::endl;
  }

  fout.close();
  delete[] buf;
  return true;
}
