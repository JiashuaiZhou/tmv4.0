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
#include "ebModel.h"

using namespace eb;

//
void Model::normalizeNormals( void ) {
  // normalize vertex normals if any
  for ( size_t i = 0; i < getNormalCount(); i++ ) {
    glm::vec3 normal = fetchNormal( i );
    normal           = glm::normalize( normal );
    if ( std::isnan( normal[0] ) ) {
      // use default z normal if invalid face
      normal = glm::vec3( 0.0F, 0.0F, 1.0F );
    }
    for ( glm::vec3::length_type c = 0; c < 3; c++ ) { normals[i * 3 + c] = normal[c]; }
  }

  // normalze triangle normals if any
  for ( size_t i = 0; i < getFaceNormalCount(); i++ ) {
    glm::vec3 normal = fetchFaceNormal( i );
    normal           = glm::normalize( normal );
    if ( std::isnan( normal[0] ) ) {
      // use default z normal if invalid face
      normal = glm::vec3( 0.0F, 0.0F, 1.0F );
    }
    for ( glm::vec3::length_type c = 0; c < 3; c++ ) { faceNormals[i * 3 + c] = normal[c]; }
  }
}

//
void Model::computeFaceNormals( bool normalize ) {
  // allocate output
  faceNormals.resize( getTriangleCount() * 3 );
  //
  for ( size_t t = 0; t < getTriangleCount(); t++ ) {
    //
    const glm::vec3 v1 = fetchPosition( t, 0 );
    const glm::vec3 v2 = fetchPosition( t, 1 );
    const glm::vec3 v3 = fetchPosition( t, 2 );
    // computes face normal
    glm::vec3       faceNormal;
    const glm::vec3 v12 = v2 - v1;
    const glm::vec3 v13 = v3 - v1;
    faceNormal          = glm::cross( v12, v13 );
    // write the result
    for ( glm::vec3::length_type c = 0; c < 3; c++ ) { faceNormals[t * 3 + c] = faceNormal[c]; }
  }
  // normalize if requested
  if ( normalize ) { normalizeNormals(); }
}

//
void Model::computeVertexNormals( bool normalize, bool noSeams ) {
  //
  if ( !hasTriangleNormals() ) { computeFaceNormals( false ); }
  //
  normals.resize( vertices.size(), 0.0F );
  // map ( position -> map ( vertexIndex, faceNormal ))
  // used in no seams mode
  auto cmp = []( const glm::vec3& a, const glm::vec3& b ) {
    if ( a.x != b.x ) return a.x < b.x;
    if ( a.y != b.y ) return a.y < b.y;
    return a.z < b.z;
  };

  typedef std::map<int, glm::vec3>                                                                       TmpVertNormals;
  typedef std::map<glm::vec3, TmpVertNormals, std::function<bool( const glm::vec3&, const glm::vec3& )>> TmpPosNormals;

  TmpPosNormals tmpNormals( cmp );

  //
  for ( size_t t = 0; t < getTriangleCount(); t++ ) {
    int idx[3];
    fetchTriangleIndices( t, idx[0], idx[1], idx[2] );
    const glm::vec3 normal = fetchFaceNormal( t );

    if ( noSeams ) {
      glm::vec3 pos[3];
      fetchTriangleVertices( t, pos[0], pos[1], pos[2] );
      for ( size_t i = 0; i < 3; i++ ) {
        // p1
        auto posIter = tmpNormals.find( pos[i] );
        bool found   = false;
        if ( posIter != tmpNormals.end() ) {
          auto vertIter = posIter->second.find( idx[i] );
          if ( vertIter != posIter->second.end() ) {
            vertIter->second = vertIter->second + normal;
            found            = true;
          }
        }
        if ( !found ) tmpNormals[pos[i]][idx[i]] = normal;
      }
    } else {
      for ( size_t i = 0; i < 3; i++ ) {
        for ( glm::vec3::length_type c = 0; c < 3; c++ ) { normals[idx[i] * 3 + c] += normal[c]; }
      }
    }
  }

  if ( noSeams ) {  // second pass in noseams mode
    TmpPosNormals::const_iterator  posIter;
    TmpVertNormals::const_iterator vertIter;
    // iteratr the positions
    for ( posIter = tmpNormals.begin(); posIter != tmpNormals.end(); posIter++ ) {
      // iterates the vertices that share the same position and cumulate normals
      glm::vec3 normal( 0.0F, 0.0F, 0.0F );
      for ( vertIter = posIter->second.begin(); vertIter != posIter->second.end(); vertIter++ ) {
        normal += vertIter->second;
      }
      // assign normal sum to all the indices of this position
      for ( vertIter = posIter->second.begin(); vertIter != posIter->second.end(); vertIter++ ) {
        for ( glm::vec3::length_type c = 0; c < 3; c++ ) { normals[vertIter->first * 3 + c] = normal[c]; }
      }
    }
  }

  if ( normalize ) { normalizeNormals(); }
}

//
void Model::computeNeighborTriangles( bool useIndices, bool skipNonManifold ) {
  std::cout << "-> Model::computeNeighborTriangles useIndices=" << ( useIndices ? "true" : "false" )
            << ", skipNonManifold = " << ( skipNonManifold ? "true" : "false" ) << std::endl;

  clock_t t1 = clock();

  // clear map information on neigborhood
  perVertexTriangles.clear();
  perTriangleNeighborTriangles.clear();
  perTriangleEdgeNeighborTriangles.clear();
  nonManifoldTriangles.clear();
  nonManifoldVertices.clear();

  // build the map with list of triangle index for all vertex
  if ( useIndices ) {
    for ( size_t triIndex = 0; triIndex < getTriangleCount(); ++triIndex ) {
      int i[3];
      fetchTriangleIndices( triIndex, i[0], i[1], i[2] );
      for ( size_t vertIdx = 0; vertIdx < 3; ++vertIdx ) { perVertexTriangles[i[vertIdx]].insert( triIndex ); }
    }
  } else {
    std::map<Vertex, std::set<size_t>, CompareVertex<true, false, false, false>> perPositionVertexIndex;
    for ( size_t triIndex = 0; triIndex < getTriangleCount(); ++triIndex ) {
      Vertex v[3];
      fetchTriangleVertices( triIndex, v[0].pos, v[1].pos, v[2].pos );
      int i[3];
      fetchTriangleIndices( triIndex, i[0], i[1], i[2] );
      for ( size_t vertIdx = 0; vertIdx < 3; ++vertIdx ) { perPositionVertexIndex[v[vertIdx]].insert( i[vertIdx] ); }
    }
    for ( size_t triIndex = 0; triIndex < getTriangleCount(); ++triIndex ) {
      Vertex v[3];
      fetchTriangleVertices( triIndex, v[0].pos, v[1].pos, v[2].pos );

      for ( size_t vertIdx = 0; vertIdx < 3; ++vertIdx ) {
        auto& vertIndices = perPositionVertexIndex[v[vertIdx]];
        // iterates over all the indices having same position
        for ( auto iter = vertIndices.begin(); iter != vertIndices.end(); ++iter ) {
          perVertexTriangles[*iter].insert( triIndex );
        }
      }
    }
  }

  // build the map with list of neighbor triangle index with same vertex
  for ( size_t triIndex = 0; triIndex < getTriangleCount(); ++triIndex ) {
    int i[3];
    fetchTriangleIndices( triIndex, i[0], i[1], i[2] );
    for ( size_t vertIdx = 0; vertIdx < 3; ++vertIdx ) {
      auto iter = perVertexTriangles.find( i[vertIdx] );
      perTriangleNeighborTriangles[triIndex].insert( iter->second.begin(), iter->second.end() );
    }

    // remove self
    perTriangleNeighborTriangles[triIndex].erase( triIndex );
  }

  // compute non-manifold vertices
  if ( useIndices ) {
    // TODO
  } else {
    // std::cout << "generate clusters" << std::endl;
    // vertex id -> list < vertex id >
    std::map<size_t, std::list<std::set<size_t>>> allVertexClusters;
    // First create a set of clusters
    // elem->first is vertex index
    // elem->second is list of triangles
    for ( auto elem : perVertexTriangles ) {
      const auto vertIndex = elem.first;
      for ( auto triIdx : elem.second ) {
        std::set<size_t> cluster;
        int              tri[3];
        fetchTriangleIndices( triIdx, tri[0], tri[1], tri[2] );
        // create a new entry in the clusters
        for ( size_t i = 0; i < 3; ++i ) {
          if ( tri[i] != vertIndex ) cluster.insert( tri[i] );
        }
        //
        allVertexClusters[elem.first].push_back( cluster );
      }
    }

    // DEBUG
    /*
    for (auto perVertexCluster : allVertexClusters) {
            std::cout << "vertexIndex = " << perVertexCluster.first << std::endl;
            for (auto cluster : perVertexCluster.second) {
                    std::cout << "(";
                    for (auto vertex : cluster) {
                            std::cout << " " << vertex << " ";
                    }
                    std::cout << ")";
            }
            std::cout << std::endl;
    }*/

    // std::cout << "Fusion the clusters two by two" << std::endl;
    for ( auto& vertexClusters : allVertexClusters ) {
      // std::cout << "Treating vertex " << vertexClusters.first << " size = " << vertexClusters.second.size() <<
      // std::endl; from first cluster to end
      auto& clusters         = vertexClusters.second;
      auto  firstClusterIter = clusters.begin();
      while ( firstClusterIter != clusters.end() ) {
        bool found = false;
        do {
          found = false;
          // from firstClusterIter + 1 to end
          auto scndClusterIter = firstClusterIter;
          scndClusterIter++;
          while ( scndClusterIter != clusters.end() ) {
            std::set<size_t> intersection;
            auto&            firstCluster = *firstClusterIter;
            auto&            scndCluster  = *scndClusterIter;
            std::set_intersection( firstCluster.begin(),
                                   firstCluster.end(),
                                   scndCluster.begin(),
                                   scndCluster.end(),
                                   std::inserter( intersection, intersection.begin() ) );

            if ( intersection.size() != 0 ) {
              found = true;
              // std::cout << "Merging clusters" << std::endl;
              firstCluster.insert( scndCluster.begin(), scndCluster.end() );
              scndClusterIter = vertexClusters.second.erase( scndClusterIter );
              /*
              // DEBUG
              for (auto cluster : vertexClusters.second) {
                      std::cout << "(";
                      for (auto vertex : cluster) {
                              std::cout << " " << vertex << " ";
                      }
                      std::cout << ")";
              }
              std::cout << std::endl;
              */
            } else {
              scndClusterIter++;
            }
          }
        } while ( found );
        firstClusterIter++;
      }
      if ( vertexClusters.second.size() > 1 ) {
        nonManifoldVertices.insert( vertexClusters.first );
        // DEBUG
        /*
        std::cout << "Error: vertex " << vertexClusters.first << " is non-manifold: " << vertexClusters.second.size() <<
        std::endl; for (auto cluster : vertexClusters.second) { std::cout << "("; for (auto vertex : cluster) {
                        std::cout << " " << vertex << " ";
                }
                std::cout << ")";
        }
        std::cout << std::endl;
        */
      }
    }

    // DEBUG
    /*
    for (auto perVertexCluster : allVertexClusters) {
            std::cout << "vertexIndex = " << perVertexCluster.first << std::endl;
            for (auto cluster : perVertexCluster.second) {
                    std::cout << "(";
                    for (auto vertex : cluster) {
                            std::cout << " " << vertex << " ";
                    }
                    std::cout << ")";
            }
            std::cout << std::endl;
    }*/
  }
  // if (nonManifoldVertices.size() != 0)
  //	std::cout << "Error: found " << nonManifoldVertices.size() << " non manifold vertices" << std::endl;

  // compute per edge triangle neighbors and detects non-manifold edges
  if ( useIndices ) {
    // TODO
  } else {
    for ( size_t triIndex = 0; triIndex < getTriangleCount(); ++triIndex ) {
      int i[3];
      fetchTriangleIndices( triIndex, i[0], i[1], i[2] );

      auto& s0 = perVertexTriangles[i[0]];
      auto& s1 = perVertexTriangles[i[1]];
      auto& s2 = perVertexTriangles[i[2]];

      std::set<size_t> intersect1;
      std::set_intersection(
        s0.begin(), s0.end(), s1.begin(), s1.end(), std::inserter( intersect1, intersect1.begin() ) );
      intersect1.erase( triIndex );  // remove self
      if ( intersect1.size() > 1 ) {
        if ( skipNonManifold ) {
          /*
          std::cout << "Error: mesh is non-manifold " << intersect1.size() << std::endl;
          std::cout << "Triangle " << triIndex << " edge s0-s1 ( " << intersect1.size() << " ) --> ";
          for (auto elem : intersect1) std::cout << elem << " ";
          std::cout << std::endl;
          */
        } else {
          perTriangleEdgeNeighborTriangles[triIndex].insert( intersect1.begin(), intersect1.end() );
        }
        nonManifoldTriangles.insert( triIndex );
      } else if ( intersect1.size() == 1 ) {
        perTriangleEdgeNeighborTriangles[triIndex].insert( *intersect1.begin() );
      } else {
        // no neighbor by this edge
      }

      std::set<size_t> intersect2;
      std::set_intersection(
        s1.begin(), s1.end(), s2.begin(), s2.end(), std::inserter( intersect2, intersect2.begin() ) );
      intersect2.erase( triIndex );  // remove self
      if ( intersect2.size() > 1 ) {
        if ( skipNonManifold ) {
          /*
          std::cout << "Error: mesh is non-manifold " << intersect2.size() << std::endl;
          std::cout << "Triangle " << triIndex << " edge s1-s2 ( " << intersect2.size() << " ) --> ";
          for (auto elem : intersect2) std::cout << elem << " ";
          std::cout << std::endl;
          */
        } else {
          perTriangleEdgeNeighborTriangles[triIndex].insert( intersect2.begin(), intersect2.end() );
        }
        nonManifoldTriangles.insert( triIndex );
      } else if ( intersect2.size() == 1 ) {
        perTriangleEdgeNeighborTriangles[triIndex].insert( *intersect2.begin() );
      } else {
        // no neighbor by this edge
      }

      std::set<size_t> intersect3;
      std::set_intersection(
        s2.begin(), s2.end(), s0.begin(), s0.end(), std::inserter( intersect3, intersect3.begin() ) );
      intersect3.erase( triIndex );  // remove self
      if ( intersect3.size() > 1 ) {
        if ( skipNonManifold ) {
          /*
          std::cout << "Error: mesh is non-manifold " << intersect3.size() << std::endl;
          std::cout << "Triangle " << triIndex << " edge s2-s0 ( " << intersect3.size() << " ) --> ";
          for (auto elem : intersect3) std::cout << elem << " ";
          std::cout << std::endl;
          */
        } else {
          perTriangleEdgeNeighborTriangles[triIndex].insert( intersect3.begin(), intersect3.end() );
        }
        nonManifoldTriangles.insert( triIndex );
      } else if ( intersect3.size() == 1 ) {
        perTriangleEdgeNeighborTriangles[triIndex].insert( *intersect3.begin() );
      } else {
        // no neighbor by this edge
      }
    }
  }

  // debug only
  /*
  for (auto triIter = perTriangleNeighborTriangles.begin(); triIter != perTriangleNeighborTriangles.end(); ++triIter) {
          std::cout << triIter->first << ": ";
          for (auto setIter = triIter->second.begin(); setIter != triIter->second.end(); ++setIter) {
                  std::cout << " " << *setIter << std::endl;
          }
          std::cout << std::endl;
  }

  for (auto triIter = perTriangleEdgeNeighborTriangles.begin(); triIter != perTriangleEdgeNeighborTriangles.end();
  ++triIter) { std::cout << triIter->first << ": "; for (auto setIter = triIter->second.begin(); setIter !=
  triIter->second.end(); ++setIter) { std::cout << " " << *setIter << std::endl;
          }
          std::cout << std::endl;
  }
          */

  clock_t t2 = clock();
  std::cout << "<- Model::computeNeighborTriangles, time=" << ( (float)( t2 - t1 ) ) / CLOCKS_PER_SEC << " sec."
            << std::endl;
}

Model& Model::operator+=( const Model& other ) {
  ModelBuilder builder( *this );
  bool         hasUVCoords = other.hasUvCoords();
  bool         hasColors   = other.hasColors();
  bool         hasNormals  = other.hasNormals();
  for ( size_t i = 0; i < other.getTriangleCount(); i++ ) {
    Vertex v1, v2, v3;
    fetchTriangle( other, i, hasUVCoords, hasColors, hasNormals, v1, v2, v3 );
    builder.pushTriangle( v1, v2, v3, true );
  }
  return *this;
}