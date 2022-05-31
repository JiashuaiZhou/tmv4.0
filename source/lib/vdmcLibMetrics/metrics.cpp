/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "metrics.hpp"
#include "mmCompare.h"
#include "mmSample.h"
#include "mmIO.h"

using namespace std;
using namespace vmeshmet;

PCCMeshMetrics::PCCMeshMetrics() {}
PCCMeshMetrics::~PCCMeshMetrics() = default;
void PCCMeshMetrics::setParameters( const PCCMeshMetricsParameters& params ) { params_ = params; }

void PCCMeshMetrics::compute( const PCCGroupOfFrames& sources,
                              const PCCGroupOfFrames& reconstructs,
                              const std::string&      srcName,
                              const std::string&      recName ) {
  if ( ( sources.getFrameCount() != reconstructs.getFrameCount() ) ) {
    printf( "Error: group of frames must have same numbers of frames. ( src = %zu rec = %zu ) \n",
            sources.getFrameCount(), reconstructs.getFrameCount() );
    exit( -1 );
  }
  for ( size_t i = 0; i < sources.getFrameCount(); i++ ) {
    if ( !sources[i].isMesh() || !reconstructs[i].isMesh() ) {
      printf( "Error: group of frames is not mesh \n" );
      exit( -1 );
    }
  }

  for ( size_t i = 0; i < sources.getFrameCount(); i++ ) {
    compute( sources[i].getModel(),                                        // Scr object
             reconstructs[i].getModel(),                                   // Rec object
             sources[i].getMap(),                                          // Scr map
             reconstructs[i].getMap(),                                     // Rec map
             srcName.empty() ? "" : stringFormat( srcName.c_str(), i ),    // Scr name
             recName.empty() ? "" : stringFormat( recName.c_str(), i ) );  // Rec map
  }
  display();
}

void PCCMeshMetrics::compute( const mm::Model& srcModel,
                              const mm::Model& recModel,
                              const mm::Image& srcMap,
                              const mm::Image& recMap, 
                              const std::string& srcName, 
                              const std::string& recName ) {
  mm::Model reindex[2];
  mm::Model sampled[2];

  // Sample
  glm::vec3 minPos( params_.minPosition_[0], params_.minPosition_[1], params_.minPosition_[2] );
  glm::vec3 maxPos( params_.maxPosition_[0], params_.maxPosition_[1], params_.maxPosition_[2] );
  for ( size_t i = 0; i < 2; i++ ) {
    auto& model =  i == 0 ? srcModel : recModel;
    printf( "Input model: \n");
    std::cout << "  Vertices: " << model.vertices.size() / 3 << std::endl;
    std::cout << "  UVs: " << model.uvcoords.size() / 2 << std::endl;
    std::cout << "  Colors: " << model.colors.size() / 3 << std::endl;
    std::cout << "  Normals: " << model.normals.size() / 3 << std::endl;
    std::cout << "  Triangles: " << model.triangles.size() / 3 << std::endl;
    std::cout << "  Trianglesuv: " << model.trianglesuv.size() / 3 << std::endl;

    // Reindex
    const std::string sort = "oriented";
    std::cout << "Reindex" << std::endl;
    std::cout << "  sort = " << sort << std::endl;
    if ( sort == "none" ) {
      mm::reindex( model, reindex[i] );
    } else if ( sort == "vertex" || sort == "oriented" || sort == "unoriented" ) {
      mm::reorder( model, sort, reindex[i] );
    } else {
      std::cout << "Error: invalid sorting method " << sort << std::endl;
    }

    const bool   useNormal     = true;
    const bool   bilinear      = true;
    const bool   hideProgress  = true;
    const size_t nbSamplesMin  = 0;
    const size_t nbSamplesMax  = 0;
    const size_t maxIterations = 10;
    const bool   useFixedPoint = true;
    std::cout << "Sampling in GRID mode" << std::endl;
    std::cout << "  Grid Size = " << params_.gridSize_ << std::endl;
    std::cout << "  Use Normal = " << useNormal << std::endl;
    std::cout << "  Bilinear = " << bilinear << std::endl;
    std::cout << "  hideProgress = " << hideProgress << std::endl;
    std::cout << "  nbSamplesMin = " << nbSamplesMin << std::endl;
    std::cout << "  nbSamplesMax = " << nbSamplesMax << std::endl;
    std::cout << "  maxIterations = " << maxIterations << std::endl;
    std::cout << "  useFixedPoint = " << useFixedPoint << std::endl;
    std::cout << "  using contrained mode with gridSize " << std::endl;
    mm::Sample::meshToPcGrid( reindex[i],                // input
                              sampled[i],                // output
                              i == 0 ? srcMap : recMap,  // map
                              params_.gridSize_,         // grid res
                              bilinear,                  // bilinear
                              !hideProgress,             // lowprogress
                              useNormal,                 // use normal
                              useFixedPoint,             // useFixedPoint
                              minPos,                    // minPos
                              maxPos );                  // maxPos
  }
  // if ( !srcName.empty() ) { mm::IO::_saveObj( srcName, sampled[0] ); }
  // if ( !recName.empty() ) { mm::IO::_saveObj( recName, sampled[1] ); }

  // PCC
  mm::Model               outPcc[2];
  pcc_quality::commandPar pccParams;
  glm::vec3               dim = maxPos - minPos;
  pccParams.singlePass        = false;
  pccParams.hausdorff         = false;
  pccParams.bColor            = true;
  pccParams.bLidar            = false;                                        // always false, no option
  pccParams.resolution        = std::max( dim.x, std::max( dim.y, dim.z ) );  // auto
  pccParams.neighborsProc     = 1;
  pccParams.dropDuplicates    = 2;
  pccParams.bAverageNormals   = true;
  
  std::cout << "Compare models using MPEG PCC distortion metric" << std::endl;
  std::cout << "  singlePass = " << pccParams.singlePass << std::endl;
  std::cout << "  hausdorff = " << pccParams.hausdorff << std::endl;
  std::cout << "  color = " << pccParams.bColor << std::endl;
  std::cout << "  resolution = " << pccParams.resolution << std::endl;
  std::cout << "  neighborsProc = " << pccParams.neighborsProc << std::endl;
  std::cout << "  dropDuplicates = " << pccParams.dropDuplicates << std::endl;
  std::cout << "  averageNormals = " << pccParams.bAverageNormals << std::endl;

  printf("Triangle numbers = %9zu \n",sampled[0].getTriangleCount() ,sampled[1].getTriangleCount() );

  if ( !srcName.empty() ) { mm::IO::_savePly( srcName, sampled[0] ); }
  if ( !recName.empty() ) { mm::IO::_savePly( recName, sampled[1] ); }

  compare_.pcc( sampled[0],   // modelA
                sampled[1],   // modelB
                srcMap,       // mapA
                recMap,       // mapB
                pccParams,    // params
                outPcc[0],    // outputA
                outPcc[1] );  // outputB

  // // PCQM
  // mm::Model outPcqm[2];
  // compare_.pcqm( sampled[0],                       // modelA
  //                sampled[1],                       // modelB
  //                srcMap,                           // mapA
  //                recMap,                           // mapB
  //                params_.pcqmRadiusCurvature_,     // radiusCurvature
  //                params_.pcqmThresholdKnnSearch_,  // thresholdKnnSearch
  //                params_.pcqmRadiusFactor_,         // radiusFactor
  //                outPcqm[0],                       // outputA
  //                outPcqm[1] );                     // outputB

  // // IBSM
  // mm::Model          outIbsm[2];
  // const unsigned int ibsmResolution        = 2048;
  // const unsigned int ibsmCameraCount       = 16;
  // const glm::vec3    ibsmCamRotParams      = { 0.0F, 0.0F, 0.0F };
  // const std::string  ibsmRenderer          = "gl12_ibsm";
  // const std::string  ibsmOutputPrefix      = "";
  // const bool         ibsmDisableReordering = false;
  // const bool         ibsmDisableCulling    = false;

  // compare_.ibsm( srcModel,               // modelA
  //                recModel,               // modelB
  //                srcMap,                 // mapA
  //                recMap,                 // mapB
  //                ibsmDisableReordering,  // disableReordering
  //                ibsmResolution,         // resolution
  //                ibsmCameraCount,        // cameraCount
  //                ibsmCamRotParams,       // camRotParams
  //                ibsmRenderer,           // renderer
  //                ibsmOutputPrefix,       // outputPrefix
  //                ibsmDisableCulling,     // disableCulling
  //                outIbsm[0],             // outputA
  //                outIbsm[1] );           // outputB
}

void PCCMeshMetrics::display() {
  printf( "Metrics results :\n" );
  printf( "PCC:\n" );
  compare_.pccFinalize();
  printf( "PCQM:\n" );
  compare_.pcqmFinalize();
  printf( "IBSM:\n" );
  compare_.ibsmFinalize();
}
