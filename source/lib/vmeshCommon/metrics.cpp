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
#include "util/image.hpp"
#include "util/checksum.hpp"

#include "mmSample.h"
#include "mmModel.h"
#include "mmImage.h"
#include "mmCompare.h"
#include "mmDequantize.h"
#include "mmIO.h"

using namespace vmesh;

//============================================================================

template<typename T>
void
convert(const TriangleMesh<T>& src, mm::Model& dst) {
  dst.reset();
  const auto& points = src.points();
  dst.vertices.resize(points.size() * 3);
  for (size_t i = 0; i < points.size(); i++) {
    dst.vertices[3 * i + 0] = points[i][0];
    dst.vertices[3 * i + 1] = points[i][1];
    dst.vertices[3 * i + 2] = points[i][2];
  }
  if (!src.texCoords().empty()) {
    const auto& texCoords = src.texCoords();
    dst.uvcoords.resize(texCoords.size() * 2);
    for (size_t i = 0; i < texCoords.size(); i++) {
      dst.uvcoords[2 * i + 0] = texCoords[i][0];
      dst.uvcoords[2 * i + 1] = texCoords[i][1];
    }
  }
  const auto& triangles = src.triangles();
  dst.triangles.resize(triangles.size() * 3);
  for (size_t i = 0; i < triangles.size(); i++) {
    dst.triangles[3 * i + 0] = triangles[i][0];
    dst.triangles[3 * i + 1] = triangles[i][1];
    dst.triangles[3 * i + 2] = triangles[i][2];
  }
  if (!src.texCoordTriangles().empty()) {
    const auto& texCoordsTri = src.texCoordTriangles();
    dst.trianglesuv.resize(texCoordsTri.size() * 3);
    for (size_t i = 0; i < texCoordsTri.size(); i++) {
      dst.trianglesuv[3 * i + 0] = texCoordsTri[i][0];
      dst.trianglesuv[3 * i + 1] = texCoordsTri[i][1];
      dst.trianglesuv[3 * i + 2] = texCoordsTri[i][2];
    }
  }
  if (!src.normals().empty()) {
    const auto& normals = src.normals();
    dst.normals.resize(normals.size() * 3);
    for (size_t i = 0; i < normals.size(); i++) {
      dst.normals[3 * i + 0] = normals[i][0];
      dst.normals[3 * i + 1] = normals[i][1];
      dst.normals[3 * i + 2] = normals[i][2];
    }
  }
}

//============================================================================

void
convert(const Frame<uint8_t>& src, mm::Image& dst) {
  const int32_t width  = src.width();
  const int32_t height = src.height();
  dst.reset(width, height);
  for (int32_t v = 0; v < height; v++) {
    for (int32_t u = 0; u < width; u++) {
      dst.data[(v * width + u) * 3 + 2] = src.plane(0).get(v, u);
      dst.data[(v * width + u) * 3 + 1] = src.plane(1).get(v, u);
      dst.data[(v * width + u) * 3 + 0] = src.plane(2).get(v, u);
    }
  }
}

//============================================================================

void
convert(const mm::Image& src, Frame<uint8_t>& dst) {
  const int32_t width  = src.width;
  const int32_t height = src.height;
  dst.resize(width, height, ColourSpace::BGR444p);
  for (int32_t v = 0; v < height; v++) {
    for (int32_t u = 0; u < width; u++) {
      dst.plane(0).set(v, u, src.data[(v * width + u) * 3 + 2]);
      dst.plane(1).set(v, u, src.data[(v * width + u) * 3 + 1]);
      dst.plane(2).set(v, u, src.data[(v * width + u) * 3 + 0]);
    }
  }
}

//============================================================================

void
log(const std::string& str, const mm::Image& image) {
  printf("%s: %dx%d nbc = %d \n",
         str.c_str(),
         image.width,
         image.height,
         image.nbc);
  fflush(stdout);
  for (int c = 2; c >= 0; c--) {
    for (int v = 0; v < std::min(image.height, 4096); v++) {
      printf("%4d: ", v);
      for (int u = 0; u < std::min(image.width, 4096); u++) {
        printf("%2x ", image.data[(v * image.width + u) * image.nbc + c]);
      }
      printf("\n");
    }
    printf("\n");
  }
}

//============================================================================

void
VMCMetrics::compute(const Sequence&             sequence0,
                    const Sequence&             sequence1,
                    const VMCMetricsParameters& params) {
  if (!compare) { compare = std::make_shared<mm::Compare>(); }
  for (int32_t i = 0; i < sequence0.frameCount(); i++) {
    compute(sequence0.mesh(i),
            sequence1.mesh(i),
            sequence0.texture(i),
            sequence1.texture(i),
            params);
  }
}

//============================================================================

void
VMCMetrics::compute(const std::string           srcMesh,
                    const std::string           recMesh,
                    const std::string           srcMap,
                    const std::string           recMap,
                    const VMCMetricsParameters& params) {
  if (!compare) { compare = std::make_shared<mm::Compare>(); }
  mm::Model srcModel;
  mm::Model recModel;
  mm::Image srcImage;
  mm::Image recImage;
  if (mm::IO::_loadModel(srcMesh, srcModel)
      && mm::IO::_loadModel(recMesh, recModel)
      && mm::IO::_loadImage(srcMap, srcImage)
      && mm::IO::_loadImage(recMap, recImage)) {
    compute(srcModel, recModel, srcImage, recImage, "", "", params);
  } else {
    printf("Error: can't load models and maps. \n");
    exit(-1);
  }
}

//============================================================================

template<typename T>
void
VMCMetrics::compute(const TriangleMesh<T>&      srcMesh,
                    const TriangleMesh<T>&      recMesh,
                    const Frame<uint8_t>&       srcMap,
                    const Frame<uint8_t>&       recMap,
                    const VMCMetricsParameters& params) {
  if (!compare) { compare = std::make_shared<mm::Compare>(); }
  mm::Model srcModel;
  mm::Model recModel;
  mm::Image srcImage;
  mm::Image recImage;
  convert(srcMesh, srcModel);
  convert(recMesh, recModel);
  convert(srcMap, srcImage);
  convert(recMap, recImage);
  compute(srcModel, recModel, srcImage, recImage, "", "", params);
  print(compare->size() - 1);
}

//============================================================================

template<typename T>
void
VMCMetrics::compute(const TriangleMesh<T>&      srcMesh,
                    const TriangleMesh<T>&      recMesh,
                    const VMCMetricsParameters& params) {
  if (!compare) { compare = std::make_shared<mm::Compare>(); }
  mm::Model srcModel;
  mm::Model recModel;
  mm::Image image(10, 10, 0);
  convert(srcMesh, srcModel);
  convert(recMesh, recModel);
  compute(srcModel, recModel, image, image, "", "", params);
}

//============================================================================

void
VMCMetrics::compute(const mm::Model&            srcModel,
                    const mm::Model&            recModel,
                    const mm::Image&            srcMap,
                    const mm::Image&            recMap,
                    const std::string&          srcName,
                    const std::string&          recName,
                    const VMCMetricsParameters& params) {
  mm::Model dequantize[2];
  mm::Model reindex[2];
  mm::Model sampled[2];

  const bool verbose = params.verbose;
  // Sample
  glm::vec3 minPos(
    params.minPosition[0], params.minPosition[1], params.minPosition[2]);
  glm::vec3 maxPos(
    params.maxPosition[0], params.maxPosition[1], params.maxPosition[2]);
  for (size_t i = 0; i < 2; i++) {
    const auto& model = i == 0 ? srcModel : recModel;
    if (verbose) {
      std::cout << (i == 0 ? "Src" : "Rec")
                << " model: " << (i == 0 ? srcName : recName) << std::endl;
      std::cout << "  Vertices: " << model.vertices.size() / 3 << std::endl;
      std::cout << "  UVs: " << model.uvcoords.size() / 2 << std::endl;
      std::cout << "  Colors: " << model.colors.size() / 3 << std::endl;
      std::cout << "  Normals: " << model.normals.size() / 3 << std::endl;
      std::cout << "  Triangles: " << model.triangles.size() / 3 << std::endl;
      std::cout << "  Trianglesuv: " << model.trianglesuv.size() / 3
                << std::endl;
    }
    // Dequantize
    glm::vec2 zero2(0, 0);
    glm::vec2 one2(1, 1);
    glm::vec3 zero3(0, 0, 0);

    auto qt = i == 0 || !params.dequantizeUV ? params.qt : 0;
    if (verbose) {
      std::cout << "De-quantizing" << std::endl;
      std::cout << "  qp = " << params.qp << std::endl;
      std::cout << "  qt = " << qt << std::endl;
      std::cout << "  qn = " << 0 << std::endl;
      std::cout << "  qc = " << 0 << std::endl;
    }
    mm::Dequantize::dequantize(model,          // input
                               dequantize[i],  // output
                               params.qp,      // qp
                               qt,             // qt,
                               0,              // qn,
                               0,              // qc,
                               minPos,         // minPos
                               maxPos,         // maxPos
                               zero2,          // minUv
                               one2,           // maxUv
                               zero3,          // minNrm
                               zero3,          // maxNrm
                               zero3,          // minCol
                               zero3,          // maxCol
                               true,           // useFixedPoint,
                               false,          // colorSpaceConversion
                               verbose);       // verbose

    if (params.computePcc || params.computePcqm) {
      // Reindex
      const std::string sort = "oriented";
      if (verbose) {
        std::cout << "Reindex" << std::endl;
        std::cout << "  sort = " << sort << std::endl;
      }
      if (sort == "none") {
        mm::reindex(dequantize[i], reindex[i]);
      } else if (sort == "vertex" || sort == "oriented"
                 || sort == "unoriented") {
        mm::reorder(dequantize[i], sort, reindex[i]);
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
      if (verbose) {
        std::cout << "Sampling in GRID mode" << std::endl;
        std::cout << "  Grid Size = " << params.gridSize << std::endl;
        std::cout << "  Use Normal = " << useNormal << std::endl;
        std::cout << "  Bilinear = " << bilinear << std::endl;
        std::cout << "  hideProgress = " << hideProgress << std::endl;
        std::cout << "  nbSamplesMin = " << nbSamplesMin << std::endl;
        std::cout << "  nbSamplesMax = " << nbSamplesMax << std::endl;
        std::cout << "  maxIterations = " << maxIterations << std::endl;
        std::cout << "  using contrained mode with gridSize " << std::endl;
      }
      mm::Sample::meshToPcGrid(reindex[i],                // input
                               sampled[i],                // output
                               i == 0 ? srcMap : recMap,  // map
                               params.gridSize,           // grid res
                               bilinear,                  // bilinear
                               !hideProgress,             // lowprogress
                               useNormal,                 // use normal
                               useFixedPoint,             // useFixedPoint
                               minPos,                    // minPos
                               maxPos,                    // maxPos
                               verbose);                  // verbose
    }
  }

  // PCC
  mm::Model               outPcc[2];
  pcc_quality::commandPar pccParams;
  glm::vec3               dim = maxPos - minPos;
  pccParams.singlePass        = false;
  pccParams.hausdorff         = false;
  pccParams.bColor            = true;
  pccParams.bLidar            = false;  // always false, no option
  pccParams.resolution      = std::max(dim.x, std::max(dim.y, dim.z));  // auto
  pccParams.neighborsProc   = 1;
  pccParams.dropDuplicates  = 2;
  pccParams.bAverageNormals = true;
  pccParams.normalCalcModificationEnable = params.normalCalcModificationEnable;

  if (params.computePcc) {
    if (verbose) {
      std::cout << "Compare models using MPEG PCC distortion metric"
                << std::endl;
      std::cout << "  singlePass     = " << pccParams.singlePass << std::endl;
      std::cout << "  hausdorff      = " << pccParams.hausdorff << std::endl;
      std::cout << "  color          = " << pccParams.bColor << std::endl;
      std::cout << "  resolution     = " << pccParams.resolution << std::endl;
      std::cout << "  neighborsProc  = " << pccParams.neighborsProc
                << std::endl;
      std::cout << "  dropDuplicates = " << pccParams.dropDuplicates
                << std::endl;
      std::cout << "  averageNormals = " << pccParams.bAverageNormals
                << std::endl;
    }
    compare->pcc(sampled[0],  // modelA
                 sampled[1],  // modelB
                 srcMap,      // mapA
                 recMap,      // mapB
                 pccParams,   // params
                 outPcc[0],   // outputA
                 outPcc[1],   // outputB
                 verbose);    // verbose
  }

  // PCQM
  if (params.computePcqm) {
    mm::Model outPcqm[2];
    compare->pcqm(sampled[0],                     // modelA
                  sampled[1],                     // modelB
                  srcMap,                         // mapA
                  recMap,                         // mapB
                  params.pcqmRadiusCurvature,     // radiusCurvature
                  params.pcqmThresholdKnnSearch,  // thresholdKnnSearch
                  params.pcqmRadiusFactor,        // radiusFactor
                  outPcqm[0],                     // outputA
                  outPcqm[1],                     // outputB
                  verbose);                       // verbose
  }

  // IBSM
  if (params.computeIbsm) {
    mm::Model          outIbsm[2];
    const unsigned int ibsmResolution   = 2048;
    const unsigned int ibsmCameraCount  = 16;
    const glm::vec3    ibsmCamRotParams = {0.0F, 0.0F, 0.0F};
    const std::string  ibsmRenderer     = "sw_raster";
    const std::string  ibsmOutputPrefix;
    const bool         ibsmDisableReordering = false;
    const bool         ibsmDisableCulling    = false;
    if (verbose) {
      std::cout << "Compare models using IBSM distortion metric" << std::endl;
      std::cout << "  ibsmRenderer =     " << ibsmRenderer << std::endl;
      std::cout << "  ibsmCameraCount    = " << ibsmCameraCount << std::endl;
      std::cout << "  ibsmCameraRotation = " << ibsmCamRotParams[0] << " "
                << ibsmCamRotParams[1] << " " << ibsmCamRotParams[2] << " "
                << std::endl;
      std::cout << "  ibsmResolution     = " << ibsmResolution << std::endl;
      std::cout << "  ibsmDisableCulling = " << ibsmDisableCulling
                << std::endl;
      std::cout << "  ibsmOutputPrefix   = "
                << "" << std::endl;
    }
    compare->ibsm(dequantize[0],          // modelA
                  dequantize[1],          // modelB
                  srcMap,                 // mapA
                  recMap,                 // mapB
                  ibsmDisableReordering,  // disableReordering
                  ibsmResolution,         // resolution
                  ibsmCameraCount,        // cameraCount
                  ibsmCamRotParams,       // camRotParams
                  ibsmRenderer,           // renderer
                  ibsmOutputPrefix,       // outputPrefix
                  ibsmDisableCulling,     // disableCulling
                  outIbsm[0],             // outputA
                  outIbsm[1],             // outputB
                  verbose);               // verbose
  }
}

//============================================================================

// void
// print(double value) {
//   if (value > 999.98) std::cout << "999.99 ";
//   else std::cout << value << ' ';
// }
void
VMCMetrics::print(int32_t frameIndex) {
  auto print = [](const std::vector<double> pcc,
                  const std::vector<double> ibsm,
                  const std::string         str) {
    std::cout << std::left << std::setw(25) << str << ": ";
    std::cout << std::right << std::setw(12) << pcc[0] << ' ';
    std::cout << std::right << std::setw(12) << pcc[1] << ' ';
    std::cout << std::right << std::setw(12) << pcc[2] << ' ';
    std::cout << std::right << std::setw(12) << pcc[3] << ' ';
    std::cout << std::right << std::setw(12) << pcc[4] << ' ';
    std::cout << std::right << std::setw(12) << ibsm[6] << ' ';
    std::cout << std::right << std::setw(12) << ibsm[2] << ' ';
    std::cout << std::left << '\n';
  };
  if (frameIndex >= 0) {
    print(compare->getPccResults(frameIndex),
          compare->getIbsmResults(frameIndex),
          "Metric_frame_" + std::to_string(frameIndex));
  } else {
    print(compare->getFinalPccResults(),
          compare->getFinalIbsmResults(),
          "Metric_results");
  }
}

void
VMCMetrics::display(bool verbose) {
  if (verbose) {
    printf("Metrics results :\n");
    printf("PCC:\n");
    compare->pccFinalize();
    printf("PCQM:\n");
    compare->pcqmFinalize();
    printf("IBSM:\n");
    compare->ibsmFinalize();
  } else {
    for (size_t i = 0; i < compare->size(); i++) { print(i); }
    print();
  }
}

//============================================================================

std::vector<double>
VMCMetrics::getPccResults() {
  return compare->getFinalPccResults();
}

//============================================================================

std::vector<double>
VMCMetrics::getPcqmResults() {
  return compare->getFinalPcqmResults();
}

//============================================================================

std::vector<double>
VMCMetrics::getIbsmResults() {
  return compare->getFinalIbsmResults();
}

//============================================================================

template void convert<float>(const TriangleMesh<float>&, mm::Model&);
template void convert<double>(const TriangleMesh<double>&, mm::Model&);

template void VMCMetrics::compute<float>(const TriangleMesh<float>&,
                                         const TriangleMesh<float>&,
                                         const Frame<uint8_t>&,
                                         const Frame<uint8_t>&,
                                         const VMCMetricsParameters&);

template void VMCMetrics::compute<double>(const TriangleMesh<double>&,
                                          const TriangleMesh<double>&,
                                          const Frame<uint8_t>&,
                                          const Frame<uint8_t>&,
                                          const VMCMetricsParameters&);

template void VMCMetrics::compute<float>(const TriangleMesh<float>&,
                                         const TriangleMesh<float>&,
                                         const VMCMetricsParameters&);

template void VMCMetrics::compute<double>(const TriangleMesh<double>&,
                                          const TriangleMesh<double>&,
                                          const VMCMetricsParameters&);

//============================================================================