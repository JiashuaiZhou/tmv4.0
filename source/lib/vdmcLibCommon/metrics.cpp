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
#include "vmc.hpp"
#include "image.hpp"

#include "mmSample.h"
#include "mmModel.h"
#include "mmImage.h"
#include "mmCompare.h"

using namespace std;
using namespace vmesh;

void
convert(const vmesh::TriangleMesh<double>& src, mm::Model& dst)
{
  mm::ModelBuilder builder(dst);
  mm::Vertex v[3];
  for (int32_t p = 0; p < 3; p++) {
    v[p].hasColor = false;
    v[p].hasUVCoord = true;
    v[p].hasNormal = false;
  }
  for (int32_t t = 0; t < src.triangleCount(); t++) {
    const auto& triCoord = src.triangle(t);
    const auto& texCoord = src.texCoordTriangle(t);
    for (int32_t p = 0; p < 3; p++) {
      auto& pos = src.point(triCoord[p]);
      auto& tex = src.texCoord(texCoord[p]);
      v[p].pos = glm::vec3(pos[0], pos[1], pos[2]);
      v[p].uv = glm::vec2(tex[0], tex[1]);
    }
    builder.pushTriangle(v[0], v[1], v[2]);
  }
}

void
convert(
  const vmesh::Frame<uint8_t, vmesh::ColourSpace::BGR444p>& src,
  mm::Image& dst)
{
  const int32_t width = src.width();
  const int32_t height = src.height();
  dst.reset(width, height);
  for (int32_t v = 0; v < height; v++) {
    for (int32_t u = 0; u < width; u++) {
      dst.data[(v * width + u) * 3 + 0] = src.plane(0).get(u, v);
      dst.data[(v * width + u) * 3 + 1] = src.plane(1).get(u, v);
      dst.data[(v * width + u) * 3 + 2] = src.plane(2).get(u, v);
    }
  }
}

void
VMCMetrics::compute(
  const vmesh::VMCGroupOfFrames& gof, const VMCMetricsParameters& params)
{
  if (!compare) {
    compare = std::make_shared<mm::Compare>();
  }
  for (int32_t i = 0; i < gof.frameCount(); i++) {
    mm::Model srcModel, recModel;
    mm::Image srcImage, recImage;

    auto& frame = gof.frame(i);
    convert(frame.input, srcModel);
    convert(frame.rec, recModel);

    convert(frame.inputTexture, srcImage);
    convert(frame.outputTexture, recImage);

    compute(
      srcModel,                    // Scr object
      recModel,                    // Rec object
      srcImage,                    // Scr map
      recImage,                    // Rec map
      "src " + std::to_string(i),  // Scr name
      "rec " + std::to_string(i),  // Rec name
      params);                     // params
  }
  display();
}

void
VMCMetrics::compute(
  const mm::Model& srcModel,
  const mm::Model& recModel,
  const mm::Image& srcMap,
  const mm::Image& recMap,
  const std::string& srcName,
  const std::string& recName,
  const VMCMetricsParameters& params)
{
  mm::Model reindex[2];
  mm::Model sampled[2];

  // Sample
  glm::vec3 minPos(
    params.minPosition[0], params.minPosition[1], params.minPosition[2]);
  glm::vec3 maxPos(
    params.maxPosition[0], params.maxPosition[1], params.maxPosition[2]);
  for (size_t i = 0; i < 2; i++) {
    auto& model = i == 0 ? srcModel : recModel;
    std::cout << (i == 0 ? "Src" : "Rec")
              << "model: " << (i == 0 ? srcName : recName) << std::endl;
    std::cout << "  Vertices: " << model.vertices.size() / 3 << std::endl;
    std::cout << "  UVs: " << model.uvcoords.size() / 2 << std::endl;
    std::cout << "  Colors: " << model.colors.size() / 3 << std::endl;
    std::cout << "  Normals: " << model.normals.size() / 3 << std::endl;
    std::cout << "  Triangles: " << model.triangles.size() / 3 << std::endl;
    std::cout << "  Trianglesuv: " << model.trianglesuv.size() / 3
              << std::endl;

    // Reindex
    const std::string sort = "oriented";
    std::cout << "Reindex" << std::endl;
    std::cout << "  sort = " << sort << std::endl;
    if (sort == "none") {
      mm::reindex(model, reindex[i]);
    } else if (
      sort == "vertex" || sort == "oriented" || sort == "unoriented") {
      mm::reorder(model, sort, reindex[i]);
    } else {
      std::cout << "Error: invalid sorting method " << sort << std::endl;
    }

    const bool useNormal = true;
    const bool bilinear = true;
    const bool hideProgress = true;
    const size_t nbSamplesMin = 0;
    const size_t nbSamplesMax = 0;
    const size_t maxIterations = 10;
    const bool useFixedPoint = true;
    std::cout << "Sampling in GRID mode" << std::endl;
    std::cout << "  Grid Size = " << params.gridSize << std::endl;
    std::cout << "  Use Normal = " << useNormal << std::endl;
    std::cout << "  Bilinear = " << bilinear << std::endl;
    std::cout << "  hideProgress = " << hideProgress << std::endl;
    std::cout << "  nbSamplesMin = " << nbSamplesMin << std::endl;
    std::cout << "  nbSamplesMax = " << nbSamplesMax << std::endl;
    std::cout << "  maxIterations = " << maxIterations << std::endl;
    std::cout << "  useFixedPoint = " << useFixedPoint << std::endl;
    std::cout << "  using contrained mode with gridSize " << std::endl;
    mm::Sample::meshToPcGrid(
      reindex[i],                // input
      sampled[i],                // output
      i == 0 ? srcMap : recMap,  // map
      params.gridSize,           // grid res
      bilinear,                  // bilinear
      !hideProgress,             // lowprogress
      useNormal,                 // use normal
      useFixedPoint,             // useFixedPoint
      minPos,                    // minPos
      maxPos);                   // maxPos
  }
  // if ( !srcName.empty() ) { mm::IO::_saveObj( srcName, sampled[0] ); }
  // if ( !recName.empty() ) { mm::IO::_saveObj( recName, sampled[1] ); }

  // PCC
  mm::Model outPcc[2];
  pcc_quality::commandPar pccParams;
  glm::vec3 dim = maxPos - minPos;
  pccParams.singlePass = false;
  pccParams.hausdorff = false;
  pccParams.bColor = true;
  pccParams.bLidar = false;  // always false, no option
  pccParams.resolution = std::max(dim.x, std::max(dim.y, dim.z));  // auto
  pccParams.neighborsProc = 1;
  pccParams.dropDuplicates = 2;
  pccParams.bAverageNormals = true;

  std::cout << "Compare models using MPEG PCC distortion metric" << std::endl;
  std::cout << "  singlePass = " << pccParams.singlePass << std::endl;
  std::cout << "  hausdorff = " << pccParams.hausdorff << std::endl;
  std::cout << "  color = " << pccParams.bColor << std::endl;
  std::cout << "  resolution = " << pccParams.resolution << std::endl;
  std::cout << "  neighborsProc = " << pccParams.neighborsProc << std::endl;
  std::cout << "  dropDuplicates = " << pccParams.dropDuplicates << std::endl;
  std::cout << "  averageNormals = " << pccParams.bAverageNormals << std::endl;

  printf(
    "Triangle numbers = %9zu / %9zu \n", sampled[0].getTriangleCount(),
    sampled[1].getTriangleCount());

  // if ( !srcName.empty() ) { mm::IO::_savePly( srcName, sampled[0] ); }
  // if ( !recName.empty() ) { mm::IO::_savePly( recName, sampled[1] ); }

  compare->pcc(
    sampled[0],  // modelA
    sampled[1],  // modelB
    srcMap,      // mapA
    recMap,      // mapB
    pccParams,   // params
    outPcc[0],   // outputA
    outPcc[1]);  // outputB

  // PCQM
  mm::Model outPcqm[2];
  compare->pcqm(
    sampled[0],                     // modelA
    sampled[1],                     // modelB
    srcMap,                         // mapA
    recMap,                         // mapB
    params.pcqmRadiusCurvature,     // radiusCurvature
    params.pcqmThresholdKnnSearch,  // thresholdKnnSearch
    params.pcqmRadiusFactor,        // radiusFactor
    outPcqm[0],                     // outputA
    outPcqm[1]);                    // outputB

  // IBSM
  mm::Model outIbsm[2];
  const unsigned int ibsmResolution = 2048;
  const unsigned int ibsmCameraCount = 16;
  const glm::vec3 ibsmCamRotParams = {0.0F, 0.0F, 0.0F};
  const std::string ibsmRenderer = "gl12_ibsm";
  const std::string ibsmOutputPrefix = "";
  const bool ibsmDisableReordering = false;
  const bool ibsmDisableCulling = false;

  compare->ibsm(
    srcModel,               // modelA
    recModel,               // modelB
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
    outIbsm[1]);            // outputB
}

void
VMCMetrics::display()
{
  printf("Metrics results :\n");
  printf("PCC:\n");
  compare->pccFinalize();
  printf("PCQM:\n");
  compare->pcqmFinalize();
  printf("IBSM:\n");
  compare->ibsmFinalize();
}
