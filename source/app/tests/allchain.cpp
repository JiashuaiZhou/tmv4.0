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
#include <chrono>
#include <iostream>

#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "encoder.hpp"

#include <gtest/gtest.h>
#include "common.hpp"

#include "util/misc.hpp"
#include "util/verbose.hpp"
#include "version.hpp"
#include "vmc.hpp"
#include "vmcStats.hpp"
#include "sequenceInfo.hpp"
#include "util/mesh.hpp"
#include "util/image.hpp"

//============================================================================

int32_t
loadGroupOfFrames(
  const vmesh::VMCGroupOfFramesInfo& gofInfo,
  vmesh::VMCGroupOfFrames& gof,
  const std::string& srcPath,
  const std::string& texPath)
{
  const auto startFrame = gofInfo.startFrameIndex_;
  const auto frameCount = gofInfo.frameCount_;
  const auto lastFrame = startFrame + frameCount - 1;
  std::cout << "Loading group of frames (" << startFrame << '-' << lastFrame
       << ") ";
  gof.resize(frameCount);
  for (int f = startFrame; f <= lastFrame; ++f) {
    const auto nameInputTexture = vmesh::expandNum(texPath, f);
    const auto findex = f - startFrame;
    auto& frame = gof.frames[findex];
    std::cout << '.' << std::flush;
    if (
      !frame.input.loadFromOBJ(srcPath, f)
      || !LoadImage(nameInputTexture, frame.inputTexture)) {
      printf("Error loading frame %d / %d \n", f, frameCount);
      return -1;
    }
  }
  return 0;
} 

//============================================================================

int32_t
saveGroupOfFrames(
  const vmesh::VMCGroupOfFramesInfo& gofInfo,
  vmesh::VMCGroupOfFrames& gof,
  const std::string& objPath,
  const std::string& texPath,
  const std::string& mtlPath)
{
  for (int f = 0; f < gofInfo.frameCount_; ++f) {
    const auto n = gofInfo.startFrameIndex_ + f;
    auto strObj = vmesh::expandNum(objPath, n);
    auto strTex = vmesh::expandNum(texPath, n);
    auto strMat = vmesh::expandNum(mtlPath, n);
    SaveImage(strTex, gof[f].outputTexture);
    vmesh::Material<double> material;
    material.texture = vmesh::basename(strTex);
    material.save(strMat);
    gof[f].rec.setMaterialLibrary(strMat);
    gof[f].rec.saveToOBJ(strObj);
  }
  return 0;
}

//============================================================================

void
performAllChain(
  const std::string srcPath,
  const std::string texPath,
  const int frameCount,
  const int startFrame,
  const int groupOfFramesMaxSize )
{
  disableSubProcessLog.disable();
  // Parameters 
  const double framerate= 30.;
  vmesh::VMCEncoderParameters params;

  // Simplify
  params.targetTriangleRatio = 0.03;
  params.minCCTriangleCount = 8;
  params.texCoordQuantizationBits = 13;

  // Reparametrisation
  params.width = 2048;
  params.height = 2048;
  params.uvOptions = DirectX::UVATLAS_GEODESIC_QUALITY;
  params.gutter = 32;
  
  // Fit sub div
  params.geometrySamplingSubdivisionIterationCount = 3;
  params.geometryFittingIterationCount = 16;
  params.geometrySmoothingCoeffcient = 0.25;
  params.geometrySmoothingCoeffcientDecayRatio = 0.75;
  params.geometryMissedVerticesSmoothingCoeffcient = 0.1;
  params.geometryMissedVerticesSmoothingIterationCount = 10;
  params.smoothDeformUpdateNormals = true;
  params.smoothDeformTriangleNormalFlipThreshold = -0.5;
  params.fitSubdivisionSurface = true;
  params.smoothingDeformSmoothMotion = false;
  params.subdivisionIterationCount = 3;
  params.applySmoothingDeform = true;
  params.initialDeformForceNormalDisplacement = false;
  params.applyVertexUnification = true;
  params.initialDeformNormalDeviationThreshold = -2.0;
  params.initialDeformNNCount = 1;
  params.smoothDeformTriangleNormalFlipThreshold = -2.0;
  params.smoothingDeformUseInitialGeometry = true;
  params.smoothingDeformSmoothMotion = false;
  params.smoothDeformSmoothingMethod = vmesh::SmoothingMethod::VERTEX_CONSTRAINT;

  // Encoder
  params.textureVideoHDRToolDecConfig = "cfg/hdrconvert/yuv420tobgr444.cfg";   
  params.keepIntermediateFiles = 1;  
  params.qpPosition = 11;
  params.qpTexCoord = 10;
  params.liftingQuantizationParameters[0] = 39;
  params.liftingQuantizationParameters[1] = 51;
  params.liftingQuantizationParameters[2] = 51;
  params.liftingQuantizationBias[0] = 0.3333;
  params.liftingQuantizationBias[1] = 0.333333;
  params.liftingQuantizationBias[2] = 0.333333;
  params.textureVideoQP = 38;
  params.bitDepthPosition = 12;
  params.bitDepthTexCoord = 13;
  params.invertOrientation = false;
  params.unifyVertices = false;
  params.encodeDisplacementsVideo = true;
  params.encodeTextureVideo = true;
  params.normalizeUV = false;
  params.geometryVideoEncoderConfig = "cfg/hm/ctc-hm-displacements-map-ai-main10.cfg";
  params.textureVideoEncoderConfig = "cfg/hm/ctc-hm-texture-ai.cfg";
  params.textureVideoHDRToolEncConfig ="cfg/hdrconvert/bgr444toyuv420.cfg";
  params.textureVideoHDRToolDecConfig ="cfg/hdrconvert/yuv420tobgr444.cfg";

  // //////////////////////////////////////////////////////////////////
  // /////////////////// ORIGINAL APPLICATIONS ////////////////////////
  // //////////////////////////////////////////////////////////////////
  std::string gofOswPath = "osw_gof.gof";
  std::string mappedOswPath = "osw_fr%04d_mapped.obj";
  std::string referenceOswPath = "osw_fr%04d_reference.obj";
  std::string decimateOswPath = "osw_fr%04d_decimate.obj";
  std::string decimateTextureOswPath = "osw_fr%04d_decimateTexture.obj";
  std::string baseOswPath = "osw_fr%04d_base.obj";  
  std::string subdivOswPath = "osw_fr%04d_subdiv.obj";
  std::string nsubdivOswPath = "osw_fr%04d_nsubdiv.obj";  
  std::string binOswPath = "osw_bistream.vdmc";
  std::string objOswPath = "osw_rec_fr%04d.obj";  
  std::string texOswPath = "osw_rec_fr%04d.png";
  std::string mtlPath = "rec_fr%04d.mtl";

  // Generate gof structure old application
  std::stringstream cmd;
  cmd << g_gengofOldPath << " "
      << "  --input=" << srcPath   << " "
      << "  --output=" << gofOswPath << " "
      << "  --startFrame=" << startFrame << " "  
      << "  --frameCount=" << frameCount;
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());  

  // Simplify
  cmd.str("");  
  cmd << g_simplifyOldPath << " "
      << "  --input=" << srcPath << " "
      << "  --decimated=" << decimateOswPath << " "
      << "  --mapped=" << mappedOswPath << " "
      << "  --reference=" << referenceOswPath << " "
      << "  --target=" << params.targetTriangleRatio << " "
      << "  --qt=" << params.texCoordQuantizationBits << " "
      << "  --cctcount=" << params.minCCTriangleCount << " ";
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Texture parametrization old application
  cmd.str("");  
  cmd << g_uvatlasOldPath << " "
      << "  --input=" << decimateOswPath << " "
      << "  --output=" << decimateTextureOswPath << " "
      << "  --width=" << params.width << " "
      << "  --height=" << params.height << " "
      << "  --gutter=" << params.gutter << " "
      << "  --quality=QUALITY ";
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Fitsubdiv
  cmd.str("");  
  /* clang-format off */
  cmd << g_fitsubdivOldPath 
      << "  --target=" << referenceOswPath
      << "  --source=" << decimateTextureOswPath 
      << "  --mapped=" << mappedOswPath
      << "  --base=" << baseOswPath 
      << "  --subdiv=" << subdivOswPath
      << "  --nsubdiv=" << nsubdivOswPath
      << "  --it=" << params.subdivisionIterationCount
      << "  --sdeform=" << params.applySmoothingDeform
      << "  --forceNormalDisp=" << params.initialDeformForceNormalDisplacement
      << "  --unifyVertices=" << params.applyVertexUnification
      << "  --deformNormalThres="
      << params.initialDeformNormalDeviationThreshold
      << "  --deformNNCount=" << params.initialDeformNNCount
      << "  --deformFlipThres="
      << params.smoothDeformTriangleNormalFlipThreshold
      << "  --useInitialGeom=" << params.smoothingDeformUseInitialGeometry
      << "  --smoothMotion=" << params.smoothingDeformSmoothMotion
      << "  --smoothMethod=" << (int)params.smoothDeformSmoothingMethod;
  /* clang-format on */
  if (disableSubProcessLog.disableLog())
    cmd << " 2>&1 > /dev/null";
  printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // Encode old application
  cmd.str("");  
  /* clang-format off */
  cmd << g_encoderOldPath << "  "
      << "  --mode="         << 0
      << "  --imesh="        << srcPath
      << "  --itex="         << texPath
      << "  --base="         << baseOswPath
      << "  --subdiv="       << subdivOswPath
      << "  --compressed="   << binOswPath
      << "  --recmesh="      << objOswPath
      << "  --rectex="       << texOswPath
      << "  --recmat="       << mtlPath
      << "  --fstart="       << startFrame
      << "  --fcount="       << frameCount
      << "  --framerate="    << framerate
      << "  --keep="         << params.keepIntermediateFiles
      << "  --it="           << params.subdivisionIterationCount
      << "  --gqp="          << params.qpPosition
      << "  --tqp="          << params.qpTexCoord
      << "  --dqp=\""        << params.liftingQuantizationParameters[0] << "," 
                             << params.liftingQuantizationParameters[1] << ","
                             << params.liftingQuantizationParameters[2] << "\""
      << "  --dqb=\""        << params.liftingQuantizationBias[0] << "," 
                             << params.liftingQuantizationBias[1] << ","
                             << params.liftingQuantizationBias[2] << "\""
      << "  --tvqp="         << params.textureVideoQP
      << "  --gdepth="       << params.bitDepthPosition
      << "  --tdepth="       << params.bitDepthTexCoord
      << "  --texwidth="     << params.width
      << "  --texheight="    << params.height
      << "  --invorient="    << params.invertOrientation
      << "  --unifvertices=" << params.unifyVertices
      << "  --encdisp="      << params.encodeDisplacementsVideo
      << "  --enctex="       << params.encodeTextureVideo
      << "  --normuv="       << params.normalizeUV
      << "  --gmenc="        << g_dracoEncoderPath
      << "  --gmdec="        << g_dracoDecoderPath
      << "  --gvenc="        << g_hmEncoderPath
      << "  --gvencconfig="  << params.geometryVideoEncoderConfig
      << "  --tvenc="        << g_hmEncoderPath
      << "  --tvencconfig="  << params.textureVideoEncoderConfig
      << "  --csc="          << g_hdrConvertPath
      << "  --cscencconfig=" << params.textureVideoHDRToolEncConfig
      << "  --cscdecconfig=" << params.textureVideoHDRToolDecConfig;
  /* clang-format on */
   if (disableSubProcessLog.disableLog())
     cmd << " 2>&1 > /dev/null";
   printf("cmd = %s \n", cmd.str().c_str());
  system(cmd.str().c_str());

  // //////////////////////////////////////////////////////////////////
  // /////////////////// ORIGINAL APPLICATIONS ////////////////////////
  // //////////////////////////////////////////////////////////////////
  printf("////////////////////////////////////////////////////////////////// \n");
  printf("/////////////////// ORIGINAL APPLICATIONS //////////////////////// \n");
  printf("////////////////////////////////////////////////////////////////// \n");
  std::string binNewPath = "new_bistream.vdmc";
  std::string objNewPath = "new_rec_fr%04d.obj";  
  std::string texNewPath = "new_rec_fr%04d.png";

  // Generate gof structure
  vmesh::SequenceInfo sequenceInfo;
  sequenceInfo.generate(frameCount, startFrame, groupOfFramesMaxSize, srcPath);

  vmesh::VMCEncoder encoder;
  vmesh::Bitstream bitstream;
  vmesh::VMCStats totalStats;
  for (int g = 0; g < sequenceInfo.gofCount(); ++g) {
    vmesh::VMCGroupOfFrames gof;
    const auto& gofInfo = sequenceInfo[g];
    printf("loadGroupOfFrames GOF = %d / %zu \n", g, sequenceInfo.gofCount());
    
    // Load group of frame
    if (loadGroupOfFrames(gofInfo, gof, srcPath, texPath )) {
      std::cerr << "Error: can't load group of frames!\n";
      return ;
    }
    
    // Compress group of frame
    auto start = std::chrono::steady_clock::now();
    if (encoder.compress(gofInfo, gof, bitstream, params)) {
      std::cerr << "Error: can't compress group of frames!\n";
      return ;
    }
    auto end = std::chrono::steady_clock::now();
    gof.stats.processingTimeInSeconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Save reconsctructed models
    if (saveGroupOfFrames(gofInfo, gof, objNewPath, texNewPath, mtlPath )) {
      std::cerr << "Error: can't save rec group of frames!\n";
      return ;
    }

    totalStats += gof.stats;
    std::cout << "\n------- Group of frames " << gofInfo.index_
                << " -----------\n";
    gof.stats.dump("GOF", framerate);
    std::cout << "---------------------------------------\n";    
  }
  std::cout << "\n------- All frames -----------\n";
  totalStats.dump("Sequence", framerate);
  std::cout << "---------------------------------------\n";

  // save bistream
  if (bitstream.save(binNewPath)) {
    std::cerr << "Error: can't save compressed bitstream!\n";
    return;
  }

  // // Compute hashes
  auto hashBinOsw = hash(binOswPath);  
  auto hashBinNew = hash(binNewPath);
  std::cout << "hashBinOsw = " << std::hex << hashBinOsw << "\n";
  std::cout << "hashBinNew = " << std::hex << hashBinNew << "\n";
  ASSERT_EQ(hashBinOsw, hashBinNew) << "Old sw and new sw bin are differentes";
  for (int f = 0; f < frameCount; f++) {
    auto hashObjNew = hash(vmesh::expandNum(objNewPath, f));    
    auto hashObjOsw = hash(vmesh::expandNum(objOswPath, f));
    auto hashTexNew = hash(vmesh::expandNum(texNewPath, f));
    auto hashTexOsw = hash(vmesh::expandNum(texOswPath, f));
    if( !disableSubProcessLog.disableLog()  ){
      std::cout << "hashObjNew " << f << " = " << std::hex << hashObjNew << "\n";      
      std::cout << "hashObjOsw " << f << " = " << std::hex << hashObjOsw << "\n";
      std::cout << "hashTexNew " << f << " = " << std::hex << hashTexNew << "\n";      
      std::cout << "hashTexOsw " << f << " = " << std::hex << hashTexOsw << "\n";
    }
    ASSERT_EQ(hashObjOsw, hashObjNew) << "Old sw and new lib obj are differentes";
    ASSERT_EQ(hashTexOsw, hashTexNew) << "Old sw and new lib tex are differentes";
  }

  // Remove tmp files
  remove(binNewPath.c_str());
  remove(binOswPath.c_str());
  for (int f = 0; f < frameCount; f++) {
    remove(vmesh::expandNum(objNewPath, f).c_str());
    remove(vmesh::expandNum(objOswPath, f).c_str());
    remove(vmesh::expandNum(texNewPath, f).c_str());
    remove(vmesh::expandNum(texOswPath, f).c_str());
    remove(vmesh::expandNum(mtlPath, f).c_str());
    remove(vmesh::expandNum(gofOswPath, f).c_str());
    remove(vmesh::expandNum(mappedOswPath, f).c_str());
    remove(vmesh::expandNum(referenceOswPath, f).c_str());
    remove(vmesh::expandNum(decimateOswPath, f).c_str());
    remove(vmesh::expandNum(decimateTextureOswPath, f).c_str());
    remove(vmesh::expandNum(baseOswPath, f).c_str());
    remove(vmesh::expandNum(subdivOswPath, f).c_str());
    remove(vmesh::expandNum(nsubdivOswPath, f).c_str());
  }
}

TEST(AllChain, OneFrame)
{
  performAllChain(
    "data/levi_fr%04d_qp12_qt13.obj", "data/levi_fr%04d.png", 1, 0, 32);
}
