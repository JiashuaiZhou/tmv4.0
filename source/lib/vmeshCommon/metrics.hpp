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

#include "vmc.hpp"

//============================================================================
namespace mm {
class Compare;
class Model;
class Image;
}  // namespace mm

namespace vmesh {
template<typename T>
class Frame;

//============================================================================

struct VMCMetricsParameters {
  // Dequantize and sampling  and
  bool   dequantizeUV   = true;
  int    qp             = 12;
  int    qt             = 13;
  int    gridSize       = 1024;
  double minPosition[3] = {0.0, 0.0, 0.0};
  double maxPosition[3] = {0.0, 0.0, 0.0};
  double resolution     = 0;

  // PCC
  bool computePcc                   = false;
  bool normalCalcModificationEnable = true;

  // IBSM
  bool computeIbsm = false;

  // PCQM
  bool   computePcqm            = false;
  double pcqmRadiusCurvature    = 0.001;
  int    pcqmThresholdKnnSearch = 20;
  double pcqmRadiusFactor       = 2.0;

  // Optionals
  bool verbose = false;
};

//============================================================================

class VMCMetrics {
public:
  VMCMetrics()                                 = default;
  VMCMetrics(const VMCMetrics& rhs)            = delete;
  VMCMetrics& operator=(const VMCMetrics& rhs) = delete;
  ~VMCMetrics()                                = default;

  void compute(const Sequence&             sequence0,
               const Sequence&             sequence1,
               const VMCMetricsParameters& params);

  void compute(const std::string           srcMesh,
               const std::string           recMesh,
               const std::string           srcMap,
               const std::string           recMap,
               const VMCMetricsParameters& params);

  template<typename T>
  void compute(const TriangleMesh<T>&      srcModel,
               const TriangleMesh<T>&      recModel,
               const Frame<uint8_t>&       srcMap,
               const Frame<uint8_t>&       recMap,
               const VMCMetricsParameters& params);

  template<typename T>
  void compute(const TriangleMesh<T>&      srcMesh,
               const TriangleMesh<T>&      recMesh,
               const VMCMetricsParameters& params);
  void display(const bool verbose = false);

  std::vector<double> getPccResults();
  std::vector<double> getIbsmResults();
  std::vector<double> getPcqmResults();

private:
  void print(int32_t frameIndex = -1);
  void compute(const mm::Model&            srcModel,
               const mm::Model&            recModel,
               const mm::Image&            srcMap,
               const mm::Image&            recMap,
               const std::string&          srcName,
               const std::string&          recName,
               const VMCMetricsParameters& params);

  std::shared_ptr<mm::Compare> compare;
};

//============================================================================

}  // namespace vmesh
