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

#include "util/misc.hpp"
#include "util/mesh.hpp"
#include "util/verbose.hpp"
#include "util/vector.hpp"
#include "vmc.hpp"

namespace vmesh {

struct VMCEncoderParameters;

//============================================================================

class TextureParametrization {
public:
  TextureParametrization()  = default;
  ~TextureParametrization() = default;

  bool generate(const TriangleMesh<MeshType>&              decimate,
                TriangleMesh<MeshType>&                    decimateTexture,
                const VMCEncoderParameters&                params,
                std::vector<ConnectedComponent<MeshType>>& curCC,
                std::vector<ConnectedComponent<MeshType>>& previousCC,
                std::string keepFilesPathPrefix);

  static bool generate_UVAtlas(const TriangleMesh<MeshType>& decimate,
                               TriangleMesh<MeshType>&       decimateTexture,
                               const VMCEncoderParameters&   params,
                               std::string keepFilesPathPrefix);

  static bool
  generate_orthoAtlas(const TriangleMesh<MeshType>& decimate,
                      TriangleMesh<MeshType>&       decimateTexture,
                      std::vector<ConnectedComponent<MeshType>>& curCC,
                      std::vector<ConnectedComponent<MeshType>>& previousCC,
                      const VMCEncoderParameters&                params,
                      std::string keepFilesPathPrefix);

  static bool create_patches(
    TriangleMesh<MeshType>&                               mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>&       projectedConnectedComponentsCategories,
    const VMCEncoderParameters& params,
    std::string                 keepFilesPathPrefix);

  static void patch_rasterization(vmesh::Plane<uint8_t>&        occupancyCC,
                                  ConnectedComponent<MeshType>& packedCC,
                                  const VMCEncoderParameters&   params);
  static void patch_horizons(vmesh::Plane<uint8_t>& occupancyCC,
                             std::vector<int>&      topHorizon,
                             std::vector<int>&      bottomHorizon,
                             std::vector<int>&      rightHorizon,
                             std::vector<int>&      leftHorizon);
  static bool pack_patch_flexible(vmesh::Plane<uint8_t>&        occupancy,
                                  vmesh::Plane<uint8_t>&        occupancyCC,
                                  ConnectedComponent<MeshType>& packedCC,
                                  const VMCEncoderParameters&   params,
                                  std::string keepFilesPathPrefix,
                                  bool        invertDirection);

  static bool
  pack_patch_flexible_tetris(vmesh::Plane<uint8_t>&        occupancy,
                             vmesh::Plane<uint8_t>&        occupancyCC,
                             ConnectedComponent<MeshType>& packedCC,
                             const VMCEncoderParameters&   params,
                             std::string                   keepFilesPathPrefix,
                             std::vector<int>&             horizon,
                             std::vector<int>&             topHorizon,
                             std::vector<int>&             bottomHorizon,
                             std::vector<int>&             rightHorizon,
                             std::vector<int>&             leftHorizon);

  static bool pack_patch_flexible_with_temporal_stability(
    vmesh::Plane<uint8_t>&        occupancy,
    vmesh::Plane<uint8_t>&        occupancyCC,
    ConnectedComponent<MeshType>& packedCC,
    ConnectedComponent<MeshType>& matchedCC,
    const VMCEncoderParameters&   params,
    std::string                   keepFilesPathPrefix);

  static bool pack_patches_with_patch_scale_and_rotation(
    TriangleMesh<MeshType>&                               mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>& projectedConnectedComponentsCategories,
    std::vector<ConnectedComponent<MeshType>>& packedCCList,
    const VMCEncoderParameters&                params,
    double                                     scalingAdjustment,
    std::string                                keepFilesPathPrefix);

  static bool
  pack_patches_with_patch_scale_and_rotation_and_temporal_stabilization(
    TriangleMesh<MeshType>&                               mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>& projectedConnectedComponentsCategories,
    std::vector<ConnectedComponent<MeshType>>& packedCCList,
    std::vector<ConnectedComponent<MeshType>>& previousFramePackedCCList,
    const VMCEncoderParameters&                params,
    double                                     scalingAdjustment,
    std::string                                keepFilesPathPrefix);
  static bool tetris_packing_patches_with_patch_scale_and_rotation(
    TriangleMesh<MeshType>&                               mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>& projectedConnectedComponentsCategories,
    std::vector<ConnectedComponent<MeshType>>& packedCCList,
    const VMCEncoderParameters&                params,
    double                                     scalingAdjustment,
    std::string                                keepFilesPathPrefix);
  static bool projection_packing_patches_with_patch_scale_and_rotation(
    TriangleMesh<MeshType>&                               mesh,
    std::vector<std::shared_ptr<TriangleMesh<MeshType>>>& projectedCCList,
    std::vector<int32_t>& projectedConnectedComponentsCategories,
    std::vector<ConnectedComponent<MeshType>>& packedCCList,
    const VMCEncoderParameters&                params,
    double                                     scalingAdjustment,
    std::string                                keepFilesPathPrefix);
};

}  // namespace vmesh