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
#pragma once

#include "v3cCommon.hpp"
#include <map>

namespace vmesh {

class Bitstream;
class V3cBitstream;
class ProfileTierLevel;
class V3CParameterSet;
class OccupancyInformation;
class GeometryInformation;
class AttributeInformation;
class AtlasTileHeader;
class AtlasTileDataUnit;
class PatchDataGroup;
class PatchDataGroupUnitPayload;
class PatchV3CParameterSet;
class PatchFrameGeometryParameterSet;
class PatchFrameAttributeParameterSet;
class GeometryPatchParameterSet;
class GeometryPatchParams;
class AttributePatchParameterSet;
class AttributePatchParams;
class GeometryFrameParams;
class PatchTileLayerUnit;
class PatchFrameParameterSet;
class PatchTileHeader;
class RefListStruct;
class PatchTileDataUnit;
class PatchInformationData;
class PatchDataUnit;
class InterPatchDataUnit;
class MergePatchDataUnit;
class SkipPatchDataUnit;
class RawPatchDataUnit;
class EOMPatchDataUnit;
class AttributeSequenceParams;
class AttributeFrameParams;
class PLRInformation;
class PLRData;
class SeiMessage;
class AtlasFrameTileInformation;
class SampleStreamNalUnit;
class NalUnit;
class AccessUnitDelimiterRbsp;
class EndOfSequenceRbsp;
class EndOfAtlasSubBitstreamRbsp;
class FillerDataRbsp;
class V3CUnit;
class VpsVpccExtension;
class AspsVpccExtension;
class AfpsVpccExtension;
class AtlasAdaptationParameterSetRbsp;
class AapsVpccExtension;
class AtlasCameraParameters;
class SampleStreamV3CUnit;
class ProfileToolsetConstraintsInformation;
class AtlasSequenceParameterSetRbsp;
class SEI;
class AtlasFrameParameterSetRbsp;
class AtlasTileLayerRbsp;
class VUIParameters;
class HrdParameters;
class HrdSubLayerParameters;
class MaxCodedVideoResolution;
class CoordinateSystemParameters;
class BaseMeshTileLayer;
class BaseMeshTileHeader;
class BaseMeshTileDataUnit;

class V3CWriter {
public:
  V3CWriter();
  ~V3CWriter();
  size_t write(SampleStreamV3CUnit& ssvu,
               Bitstream&           bitstream,
               uint32_t             forcedSsvhUnitSizePrecisionBytes = 0);
  int    encode(V3cBitstream& syntax, SampleStreamV3CUnit& ssvu);

#if defined(BITSTREAM_TRACE) || defined(CONFORMANCE_TRACE)
  void setLogger(Logger& logger) { logger_ = &logger; }
#endif
private:
  // 8.3.2 V3C unit syntax
  // 8.3.2.1 General V3C unit syntax
  void
  v3cUnit(V3cBitstream& syntax, Bitstream& bitstream, V3CUnitType V3CUnitType);

  // 8.3.2.2 V3C unit header syntax
  void v3cUnitHeader(V3cBitstream& syntax,
                     Bitstream&    bitstream,
                     V3CUnitType   V3CUnitType);

  // 8.3.2.3 V3C unit payload syntax
  void v3cUnitPayload(V3cBitstream& syntax,
                      Bitstream&    bitstream,
                      V3CUnitType   V3CUnitType);

  void videoSubStream(V3cBitstream& syntax,
                      Bitstream&    bitstream,
                      V3CUnitType   V3CUnitType);

  // Base mesh sub-bitstream syntax
  void baseMeshSubStream(V3cBitstream& syntax, Bitstream& bitstream);

  // Base mesh sequence parameter set Rbsp
  void
  baseMeshSequenceParameterSetRbsp(BaseMeshSequenceParameterSetRbsp& bmsps,
                                   V3cBitstream&                     syntax,
                                   Bitstream& bitstream);

  // Base mesh frame parameter set Rbsp
  void baseMeshFrameParameterSetRbsp(BaseMeshFrameParameterSetRbsp& bmfps,
                                     V3cBitstream&                  syntax,
                                     Bitstream&                     bitstream);

  // Base mesh frame tile information syntax
  void baseMeshFrameTileInformation(BaseMeshFrameTileInformation&     bmfti,
                                    BaseMeshSequenceParameterSetRbsp& bmsps,
                                    Bitstream& bitstream);

  // Base mesh tile layer
  void baseMeshTileLayerRbsp(BaseMeshTileLayer&  bmtl,
                             BaseMeshNalUnitType nalUnitType,
                             V3cBitstream&       syntax,
                             Bitstream&          bitstream);

  // Base mesh tile header
  void baseMeshTileHeader(BaseMeshTileHeader& bmth,
                          BaseMeshNalUnitType nalUnitType,
                          V3cBitstream&       syntax,
                          Bitstream&          bitstream);

  // Reference list structure syntax
  void refListStruct(RefListStruct&                    rls,
                     BaseMeshSequenceParameterSetRbsp& asps,
                     Bitstream&                        bitstream);

  // base mesh tile data unit
  void baseMeshTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                            BaseMeshTileHeader&   bmth,
                            V3cBitstream&         syntax,
                            Bitstream&            bitstream);

  // base mesh intra tile data unit
  void baseMeshIntraTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                 BaseMeshTileHeader&   bmth,
                                 V3cBitstream&         syntax,
                                 Bitstream&            bitstream);

  // base mesh inter tile data unit
  void baseMeshInterTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                 BaseMeshTileHeader&   bmth,
                                 V3cBitstream&         syntax,
                                 Bitstream&            bitstream);

  // base mesh skip tile data unit
  void baseMeshSkipTileDataUnit(BaseMeshTileDataUnit& bmtdu,
                                BaseMeshTileHeader&   bmth,
                                V3cBitstream&         syntax,
                                Bitstream&            bitstream);

  // V3C Unit
  void addV3CUnit(V3cBitstream&        syntax,
                  SampleStreamV3CUnit& ssvu,
                  V3CUnitType          v3cUnitType);

  // V3C VPS extension
  void vpsExtension(V3CParameterSet& vps, uint8_t index, Bitstream& bitstream);

  // V3C VDMC extension
  void vpsVdmcExtension(V3CParameterSet&  vps,
                        VpsVdmcExtension& ext,
                        Bitstream&        bitstream);

  // ASPS V-DMC extension syntax
  void aspsVdmcExtension(Bitstream&                     bitstream,
                         AtlasSequenceParameterSetRbsp& asps,
                         AspsVdmcExtension&             ext);

  // AFPS V-DMC extension syntax
  void afpsVdmcExtension(Bitstream& bitstream, AfpsVdmcExtension& ext);

  // AAPS V-DMC extension syntax
  void aapsVdmcExtension(Bitstream& bitstream, AapsVdmcExtension& ext);

  // 8.3.2.4 Atlas sub-bitstream syntax
  void atlasSubStream(V3cBitstream& syntax, Bitstream& bitstream);

  // 8.3.3 Byte alignment syntax
  void byteAlignment(Bitstream& bitstream);

  // 8.3.4 Byte alignment syntax
  void lengthAlignment(Bitstream& bitstream);

  // 8.3.4 V3C parameter set syntax
  // 8.3.4.1 General V3C parameter set syntax
  void v3cParameterSet(V3CParameterSet& vps,
                       V3cBitstream&    syntax,
                       Bitstream&       bitstream);

  // 8.3.4.2 Profile, tier, and level syntax
  void profileTierLevel(ProfileTierLevel& ptl, Bitstream& bitstream);

  // 8.3.4.3 Occupancy parameter information syntax
  void occupancyInformation(OccupancyInformation& oi, Bitstream& bitstream);

  // 8.3.4.4 Geometry information syntax
  void geometryInformation(GeometryInformation& gi,
                           V3CParameterSet&     vps,
                           Bitstream&           bitstream);

  // 8.3.4.5 Attribute information syntax
  void attributeInformation(AttributeInformation& ai,
                            V3CParameterSet&      sps,
                            Bitstream&            bitstream);

  // 8.3.4.6 Profile toolset constraints information syntax
  void profileToolsetConstraintsInformation(
    ProfileToolsetConstraintsInformation& ptci,
    Bitstream&                            bitstream);

  // 8.2 Specification of syntax functions and descriptors
  bool byteAligned(Bitstream& bitstream);
  bool moreDataInPayload(Bitstream& bitstream);
  bool moreRbspData(Bitstream& bitstream);
  bool moreRbspTrailingData(Bitstream& bitstream);
  bool moreDataInV3CUnit(Bitstream& bitstream);
  bool payloadExtensionPresent(Bitstream& bitstream);

  // 8.3.5 NAL unit syntax
  // 8.3.5.1 General NAL unit syntax
  void nalUnit(Bitstream& bitstream, NalUnit& nalUnit);

  // 8.3.5.2 NAL unit header syntax
  void nalUnitHeader(Bitstream& bitstream, NalUnit& nalUnit);

  // 8.3.5.2  NAL unit header syntax
  // 8.3.6.1 Atlas sequence parameter set Rbsp
  // 8.3.6.1.1 General Atlas sequence parameter set Rbsp
  void atlasSequenceParameterSetRbsp(AtlasSequenceParameterSetRbsp& asps,
                                     V3cBitstream&                  syntax,
                                     Bitstream&                     bitstream);

  // 8.3.6.1.2 Point local reconstruction information syntax
  void plrInformation(AtlasSequenceParameterSetRbsp& asps,
                      V3cBitstream&                  syntax,
                      Bitstream&                     bitstream);

  // 8.3.6.2 Atlas frame parameter set Rbsp syntax
  // 8.3.6.2.1 General atlas frame parameter set Rbsp syntax
  void atlasFrameParameterSetRbsp(AtlasFrameParameterSetRbsp& afps,
                                  V3cBitstream&               syntax,
                                  Bitstream&                  bitstream);

  // 8.3.6.2.2 Atlas frame tile information syntax
  void atlasFrameTileInformation(AtlasFrameTileInformation&     afti,
                                 AtlasSequenceParameterSetRbsp& asps,
                                 Bitstream&                     bitstream);

  // 8.3.6.2 Atlas adaptation parameter set RBSP syntax
  // 8.3.6.2.1 General atlas adaptation parameter set RBSP syntax
  void atlasAdaptationParameterSetRbsp(AtlasAdaptationParameterSetRbsp& aaps,
                                       Bitstream& bitstream);

  // 8.3.6.4  Supplemental enhancement information Rbsp
  void seiRbsp(V3cBitstream&    syntax,
               Bitstream&       bitstream,
               AtlasNalUnitType nalUnitType,
               SEI&             sei,
               size_t           atglIndex);

  // 8.3.6.5  Access unit delimiter Rbsp syntax
  void accessUnitDelimiterRbsp(AccessUnitDelimiterRbsp& aud,
                               V3cBitstream&            syntax,
                               Bitstream&               bitstream);

  // 8.3.6.6  End of sequence Rbsp syntax
  void endOfSequenceRbsp(EndOfSequenceRbsp& eosbsp,
                         V3cBitstream&      syntax,
                         Bitstream&         bitstream);

  // 8.3.6.7  End of bitstream Rbsp syntax
  void endOfAtlasSubBitstreamRbsp(EndOfAtlasSubBitstreamRbsp& eoasb,
                                  V3cBitstream&               syntax,
                                  Bitstream&                  bitstream);

  // 8.3.6.8  Filler data Rbsp syntax
  void fillerDataRbsp(FillerDataRbsp& fdrbsp,
                      V3cBitstream&   syntax,
                      Bitstream&      bitstream);

  // 8.3.6.9 Atlas tile group layer Rbsp syntax = patchTileLayerUnit
  void atlasTileLayerRbsp(AtlasTileLayerRbsp& atgl,
                          V3cBitstream&       syntax,
                          AtlasNalUnitType    nalUnitType,
                          Bitstream&          bitstream);

  // 8.3.6.10 RBSP trailing bit syntax
  void rbspTrailingBits(Bitstream& bitstream);

  // 8.3.6.11  Atlas tile group header syntax
  void atlasTileHeader(AtlasTileHeader& ath,
                       V3cBitstream&    syntax,
                       AtlasNalUnitType nalUnitType,
                       Bitstream&       bitstream);

  // 8.3.6.12  Reference list structure syntax
  void refListStruct(RefListStruct&                 rls,
                     AtlasSequenceParameterSetRbsp& asps,
                     Bitstream&                     bitstream);

  // 8.3.7.1  General atlas tile group data unit syntax =patchTileDataUnit
  void atlasTileDataUnit(AtlasTileDataUnit& atdu,
                         AtlasTileHeader&   ath,
                         V3cBitstream&      syntax,
                         Bitstream&         bitstream);

  // 8.3.7.2  Patch information data syntax
  void patchInformationData(PatchInformationData& pid,
                            size_t                patchMode,
                            AtlasTileHeader&      ath,
                            V3cBitstream&         syntax,
                            Bitstream&            bitstream);

  // 8.3.7.3  Patch data unit syntax : AtlasTileHeader instead of
  // PatchTileHeader
  void patchDataUnit(PatchDataUnit&   pdu,
                     AtlasTileHeader& ath,
                     V3cBitstream&    syntax,
                     Bitstream&       bitstream);

  // 8.3.7.4  Skip patch data unit syntax
  void skipPatchDataUnit(Bitstream& bitstream);

  // 8.3.7.5  Merge patch data unit syntax
  void mergePatchDataUnit(MergePatchDataUnit& mpdu,
                          AtlasTileHeader&    ath,
                          V3cBitstream&       syntax,
                          Bitstream&          bitstream);

  // 8.3.7.6  Inter patch data unit syntax
  void interPatchDataUnit(InterPatchDataUnit& ipdu,
                          AtlasTileHeader&    ath,
                          V3cBitstream&       syntax,
                          Bitstream&          bitstream);

  // 8.3.7.7  Raw patch data unit syntax
  void rawPatchDataUnit(RawPatchDataUnit& rpdu,
                        AtlasTileHeader&  ath,
                        V3cBitstream&     syntax,
                        Bitstream&        bitstream);
  // 8.3.7.8  EOM patch data unit syntax
  void eomPatchDataUnit(EOMPatchDataUnit& epdu,
                        AtlasTileHeader&  ptgh,
                        V3cBitstream&     syntax,
                        Bitstream&        bitstream);

  // 8.3.7.9  Point local reconstruction data syntax
  void plrData(PLRData&                       plrd,
               V3cBitstream&                  syntax,
               AtlasSequenceParameterSetRbsp& asps,
               Bitstream&                     bitstream);

  // 8.3.8 Supplemental enhancement information message syntax
  void seiMessage(Bitstream&       bitstream,
                  V3cBitstream&    syntax,
                  AtlasNalUnitType nalUnitType,
                  SEI&             sei,
                  size_t           atglIndex);

  // 8.3.5.3 Patch sequence parameter set syntax
  void patchV3CParameterSet(PatchDataGroup& pdg,
                            size_t          index,
                            Bitstream&      bitstream);

  // 8.3.5.4 Patch frame geometry parameter set syntax
  void patchFrameGeometryParameterSet(PatchDataGroup&  pdg,
                                      size_t           index,
                                      V3CParameterSet& v3cParameterSet,
                                      Bitstream&       bitstream);

  // 8.3.5.5 Geometry frame Params syntax
  void geometryFrameParams(GeometryFrameParams& geometryFrameParams,
                           Bitstream&           bitstream);

  // 8.3.5.6 Patch frame attribute parameter set syntax
  void patchFrameAttributeParameterSet(PatchDataGroup&  pdg,
                                       size_t           index,
                                       V3CParameterSet& v3cParameterSet,
                                       Bitstream&       bitstream);

  // 8.3.5.7 Attribute frame Params syntax
  void attributeFrameParams(AttributeFrameParams& attributeFrameParams,
                            size_t                dimension,
                            Bitstream&            bitstream);

  // 8.3.5.8 Geometry patch parameter set syntax
  void geometryPatchParameterSet(PatchDataGroup& pdg,
                                 size_t          index,
                                 Bitstream&      bitstream);

  // 8.3.5.9 Geometry patch Params syntax
  void geometryPatchParams(
    GeometryPatchParams&            geometryPatchParams,
    PatchFrameGeometryParameterSet& patchFrameGeometryParameterSet,
    Bitstream&                      bitstream);

  // 8.3.5.10 Attribute patch parameter set syntax
  void attributePatchParameterSet(PatchDataGroup&  pdg,
                                  size_t           index,
                                  V3CParameterSet& v3cParameterSet,
                                  Bitstream&       bitstream);

  // 8.3.5.11 Attribute patch Params syntax
  void attributePatchParams(AttributePatchParams& attributePatchParams,
                            PatchFrameAttributeParameterSet& afps,
                            size_t                           dimension,
                            Bitstream&                       bitstream);

  // 8.3.5.12 Patch frame parameter set syntax
  void patchFrameParameterSet(PatchDataGroup&  pdg,
                              size_t           index,
                              V3CParameterSet& v3cParameterSet,
                              Bitstream&       bitstream);

  // 8.3.5.13 Patch frame layer unit syntax
  void patchTileLayerUnit(PatchDataGroup& pdg,
                          size_t          index,
                          V3cBitstream&   syntax,
                          Bitstream&      bitstream);

  // 8.3.5.14 Patch frame header syntax
  void patchTileHeader(PatchTileHeader& patchTileHeader,
                       PatchTileHeader& pfhPrev,
                       V3cBitstream&    syntax,
                       Bitstream&       bitstream);

  // 8.3.5.15 Reference list structure syntax
  void refListStruct(RefListStruct&        refListStruct,
                     PatchV3CParameterSet& patchV3CParameterSet,
                     Bitstream&            bitstream);

  // 8.3.5.16 Patch frame data unit syntax
  void patchTileDataUnit(PatchTileDataUnit& ptgdu,
                         PatchTileHeader&   pth,
                         V3cBitstream&      syntax,
                         Bitstream&         bitstream);

  // 8.3.5.17 Patch information data syntax
  void patchInformationData(PatchInformationData& pid,
                            size_t                patchIndex,
                            size_t                patchMode,
                            PatchTileHeader&      ptgh,
                            V3cBitstream&         syntax,
                            Bitstream&            bitstream);

  // 8.3.7.3 Patch data unit syntax
  void patchDataUnit(PatchDataUnit&   pdu,
                     PatchTileHeader& ptgh,
                     V3cBitstream&    syntax,
                     Bitstream&       bitstream);

  // 8.3.5.19  Delta Patch data unit syntax
  void deltaPatchDataUnit(InterPatchDataUnit& ipdu,
                          PatchTileHeader&    ptgh,
                          V3cBitstream&       syntax,
                          Bitstream&          bitstream);

  // 8.3.5.20 raw patch data unit syntax
  void pcmPatchDataUnit(RawPatchDataUnit& rpdu,
                        PatchTileHeader&  ptgh,
                        V3cBitstream&     syntax,
                        Bitstream&        bitstream);

  // 8.3.5.x EOM patch data unit syntax
  void eomPatchDataUnit(EOMPatchDataUnit& epdu,
                        PatchTileHeader&  ptgh,
                        V3cBitstream&     syntax,
                        Bitstream&        bitstream);

  // C.2 Sample stream V3C unit syntax and semantics
  // C.2.1 Sample stream V3C header syntax
  void sampleStreamV3CHeader(Bitstream& bitstream, SampleStreamV3CUnit& ssvu);

  // C.2.2 Sample stream NAL unit syntax
  void sampleStreamV3CUnit(Bitstream&           bitstream,
                           SampleStreamV3CUnit& ssvu,
                           V3CUnit&             v3cUnit);

  // D.2 Sample stream NAL unit syntax and semantics
  // D.2.1 Sample stream NAL header syntax
  void sampleStreamNalHeader(Bitstream& bitstream, SampleStreamNalUnit& ssnu);

  // D.2.2 Sample stream NAL unit syntax
  void sampleStreamNalUnit(V3cBitstream&        syntax,
                           bool                 isAtlas,
                           Bitstream&           bitstream,
                           SampleStreamNalUnit& ssnu,
                           NalUnit&             nalUnit,
                           size_t               index     = 0,
                           size_t               atglIndex = 0);

  // F.2  SEI payload syntax
  // F.2.1  General SEI message syntax
  void seiPayload(Bitstream&       bitstream,
                  V3cBitstream&    syntax,
                  SEI&             sei,
                  AtlasNalUnitType nalUnitType,
                  size_t           atglIndex);

  // F.2.2  Filler payload SEI message syntax
  void fillerPayload(Bitstream& bitstream, SEI& sei);

  // F.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
  void userDataRegisteredItuTT35(Bitstream& bitstream, SEI& sei);

  // F.2.4  User data unregistered SEI message syntax
  void userDataUnregistered(Bitstream& bitstream, SEI& sei);

  // F.2.5  Recovery point SEI message syntax
  void recoveryPoint(Bitstream& bitstream, SEI& sei);

  // F.2.6  No reconstruction SEI message syntax
  void noReconstruction(Bitstream& bitstream, SEI& sei);

  // F.2.7  Reserved SEI message syntax
  void reservedSeiMessage(Bitstream& bitstream, SEI& sei);

  // F.2.8  SEI manifest SEI message syntax
  void seiManifest(Bitstream& bitstream, SEI& sei);

  // F.2.9  SEI prefix indication SEI message syntax
  void seiPrefixIndication(Bitstream& bitstream, SEI& sei);

  // F.2.10  Active substreams SEI message syntax
  void activeSubBitstreams(Bitstream& bitstream, SEI& sei);

  // F.2.11  Component codec mapping SEI message syntax
  void componentCodecMapping(Bitstream& bitstream, SEI& sei);

  // F.2.12  Volumetric Tiling SEI message syntax
  // F.2.12.1 Scene object information SEI message syntax
  void sceneObjectInformation(Bitstream& bitstream, SEI& sei);

  // F.2.12.2 Object label information SEI message syntax
  void objectLabelInformation(Bitstream& bitstream, SEI& sei);

  // F.2.12.3 Patch information SEI message syntax
  void patchInformation(Bitstream& bitstream, SEI& sei);

  // F.2.12.4 Volumetric rectangle information SEI message syntax
  void volumetricRectangleInformation(Bitstream& bitstream, SEI& sei);

  // F.2.12.5  Atlas object information  SEI message syntax
  void atlasObjectInformation(Bitstream& bitstream, SEI& seiAbstract);

  // F.2.13  Buffering period SEI message syntax
  void bufferingPeriod(Bitstream& bitstream, SEI& sei);

  // F.2.14  Atlas frame timing SEI message syntax
  void atlasFrameTiming(Bitstream& bitstream,
                        SEI&       sei,
                        SEI&       seiBufferingPeriodAbstract,
                        bool       cabDabDelaysPresentFlag);

  // F.2.15.1 Viewport camera parameters SEI messages syntax
  void viewportCameraParameters(Bitstream& bitstream, SEI& seiAbstract);

  // F.2.15.2	Viewport position SEI messages syntax
  void viewportPosition(Bitstream& bitstream, SEI& seiAbstract);

  // F.2.16 Decoded Atlas Information Hash SEI message syntax
  void decodedAtlasInformationHash(Bitstream& bitstream, SEI& seiAbstract);
  // F.2.16.1 Decoded high-level hash unit syntax
  void decodedHighLevelHash(Bitstream& bitstream, SEI& seiAbstract);

  // F.2.16.2 Decoded atlas hash unit syntax
  void decodedAtlasHash(Bitstream& bitstream, SEI& seiAbstract);

  // F.2.16.3 Decoded atlas b2p hash unit syntax
  void decodedAtlasB2pHash(Bitstream& bitstream, SEI& seiAbstract);

  // F.2.16.4 Decoded atlas tiles hash unit syntax
  void
  decodedAtlasTilesHash(Bitstream& bitstream, SEI& seiAbstract, size_t id);

  // F.2.16.5 Decoded atlas tile b2p hash unit syntax
  void
  decodedAtlasTilesB2pHash(Bitstream& bitstream, SEI& seiAbstract, size_t id);

  // F.2.17 Time code SEI message syntax
  void timeCode(Bitstream& bitstream, SEI& seiAbstract);

  // H.20.2.17 Attribute transformation parameters SEI message syntax
  void attributeTransformationParams(Bitstream& bitstream, SEI& sei);

  // H.20.2.18 Occupancy synthesis SEI message syntax
  void occupancySynthesis(Bitstream& bitstream, SEI& seiAbstract);

  // H.20.2.19 Geometry smoothing SEI message syntax
  void geometrySmoothing(Bitstream& bitstream, SEI& seiAbstract);

  // H.20.2.20 Attribute smoothing SEI message syntax
  void attributeSmoothing(Bitstream& bitstream, SEI& seiAbstract);

  // G.2  VUI syntax
  // G.2.1  VUI parameters syntax
  void vuiParameters(Bitstream& bitstream, VUIParameters& vp);

  // G.2.2  HRD parameters syntax
  void hrdParameters(Bitstream& bitstream, HrdParameters& hp);
  // G.2.3  Sub-layer HRD parameters syntax
  void hrdSubLayerParameters(Bitstream&             bitstream,
                             HrdSubLayerParameters& hlsp,
                             size_t                 cabCnt);
  // G.2.4 Maximum coded video resolution syntax
  void maxCodedVideoResolution(Bitstream&               bitstream,
                               MaxCodedVideoResolution& mcvr);
  // G.2.5	Coordinate system parameters syntax
  void coordinateSystemParameters(Bitstream&                  bitstream,
                                  CoordinateSystemParameters& csp);

  // H.7.3.4.1 VPS V-PCC extension syntax
  void vpsVpccExtension(Bitstream& bitstream, VpsVpccExtension& ext);

  // H.7.3.6.1.1 ASPS V-PCC extension syntax
  void aspsVpccExtension(Bitstream&                     bitstream,
                         AtlasSequenceParameterSetRbsp& asps,
                         AspsVpccExtension&             ext);

  // H.7.3.6.1.2 ASPS MIV extension syntax
  void aspsMivExtension(Bitstream& bitstream);

  // H.7.3.6.2.1 AFPS V-PCC extension syntax
  void afpsVpccExtension(Bitstream& bitstream, AfpsVpccExtension& ext);

  // H.7.3.6.2.1 AAPS V-PCC extension syntax
  void aapsVpccExtension(Bitstream& bitstream, AapsVpccExtension& ext);

  // H.7.3.6.2.2 Atlas camera parameters syntax
  void atlasCameraParameters(Bitstream& bitstream, AtlasCameraParameters& acp);

#if defined(BITSTREAM_TRACE) || defined(CONFORMANCE_TRACE)
  Logger* logger_ = nullptr;
#endif
};

};  // namespace vmesh
