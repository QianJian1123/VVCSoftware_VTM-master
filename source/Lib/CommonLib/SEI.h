/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#ifndef __SEI__
#define __SEI__

#pragma once
#include <list>
#include <vector>
#include <cstring>

#include "CommonDef.h"
#include "libmd5/MD5.h"

//! \ingroup CommonLib
//! \{
class SPS;

/**
 * Abstract class representing an SEI message with lightweight RTTI.
 */
class SEI
{
public:
  enum PayloadType
  {
    BUFFERING_PERIOD                     = 0,
    PICTURE_TIMING                       = 1,
    FILLER_PAYLOAD                       = 3,
    USER_DATA_REGISTERED_ITU_T_T35       = 4,
    USER_DATA_UNREGISTERED               = 5,
    FILM_GRAIN_CHARACTERISTICS           = 19,
    FRAME_PACKING                        = 45,
    DISPLAY_ORIENTATION                  = 47,
    PARAMETER_SETS_INCLUSION_INDICATION  = 129,
    DECODING_UNIT_INFO                   = 130,
    DECODED_PICTURE_HASH                 = 132,
    SCALABLE_NESTING                     = 133,
    MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
    COLOUR_TRANSFORM_INFO                = 142,
    DEPENDENT_RAP_INDICATION             = 145,
    EQUIRECTANGULAR_PROJECTION           = 150,
    SPHERE_ROTATION                      = 154,
    REGION_WISE_PACKING                  = 155,
    OMNI_VIEWPORT                        = 156,
    GENERALIZED_CUBEMAP_PROJECTION       = 153,
    ALPHA_CHANNEL_INFO                   = 165,
    FRAME_FIELD_INFO                     = 168,
    DEPTH_REPRESENTATION_INFO            = 177,
    MULTIVIEW_ACQUISITION_INFO           = 179,
    SUBPICTURE_LEVEL_INFO                = 203,
    SAMPLE_ASPECT_RATIO_INFO             = 204,
    CONTENT_LIGHT_LEVEL_INFO             = 144,
    ALTERNATIVE_TRANSFER_CHARACTERISTICS = 147,
    AMBIENT_VIEWING_ENVIRONMENT          = 148,
    CONTENT_COLOUR_VOLUME                = 149,
    ANNOTATED_REGIONS                    = 202,
    SCALABILITY_DIMENSION_INFO           = 205,
    EXTENDED_DRAP_INDICATION             = 206,
  };

  SEI() {}
  virtual ~SEI() {}

  static const char *getSEIMessageString(SEI::PayloadType payloadType);

  virtual PayloadType payloadType() const = 0;
};


class SEIEquirectangularProjection : public SEI
{
public:
  PayloadType payloadType() const { return EQUIRECTANGULAR_PROJECTION; }

  SEIEquirectangularProjection()  {}
  virtual ~SEIEquirectangularProjection() {}

  bool    m_erpCancelFlag;
  bool    m_erpPersistenceFlag;
  bool    m_erpGuardBandFlag;
  uint8_t m_erpGuardBandType;
  uint8_t m_erpLeftGuardBandWidth;
  uint8_t m_erpRightGuardBandWidth;
};

class SEISphereRotation : public SEI
{
public:
  PayloadType payloadType() const { return SPHERE_ROTATION; }

  SEISphereRotation()  {}
  virtual ~SEISphereRotation() {}

  bool  m_sphereRotationCancelFlag;
  bool  m_sphereRotationPersistenceFlag;
  int   m_sphereRotationYaw;
  int   m_sphereRotationPitch;
  int   m_sphereRotationRoll;
};

class SEIOmniViewport : public SEI
{
public:
  PayloadType payloadType() const { return OMNI_VIEWPORT; }

  SEIOmniViewport() {}
  virtual ~SEIOmniViewport() {}

  struct OmniViewport
  {
    int      azimuthCentre;
    int      elevationCentre;
    int      tiltCentre;
    uint32_t horRange;
    uint32_t verRange;
  };

  uint32_t m_omniViewportId;
  bool     m_omniViewportCancelFlag;
  bool     m_omniViewportPersistenceFlag;
  uint8_t  m_omniViewportCntMinus1;
  std::vector<OmniViewport> m_omniViewportRegions;
};

class SEIRegionWisePacking : public SEI
{
public:
  PayloadType payloadType() const { return REGION_WISE_PACKING; }
  SEIRegionWisePacking() {}
  virtual ~SEIRegionWisePacking() {}
  bool                  m_rwpCancelFlag;
  bool                  m_rwpPersistenceFlag;
  bool                  m_constituentPictureMatchingFlag;
  int                   m_numPackedRegions;
  int                   m_projPictureWidth;
  int                   m_projPictureHeight;
  int                   m_packedPictureWidth;
  int                   m_packedPictureHeight;
  std::vector<uint8_t>  m_rwpTransformType;
  std::vector<bool>     m_rwpGuardBandFlag;
  std::vector<uint32_t> m_projRegionWidth;
  std::vector<uint32_t> m_projRegionHeight;
  std::vector<uint32_t> m_rwpProjRegionTop;
  std::vector<uint32_t> m_projRegionLeft;
  std::vector<uint16_t> m_packedRegionWidth;
  std::vector<uint16_t> m_packedRegionHeight;
  std::vector<uint16_t> m_packedRegionTop;
  std::vector<uint16_t> m_packedRegionLeft;
  std::vector<uint8_t>  m_rwpLeftGuardBandWidth;
  std::vector<uint8_t>  m_rwpRightGuardBandWidth;
  std::vector<uint8_t>  m_rwpTopGuardBandHeight;
  std::vector<uint8_t>  m_rwpBottomGuardBandHeight;
  std::vector<bool>     m_rwpGuardBandNotUsedForPredFlag;
  std::vector<uint8_t>  m_rwpGuardBandType;
};

class SEIGeneralizedCubemapProjection : public SEI
{
public:
  PayloadType payloadType() const { return GENERALIZED_CUBEMAP_PROJECTION; }

  SEIGeneralizedCubemapProjection()  {}
  virtual ~SEIGeneralizedCubemapProjection() {}

  bool                 m_gcmpCancelFlag;
  bool                 m_gcmpPersistenceFlag;
  uint8_t              m_gcmpPackingType;
  uint8_t              m_gcmpMappingFunctionType;
  std::vector<uint8_t> m_gcmpFaceIndex;
  std::vector<uint8_t> m_gcmpFaceRotation;
  std::vector<uint8_t> m_gcmpFunctionCoeffU;
  std::vector<bool>    m_gcmpFunctionUAffectedByVFlag;
  std::vector<uint8_t> m_gcmpFunctionCoeffV;
  std::vector<bool>    m_gcmpFunctionVAffectedByUFlag;
  bool                 m_gcmpGuardBandFlag;
  uint8_t              m_gcmpGuardBandType;
  bool                 m_gcmpGuardBandBoundaryExteriorFlag;
  uint8_t              m_gcmpGuardBandSamplesMinus1;
};

class SEIScalabilityDimensionInfo : public SEI
{
public:
  PayloadType payloadType() const { return SCALABILITY_DIMENSION_INFO; }
  SEIScalabilityDimensionInfo()
  : m_sdiNumViews (0)
  , m_sdiMaxLayersMinus1 (0)
  , m_sdiMultiviewInfoFlag (false)
  , m_sdiAuxiliaryInfoFlag (false)
  , m_sdiViewIdLenMinus1 (0)
  {
  }
  virtual ~SEIScalabilityDimensionInfo() {}
  bool isSDISameContent(SEIScalabilityDimensionInfo* sdiB);

  int                   m_sdiNumViews;
  int                   m_sdiMaxLayersMinus1;
  bool                  m_sdiMultiviewInfoFlag;
  bool                  m_sdiAuxiliaryInfoFlag;
  int                   m_sdiViewIdLenMinus1;
  std::vector<int>      m_sdiLayerId;
  std::vector<int>      m_sdiViewIdVal;
  std::vector<int>      m_sdiAuxId;
  std::vector<int>      m_sdiNumAssociatedPrimaryLayersMinus1;
  std::vector<std::vector<int>> m_sdiAssociatedPrimaryLayerIdx;
};

class SEIMultiviewAcquisitionInfo : public SEI
{
public:
  PayloadType payloadType( ) const { return MULTIVIEW_ACQUISITION_INFO; }
  SEIMultiviewAcquisitionInfo ( ) { };
  ~SEIMultiviewAcquisitionInfo( ) { };
  SEI* getCopy( ) const { return new SEIMultiviewAcquisitionInfo(*this); };
  bool isMAISameContent(SEIMultiviewAcquisitionInfo* maiB);

  void resizeArrays( )
  {
    int numViews = m_maiIntrinsicParamsEqualFlag ? 1 : m_maiNumViewsMinus1 + 1;
    m_maiSignFocalLengthX       .resize( numViews );
    m_maiExponentFocalLengthX   .resize( numViews );
    m_maiMantissaFocalLengthX   .resize( numViews );
    m_maiSignFocalLengthY       .resize( numViews );
    m_maiExponentFocalLengthY   .resize( numViews );
    m_maiMantissaFocalLengthY   .resize( numViews );
    m_maiSignPrincipalPointX    .resize( numViews );
    m_maiExponentPrincipalPointX.resize( numViews );
    m_maiMantissaPrincipalPointX.resize( numViews );
    m_maiSignPrincipalPointY    .resize( numViews );
    m_maiExponentPrincipalPointY.resize( numViews );
    m_maiMantissaPrincipalPointY.resize( numViews );
    m_maiSignSkewFactor         .resize( numViews );
    m_maiExponentSkewFactor     .resize( numViews );
    m_maiMantissaSkewFactor     .resize( numViews );

    m_maiSignR                  .resize( m_maiNumViewsMinus1 + 1 );
    m_maiExponentR              .resize( m_maiNumViewsMinus1 + 1 );
    m_maiMantissaR              .resize( m_maiNumViewsMinus1 + 1 );
    m_maiSignT                  .resize( m_maiNumViewsMinus1 + 1 );
    m_maiExponentT              .resize( m_maiNumViewsMinus1 + 1 );
    m_maiMantissaT              .resize( m_maiNumViewsMinus1 + 1 );

    for( int i = 0; i <= m_maiNumViewsMinus1 ; i++ )
    {
      m_maiSignR    [i].resize( 3 );
      m_maiExponentR[i].resize( 3 );
      m_maiMantissaR[i].resize( 3 );
      m_maiSignT    [i].resize( 3 );
      m_maiExponentT[i].resize( 3 );
      m_maiMantissaT[i].resize( 3 );

      for (int j = 0; j < 3; j++)
      {
        m_maiSignR    [i][j].resize( 3 );
        m_maiExponentR[i][j].resize( 3 );
        m_maiMantissaR[i][j].resize( 3 );
      }
    }
  }

  uint32_t getMantissaFocalLengthXLen   (int i) const;
  uint32_t getMantissaFocalLengthYLen   (int i) const;
  uint32_t getMantissaPrincipalPointXLen(int i) const;
  uint32_t getMantissaPrincipalPointYLen(int i) const;
  uint32_t getMantissaSkewFactorLen     (int i) const;
  uint32_t getMantissaRLen              (int i, int j, int k ) const;
  uint32_t getMantissaTLen              (int i, int j )        const;

  bool              m_maiIntrinsicParamFlag;
  bool              m_maiExtrinsicParamFlag;
  int               m_maiNumViewsMinus1;
  bool              m_maiIntrinsicParamsEqualFlag;
  int               m_maiPrecFocalLength;
  int               m_maiPrecPrincipalPoint;
  int               m_maiPrecSkewFactor;
  std::vector<bool> m_maiSignFocalLengthX;
  std::vector<int>  m_maiExponentFocalLengthX;
  std::vector<int>  m_maiMantissaFocalLengthX;
  std::vector<bool> m_maiSignFocalLengthY;
  std::vector<int>  m_maiExponentFocalLengthY;
  std::vector<int>  m_maiMantissaFocalLengthY;
  std::vector<bool> m_maiSignPrincipalPointX;
  std::vector<int>  m_maiExponentPrincipalPointX;
  std::vector<int>  m_maiMantissaPrincipalPointX;
  std::vector<bool> m_maiSignPrincipalPointY;
  std::vector<int>  m_maiExponentPrincipalPointY;
  std::vector<int>  m_maiMantissaPrincipalPointY;
  std::vector<bool> m_maiSignSkewFactor;
  std::vector<int>  m_maiExponentSkewFactor;
  std::vector<int>  m_maiMantissaSkewFactor;
  int               m_maiPrecRotationParam;
  int               m_maiPrecTranslationParam;
  std::vector< std::vector<std::vector<bool>>> m_maiSignR;
  std::vector< std::vector<std::vector<int>>>  m_maiExponentR;
  std::vector< std::vector<std::vector<int>>>  m_maiMantissaR;
  std::vector< std::vector<bool>> m_maiSignT;
  std::vector< std::vector<int>>  m_maiExponentT;
  std::vector< std::vector<int>>  m_maiMantissaT;
private:
  uint32_t xGetSyntaxElementLen( int expo, int prec, int val ) const;
};

class SEIAlphaChannelInfo : public SEI
{
public:
  PayloadType payloadType() const { return ALPHA_CHANNEL_INFO; }
  SEIAlphaChannelInfo()
  : m_aciCancelFlag (false)
  , m_aciUseIdc (0)
  , m_aciBitDepthMinus8 (0)
  , m_aciTransparentValue (0)
  , m_aciOpaqueValue (255)
  , m_aciIncrFlag (false)
  , m_aciClipFlag (false)
  , m_aciClipTypeFlag (false)
  {};
  virtual ~SEIAlphaChannelInfo() {};

  bool m_aciCancelFlag;
  int  m_aciUseIdc;
  int  m_aciBitDepthMinus8;
  int  m_aciTransparentValue;
  int  m_aciOpaqueValue;
  bool m_aciIncrFlag;
  bool m_aciClipFlag;
  bool m_aciClipTypeFlag;
};

class SEIDepthRepresentationInfo : public SEI
{
public:
  PayloadType payloadType() const { return DEPTH_REPRESENTATION_INFO; }
  SEIDepthRepresentationInfo()
  : m_driZNearFlag (false)
  , m_driZFarFlag (false)
  , m_driDMinFlag (false)
  , m_driDMaxFlag (false)
  , m_driZNear (0.0)
  , m_driZFar (0.0)
  , m_driDMin (0.0)
  , m_driDMax (0.0)
  , m_driDepthRepresentationType (0)
  , m_driDisparityRefViewId (0)
  , m_driDepthNonlinearRepresentationNumMinus1 (0)
  {};
  virtual ~SEIDepthRepresentationInfo() {};

  bool m_driZNearFlag;
  bool m_driZFarFlag;
  bool m_driDMinFlag;
  bool m_driDMaxFlag;
  double m_driZNear;
  double m_driZFar;
  double m_driDMin;
  double m_driDMax;
  int m_driDepthRepresentationType;
  int m_driDisparityRefViewId;
  int m_driDepthNonlinearRepresentationNumMinus1;
  std::vector<int> m_driDepthNonlinearRepresentationModel;
};

class SEISampleAspectRatioInfo : public SEI
{
public:
  PayloadType payloadType() const { return SAMPLE_ASPECT_RATIO_INFO; }
  SEISampleAspectRatioInfo() {}
  virtual ~SEISampleAspectRatioInfo() {}
  bool                  m_sariCancelFlag;
  bool                  m_sariPersistenceFlag;
  int                   m_sariAspectRatioIdc;
  int                   m_sariSarWidth;
  int                   m_sariSarHeight;
};

static const uint32_t ISO_IEC_11578_LEN=16;

class SEIuserDataUnregistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_UNREGISTERED; }

  SEIuserDataUnregistered()
    : userData(0)
    {}

  virtual ~SEIuserDataUnregistered()
  {
    delete userData;
  }

  uint8_t uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  uint32_t  userDataLength;
  uint8_t *userData;
};

class SEIDecodedPictureHash : public SEI
{
public:
  PayloadType payloadType() const { return DECODED_PICTURE_HASH; }

  SEIDecodedPictureHash() {}
  virtual ~SEIDecodedPictureHash() {}

  HashType method;
  bool     singleCompFlag;

  PictureHash m_pictureHash;
};

class SEIDependentRAPIndication : public SEI
{
public:
  PayloadType payloadType() const { return DEPENDENT_RAP_INDICATION; }
  SEIDependentRAPIndication() { }

  virtual ~SEIDependentRAPIndication() { }
};


class SEIBufferingPeriod : public SEI
{
public:
  PayloadType payloadType() const { return BUFFERING_PERIOD; }
  void copyTo (SEIBufferingPeriod& target) const;

  SEIBufferingPeriod()
  : m_bpNalCpbParamsPresentFlag (false)
  , m_bpVclCpbParamsPresentFlag (false)
  , m_initialCpbRemovalDelayLength (0)
  , m_cpbRemovalDelayLength (0)
  , m_dpbOutputDelayLength (0)
  , m_bpCpbCnt(0)
  , m_duCpbRemovalDelayIncrementLength (0)
  , m_dpbOutputDelayDuLength (0)
  , m_cpbRemovalDelayDeltasPresentFlag (false)
  , m_numCpbRemovalDelayDeltas (0)
  , m_bpMaxSubLayers (0)
  , m_bpDecodingUnitHrdParamsPresentFlag (false)
  , m_decodingUnitCpbParamsInPicTimingSeiFlag (false)
  , m_decodingUnitDpbDuParamsInPicTimingSeiFlag(false)
    , m_sublayerInitialCpbRemovalDelayPresentFlag(false)
    , m_additionalConcatenationInfoPresentFlag (false)
    , m_maxInitialRemovalDelayForConcatenation (0)
    , m_sublayerDpbOutputOffsetsPresentFlag (false)
    , m_altCpbParamsPresentFlag (false)
    , m_useAltCpbParamsFlag (false)
  {
    ::memset(m_initialCpbRemovalDelay, 0, sizeof(m_initialCpbRemovalDelay));
    ::memset(m_initialCpbRemovalOffset, 0, sizeof(m_initialCpbRemovalOffset));
    ::memset(m_cpbRemovalDelayDelta, 0, sizeof(m_cpbRemovalDelayDelta));
    ::memset(m_dpbOutputTidOffset, 0, sizeof(m_dpbOutputTidOffset));
  }
  virtual ~SEIBufferingPeriod() {}

  void      setDuCpbRemovalDelayIncrementLength( uint32_t value )        { m_duCpbRemovalDelayIncrementLength = value;        }
  uint32_t  getDuCpbRemovalDelayIncrementLength( ) const                 { return m_duCpbRemovalDelayIncrementLength;         }
  void      setDpbOutputDelayDuLength( uint32_t value )                  { m_dpbOutputDelayDuLength = value;                  }
  uint32_t  getDpbOutputDelayDuLength( ) const                           { return m_dpbOutputDelayDuLength;                   }
  bool m_bpNalCpbParamsPresentFlag;
  bool m_bpVclCpbParamsPresentFlag;
  uint32_t m_initialCpbRemovalDelayLength;
  uint32_t m_cpbRemovalDelayLength;
  uint32_t m_dpbOutputDelayLength;
  int      m_bpCpbCnt;
  uint32_t m_duCpbRemovalDelayIncrementLength;
  uint32_t m_dpbOutputDelayDuLength;
  uint32_t m_initialCpbRemovalDelay         [MAX_TLAYER][MAX_CPB_CNT][2];
  uint32_t m_initialCpbRemovalOffset        [MAX_TLAYER][MAX_CPB_CNT][2];
  bool m_concatenationFlag;
  uint32_t m_auCpbRemovalDelayDelta;
  bool m_cpbRemovalDelayDeltasPresentFlag;
  int  m_numCpbRemovalDelayDeltas;
  int  m_bpMaxSubLayers;
  uint32_t m_cpbRemovalDelayDelta    [15];
  bool m_bpDecodingUnitHrdParamsPresentFlag;
  bool m_decodingUnitCpbParamsInPicTimingSeiFlag;
  bool m_decodingUnitDpbDuParamsInPicTimingSeiFlag;
  bool m_sublayerInitialCpbRemovalDelayPresentFlag;
  bool     m_additionalConcatenationInfoPresentFlag;
  uint32_t m_maxInitialRemovalDelayForConcatenation;
  bool     m_sublayerDpbOutputOffsetsPresentFlag;
  uint32_t m_dpbOutputTidOffset      [MAX_TLAYER];
  bool     m_altCpbParamsPresentFlag;
  bool     m_useAltCpbParamsFlag;
};

class SEIPictureTiming : public SEI
{
public:
  PayloadType payloadType() const { return PICTURE_TIMING; }
  void copyTo (SEIPictureTiming& target) const;

  SEIPictureTiming()
  : m_picDpbOutputDelay (0)
  , m_picDpbOutputDuDelay (0)
  , m_numDecodingUnitsMinus1 (0)
  , m_duCommonCpbRemovalDelayFlag (false)
  , m_cpbAltTimingInfoPresentFlag (false)
  , m_ptDisplayElementalPeriodsMinus1(0)
  {
    ::memset(m_ptSubLayerDelaysPresentFlag, 0, sizeof(m_ptSubLayerDelaysPresentFlag));
    ::memset(m_duCommonCpbRemovalDelayMinus1, 0, sizeof(m_duCommonCpbRemovalDelayMinus1));
    ::memset(m_cpbRemovalDelayDeltaEnabledFlag, 0, sizeof(m_cpbRemovalDelayDeltaEnabledFlag));
    ::memset(m_cpbRemovalDelayDeltaIdx, 0, sizeof(m_cpbRemovalDelayDeltaIdx));
    ::memset(m_auCpbRemovalDelay, 0, sizeof(m_auCpbRemovalDelay));
  }
  virtual ~SEIPictureTiming()
  {
  }


  bool  m_ptSubLayerDelaysPresentFlag[MAX_TLAYER];
  bool  m_cpbRemovalDelayDeltaEnabledFlag[MAX_TLAYER];
  uint32_t  m_cpbRemovalDelayDeltaIdx[MAX_TLAYER];
  uint32_t  m_auCpbRemovalDelay[MAX_TLAYER];
  uint32_t  m_picDpbOutputDelay;
  uint32_t  m_picDpbOutputDuDelay;
  uint32_t  m_numDecodingUnitsMinus1;
  bool  m_duCommonCpbRemovalDelayFlag;
  uint32_t  m_duCommonCpbRemovalDelayMinus1[MAX_TLAYER];
  std::vector<uint32_t> m_numNalusInDuMinus1;
  std::vector<uint32_t> m_duCpbRemovalDelayMinus1;
  bool     m_cpbAltTimingInfoPresentFlag;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>              m_nalCpbDelayOffset;
  std::vector<uint32_t>              m_nalDpbDelayOffset;
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>              m_vclCpbDelayOffset;
  std::vector<uint32_t>              m_vclDpbDelayOffset;
  int m_ptDisplayElementalPeriodsMinus1;
};

class SEIDecodingUnitInfo : public SEI
{
public:
  PayloadType payloadType() const { return DECODING_UNIT_INFO; }

  SEIDecodingUnitInfo()
    : m_decodingUnitIdx(0)
    , m_dpbOutputDuDelayPresentFlag(false)
    , m_picSptDpbOutputDuDelay(-1)
  {
    ::memset(m_duiSubLayerDelaysPresentFlag, 0, sizeof(m_duiSubLayerDelaysPresentFlag));
    ::memset(m_duSptCpbRemovalDelayIncrement, 0, sizeof(m_duSptCpbRemovalDelayIncrement));
  }
  virtual ~SEIDecodingUnitInfo() {}
  int m_decodingUnitIdx;
  bool m_duiSubLayerDelaysPresentFlag[MAX_TLAYER];
  int m_duSptCpbRemovalDelayIncrement[MAX_TLAYER];
  bool m_dpbOutputDuDelayPresentFlag;
  int m_picSptDpbOutputDuDelay;
};


class SEIFrameFieldInfo : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_FIELD_INFO; }

  SEIFrameFieldInfo()
    : m_fieldPicFlag(false)
    , m_bottomFieldFlag (false)
    , m_pairingIndicatedFlag (false)
    , m_pairedWithNextFieldFlag(false)
    , m_displayFieldsFromFrameFlag(false)
    , m_topFieldFirstFlag(false)
    , m_displayElementalPeriodsMinus1(0)
    , m_sourceScanType(0)
    , m_duplicateFlag(false)
  {}
  virtual ~SEIFrameFieldInfo() {}

  bool m_fieldPicFlag;
  bool m_bottomFieldFlag;
  bool m_pairingIndicatedFlag;
  bool m_pairedWithNextFieldFlag;
  bool m_displayFieldsFromFrameFlag;
  bool m_topFieldFirstFlag;
  int  m_displayElementalPeriodsMinus1;
  int  m_sourceScanType;
  bool m_duplicateFlag;
};


class SEIFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_PACKING; }

  SEIFramePacking() {}
  virtual ~SEIFramePacking() {}

  int  m_arrangementId;
  bool m_arrangementCancelFlag;
  int  m_arrangementType;
  bool m_quincunxSamplingFlag;
  int  m_contentInterpretationType;
  bool m_spatialFlippingFlag;
  bool m_frame0FlippedFlag;
  bool m_fieldViewsFlag;
  bool m_currentFrameIsFrame0Flag;
  bool m_frame0SelfContainedFlag;
  bool m_frame1SelfContainedFlag;
  int  m_frame0GridPositionX;
  int  m_frame0GridPositionY;
  int  m_frame1GridPositionX;
  int  m_frame1GridPositionY;
  int  m_arrangementReservedByte;
  bool m_arrangementPersistenceFlag;
  bool m_upsampledAspectRatio;
};

class SEIDisplayOrientation : public SEI
{
public:
  PayloadType payloadType() const { return DISPLAY_ORIENTATION; }

  SEIDisplayOrientation() {}
  virtual ~SEIDisplayOrientation() {}

  bool                  m_doCancelFlag;
  bool                  m_doPersistenceFlag;
  int                   m_doTransformType;
};

class SEIParameterSetsInclusionIndication : public SEI
{
public:
  PayloadType payloadType() const { return PARAMETER_SETS_INCLUSION_INDICATION; }
  SEIParameterSetsInclusionIndication() {}
  virtual ~SEIParameterSetsInclusionIndication() {}

  int m_selfContainedClvsFlag;
};

class SEIMasteringDisplayColourVolume : public SEI
{
public:
    PayloadType payloadType() const { return MASTERING_DISPLAY_COLOUR_VOLUME; }
    SEIMasteringDisplayColourVolume() {}
    virtual ~SEIMasteringDisplayColourVolume(){}

    SEIMasteringDisplay values;
};

typedef std::list<SEI*> SEIMessages;

/// output a selection of SEI messages by payload type. Ownership stays in original message list.
SEIMessages getSeisByType(const SEIMessages &seiList, SEI::PayloadType seiType);

/// remove a selection of SEI messages by payload type from the original list and return them in a new list.
SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// delete list of SEI messages (freeing the referenced objects)
void deleteSEIs (SEIMessages &seiList);

class SEIScalableNesting : public SEI
{
public:
  PayloadType payloadType() const { return SCALABLE_NESTING; }

  SEIScalableNesting()
  : m_snOlsFlag (false)
  , m_snSubpicFlag (false)
  , m_snNumOlssMinus1 (0)
  , m_snAllLayersFlag (false)
  , m_snNumLayersMinus1 (0)
  , m_snNumSubpics (1)
  , m_snSubpicIdLen (0)
  , m_snNumSEIs(0)
  {}

  virtual ~SEIScalableNesting()
  {
    deleteSEIs(m_nestedSEIs);
  }

  bool      m_snOlsFlag;
  bool      m_snSubpicFlag;
  uint32_t  m_snNumOlssMinus1;
  uint32_t  m_snOlsIdxDeltaMinus1[MAX_NESTING_NUM_LAYER];
  uint32_t  m_snOlsIdx[MAX_NESTING_NUM_LAYER];
  bool      m_snAllLayersFlag;                      //value valid if m_nestingOlsFlag == 0
  uint32_t  m_snNumLayersMinus1;                    //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0
  uint8_t   m_snLayerId[MAX_NESTING_NUM_LAYER];     //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0. This can e.g. be a static array of 64 uint8_t values
  uint32_t  m_snNumSubpics;
  uint8_t   m_snSubpicIdLen;
  std::vector<uint16_t> m_snSubpicId;
  uint32_t  m_snNumSEIs;

  SEIMessages m_nestedSEIs;
};


#if ENABLE_TRACING
void xTraceSEIHeader();
void xTraceSEIMessageType( SEI::PayloadType payloadType );
#endif

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
class SEIAlternativeTransferCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return ALTERNATIVE_TRANSFER_CHARACTERISTICS; }

  SEIAlternativeTransferCharacteristics() : m_preferredTransferCharacteristics(18)
  { }

  virtual ~SEIAlternativeTransferCharacteristics() {}

  uint32_t m_preferredTransferCharacteristics;
};
#endif
class SEIUserDataRegistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_REGISTERED_ITU_T_T35; }

  SEIUserDataRegistered() {}
  virtual ~SEIUserDataRegistered() {}

  uint16_t m_ituCountryCode;
  std::vector<uint8_t> m_userData;
};

class SEIFilmGrainCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return FILM_GRAIN_CHARACTERISTICS; }

  SEIFilmGrainCharacteristics() {}
  virtual ~SEIFilmGrainCharacteristics() {}

  bool        m_filmGrainCharacteristicsCancelFlag;
  uint8_t     m_filmGrainModelId;
  bool        m_separateColourDescriptionPresentFlag;
  uint8_t     m_filmGrainBitDepthLumaMinus8;
  uint8_t     m_filmGrainBitDepthChromaMinus8;
  bool        m_filmGrainFullRangeFlag;
  uint8_t     m_filmGrainColourPrimaries;
  uint8_t     m_filmGrainTransferCharacteristics;
  uint8_t     m_filmGrainMatrixCoeffs;
  uint8_t     m_blendingModeId;
  uint8_t     m_log2ScaleFactor;

  struct CompModelIntensityValues
  {
    uint8_t intensityIntervalLowerBound;
    uint8_t intensityIntervalUpperBound;
    std::vector<int> compModelValue;
  };

  struct CompModel
  {
    bool  presentFlag;
    uint8_t numModelValues;
    std::vector<CompModelIntensityValues> intensityValues;
  };

  CompModel m_compModel[MAX_NUM_COMPONENT];
  bool      m_filmGrainCharacteristicsPersistenceFlag;
};

class SEIContentLightLevelInfo : public SEI
{
public:
  PayloadType payloadType() const { return CONTENT_LIGHT_LEVEL_INFO; }
  SEIContentLightLevelInfo() { }

  virtual ~SEIContentLightLevelInfo() { }

  uint32_t m_maxContentLightLevel;
  uint32_t m_maxPicAverageLightLevel;
};

class SEIAmbientViewingEnvironment : public SEI
{
public:
  PayloadType payloadType() const { return AMBIENT_VIEWING_ENVIRONMENT; }
  SEIAmbientViewingEnvironment() { }

  virtual ~SEIAmbientViewingEnvironment() { }

  uint32_t m_ambientIlluminance;
  uint16_t m_ambientLightX;
  uint16_t m_ambientLightY;
};

class SEIColourTransformInfo : public SEI
{
public:
  PayloadType payloadType() const { return COLOUR_TRANSFORM_INFO; }
  SEIColourTransformInfo() { }

  virtual ~SEIColourTransformInfo() { }

  uint16_t m_id;
  bool     m_signalInfoFlag;
  bool     m_fullRangeFlag;
  uint16_t m_primaries;
  uint16_t m_transferFunction;
  uint16_t m_matrixCoefs;
  bool     m_crossComponentFlag;
  bool     m_crossComponentInferred;
  uint16_t m_numberChromaLutMinus1;
  int      m_chromaOffset;
  uint16_t m_bitdepth;
  uint16_t m_log2NumberOfPointsPerLut;
  LutModel m_lut[MAX_NUM_COMPONENT];
};
class SEIContentColourVolume : public SEI
{
public:
  PayloadType payloadType() const { return CONTENT_COLOUR_VOLUME; }
  SEIContentColourVolume() {}
  virtual ~SEIContentColourVolume() {}

  bool      m_ccvCancelFlag;
  bool      m_ccvPersistenceFlag;
  bool      m_ccvPrimariesPresentFlag;
  bool      m_ccvMinLuminanceValuePresentFlag;
  bool      m_ccvMaxLuminanceValuePresentFlag;
  bool      m_ccvAvgLuminanceValuePresentFlag;
  int       m_ccvPrimariesX[MAX_NUM_COMPONENT];
  int       m_ccvPrimariesY[MAX_NUM_COMPONENT];
  uint32_t  m_ccvMinLuminanceValue;
  uint32_t  m_ccvMaxLuminanceValue;
  uint32_t  m_ccvAvgLuminanceValue;
};

#endif


class SEISubpicureLevelInfo : public SEI
{
public:
  PayloadType payloadType() const { return SUBPICTURE_LEVEL_INFO; }
  SEISubpicureLevelInfo()
  : m_numRefLevels(0)
  , m_explicitFractionPresentFlag (false)
  , m_cbrConstraintFlag (false)
  , m_numSubpics(0)
  , m_sliMaxSublayers(1)
  , m_sliSublayerInfoPresentFlag(false)
  {}
  virtual ~SEISubpicureLevelInfo() {}

  int       m_numRefLevels;
  bool      m_explicitFractionPresentFlag;
  bool      m_cbrConstraintFlag;
  int       m_numSubpics;
  int       m_sliMaxSublayers;
  bool      m_sliSublayerInfoPresentFlag;
  std::vector<std::vector<int>>              m_nonSubpicLayersFraction;
  std::vector<std::vector<Level::Name>>      m_refLevelIdc;
  std::vector<std::vector<std::vector<int>>> m_refLevelFraction;
};

class SEIAnnotatedRegions : public SEI
{
public:
  PayloadType payloadType() const { return ANNOTATED_REGIONS; }
  SEIAnnotatedRegions() {}
  virtual ~SEIAnnotatedRegions() {}

  void copyFrom(const SEIAnnotatedRegions &seiAnnotatedRegions)
  {
    (*this) = seiAnnotatedRegions;
  }

  struct AnnotatedRegionObject
  {
    AnnotatedRegionObject() :
      objectCancelFlag(false),
      objectLabelValid(false),
      boundingBoxValid(false)
    { }
    bool objectCancelFlag;

    bool objectLabelValid;
    uint32_t objLabelIdx;            // only valid if bObjectLabelValid

    bool boundingBoxValid;
    uint32_t boundingBoxTop;         // only valid if bBoundingBoxValid
    uint32_t boundingBoxLeft;
    uint32_t boundingBoxWidth;
    uint32_t boundingBoxHeight;

    bool partialObjectFlag;        // only valid if bPartialObjectFlagValid
    uint32_t objectConfidence;
  };
  struct AnnotatedRegionLabel
  {
    AnnotatedRegionLabel() : labelValid(false) { }
    bool        labelValid;
    std::string label;           // only valid if bLabelValid
  };

  struct AnnotatedRegionHeader
  {
    AnnotatedRegionHeader() : m_cancelFlag(true), m_receivedSettingsOnce(false) { }
    bool      m_cancelFlag;
    bool      m_receivedSettingsOnce; // used for decoder conformance checking. Other confidence flags must be unchanged once this flag is set.

    bool      m_notOptimizedForViewingFlag;
    bool      m_trueMotionFlag;
    bool      m_occludedObjectFlag;
    bool      m_partialObjectFlagPresentFlag;
    bool      m_objectLabelPresentFlag;
    bool      m_objectConfidenceInfoPresentFlag;
    uint32_t      m_objectConfidenceLength;         // Only valid if m_objectConfidenceInfoPresentFlag
    bool      m_objectLabelLanguagePresentFlag; // Only valid if m_objectLabelPresentFlag
    std::string m_annotatedRegionsObjectLabelLang;
  };
  typedef uint32_t AnnotatedRegionObjectIndex;
  typedef uint32_t AnnotatedRegionLabelIndex;

  AnnotatedRegionHeader m_hdr;
  std::vector<std::pair<AnnotatedRegionObjectIndex, AnnotatedRegionObject> > m_annotatedRegions;
  std::vector<std::pair<AnnotatedRegionLabelIndex,  AnnotatedRegionLabel>  > m_annotatedLabels;
};

class SEIExtendedDrapIndication : public SEI
{
public:
  PayloadType payloadType() const { return EXTENDED_DRAP_INDICATION; }

  SEIExtendedDrapIndication() {}
  virtual ~SEIExtendedDrapIndication() {}

  int               m_edrapIndicationRapIdMinus1;
  bool              m_edrapIndicationLeadingPicturesDecodableFlag;
  int               m_edrapIndicationReservedZero12Bits;
  int               m_edrapIndicationNumRefRapPicsMinus1;
  std::vector<int>  m_edrapIndicationRefRapId;
};
//! \}


