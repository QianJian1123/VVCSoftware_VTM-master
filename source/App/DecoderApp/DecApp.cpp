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

/** \file     DecApp.cpp
    \brief    Decoder application class
*/

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>

#include "DecApp.h"
#include "DecoderLib/AnnexBread.h"
#include "DecoderLib/NALread.h"
#if RExt__DECODER_DEBUG_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#include "CommonLib/dtrace_codingstruct.h"


//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

DecApp::DecApp()
: m_iPOCLastDisplay(-MAX_INT)
{
  for (int i = 0; i < MAX_NUM_LAYER_IDS; i++)
  {
    m_newCLVS[i] = true;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal class
 - until the end of the bitstream, call decoding function in DecApp class
 - delete allocated buffers
 - destroy internal class
 - returns the number of mismatching pictures
 */
uint32_t DecApp::decode()
{
  int                 poc;
  PicList* pcListPic = NULL;

  ifstream bitstreamFile(m_bitstreamFileName.c_str(), ifstream::in | ifstream::binary);
  if (!bitstreamFile)
  {
    EXIT( "Failed to open bitstream file " << m_bitstreamFileName.c_str() << " for reading" ) ;
  }

  InputByteStream bytestream(bitstreamFile);

  if (!m_outputDecodedSEIMessagesFilename.empty() && m_outputDecodedSEIMessagesFilename!="-")
  {
    m_seiMessageFileStream.open(m_outputDecodedSEIMessagesFilename.c_str(), std::ios::out);
    if (!m_seiMessageFileStream.is_open() || !m_seiMessageFileStream.good())
    {
      EXIT( "Unable to open file "<< m_outputDecodedSEIMessagesFilename.c_str() << " for writing decoded SEI messages");
    }
  }

  if (!m_oplFilename.empty() && m_oplFilename!="-")
  {
    m_oplFileStream.open(m_oplFilename.c_str(), std::ios::out);
    if (!m_oplFileStream.is_open() || !m_oplFileStream.good())
    {
      EXIT( "Unable to open file "<< m_oplFilename.c_str() << " to write an opl-file for conformance testing (see JVET-P2008 for details)");
    }
  }

  // create & initialize internal classes
  xCreateDecLib();

  m_iPOCLastDisplay += m_iSkipFrame;      // set the last displayed POC correctly for skip forward.

  // clear contents of colour-remap-information-SEI output file
  if (!m_colourRemapSEIFileName.empty())
  {
    std::ofstream ofile(m_colourRemapSEIFileName.c_str());
    if (!ofile.good() || !ofile.is_open())
    {
      EXIT( "Unable to open file " << m_colourRemapSEIFileName.c_str() << " for writing colour-remap-information-SEI video");
    }
  }

  // clear contents of annotated-Regions-SEI output file
  if (!m_annotatedRegionsSEIFileName.empty())
  {
    std::ofstream ofile(m_annotatedRegionsSEIFileName.c_str());
    if (!ofile.good() || !ofile.is_open())
    {
      fprintf(stderr, "\nUnable to open file '%s' for writing annotated-Regions-SEI\n", m_annotatedRegionsSEIFileName.c_str());
      exit(EXIT_FAILURE);
    }
  }

  // main decoder loop
  bool loopFiltered[MAX_VPS_LAYERS] = { false };

  bool bPicSkipped = false;

  bool isEosPresentInPu = false;
  bool isEosPresentInLastPu = false;

  bool outputPicturePresentInBitstream = false;
  auto setOutputPicturePresentInStream = [&]()
  {
    if( !outputPicturePresentInBitstream )
    {
      PicList::iterator iterPic = pcListPic->begin();
      while (!outputPicturePresentInBitstream && iterPic != pcListPic->end())
      {
        Picture *pcPic = *(iterPic++);
        if (pcPic->neededForOutput)
        {
          outputPicturePresentInBitstream = true;
        }
      }
    }
  };

    m_cDecLib.setHTidExternalSetFlag(m_mTidExternalSet);
    m_cDecLib.setTOlsIdxExternalFlag(m_tOlsIdxTidExternalSet);

  bool gdrRecoveryPeriod[MAX_NUM_LAYER_IDS] = { false };
  bool prevPicSkipped = true;

  while (!!bitstreamFile)
  {
    InputNALUnit nalu;
    nalu.m_nalUnitType = NAL_UNIT_INVALID;

    // determine if next NAL unit will be the first one from a new picture
    bool bNewPicture = m_cDecLib.isNewPicture(&bitstreamFile, &bytestream);
    bool bNewAccessUnit = bNewPicture && m_cDecLib.isNewAccessUnit( bNewPicture, &bitstreamFile, &bytestream );
    if(!bNewPicture)
    {
      AnnexBStats stats = AnnexBStats();

      // find next NAL unit in stream
      byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);
      if (nalu.getBitstream().getFifo().empty())
      {
        /* this can happen if the following occur:
         *  - empty input file
         *  - two back-to-back start_code_prefixes
         *  - start_code_prefix immediately followed by EOF
         */
        msg( ERROR, "Warning: Attempt to decode an empty NAL unit\n");
      }
      else
      {
        // read NAL unit header
        read(nalu);

        // flush output for first slice of an IDR picture
        if(m_cDecLib.getFirstSliceInPicture() &&
            (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
             nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP))
        {
          if (!m_cDecLib.getMixedNaluTypesInPicFlag())
          {
            m_newCLVS[nalu.m_nuhLayerId] = true;   // An IDR picture starts a new CLVS
            xFlushOutput(pcListPic, nalu.m_nuhLayerId);
          }
          else
          {
            m_newCLVS[nalu.m_nuhLayerId] = false;
          }
        }
        else if (m_cDecLib.getFirstSliceInPicture() && nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA && isEosPresentInLastPu)
        {
          // A CRA that is immediately preceded by an EOS is a CLVSS
          m_newCLVS[nalu.m_nuhLayerId] = true;
          xFlushOutput(pcListPic, nalu.m_nuhLayerId);
        }
        else if (m_cDecLib.getFirstSliceInPicture() && nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA && !isEosPresentInLastPu)
        {
          // A CRA that is not immediately precede by an EOS is not a CLVSS
          m_newCLVS[nalu.m_nuhLayerId] = false;
        }
        else if(m_cDecLib.getFirstSliceInPicture() && !isEosPresentInLastPu)
        {
          m_newCLVS[nalu.m_nuhLayerId] = false;
        }

        // parse NAL unit syntax if within target decoding layer
        if( ( m_iMaxTemporalLayer < 0 || nalu.m_temporalId <= m_iMaxTemporalLayer ) && xIsNaluWithinTargetDecLayerIdSet( &nalu ) )
        {
          CHECK(nalu.m_temporalId > m_iMaxTemporalLayer, "bitstream shall not include any NAL unit with TemporalId greater than HighestTid");
          if (m_targetDecLayerIdSet.size())
          {
            CHECK(std::find(m_targetDecLayerIdSet.begin(), m_targetDecLayerIdSet.end(), nalu.m_nuhLayerId) == m_targetDecLayerIdSet.end(), "bitstream shall not contain any other layers than included in the OLS with OlsIdx");
          }
          if (bPicSkipped)
          {
            if ((nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR))
            {
              if (m_cDecLib.isSliceNaluFirstInAU(true, nalu))
              {
                m_cDecLib.resetAccessUnitNals();
                m_cDecLib.resetAccessUnitApsNals();
                m_cDecLib.resetAccessUnitPicInfo();
              }
              bPicSkipped = false;
            }
          }

          int skipFrameCounter = m_iSkipFrame;
          m_cDecLib.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay, m_targetOlsIdx);

          if ( prevPicSkipped && nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR )
          {
            gdrRecoveryPeriod[nalu.m_nuhLayerId] = true;
          }

          if ( skipFrameCounter == 1 && ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR  || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA ))
          {
            skipFrameCounter--;
          }

          if ( m_iSkipFrame < skipFrameCounter  &&
              ((nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR)))
          {
            if (m_cDecLib.isSliceNaluFirstInAU(true, nalu))
            {
              m_cDecLib.checkSeiInPictureUnit();
              m_cDecLib.resetPictureSeiNalus();
              m_cDecLib.checkAPSInPictureUnit();
              m_cDecLib.resetPictureUnitNals();
              m_cDecLib.resetAccessUnitSeiTids();
              m_cDecLib.checkSEIInAccessUnit();
              m_cDecLib.resetAccessUnitSeiPayLoadTypes();
              m_cDecLib.resetAccessUnitNals();
              m_cDecLib.resetAccessUnitApsNals();
              m_cDecLib.resetAccessUnitPicInfo();
            }
            bPicSkipped = true;
            m_iSkipFrame++;   // skipFrame count restore, the real decrement occur at the begin of next frame
          }

          if (nalu.m_nalUnitType == NAL_UNIT_OPI)
          {
            if (!m_cDecLib.getHTidExternalSetFlag() && m_cDecLib.getOPI()->getHtidInfoPresentFlag())
            {
              m_iMaxTemporalLayer = m_cDecLib.getOPI()->getOpiHtidPlus1()-1;
            }
            m_cDecLib.setHTidOpiSetFlag(m_cDecLib.getOPI()->getHtidInfoPresentFlag());
          }
          if (nalu.m_nalUnitType == NAL_UNIT_VPS)
          {
            m_cDecLib.deriveTargetOutputLayerSet( m_cDecLib.getVPS()->m_targetOlsIdx );
            m_targetDecLayerIdSet = m_cDecLib.getVPS()->m_targetLayerIdSet;
            m_targetOutputLayerIdSet = m_cDecLib.getVPS()->m_targetOutputLayerIdSet;
          }
        }
        else
        {
          bPicSkipped = true;
        }
      }

      if( nalu.isSlice() && nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_RASL)
      {
        prevPicSkipped = bPicSkipped;
      }

      // once an EOS NAL unit appears in the current PU, mark the variable isEosPresentInPu as true
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        isEosPresentInPu = true;
        m_newCLVS[nalu.m_nuhLayerId] = true;  //The presence of EOS means that the next picture is the beginning of new CLVS
      }
      // within the current PU, only EOS and EOB are allowed to be sent after an EOS nal unit
      if(isEosPresentInPu)
      {
        CHECK(nalu.m_nalUnitType != NAL_UNIT_EOS && nalu.m_nalUnitType != NAL_UNIT_EOB, "When an EOS NAL unit is present in a PU, it shall be the last NAL unit among all NAL units within the PU other than other EOS NAL units or an EOB NAL unit");
      }
    }

    if ((bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) && !m_cDecLib.getFirstSliceInSequence(nalu.m_nuhLayerId) && !bPicSkipped)
    {
      if (!loopFiltered[nalu.m_nuhLayerId] || bitstreamFile)
      {
        m_cDecLib.executeLoopFilters();
        m_cDecLib.finishPicture(poc, pcListPic, INFO, m_newCLVS[nalu.m_nuhLayerId]);
      }
      loopFiltered[nalu.m_nuhLayerId] = (nalu.m_nalUnitType == NAL_UNIT_EOS);
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        m_cDecLib.setFirstSliceInSequence(true, nalu.m_nuhLayerId);
      }

      m_cDecLib.updateAssociatedIRAP();
      m_cDecLib.updatePrevGDRInSameLayer();
      m_cDecLib.updatePrevIRAPAndGDRSubpic();

      if ( gdrRecoveryPeriod[nalu.m_nuhLayerId] )
      {
        if ( m_cDecLib.getGDRRecoveryPocReached() )
        {
          gdrRecoveryPeriod[nalu.m_nuhLayerId] = false;
        }
      }
    }
    else if ( (bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) &&
      m_cDecLib.getFirstSliceInSequence(nalu.m_nuhLayerId))
    {
      m_cDecLib.setFirstSliceInPicture (true);
    }

    if( pcListPic )
    {
      if ( gdrRecoveryPeriod[nalu.m_nuhLayerId] ) // Suppress YUV and OPL output during GDR recovery
      {
        PicList::iterator iterPic = pcListPic->begin();
        while (iterPic != pcListPic->end())
        {
          Picture *pcPic = *(iterPic++);
          if (pcPic->layerId == nalu.m_nuhLayerId)
          {
            pcPic->neededForOutput = false;
          }
        }
      }

      if( !m_reconFileName.empty() && !m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].isOpen() )
      {
        const BitDepths &bitDepths=pcListPic->front()->cs->sps->getBitDepths(); // use bit depths of first reconstructed picture.
        for( uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++ )
        {
            if( m_outputBitDepth[channelType] == 0 )
            {
                m_outputBitDepth[channelType] = bitDepths.recon[channelType];
            }
        }

        if (m_packedYUVMode && (m_outputBitDepth[CH_L] != 10 && m_outputBitDepth[CH_L] != 12))
        {
          EXIT ("Invalid output bit-depth for packed YUV output, aborting\n");
        }

        std::string reconFileName = m_reconFileName;
        if( m_reconFileName.compare( "/dev/null" ) && m_cDecLib.getVPS() != nullptr && m_cDecLib.getVPS()->getMaxLayers() > 1 && xIsNaluWithinTargetOutputLayerIdSet( &nalu ) )
        {
          size_t pos = reconFileName.find_last_of('.');
          std::string layerString = std::string(".layer") + std::to_string(nalu.m_nuhLayerId);
          if (pos != string::npos)
          {
            reconFileName.insert(pos, layerString);
          }
          else
          {
            reconFileName.append(layerString);
          }
        }
        if( ( m_cDecLib.getVPS() != nullptr && ( m_cDecLib.getVPS()->getMaxLayers() == 1 || xIsNaluWithinTargetOutputLayerIdSet( &nalu ) ) ) || m_cDecLib.getVPS() == nullptr )
        {
          m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].open( reconFileName, true, m_outputBitDepth, m_outputBitDepth, bitDepths.recon ); // write mode
        }
      }
      // update file bitdepth shift if recon bitdepth changed between sequences
      for( uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++ )
      {
        int reconBitdepth = pcListPic->front()->cs->sps->getBitDepth((ChannelType)channelType);
        int fileBitdepth  = m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].getFileBitdepth(channelType);
        int bitdepthShift = m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].getBitdepthShift(channelType);
        if( fileBitdepth + bitdepthShift != reconBitdepth )
        {
          m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].setBitdepthShift(channelType, reconBitdepth - fileBitdepth);
        }
      }
      if (!m_SEICTIFileName.empty() && !m_cVideoIOYuvSEICTIFile[nalu.m_nuhLayerId].isOpen())
      {
        const BitDepths& bitDepths = pcListPic->front()->cs->sps->getBitDepths(); // use bit depths of first reconstructed picture.
        for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
        {
          if (m_outputBitDepth[channelType] == 0)
          {
            m_outputBitDepth[channelType] = bitDepths.recon[channelType];
          }
        }

        if (m_packedYUVMode && (m_outputBitDepth[CH_L] != 10 && m_outputBitDepth[CH_L] != 12))
        {
          EXIT("Invalid output bit-depth for packed YUV output, aborting\n");
        }

        std::string SEICTIFileName = m_SEICTIFileName;
        if (m_SEICTIFileName.compare("/dev/null") && m_cDecLib.getVPS() != nullptr && m_cDecLib.getVPS()->getMaxLayers() > 1 && xIsNaluWithinTargetOutputLayerIdSet(&nalu))
        {
          size_t pos = SEICTIFileName.find_last_of('.');
          if (pos != string::npos)
          {
            SEICTIFileName.insert(pos, std::to_string(nalu.m_nuhLayerId));
          }
          else
          {
            SEICTIFileName.append(std::to_string(nalu.m_nuhLayerId));
          }
        }
        if ((m_cDecLib.getVPS() != nullptr && (m_cDecLib.getVPS()->getMaxLayers() == 1 || xIsNaluWithinTargetOutputLayerIdSet(&nalu))) || m_cDecLib.getVPS() == nullptr)
        {
          m_cVideoIOYuvSEICTIFile[nalu.m_nuhLayerId].open(SEICTIFileName, true, m_outputBitDepth, m_outputBitDepth, bitDepths.recon); // write mode
        }
      }
      if (!m_annotatedRegionsSEIFileName.empty())
      {
        xOutputAnnotatedRegions(pcListPic);
      }
      // write reconstruction to file
      if( bNewPicture )
      {
        setOutputPicturePresentInStream();
        xWriteOutput( pcListPic, nalu.m_temporalId );
      }
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        if (!m_annotatedRegionsSEIFileName.empty() && bNewPicture)
        {
          xOutputAnnotatedRegions(pcListPic);
        }
        setOutputPicturePresentInStream();
        xWriteOutput( pcListPic, nalu.m_temporalId );
        m_cDecLib.setFirstSliceInPicture (false);
      }
      // write reconstruction to file -- for additional bumping as defined in C.5.2.3
      if (!bNewPicture && ((nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_IRAP_VCL_11)
        || (nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR)))
      {
        setOutputPicturePresentInStream();
        xWriteOutput( pcListPic, nalu.m_temporalId );
      }
    }
    if( bNewPicture )
    {
      m_cDecLib.checkSeiInPictureUnit();
      m_cDecLib.resetPictureSeiNalus();
      // reset the EOS present status for the next PU check
      isEosPresentInLastPu = isEosPresentInPu;
      isEosPresentInPu = false;
    }
    if (bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      m_cDecLib.checkAPSInPictureUnit();
      m_cDecLib.resetPictureUnitNals();
    }
    if (bNewAccessUnit || !bitstreamFile)
    {
      m_cDecLib.CheckNoOutputPriorPicFlagsInAccessUnit();
      m_cDecLib.resetAccessUnitNoOutputPriorPicFlags();
      m_cDecLib.checkLayerIdIncludedInCvss();
      m_cDecLib.resetAccessUnitEos();
      m_cDecLib.resetAudIrapOrGdrAuFlag();
    }
    if(bNewAccessUnit)
    {
      m_cDecLib.checkTidLayerIdInAccessUnit();
      m_cDecLib.resetAccessUnitSeiTids();
      m_cDecLib.checkSEIInAccessUnit();
      m_cDecLib.resetAccessUnitSeiPayLoadTypes();
      m_cDecLib.resetAccessUnitNals();
      m_cDecLib.resetAccessUnitApsNals();
      m_cDecLib.resetAccessUnitPicInfo();
    }
  }
  if (!m_annotatedRegionsSEIFileName.empty())
  {
    xOutputAnnotatedRegions(pcListPic);
  }
  // May need to check again one more time as in case one the bitstream has only one picture, the first check may miss it
  setOutputPicturePresentInStream();
  CHECK(!outputPicturePresentInBitstream, "It is required that there shall be at least one picture with PictureOutputFlag equal to 1 in the bitstream")

  xFlushOutput( pcListPic );

  // get the number of checksum errors
  uint32_t nRet = m_cDecLib.getNumberOfChecksumErrorsDetected();

  // delete buffers
  m_cDecLib.deletePicBuffer();
  // destroy internal classes
  xDestroyDecLib();

#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::DestroyInstance();
#endif

  destroyROM();

  return nRet;
}



void DecApp::writeLineToOutputLog(Picture * pcPic)
{
  if (m_oplFileStream.is_open() && m_oplFileStream.good())
  {
    const SPS *   sps             = pcPic->cs->sps;
    ChromaFormat  chromaFormatIDC = sps->getChromaFormatIdc();
    const Window &conf            = pcPic->getConformanceWindow();
    const int     leftOffset      = conf.getWindowLeftOffset() * SPS::getWinUnitX(chromaFormatIDC);
    const int     rightOffset     = conf.getWindowRightOffset() * SPS::getWinUnitX(chromaFormatIDC);
    const int     topOffset       = conf.getWindowTopOffset() * SPS::getWinUnitY(chromaFormatIDC);
    const int     bottomOffset    = conf.getWindowBottomOffset() * SPS::getWinUnitY(chromaFormatIDC);
    PictureHash   recon_digest;
    auto numChar = calcMD5WithCropping(((const Picture *) pcPic)->getRecoBuf(), recon_digest, sps->getBitDepths(),
                                       leftOffset, rightOffset, topOffset, bottomOffset);

    const int croppedWidth  = pcPic->Y().width - leftOffset - rightOffset;
    const int croppedHeight = pcPic->Y().height - topOffset - bottomOffset;

    m_oplFileStream << std::setw(3) << pcPic->layerId << ",";
    m_oplFileStream << std::setw(8) << pcPic->getPOC() << "," << std::setw(5) << croppedWidth << "," << std::setw(5)
                    << croppedHeight << "," << hashToString(recon_digest, numChar) << "\n";
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

void DecApp::xCreateDecLib()
{
  initROM();

  // create decoder class
  m_cDecLib.create();

  // initialize decoder class
  m_cDecLib.init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
    m_cacheCfgFile
#endif
  );
  m_cDecLib.setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);


  if (!m_outputDecodedSEIMessagesFilename.empty())
  {
    std::ostream &os=m_seiMessageFileStream.is_open() ? m_seiMessageFileStream : std::cout;
    m_cDecLib.setDecodedSEIMessageOutputStream(&os);
  }
#if JVET_S0257_DUMP_360SEI_MESSAGE
  if (!m_outputDecoded360SEIMessagesFilename.empty())
  {
    m_cDecLib.setDecoded360SEIMessageFileName(m_outputDecoded360SEIMessagesFilename);
  }
#endif
  m_cDecLib.m_targetSubPicIdx = this->m_targetSubPicIdx;
  m_cDecLib.initScalingList();
#if GDR_LEAK_TEST
  m_cDecLib.m_gdrPocRandomAccess = this->m_gdrPocRandomAccess;
#endif // GDR_LEAK_TEST
}

void DecApp::xDestroyDecLib()
{
  if( !m_reconFileName.empty() )
  {
    for( auto & recFile : m_cVideoIOYuvReconFile )
    {
      recFile.second.close();
    }
  }
  if (!m_SEICTIFileName.empty())
  {
    for (auto& recFile : m_cVideoIOYuvSEICTIFile)
    {
      recFile.second.close();
    }
  }

  // destroy decoder class
  m_cDecLib.destroy();
}


/** \param pcListPic list of pictures to be written to file
    \param tId       temporal sub-layer ID
 */
void DecApp::xWriteOutput( PicList* pcListPic, uint32_t tId )
{
  if (pcListPic->empty())
  {
    return;
  }

  PicList::iterator iterPic   = pcListPic->begin();
  int numPicsNotYetDisplayed = 0;
  int dpbFullness = 0;
  const SPS* activeSPS = (pcListPic->front()->cs->sps);
  uint32_t maxNumReorderPicsHighestTid;
  uint32_t maxDecPicBufferingHighestTid;
  uint32_t maxNrSublayers = activeSPS->getMaxTLayers();

  const VPS* referredVPS = pcListPic->front()->cs->vps;
  const int temporalId = ( m_iMaxTemporalLayer == -1 || m_iMaxTemporalLayer >= maxNrSublayers ) ? maxNrSublayers - 1 : m_iMaxTemporalLayer;

  if( referredVPS == nullptr || referredVPS->m_numLayersInOls[referredVPS->m_targetOlsIdx] == 1 )
  {
    maxNumReorderPicsHighestTid = activeSPS->getMaxNumReorderPics( temporalId );
    maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering( temporalId );
  }
  else
  {
    maxNumReorderPicsHighestTid = referredVPS->getMaxNumReorderPics( temporalId );
    maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering( temporalId );
  }

  while (iterPic != pcListPic->end())
  {
    Picture* pcPic = *(iterPic);
    if(pcPic->neededForOutput && pcPic->getPOC() >= m_iPOCLastDisplay)
    {
       numPicsNotYetDisplayed++;
      dpbFullness++;
    }
    else if(pcPic->referenced)
    {
      dpbFullness++;
    }
    iterPic++;
  }

  iterPic = pcListPic->begin();

  if (numPicsNotYetDisplayed>2)
  {
    iterPic++;
  }

  Picture* pcPic = *(iterPic);
  if( numPicsNotYetDisplayed>2 && pcPic->fieldPic ) //Field Decoding
  {
    PicList::iterator endPic   = pcListPic->end();
    endPic--;
    iterPic   = pcListPic->begin();
    while (iterPic != endPic)
    {
      Picture* pcPicTop = *(iterPic);
      iterPic++;
      Picture* pcPicBottom = *(iterPic);

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput &&
          (numPicsNotYetDisplayed >  maxNumReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid) &&
          (!(pcPicTop->getPOC()%2) && pcPicBottom->getPOC() == pcPicTop->getPOC()+1) &&
          (pcPicTop->getPOC() == m_iPOCLastDisplay+1 || m_iPOCLastDisplay < 0))
      {
        // write to file
        numPicsNotYetDisplayed = numPicsNotYetDisplayed-2;
        if ( !m_reconFileName.empty() )
        {
          const Window &conf = pcPicTop->cs->pps->getConformanceWindow();
          const bool isTff = pcPicTop->topField;

          bool display = true;

          if (display)
          {
            m_cVideoIOYuvReconFile[pcPicTop->layerId].write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
                                          m_outputColourSpaceConvert,
                                          false, // TODO: m_packedYUVMode,
                                          conf.getWindowLeftOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowRightOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowTopOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowBottomOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          NUM_CHROMA_FORMAT, isTff );
          }
        }
        writeLineToOutputLog(pcPicTop);
        writeLineToOutputLog(pcPicBottom);

        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( ! pcPicTop->referenced && pcPicTop->reconstructed )
        {
          pcPicTop->reconstructed = false;
        }
        if ( ! pcPicBottom->referenced && pcPicBottom->reconstructed )
        {
          pcPicBottom->reconstructed = false;
        }
        pcPicTop->neededForOutput = false;
        pcPicBottom->neededForOutput = false;
      }
    }
  }
  else if( !pcPic->fieldPic ) //Frame Decoding
  {
    iterPic = pcListPic->begin();

    while (iterPic != pcListPic->end())
    {
      pcPic = *(iterPic);

      if(pcPic->neededForOutput && pcPic->getPOC() >= m_iPOCLastDisplay &&
        (numPicsNotYetDisplayed >  maxNumReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid))
      {
        // write to file
        numPicsNotYetDisplayed--;
        if (!pcPic->referenced)
        {
          dpbFullness--;
        }


        if (!m_reconFileName.empty())
        {
          const Window &conf = pcPic->getConformanceWindow();
          const SPS* sps = pcPic->cs->sps;
          ChromaFormat chromaFormatIDC = sps->getChromaFormatIdc();
          if( m_upscaledOutput )
          {
            m_cVideoIOYuvReconFile[pcPic->layerId].writeUpscaledPicture( *sps, *pcPic->cs->pps, pcPic->getRecoBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
          }
          else
          {
            m_cVideoIOYuvReconFile[pcPic->layerId].write( pcPic->getRecoBuf().get( COMPONENT_Y ).width, pcPic->getRecoBuf().get( COMPONENT_Y ).height, pcPic->getRecoBuf(),
                                        m_outputColourSpaceConvert,
                                        m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
            }
        }
        // Perform CTI on decoded frame and write to output CTI file
        if (!m_SEICTIFileName.empty())
        {
          const Window& conf = pcPic->getConformanceWindow();
          const SPS* sps = pcPic->cs->sps;
          ChromaFormat chromaFormatIDC = sps->getChromaFormatIdc();
          if (m_upscaledOutput)
          {
            m_cVideoIOYuvSEICTIFile[pcPic->layerId].writeUpscaledPicture(*sps, *pcPic->cs->pps, pcPic->getDisplayBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range);
          }
          else
          {
            m_cVideoIOYuvSEICTIFile[pcPic->layerId].write(pcPic->getRecoBuf().get(COMPONENT_Y).width, pcPic->getRecoBuf().get(COMPONENT_Y).height, pcPic->getDisplayBuf(),
              m_outputColourSpaceConvert, m_packedYUVMode,
              conf.getWindowLeftOffset() * SPS::getWinUnitX(chromaFormatIDC),
              conf.getWindowRightOffset() * SPS::getWinUnitX(chromaFormatIDC),
              conf.getWindowTopOffset() * SPS::getWinUnitY(chromaFormatIDC),
              conf.getWindowBottomOffset() * SPS::getWinUnitY(chromaFormatIDC),
              NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range);
          }
        }
        writeLineToOutputLog(pcPic);

        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if (!pcPic->referenced && pcPic->reconstructed)
        {
          pcPic->reconstructed = false;
        }
        pcPic->neededForOutput = false;
      }

      iterPic++;
    }
  }
}

/** \param pcListPic list of pictures to be written to file
 */
void DecApp::xFlushOutput( PicList* pcListPic, const int layerId )
{
  if(!pcListPic || pcListPic->empty())
  {
    return;
  }
  PicList::iterator iterPic   = pcListPic->begin();

  iterPic   = pcListPic->begin();
  Picture* pcPic = *(iterPic);

  if (pcPic->fieldPic ) //Field Decoding
  {
    PicList::iterator endPic   = pcListPic->end();
    endPic--;
    Picture *pcPicTop, *pcPicBottom = NULL;
    while (iterPic != endPic)
    {
      pcPicTop = *(iterPic);
      iterPic++;
      pcPicBottom = *(iterPic);

      if( pcPicTop->layerId != layerId && layerId != NOT_VALID )
      {
        continue;
      }

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput && !(pcPicTop->getPOC()%2) && (pcPicBottom->getPOC() == pcPicTop->getPOC()+1) )
      {
          // write to file
          if ( !m_reconFileName.empty() )
          {
            const Window &conf = pcPicTop->cs->pps->getConformanceWindow();
            const bool    isTff   = pcPicTop->topField;

            m_cVideoIOYuvReconFile[pcPicTop->layerId].write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
                                        m_outputColourSpaceConvert,
                                        false, // TODO: m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        NUM_CHROMA_FORMAT, isTff );
          }
          writeLineToOutputLog(pcPicTop);
          writeLineToOutputLog(pcPicBottom);
        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if( ! pcPicTop->referenced && pcPicTop->reconstructed )
        {
          pcPicTop->reconstructed = false;
        }
        if( ! pcPicBottom->referenced && pcPicBottom->reconstructed )
        {
          pcPicBottom->reconstructed = false;
        }
        pcPicTop->neededForOutput = false;
        pcPicBottom->neededForOutput = false;

        if(pcPicTop)
        {
          pcPicTop->destroy();
          delete pcPicTop;
          pcPicTop = NULL;
        }
      }
    }
    if(pcPicBottom)
    {
      pcPicBottom->destroy();
      delete pcPicBottom;
      pcPicBottom = NULL;
    }
  }
  else //Frame decoding
  {
    while (iterPic != pcListPic->end())
    {
      pcPic = *(iterPic);

      if( pcPic->layerId != layerId && layerId != NOT_VALID )
      {
        iterPic++;
        continue;
      }

      if (pcPic->neededForOutput)
      {
          // write to file
          if (!m_reconFileName.empty())
          {
            const Window &conf = pcPic->getConformanceWindow();
            const SPS* sps = pcPic->cs->sps;
            ChromaFormat chromaFormatIDC = sps->getChromaFormatIdc();
            if( m_upscaledOutput )
            {
              m_cVideoIOYuvReconFile[pcPic->layerId].writeUpscaledPicture( *sps, *pcPic->cs->pps, pcPic->getRecoBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
            }
            else
            {
              m_cVideoIOYuvReconFile[pcPic->layerId].write( pcPic->getRecoBuf().get( COMPONENT_Y ).width, pcPic->getRecoBuf().get( COMPONENT_Y ).height, pcPic->getRecoBuf(),
                                        m_outputColourSpaceConvert,
                                        m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
              }
          }
          // Perform CTI on decoded frame and write to output CTI file
          if (!m_SEICTIFileName.empty())
          {
            const Window& conf = pcPic->getConformanceWindow();
            const SPS* sps = pcPic->cs->sps;
            ChromaFormat chromaFormatIDC = sps->getChromaFormatIdc();
            if (m_upscaledOutput)
            {
              m_cVideoIOYuvSEICTIFile[pcPic->layerId].writeUpscaledPicture(*sps, *pcPic->cs->pps, pcPic->getDisplayBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range);
            }
            else
            {
              m_cVideoIOYuvSEICTIFile[pcPic->layerId].write(pcPic->getRecoBuf().get(COMPONENT_Y).width, pcPic->getRecoBuf().get(COMPONENT_Y).height, pcPic->getDisplayBuf(),
                m_outputColourSpaceConvert, m_packedYUVMode,
                conf.getWindowLeftOffset() * SPS::getWinUnitX(chromaFormatIDC),
                conf.getWindowRightOffset() * SPS::getWinUnitX(chromaFormatIDC),
                conf.getWindowTopOffset() * SPS::getWinUnitY(chromaFormatIDC),
                conf.getWindowBottomOffset() * SPS::getWinUnitY(chromaFormatIDC),
                NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range);
            }
          }
          writeLineToOutputLog(pcPic);
        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if (!pcPic->referenced && pcPic->reconstructed)
        {
          pcPic->reconstructed = false;
        }
        pcPic->neededForOutput = false;
      }
      if(pcPic != NULL)
      {
        pcPic->destroy();
        delete pcPic;
        pcPic = NULL;
        *iterPic = nullptr;
      }
      iterPic++;
    }
  }

  if( layerId != NOT_VALID )
  {
    pcListPic->remove_if([](Picture* p) { return p == nullptr; });
  }
  else
  pcListPic->clear();
  m_iPOCLastDisplay = -MAX_INT;
}

/** \param pcListPic list of pictures to be written to file
 */
void DecApp::xOutputAnnotatedRegions(PicList* pcListPic)
{
  if(!pcListPic || pcListPic->empty())
  {
    return;
  }
  PicList::iterator iterPic   = pcListPic->begin();

  while (iterPic != pcListPic->end())
  {
    Picture* pcPic = *(iterPic);
    if (pcPic->neededForOutput)
    {
      // Check if any annotated region SEI has arrived
      SEIMessages annotatedRegionSEIs = getSeisByType(pcPic->SEIs, SEI::ANNOTATED_REGIONS);
      for(auto it=annotatedRegionSEIs.begin(); it!=annotatedRegionSEIs.end(); it++)
      {
        const SEIAnnotatedRegions &seiAnnotatedRegions = *(SEIAnnotatedRegions*)(*it);

        if (seiAnnotatedRegions.m_hdr.m_cancelFlag)
        {
          m_arObjects.clear();
          m_arLabels.clear();
        }
        else
        {
          if (m_arHeader.m_receivedSettingsOnce)
          {
            // validate those settings that must stay constant are constant.
            assert(m_arHeader.m_occludedObjectFlag              == seiAnnotatedRegions.m_hdr.m_occludedObjectFlag);
            assert(m_arHeader.m_partialObjectFlagPresentFlag    == seiAnnotatedRegions.m_hdr.m_partialObjectFlagPresentFlag);
            assert(m_arHeader.m_objectConfidenceInfoPresentFlag == seiAnnotatedRegions.m_hdr.m_objectConfidenceInfoPresentFlag);
            assert((!m_arHeader.m_objectConfidenceInfoPresentFlag) || m_arHeader.m_objectConfidenceLength == seiAnnotatedRegions.m_hdr.m_objectConfidenceLength);
          }
          else
          {
            m_arHeader.m_receivedSettingsOnce=true;
            m_arHeader=seiAnnotatedRegions.m_hdr; // copy the settings.
          }
          // Process label updates
          if (seiAnnotatedRegions.m_hdr.m_objectLabelPresentFlag)
          {
            for(auto srcIt=seiAnnotatedRegions.m_annotatedLabels.begin(); srcIt!=seiAnnotatedRegions.m_annotatedLabels.end(); srcIt++)
            {
              const uint32_t labIdx = srcIt->first;
              if (srcIt->second.labelValid)
              {
                m_arLabels[labIdx] = srcIt->second.label;
              }
              else
              {
                m_arLabels.erase(labIdx);
              }
            }
          }

          // Process object updates
          for(auto srcIt=seiAnnotatedRegions.m_annotatedRegions.begin(); srcIt!=seiAnnotatedRegions.m_annotatedRegions.end(); srcIt++)
          {
            uint32_t objIdx = srcIt->first;
            const SEIAnnotatedRegions::AnnotatedRegionObject &src =srcIt->second;

            if (src.objectCancelFlag)
            {
              m_arObjects.erase(objIdx);
            }
            else
            {
              auto destIt = m_arObjects.find(objIdx);

              if (destIt == m_arObjects.end())
              {
                //New object arrived, needs to be appended to the map of tracked objects
                m_arObjects[objIdx] = src;
              }
              else //Existing object, modifications to be done
              {
                SEIAnnotatedRegions::AnnotatedRegionObject &dst=destIt->second;

                if (seiAnnotatedRegions.m_hdr.m_objectLabelPresentFlag && src.objectLabelValid)
                {
                  dst.objectLabelValid=true;
                  dst.objLabelIdx = src.objLabelIdx;
                }
                if (src.boundingBoxValid)
                {
                  dst.boundingBoxTop    = src.boundingBoxTop   ;
                  dst.boundingBoxLeft   = src.boundingBoxLeft  ;
                  dst.boundingBoxWidth  = src.boundingBoxWidth ;
                  dst.boundingBoxHeight = src.boundingBoxHeight;
                  if (seiAnnotatedRegions.m_hdr.m_partialObjectFlagPresentFlag)
                  {
                    dst.partialObjectFlag = src.partialObjectFlag;
                  }
                  if (seiAnnotatedRegions.m_hdr.m_objectConfidenceInfoPresentFlag)
                  {
                    dst.objectConfidence = src.objectConfidence;
                  }
                }
              }
            }
          }
        }
      }

      if (!m_arObjects.empty())
      {
        FILE *fpPersist = fopen(m_annotatedRegionsSEIFileName.c_str(), "ab");
        if (fpPersist == NULL)
        {
          std::cout << "Not able to open file for writing persist SEI messages" << std::endl;
        }
        else
        {
          fprintf(fpPersist, "\n");
          fprintf(fpPersist, "Number of objects = %d\n", (int)m_arObjects.size());
          for (auto it = m_arObjects.begin(); it != m_arObjects.end(); ++it)
          {
            fprintf(fpPersist, "Object Idx = %d\n",    it->first);
            fprintf(fpPersist, "Object Top = %d\n",    it->second.boundingBoxTop);
            fprintf(fpPersist, "Object Left = %d\n",   it->second.boundingBoxLeft);
            fprintf(fpPersist, "Object Width = %d\n",  it->second.boundingBoxWidth);
            fprintf(fpPersist, "Object Height = %d\n", it->second.boundingBoxHeight);
            if (it->second.objectLabelValid)
            {
              auto labelIt=m_arLabels.find(it->second.objLabelIdx);
              fprintf(fpPersist, "Object Label = %s\n", labelIt!=m_arLabels.end() ? (labelIt->second.c_str()) : "<UNKNOWN>");
            }
            if (m_arHeader.m_partialObjectFlagPresentFlag)
            {
              fprintf(fpPersist, "Object Partial = %d\n", it->second.partialObjectFlag?1:0);
            }
            if (m_arHeader.m_objectConfidenceInfoPresentFlag)
            {
              fprintf(fpPersist, "Object Conf = %d\n", it->second.objectConfidence);
            }
          }
          fclose(fpPersist);
        }
      }
    }
   iterPic++;
  }
}

/** \param nalu Input nalu to check whether its LayerId is within targetDecLayerIdSet
 */
bool DecApp::xIsNaluWithinTargetDecLayerIdSet( const InputNALUnit* nalu ) const
{
  if( !m_targetDecLayerIdSet.size() ) // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }

  return std::find( m_targetDecLayerIdSet.begin(), m_targetDecLayerIdSet.end(), nalu->m_nuhLayerId ) != m_targetDecLayerIdSet.end();
}

/** \param nalu Input nalu to check whether its LayerId is within targetOutputLayerIdSet
 */
bool DecApp::xIsNaluWithinTargetOutputLayerIdSet( const InputNALUnit* nalu ) const
{
  if( !m_targetOutputLayerIdSet.size() ) // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }

  return std::find( m_targetOutputLayerIdSet.begin(), m_targetOutputLayerIdSet.end(), nalu->m_nuhLayerId ) != m_targetOutputLayerIdSet.end();
}

//! \}
