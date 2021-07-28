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

/**
 * \file
 * \brief Implementation of AffineGradientSearch class
 */
//#define USE_AVX2
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "CommonDefX86.h"
#include "../AffineGradientSearch.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86

#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

#define CALC_EQUAL_COEFF_8PXLS(x1,x2,y1,y2,tmp0,tmp1,tmp2,tmp3,inter0,inter1,inter2,inter3,loadLocation)       \
{                                                                                                              \
inter0 = _mm_mul_epi32(x1, y1);                                                                                \
inter1 = _mm_mul_epi32(tmp0, tmp2);                                                                            \
inter2 = _mm_mul_epi32(x2, y2);                                                                                \
inter3 = _mm_mul_epi32(tmp1, tmp3);                                                                            \
inter2 = _mm_add_epi64(inter0, inter2);                                                                        \
inter3 = _mm_add_epi64(inter1, inter3);                                                                        \
inter0 = _mm_loadl_epi64(loadLocation);                                                                        \
inter3 = _mm_add_epi64(inter2, inter3);                                                                        \
inter1 = _mm_srli_si128(inter3, 8);                                                                            \
inter3 = _mm_add_epi64(inter1, inter3);                                                                        \
inter3 = _mm_add_epi64(inter0, inter3);                                                                        \
}

template<X86_VEXT vext>
static void simdHorizontalSobelFilter( Pel *const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height )
{
  __m128i mmPred[4];
  __m128i mm2xPred[2];
  __m128i mmIntermediates[4];
  __m128i mmDerivate[2];

  assert( !(height % 2) );
  assert( !(width % 4) );

  /* Derivates of the rows and columns at the boundary are done at the end of this function */
  /* The value of col and row indicate the columns and rows for which the derivates have already been computed */
  for ( int col = 1; (col + 2) < width; col += 2 )
  {
    mmPred[0] = _mm_loadl_epi64( reinterpret_cast<const __m128i *>(&pPred[0 * predStride + col - 1]) );
    mmPred[1] = _mm_loadl_epi64( reinterpret_cast<const __m128i *>(&pPred[1 * predStride + col - 1]) );

    mmPred[0] = _mm_cvtepi16_epi32( mmPred[0] );
    mmPred[1] = _mm_cvtepi16_epi32( mmPred[1] );

    for ( int row = 1; row < (height - 1); row += 2 )
    {
      mmPred[2] = _mm_loadl_epi64( reinterpret_cast<const __m128i *>(&pPred[(row + 1) * predStride + col - 1]) );
      mmPred[3] = _mm_loadl_epi64( reinterpret_cast<const __m128i *>(&pPred[(row + 2) * predStride + col - 1]) );

      mmPred[2] = _mm_cvtepi16_epi32( mmPred[2] );
      mmPred[3] = _mm_cvtepi16_epi32( mmPred[3] );

      mm2xPred[0] = _mm_slli_epi32( mmPred[1], 1 );
      mm2xPred[1] = _mm_slli_epi32( mmPred[2], 1 );

      mmIntermediates[0] = _mm_add_epi32( mm2xPred[0], mmPred[0] );
      mmIntermediates[2] = _mm_add_epi32( mm2xPred[1], mmPred[1] );

      mmIntermediates[0] = _mm_add_epi32( mmIntermediates[0], mmPred[2] );
      mmIntermediates[2] = _mm_add_epi32( mmIntermediates[2], mmPred[3] );

      mmPred[0] = mmPred[2];
      mmPred[1] = mmPred[3];

      mmIntermediates[1] = _mm_srli_si128( mmIntermediates[0], 8 );
      mmIntermediates[3] = _mm_srli_si128( mmIntermediates[2], 8 );

      mmDerivate[0] = _mm_sub_epi32( mmIntermediates[1], mmIntermediates[0] );
      mmDerivate[1] = _mm_sub_epi32( mmIntermediates[3], mmIntermediates[2] );

      _mm_storel_epi64( reinterpret_cast<__m128i *> (&pDerivate[col + (row + 0) * derivateBufStride]), mmDerivate[0] );
      _mm_storel_epi64( reinterpret_cast<__m128i *> (&pDerivate[col + (row + 1) * derivateBufStride]), mmDerivate[1] );
    }
  }

  for ( int j = 1; j < (height - 1); j++ )
  {
    pDerivate[j * derivateBufStride] = pDerivate[j * derivateBufStride + 1];
    pDerivate[j * derivateBufStride + (width - 1)] = pDerivate[j * derivateBufStride + (width - 2)];
  }

  memcpy( pDerivate, pDerivate + derivateBufStride, width * sizeof( pDerivate[0] ) );
  memcpy( pDerivate + (height - 1) * derivateBufStride, pDerivate + (height - 2) * derivateBufStride, width * sizeof( pDerivate[0] )
  );
}

template<X86_VEXT vext>
static void simdVerticalSobelFilter( Pel *const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height )
{
  __m128i mmPred[4];
  __m128i mmIntermediates[6];
  __m128i mmDerivate[2];

  assert( !(height % 2) );
  assert( !(width % 4) );

  /* Derivates of the rows and columns at the boundary are done at the end of this function */
  /* The value of col and row indicate the columns and rows for which the derivates have already been computed */
  for ( int col = 1; col < (width - 1); col += 2 )
  {
    mmPred[0] = _mm_loadl_epi64( reinterpret_cast<const __m128i *>(&pPred[0 * predStride + col - 1]) );
    mmPred[1] = _mm_loadl_epi64( reinterpret_cast<const __m128i *>(&pPred[1 * predStride + col - 1]) );

    mmPred[0] = _mm_cvtepi16_epi32( mmPred[0] );
    mmPred[1] = _mm_cvtepi16_epi32( mmPred[1] );

    for ( int row = 1; row < (height - 1); row += 2 )
    {
      mmPred[2] = _mm_loadl_epi64( reinterpret_cast<const __m128i *>(&pPred[(row + 1) * predStride + col - 1]) );
      mmPred[3] = _mm_loadl_epi64( reinterpret_cast<const __m128i *>(&pPred[(row + 2) * predStride + col - 1]) );

      mmPred[2] = _mm_cvtepi16_epi32( mmPred[2] );
      mmPred[3] = _mm_cvtepi16_epi32( mmPred[3] );

      mmIntermediates[0] = _mm_sub_epi32( mmPred[2], mmPred[0] );
      mmIntermediates[3] = _mm_sub_epi32( mmPred[3], mmPred[1] );

      mmPred[0] = mmPred[2];
      mmPred[1] = mmPred[3];

      mmIntermediates[1] = _mm_srli_si128( mmIntermediates[0], 4 );
      mmIntermediates[4] = _mm_srli_si128( mmIntermediates[3], 4 );
      mmIntermediates[2] = _mm_srli_si128( mmIntermediates[0], 8 );
      mmIntermediates[5] = _mm_srli_si128( mmIntermediates[3], 8 );

      mmIntermediates[1] = _mm_slli_epi32( mmIntermediates[1], 1 );
      mmIntermediates[4] = _mm_slli_epi32( mmIntermediates[4], 1 );

      mmIntermediates[0] = _mm_add_epi32( mmIntermediates[0], mmIntermediates[2] );
      mmIntermediates[3] = _mm_add_epi32( mmIntermediates[3], mmIntermediates[5] );

      mmDerivate[0] = _mm_add_epi32( mmIntermediates[0], mmIntermediates[1] );
      mmDerivate[1] = _mm_add_epi32( mmIntermediates[3], mmIntermediates[4] );

      _mm_storel_epi64( reinterpret_cast<__m128i *> (&pDerivate[col + (row + 0) * derivateBufStride]), mmDerivate[0] );
      _mm_storel_epi64( reinterpret_cast<__m128i *> (&pDerivate[col + (row + 1) * derivateBufStride]), mmDerivate[1] );
    }
  }

  for ( int j = 1; j < (height - 1); j++ )
  {
    pDerivate[j * derivateBufStride] = pDerivate[j * derivateBufStride + 1];
    pDerivate[j * derivateBufStride + (width - 1)] = pDerivate[j * derivateBufStride + (width - 2)];
  }

  memcpy( pDerivate, pDerivate + derivateBufStride, width * sizeof( pDerivate[0] ) );
  memcpy( pDerivate + (height - 1) * derivateBufStride, pDerivate + (height - 2) * derivateBufStride, width * sizeof( pDerivate[0] ) );
}

template<X86_VEXT vext>
static void simdEqualCoeffComputer( Pel *pResidue, int residueStride, int **ppDerivate, int derivateBufStride, int64_t( *pEqualCoeff )[7], int width, int height, bool b6Param )
{
  __m128i mmFour;
  __m128i mmTmp[4];
  __m128i mmIntermediate[4];
  __m128i mmIndxK, mmIndxJ;
  __m128i mmResidue[2];
  __m128i mmC[12];

  // Add directly to indexes to get new index
  mmFour = _mm_set1_epi32(4);
  mmIndxJ = _mm_set1_epi32(-2);


  int n = b6Param ? 6 : 4;
  int idx1 = 0, idx2 = 0;
  idx1 = -2 * derivateBufStride - 4;
  idx2 = -derivateBufStride - 4;

  for ( int j = 0; j < height; j += 2 )
  {
    if (!(j & 3))
      mmIndxJ = _mm_add_epi32(mmIndxJ, mmFour);
    mmIndxK = _mm_set1_epi32(-2);
    idx1 += (derivateBufStride << 1);
    idx2 += (derivateBufStride << 1);

    for ( int k = 0; k < width; k += 4 )
    {
      idx1 += 4;
      idx2 += 4;
      mmIndxK = _mm_add_epi32( mmIndxK, mmFour );

      if ( b6Param )
      {
        // mmC[0-5] for iC[0-5] of 1st row of pixels
        mmC[0] = _mm_loadu_si128( (const __m128i*)&ppDerivate[0][idx1] );
        mmC[2] = _mm_loadu_si128( (const __m128i*)&ppDerivate[1][idx1] );
        mmC[1] = _mm_mullo_epi32( mmIndxK, mmC[0] );
        mmC[3] = _mm_mullo_epi32( mmIndxK, mmC[2] );
        mmC[4] = _mm_mullo_epi32(mmIndxJ, mmC[0]);
        mmC[5] = _mm_mullo_epi32(mmIndxJ, mmC[2]);

        // mmC[6-11] for iC[0-5] of 2nd row of pixels
        mmC[6] = _mm_loadu_si128( (const __m128i*)&ppDerivate[0][idx2] );
        mmC[8] = _mm_loadu_si128( (const __m128i*)&ppDerivate[1][idx2] );
        mmC[7] = _mm_mullo_epi32( mmIndxK, mmC[6] );
        mmC[9] = _mm_mullo_epi32( mmIndxK, mmC[8] );
        mmC[10] = _mm_mullo_epi32(mmIndxJ, mmC[6]);
        mmC[11] = _mm_mullo_epi32(mmIndxJ, mmC[8]);
      }
      else
      {
        // mmC[0-3] for iC[0-3] of 1st row of pixels
        mmC[0] = _mm_loadu_si128( (const __m128i*)&ppDerivate[0][idx1] );
        mmC[2] = _mm_loadu_si128( (const __m128i*)&ppDerivate[1][idx1] );
        mmC[1] = _mm_mullo_epi32( mmIndxK, mmC[0] );
        mmC[3] = _mm_mullo_epi32(mmIndxJ, mmC[0]);
        mmTmp[0] = _mm_mullo_epi32(mmIndxJ, mmC[2]);
        mmTmp[1] = _mm_mullo_epi32( mmIndxK, mmC[2] );
        mmC[1] = _mm_add_epi32( mmC[1], mmTmp[0] );
        mmC[3] = _mm_sub_epi32( mmC[3], mmTmp[1] );

        // mmC[4-7] for iC[0-3] of 1st row of pixels
        mmC[4] = _mm_loadu_si128( (const __m128i*)&ppDerivate[0][idx2] );
        mmC[6] = _mm_loadu_si128( (const __m128i*)&ppDerivate[1][idx2] );
        mmC[5] = _mm_mullo_epi32( mmIndxK, mmC[4] );
        mmC[7] = _mm_mullo_epi32(mmIndxJ, mmC[4]);
        mmTmp[2] = _mm_mullo_epi32(mmIndxJ, mmC[6]);
        mmTmp[3] = _mm_mullo_epi32( mmIndxK, mmC[6] );
        mmC[5] = _mm_add_epi32( mmC[5], mmTmp[2] );
        mmC[7] = _mm_sub_epi32( mmC[7], mmTmp[3] );
      }

      // Residue
      mmResidue[0] = _mm_loadl_epi64( (const __m128i*)&pResidue[idx1] );
      mmResidue[1] = _mm_loadl_epi64( (const __m128i*)&pResidue[idx2] );
      mmResidue[0] = _mm_cvtepi16_epi32( mmResidue[0] );
      mmResidue[1] = _mm_cvtepi16_epi32( mmResidue[1] );
      mmResidue[0] = _mm_slli_epi32( mmResidue[0], 3 );
      mmResidue[1] = _mm_slli_epi32( mmResidue[1], 3 );

      // Calculation of coefficient matrix
      for ( int col = 0; col < n; col++ )
      {
        mmTmp[0] = _mm_srli_si128( mmC[0 + col], 4 );
        mmTmp[1] = _mm_srli_si128( mmC[n + col], 4 );
        CALC_EQUAL_COEFF_8PXLS( mmC[0 + col], mmC[n + col], mmC[0 + col], mmC[n + col], mmTmp[0], mmTmp[1], mmTmp[0], mmTmp[1], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], (const __m128i*)&pEqualCoeff[col + 1][col] );
        _mm_storel_epi64( (__m128i*)&pEqualCoeff[col + 1][col], mmIntermediate[3] );

        for ( int row = col + 1; row < n; row++ )
        {
          mmTmp[2] = _mm_srli_si128( mmC[0 + row], 4 );
          mmTmp[3] = _mm_srli_si128( mmC[n + row], 4 );
          CALC_EQUAL_COEFF_8PXLS( mmC[0 + col], mmC[n + col], mmC[0 + row], mmC[n + row], mmTmp[0], mmTmp[1], mmTmp[2], mmTmp[3], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], (const __m128i*)&pEqualCoeff[col + 1][row] );
          _mm_storel_epi64( (__m128i*)&pEqualCoeff[col + 1][row], mmIntermediate[3] );
          _mm_storel_epi64( (__m128i*)&pEqualCoeff[row + 1][col], mmIntermediate[3] );
        }

        mmTmp[2] = _mm_srli_si128( mmResidue[0], 4 );
        mmTmp[3] = _mm_srli_si128( mmResidue[1], 4 );
        CALC_EQUAL_COEFF_8PXLS( mmC[0 + col], mmC[n + col], mmResidue[0], mmResidue[1], mmTmp[0], mmTmp[1], mmTmp[2], mmTmp[3], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], (const __m128i*)&pEqualCoeff[col + 1][n] );
        _mm_storel_epi64( (__m128i*)&pEqualCoeff[col + 1][n], mmIntermediate[3] );
      }
    }

    idx1 -= (width);
    idx2 -= (width);
  }
}
#if RExt__HIGH_BIT_DEPTH_SUPPORT
template<X86_VEXT vext>
static void simdHorizontalSobelFilter_HBD_SIMD(Pel *const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height)
{
  __m128i pred[4];
  __m128i pred2x[2];
  __m128i intermediates[4];
  __m128i derivate[2];

  assert(!(height % 2));
  assert(!(width % 4));

  /* Derivates of the rows and columns at the boundary are done at the end of this function */
  /* The value of col and row indicate the columns and rows for which the derivates have already been computed */

  int col = 1;
#if USE_AVX2
  if (vext >= AVX2)
  {
    __m256i pred256[4];
    __m256i pred2x256[2];
    __m256i intermediates256[4];
    __m256i derivate256[2];

    for (; (col + 6) < width; col += 6)
    {
      pred256[0] = _mm256_lddqu_si256((__m256i *)&pPred[col - 1]);
      pred256[1] = _mm256_lddqu_si256((__m256i *)&pPred[predStride + col - 1]);

      for (int row = 1; row < (height - 1); row += 2)
      {
        pred256[2] = _mm256_lddqu_si256((__m256i *)&pPred[(row + 1) * predStride + col - 1]);
        pred256[3] = _mm256_lddqu_si256((__m256i *)&pPred[(row + 2) * predStride + col - 1]);

        pred2x256[0] = _mm256_slli_epi32(pred256[1], 1);
        pred2x256[1] = _mm256_slli_epi32(pred256[2], 1);

        intermediates256[0] = _mm256_add_epi32(pred2x256[0], pred256[0]);
        intermediates256[2] = _mm256_add_epi32(pred2x256[1], pred256[1]);

        intermediates256[0] = _mm256_add_epi32(intermediates256[0], pred256[2]);
        intermediates256[2] = _mm256_add_epi32(intermediates256[2], pred256[3]);

        pred256[0] = pred256[2];
        pred256[1] = pred256[3];

        intermediates256[1] = _mm256_permute4x64_epi64(intermediates256[0], 0xf9);
        intermediates256[3] = _mm256_permute4x64_epi64(intermediates256[2], 0xf9);

        derivate256[0] = _mm256_sub_epi32(intermediates256[1], intermediates256[0]);
        derivate256[1] = _mm256_sub_epi32(intermediates256[3], intermediates256[2]);

        _mm_storeu_si128((__m128i *)&pDerivate[col + row * derivateBufStride], _mm256_castsi256_si128(derivate256[0]));
        _mm_storel_epi64((__m128i *)&pDerivate[col + 4 + row * derivateBufStride], _mm256_castsi256_si128(_mm256_permute4x64_epi64(derivate256[0], 0xaa)));

        _mm_storeu_si128((__m128i *)&pDerivate[col + (row + 1) * derivateBufStride], _mm256_castsi256_si128(derivate256[1]));
        _mm_storel_epi64((__m128i *)&pDerivate[col + 4 + (row + 1) * derivateBufStride], _mm256_castsi256_si128(_mm256_permute4x64_epi64(derivate256[1], 0xaa)));
      }
    }
  }
#endif

  for (; (col + 2) < width; col += 2)
  {
    pred[0] = _mm_lddqu_si128((__m128i *)&pPred[col - 1]);
    pred[1] = _mm_lddqu_si128((__m128i *)&pPred[predStride + col - 1]);

    for (int row = 1; row < (height - 1); row += 2)
    {
      pred[2] = _mm_lddqu_si128((__m128i *)&pPred[(row + 1) * predStride + col - 1]);
      pred[3] = _mm_lddqu_si128((__m128i *)&pPred[(row + 2) * predStride + col - 1]);

      pred2x[0] = _mm_slli_epi32(pred[1], 1);
      pred2x[1] = _mm_slli_epi32(pred[2], 1);

      intermediates[0] = _mm_add_epi32(pred2x[0], pred[0]);
      intermediates[2] = _mm_add_epi32(pred2x[1], pred[1]);

      intermediates[0] = _mm_add_epi32(intermediates[0], pred[2]);
      intermediates[2] = _mm_add_epi32(intermediates[2], pred[3]);

      pred[0] = pred[2];
      pred[1] = pred[3];

      intermediates[1] = _mm_srli_si128(intermediates[0], 8);
      intermediates[3] = _mm_srli_si128(intermediates[2], 8);

      derivate[0] = _mm_sub_epi32(intermediates[1], intermediates[0]);
      derivate[1] = _mm_sub_epi32(intermediates[3], intermediates[2]);

      _mm_storel_epi64((__m128i *)&pDerivate[col + row * derivateBufStride], derivate[0]);
      _mm_storel_epi64((__m128i *)&pDerivate[col + (row + 1) * derivateBufStride], derivate[1]);
    }
  }

  for (int j = 1; j < (height - 1); j++)
  {
    pDerivate[j * derivateBufStride] = pDerivate[j * derivateBufStride + 1];
    pDerivate[j * derivateBufStride + (width - 1)] = pDerivate[j * derivateBufStride + (width - 2)];
  }

  memcpy(pDerivate, pDerivate + derivateBufStride, width * sizeof(pDerivate[0]));
  memcpy(pDerivate + (height - 1) * derivateBufStride, pDerivate + (height - 2) * derivateBufStride, width * sizeof(pDerivate[0])
  );
}

template<X86_VEXT vext>
static void simdVerticalSobelFilter_HBD_SIMD(Pel *const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height)
{
  __m128i pred[4];
  __m128i intermediates[6];
  __m128i derivate[2];

  assert(!(height % 2));
  assert(!(width % 4));

  int col = 1;
#if USE_AVX2
  if (vext >= AVX2)
  {
    __m256i pred256[4];
    __m256i intermediates256[6];
    __m256i derivate256[2];
    __m256i shuffle256 = _mm256_set_epi32(0x00000007, 0x00000007, 0x00000006, 0x00000005, 0x00000004, 0x00000003, 0x00000002, 0x00000001);

    for (; (col + 6) < width; col += 6)
    {
      pred256[0] = _mm256_lddqu_si256((__m256i *)&pPred[col - 1]);
      pred256[1] = _mm256_lddqu_si256((__m256i *)&pPred[predStride + col - 1]);

      for (int row = 1; row < (height - 1); row += 2)
      {
        pred256[2] = _mm256_lddqu_si256((__m256i *)&pPred[(row + 1) * predStride + col - 1]);
        pred256[3] = _mm256_lddqu_si256((__m256i *)&pPred[(row + 2) * predStride + col - 1]);

        intermediates256[0] = _mm256_sub_epi32(pred256[2], pred256[0]);
        intermediates256[3] = _mm256_sub_epi32(pred256[3], pred256[1]);

        pred256[0] = pred256[2];
        pred256[1] = pred256[3];

        intermediates256[1] = _mm256_permutevar8x32_epi32(intermediates256[0], shuffle256);
        intermediates256[4] = _mm256_permutevar8x32_epi32(intermediates256[3], shuffle256);
        intermediates256[2] = _mm256_permute4x64_epi64(intermediates256[0], 0xf9);
        intermediates256[5] = _mm256_permute4x64_epi64(intermediates256[3], 0xf9);

        intermediates256[1] = _mm256_slli_epi32(intermediates256[1], 1);
        intermediates256[4] = _mm256_slli_epi32(intermediates256[4], 1);

        intermediates256[0] = _mm256_add_epi32(intermediates256[0], intermediates256[2]);
        intermediates256[3] = _mm256_add_epi32(intermediates256[3], intermediates256[5]);

        derivate256[0] = _mm256_add_epi32(intermediates256[0], intermediates256[1]);
        derivate256[1] = _mm256_add_epi32(intermediates256[3], intermediates256[4]);

        _mm_storeu_si128((__m128i *)&pDerivate[col + row * derivateBufStride], _mm256_castsi256_si128(derivate256[0]));
        _mm_storel_epi64((__m128i *)&pDerivate[col + 4 + row * derivateBufStride], _mm256_castsi256_si128(_mm256_permute4x64_epi64(derivate256[0], 0xaa)));

        _mm_storeu_si128((__m128i *)&pDerivate[col + (row + 1) * derivateBufStride], _mm256_castsi256_si128(derivate256[1]));
        _mm_storel_epi64((__m128i *)&pDerivate[col + 4 + (row + 1) * derivateBufStride], _mm256_castsi256_si128(_mm256_permute4x64_epi64(derivate256[1], 0xaa)));
      }
    }
  }
#endif

  /* Derivates of the rows and columns at the boundary are done at the end of this function */
  /* The value of col and row indicate the columns and rows for which the derivates have already been computed */
  for (; (col + 2) < width; col += 2)
  {
    pred[0] = _mm_lddqu_si128((__m128i *)&pPred[col - 1]);
    pred[1] = _mm_lddqu_si128((__m128i *)&pPred[predStride + col - 1]);

    for (int row = 1; row < (height - 1); row += 2)
    {
      pred[2] = _mm_lddqu_si128((__m128i *)&pPred[(row + 1) * predStride + col - 1]);
      pred[3] = _mm_lddqu_si128((__m128i *)&pPred[(row + 2) * predStride + col - 1]);

      intermediates[0] = _mm_sub_epi32(pred[2], pred[0]);
      intermediates[3] = _mm_sub_epi32(pred[3], pred[1]);

      pred[0] = pred[2];
      pred[1] = pred[3];

      intermediates[1] = _mm_srli_si128(intermediates[0], 4);
      intermediates[4] = _mm_srli_si128(intermediates[3], 4);
      intermediates[2] = _mm_srli_si128(intermediates[0], 8);
      intermediates[5] = _mm_srli_si128(intermediates[3], 8);

      intermediates[1] = _mm_slli_epi32(intermediates[1], 1);
      intermediates[4] = _mm_slli_epi32(intermediates[4], 1);

      intermediates[0] = _mm_add_epi32(intermediates[0], intermediates[2]);
      intermediates[3] = _mm_add_epi32(intermediates[3], intermediates[5]);

      derivate[0] = _mm_add_epi32(intermediates[0], intermediates[1]);
      derivate[1] = _mm_add_epi32(intermediates[3], intermediates[4]);

      _mm_storel_epi64((__m128i *)&pDerivate[col + row * derivateBufStride], derivate[0]);
      _mm_storel_epi64((__m128i *)&pDerivate[col + (row + 1) * derivateBufStride], derivate[1]);
    }
  }

  for (int j = 1; j < (height - 1); j++)
  {
    pDerivate[j * derivateBufStride] = pDerivate[j * derivateBufStride + 1];
    pDerivate[j * derivateBufStride + (width - 1)] = pDerivate[j * derivateBufStride + (width - 2)];
  }

  memcpy(pDerivate, pDerivate + derivateBufStride, width * sizeof(pDerivate[0]));
  memcpy(pDerivate + (height - 1) * derivateBufStride, pDerivate + (height - 2) * derivateBufStride, width * sizeof(pDerivate[0]));
}

#define CALC_EQUAL_COEFF_16PXLS(x1,x2,y1,y2,tmp0,tmp1,tmp2,tmp3,inter0,inter1,inter2,inter3,loadLocation)         \
{                                                                                                                 \
inter0 = _mm256_mul_epi32(x1, y1);                                                                                \
inter1 = _mm256_mul_epi32(tmp0, tmp2);                                                                            \
inter2 = _mm256_mul_epi32(x2, y2);                                                                                \
inter3 = _mm256_mul_epi32(tmp1, tmp3);                                                                            \
inter2 = _mm256_add_epi64(inter0, inter2);                                                                        \
inter3 = _mm256_add_epi64(inter1, inter3);                                                                        \
inter0 = _mm256_castsi128_si256(_mm_loadl_epi64(loadLocation));                                                   \
inter3 = _mm256_add_epi64(inter2, inter3);                                                                        \
inter1 = _mm256_permute4x64_epi64(inter3, 0x4e);                                                                  \
inter3 = _mm256_add_epi64(inter1, inter3);                                                                        \
inter1 = _mm256_permute4x64_epi64(inter3, 0xb1);                                                                  \
inter3 = _mm256_add_epi64(inter1, inter3);                                                                        \
inter3 = _mm256_add_epi64(inter0, inter3);                                                                        \
}

template<X86_VEXT vext>
static void simdEqualCoeffComputer_HBD_SIMD(Pel *pResidue, int residueStride, int **ppDerivate, int derivateBufStride, int64_t(*pEqualCoeff)[7], int width, int height, bool b6Param)
{
  int n = b6Param ? 6 : 4;
  CHECK((width & 8), "width of affine block should be multiple of 8");

#if USE_AVX2
  if (vext >= AVX2)
  {
    int idx1 = -2 * derivateBufStride - 8;
    int idx2 = -derivateBufStride - 8;

    __m256i tmp[4];
    __m256i intermediate[4];
    __m256i residue[2];
    __m256i coef[12];

    // Add directly to indexes to get new index
    __m256i four = _mm256_set1_epi32(4);
    __m256i eight = _mm256_set1_epi32(8);
    __m256i shuffle = _mm256_set_epi32(0x00000007, 0x00000007, 0x00000006, 0x00000005, 0x00000004, 0x00000003, 0x00000002, 0x00000001);
    __m256i indxJ = _mm256_set1_epi32(-2);

    for (int j = 0; j < height; j += 2)
    {
      if (!(j & 3))
        indxJ = _mm256_add_epi32(indxJ, four);
      __m256i indxK = _mm256_set_epi32(-2, -2, -2, -2, -6, -6, -6, -6);
      idx1 += (derivateBufStride << 1);
      idx2 += (derivateBufStride << 1);

      for (int k = 0; k < width; k += 8)
      {
        idx1 += 8;
        idx2 += 8;
        indxK = _mm256_add_epi32(indxK, eight);

        if (b6Param)
        {
          // coef[0-5] for iC[0-5] of 1st row of pixels
          coef[0] = _mm256_lddqu_si256((__m256i *)&ppDerivate[0][idx1]);
          coef[2] = _mm256_lddqu_si256((__m256i *)&ppDerivate[1][idx1]);
          coef[1] = _mm256_mullo_epi32(indxK, coef[0]);
          coef[3] = _mm256_mullo_epi32(indxK, coef[2]);
          coef[4] = _mm256_mullo_epi32(indxJ, coef[0]);
          coef[5] = _mm256_mullo_epi32(indxJ, coef[2]);

          // coef[6-11] for iC[0-5] of 2nd row of pixels
          coef[6] = _mm256_lddqu_si256((__m256i *)&ppDerivate[0][idx2]);
          coef[8] = _mm256_lddqu_si256((__m256i *)&ppDerivate[1][idx2]);
          coef[7] = _mm256_mullo_epi32(indxK, coef[6]);
          coef[9] = _mm256_mullo_epi32(indxK, coef[8]);
          coef[10] = _mm256_mullo_epi32(indxJ, coef[6]);
          coef[11] = _mm256_mullo_epi32(indxJ, coef[8]);
        }
        else
        {
          // coef[0-3] for iC[0-3] of 1st row of pixels
          coef[0] = _mm256_lddqu_si256((__m256i *)&ppDerivate[0][idx1]);
          coef[2] = _mm256_lddqu_si256((__m256i *)&ppDerivate[1][idx1]);
          coef[1] = _mm256_mullo_epi32(indxK, coef[0]);
          coef[3] = _mm256_mullo_epi32(indxJ, coef[0]);
          tmp[0] = _mm256_mullo_epi32(indxJ, coef[2]);
          tmp[1] = _mm256_mullo_epi32(indxK, coef[2]);
          coef[1] = _mm256_add_epi32(coef[1], tmp[0]);
          coef[3] = _mm256_sub_epi32(coef[3], tmp[1]);

          // coef[4-7] for iC[0-3] of 1st row of pixels
          coef[4] = _mm256_lddqu_si256((__m256i *)&ppDerivate[0][idx2]);
          coef[6] = _mm256_lddqu_si256((__m256i *)&ppDerivate[1][idx2]);
          coef[5] = _mm256_mullo_epi32(indxK, coef[4]);
          coef[7] = _mm256_mullo_epi32(indxJ, coef[4]);
          tmp[2] = _mm256_mullo_epi32(indxJ, coef[6]);
          tmp[3] = _mm256_mullo_epi32(indxK, coef[6]);
          coef[5] = _mm256_add_epi32(coef[5], tmp[2]);
          coef[7] = _mm256_sub_epi32(coef[7], tmp[3]);
        }

        // Residue
        residue[0] = _mm256_lddqu_si256((__m256i *)&pResidue[idx1]);
        residue[1] = _mm256_lddqu_si256((__m256i *)&pResidue[idx2]);
        residue[0] = _mm256_slli_epi32(residue[0], 3);
        residue[1] = _mm256_slli_epi32(residue[1], 3);

        // Calculation of coefficient matrix
        for (int col = 0; col < n; col++)
        {
          tmp[0] = _mm256_permutevar8x32_epi32(coef[0 + col], shuffle);
          tmp[1] = _mm256_permutevar8x32_epi32(coef[n + col], shuffle);
          CALC_EQUAL_COEFF_16PXLS(coef[0 + col], coef[n + col], coef[0 + col], coef[n + col], tmp[0], tmp[1], tmp[0], tmp[1], intermediate[0], intermediate[1], intermediate[2], intermediate[3], (const __m128i*)&pEqualCoeff[col + 1][col]);
          _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][col], _mm256_castsi256_si128(intermediate[3]));

          for (int row = col + 1; row < n; row++)
          {
            tmp[2] = _mm256_permutevar8x32_epi32(coef[0 + row], shuffle);
            tmp[3] = _mm256_permutevar8x32_epi32(coef[n + row], shuffle);
            CALC_EQUAL_COEFF_16PXLS(coef[0 + col], coef[n + col], coef[0 + row], coef[n + row], tmp[0], tmp[1], tmp[2], tmp[3], intermediate[0], intermediate[1], intermediate[2], intermediate[3], (const __m128i*)&pEqualCoeff[col + 1][row]);
            _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][row], _mm256_castsi256_si128(intermediate[3]));
            _mm_storel_epi64((__m128i*)&pEqualCoeff[row + 1][col], _mm256_castsi256_si128(intermediate[3]));
          }

          tmp[2] = _mm256_permutevar8x32_epi32(residue[0], shuffle);
          tmp[3] = _mm256_permutevar8x32_epi32(residue[1], shuffle);
          CALC_EQUAL_COEFF_16PXLS(coef[0 + col], coef[n + col], residue[0], residue[1], tmp[0], tmp[1], tmp[2], tmp[3], intermediate[0], intermediate[1], intermediate[2], intermediate[3], (const __m128i*)&pEqualCoeff[col + 1][n]);
          _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][n], _mm256_castsi256_si128(intermediate[3]));
        }
      }

      idx1 -= (width);
      idx2 -= (width);
    }
  }
  else
#endif
  {
    int idx1 = -2 * derivateBufStride - 4;
    int idx2 = -derivateBufStride - 4;

    __m128i four;
    __m128i tmp[4];
    __m128i intermediate[4];
    __m128i indxK, indxJ;
    __m128i residue[2];
    __m128i coef[12];

    // Add directly to indexes to get new index
    four = _mm_set1_epi32(4);
    indxJ = _mm_set1_epi32(-2);

    for (int j = 0; j < height; j += 2)
    {
      if (!(j & 3))
        indxJ = _mm_add_epi32(indxJ, four);
      indxK = _mm_set1_epi32(-2);
      idx1 += (derivateBufStride << 1);
      idx2 += (derivateBufStride << 1);

      for (int k = 0; k < width; k += 4)
      {
        idx1 += 4;
        idx2 += 4;
        indxK = _mm_add_epi32(indxK, four);

        if (b6Param)
        {
          // coef[0-5] for iC[0-5] of 1st row of pixels
          coef[0] = _mm_lddqu_si128((const __m128i*)&ppDerivate[0][idx1]);
          coef[2] = _mm_lddqu_si128((const __m128i*)&ppDerivate[1][idx1]);
          coef[1] = _mm_mullo_epi32(indxK, coef[0]);
          coef[3] = _mm_mullo_epi32(indxK, coef[2]);
          coef[4] = _mm_mullo_epi32(indxJ, coef[0]);
          coef[5] = _mm_mullo_epi32(indxJ, coef[2]);

          // coef[6-11] for iC[0-5] of 2nd row of pixels
          coef[6] = _mm_lddqu_si128((const __m128i*)&ppDerivate[0][idx2]);
          coef[8] = _mm_lddqu_si128((const __m128i*)&ppDerivate[1][idx2]);
          coef[7] = _mm_mullo_epi32(indxK, coef[6]);
          coef[9] = _mm_mullo_epi32(indxK, coef[8]);
          coef[10] = _mm_mullo_epi32(indxJ, coef[6]);
          coef[11] = _mm_mullo_epi32(indxJ, coef[8]);
        }
        else
        {
          // coef[0-3] for iC[0-3] of 1st row of pixels
          coef[0] = _mm_lddqu_si128((const __m128i*)&ppDerivate[0][idx1]);
          coef[2] = _mm_lddqu_si128((const __m128i*)&ppDerivate[1][idx1]);
          coef[1] = _mm_mullo_epi32(indxK, coef[0]);
          coef[3] = _mm_mullo_epi32(indxJ, coef[0]);
          tmp[0] = _mm_mullo_epi32(indxJ, coef[2]);
          tmp[1] = _mm_mullo_epi32(indxK, coef[2]);
          coef[1] = _mm_add_epi32(coef[1], tmp[0]);
          coef[3] = _mm_sub_epi32(coef[3], tmp[1]);

          // coef[4-7] for iC[0-3] of 1st row of pixels
          coef[4] = _mm_lddqu_si128((const __m128i*)&ppDerivate[0][idx2]);
          coef[6] = _mm_lddqu_si128((const __m128i*)&ppDerivate[1][idx2]);
          coef[5] = _mm_mullo_epi32(indxK, coef[4]);
          coef[7] = _mm_mullo_epi32(indxJ, coef[4]);
          tmp[2] = _mm_mullo_epi32(indxJ, coef[6]);
          tmp[3] = _mm_mullo_epi32(indxK, coef[6]);
          coef[5] = _mm_add_epi32(coef[5], tmp[2]);
          coef[7] = _mm_sub_epi32(coef[7], tmp[3]);
        }

        // Residue
        residue[0] = _mm_lddqu_si128((__m128i *)&pResidue[idx1]);
        residue[1] = _mm_lddqu_si128((__m128i *)&pResidue[idx2]);
        residue[0] = _mm_slli_epi32(residue[0], 3);
        residue[1] = _mm_slli_epi32(residue[1], 3);

        // Calculation of coefficient matrix
        for (int col = 0; col < n; col++)
        {
          tmp[0] = _mm_srli_si128(coef[0 + col], 4);
          tmp[1] = _mm_srli_si128(coef[n + col], 4);
          CALC_EQUAL_COEFF_8PXLS(coef[0 + col], coef[n + col], coef[0 + col], coef[n + col], tmp[0], tmp[1], tmp[0], tmp[1], intermediate[0], intermediate[1], intermediate[2], intermediate[3], (const __m128i*)&pEqualCoeff[col + 1][col]);
          _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][col], intermediate[3]);

          for (int row = col + 1; row < n; row++)
          {
            tmp[2] = _mm_srli_si128(coef[0 + row], 4);
            tmp[3] = _mm_srli_si128(coef[n + row], 4);
            CALC_EQUAL_COEFF_8PXLS(coef[0 + col], coef[n + col], coef[0 + row], coef[n + row], tmp[0], tmp[1], tmp[2], tmp[3], intermediate[0], intermediate[1], intermediate[2], intermediate[3], (const __m128i*)&pEqualCoeff[col + 1][row]);
            _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][row], intermediate[3]);
            _mm_storel_epi64((__m128i*)&pEqualCoeff[row + 1][col], intermediate[3]);
          }

          tmp[2] = _mm_srli_si128(residue[0], 4);
          tmp[3] = _mm_srli_si128(residue[1], 4);
          CALC_EQUAL_COEFF_8PXLS(coef[0 + col], coef[n + col], residue[0], residue[1], tmp[0], tmp[1], tmp[2], tmp[3], intermediate[0], intermediate[1], intermediate[2], intermediate[3], (const __m128i*)&pEqualCoeff[col + 1][n]);
          _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][n], intermediate[3]);
        }
      }

      idx1 -= (width);
      idx2 -= (width);
    }
  }
}
#endif

template <X86_VEXT vext>
void AffineGradientSearch::_initAffineGradientSearchX86()
{
#if RExt__HIGH_BIT_DEPTH_SUPPORT
  m_HorizontalSobelFilter = simdHorizontalSobelFilter_HBD_SIMD<vext>;
  m_VerticalSobelFilter = simdVerticalSobelFilter_HBD_SIMD<vext>;
  m_EqualCoeffComputer = simdEqualCoeffComputer_HBD_SIMD<vext>;
#else
  m_HorizontalSobelFilter = simdHorizontalSobelFilter<vext>;
  m_VerticalSobelFilter = simdVerticalSobelFilter<vext>;
  m_EqualCoeffComputer = simdEqualCoeffComputer<vext>;
#endif
}

template void AffineGradientSearch::_initAffineGradientSearchX86<SIMDX86>();

#endif //#ifdef TARGET_SIMD_X86
//! \}
