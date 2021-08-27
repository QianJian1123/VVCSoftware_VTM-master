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


#include "BinEncoder.h"

#include "CommonLib/Rom.h"
#include "CommonLib/dtrace_next.h"

BinCounter::BinCounter()
  : m_CtxBinsCodedBuffer( Ctx::NumberOfContexts )
  , m_NumBinsCtx        ( m_CtxBinsCodedBuffer.data() )
  , m_NumBinsEP         ( 0 )
  , m_NumBinsTrm        ( 0 )
{}


void BinCounter::reset()
{
  for( std::size_t k = 0; k < m_CtxBinsCodedBuffer.size(); k++ )
  {
    m_NumBinsCtx[k] = 0;
  }
  m_NumBinsEP       = 0;
  m_NumBinsTrm      = 0;
}


uint32_t BinCounter::getAll() const
{
  uint32_t  count = m_NumBinsEP + m_NumBinsTrm;
  for( std::size_t k = 0; k < m_CtxBinsCodedBuffer.size(); k++ )
  {
    count += m_NumBinsCtx[k];
  }
  return count;
}





template <class BinProbModel>
BinEncoderBase::BinEncoderBase( const BinProbModel* dummy )
  : BinEncIf          ( dummy )
  , m_Bitstream       ( 0 )
  , m_Low             ( 0 )
  , m_Range           ( 0 )
  , m_bufferedByte    ( 0 )
  , m_numBufferedBytes( 0 )
  , m_bitsLeft        ( 0 )
{}

void BinEncoderBase::init( OutputBitstream* bitstream )
{
  m_Bitstream = bitstream;
}

void BinEncoderBase::uninit()
{
  m_Bitstream = 0;
}
//间起始点(low)的初始值设置为0，存储在一个32位的寄存器中，有效位为后9位。
//区间宽度(range)将区间[0,1]扩展到初始值长度为510，用9位二进制数表示。当(range)的值小于初始值510的一半时，将会进行重归一化操作（RenormE），将对(range)左移扩展到区间初始值的一半以上，并对(low)左移进行比特输出，即(range)的取值需在[2^8 ,2^9 ) 之间。

//m_bitsLeft表示存储(low)的寄存器中还剩余多少bit。寄存器大小为32位，(low)的初始值占9位
//（9个0），所以剩余比特数的初始值为23位。
// m_bufferedByte和m_numBufferedBytes是与输出缓存器有关的参数。

void BinEncoderBase::start()  //算数编码初始化函数
{
  m_Low               = 0; //区间起始点
  m_Range             = 510;//区间宽度
  m_bufferedByte      = 0xff;//缓冲区大小
  m_numBufferedBytes  = 0;//已缓存的字节数
  m_bitsLeft          = 23;//寄存器剩余比特数
  BinCounter::reset();
  m_BinStore. reset();
}

void BinEncoderBase::finish()
{
  if( m_Low >> ( 32 - m_bitsLeft ) )
  {
    m_Bitstream->write( m_bufferedByte + 1, 8 );
    while( m_numBufferedBytes > 1 )
    {
      m_Bitstream->write( 0x00, 8 );
      m_numBufferedBytes--;
    }
    m_Low -= 1 << ( 32 - m_bitsLeft );
  }
  else
  {
    if( m_numBufferedBytes > 0 )
    {
      m_Bitstream->write( m_bufferedByte, 8 );
    }
    while( m_numBufferedBytes > 1 )
    {
      m_Bitstream->write( 0xff, 8 );
      m_numBufferedBytes--;
    }
  }
  m_Bitstream->write( m_Low >> 8, 24 - m_bitsLeft );
}

void BinEncoderBase::restart()
{
  m_Low               = 0;
  m_Range             = 510;
  m_bufferedByte      = 0xff;
  m_numBufferedBytes  = 0;
  m_bitsLeft          = 23;
}

void BinEncoderBase::reset( int qp, int initId )
{
  Ctx::init( qp, initId );
  start();
}

void BinEncoderBase::resetBits()
{
  m_Low               = 0;
  m_bufferedByte      = 0xff;
  m_numBufferedBytes  = 0;
  m_bitsLeft          = 23;
  BinCounter::reset();
}

void BinEncoderBase::encodeBinEP( unsigned bin )
{
  DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, bin );

  BinCounter::addEP();
  m_Low <<= 1;
  if( bin )
  {
    m_Low += m_Range;
  }
  m_bitsLeft--;
  if( m_bitsLeft < 12 )
  {
    writeOut();
  }
}
/*
有些语法元素在二值化后选择的可能不是上述的算术编码，而是旁路编码。旁路编码器假设符 号0和1的概率各占1 2
的固定概率进行编码，不需要对(range)进行查表划分，不需要进行概率模型的更新。
传入的range_input在256到510之间，因为0和1的区间长度各占一半，所以新d的range=1/2range_input。故每
编码一次symbol都需要进行一次重归一化，左移位数为1，并输出1个比特。这里为了更简便，将对low的
移位操作提前，并保持range不变。需要注意的是旁路编码器中，0的区间在前，1的区间在后。
• 在常规编码其中，若LPS在前，MPS在后
symbol = 0时：low << numBits, range = R0 << numBits;
symbol = 1时：low = (low + R0) << numBits, range = R1 << numBits;
在旁路编码器中有R0 = R1 = 1 2range，numBits = 1：
symbol = 0时：low << 1, range = (1 2range) << 1 = range;
symbol = 1时：low = (low + 1 2range) << 1 = (low << 1) + range, range = (1 2range) << 1 = range;

*/
void BinEncoderBase::encodeBinsEP( unsigned bins, unsigned numBins )
{
  for(int i = 0; i < numBins; i++)
  {
    DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, ( bins >> ( numBins - 1 - i ) ) & 1 );
  }

  BinCounter::addEP( numBins );//重归一化
  if( m_Range == 256 )
  {
    encodeAlignedBinsEP( bins, numBins );
    return;
  }
  //写入码流
  while( numBins > 8 )
  {
    numBins          -= 8;
    unsigned pattern  = bins >> numBins;
    m_Low           <<= 8;
    m_Low            += m_Range * pattern;
    bins             -= pattern << numBins;
    m_bitsLeft       -= 8;
    if( m_bitsLeft < 12 )
    {
      writeOut();
    }
  }
  m_Low     <<= numBins;
  m_Low      += m_Range * bins;
  m_bitsLeft -= numBins;
  if( m_bitsLeft < 12 )
  {
    writeOut();
  }
}

void BinEncoderBase::encodeRemAbsEP(unsigned bins, unsigned goRicePar, unsigned cutoff, int maxLog2TrDynamicRange)
{
  const unsigned threshold = cutoff << goRicePar;
  if (bins < threshold)
  {
    const unsigned bitMask = (1 << goRicePar) - 1;
    const unsigned length = (bins >> goRicePar) + 1;
    encodeBinsEP((1 << length) - 2, length);
    encodeBinsEP(bins & bitMask, goRicePar);
  }
  else
  {
    const unsigned  maxPrefixLength = 32 - cutoff - maxLog2TrDynamicRange;
    unsigned        prefixLength = 0;
    unsigned        codeValue = (bins >> goRicePar) - cutoff;
    unsigned        suffixLength;
    if (codeValue >= ((1 << maxPrefixLength) - 1))
    {
      prefixLength = maxPrefixLength;
      suffixLength = maxLog2TrDynamicRange;
    }
    else
    {
      while (codeValue > ((2 << prefixLength) - 2))
      {
        prefixLength++;
      }
      suffixLength = prefixLength + goRicePar + 1; //+1 for the separator bit
    }
    const unsigned totalPrefixLength = prefixLength + cutoff;
    const unsigned bitMask = (1 << goRicePar) - 1;
    const unsigned prefix = (1 << totalPrefixLength) - 1;
    const unsigned suffix = ((codeValue - ((1 << prefixLength) - 1)) << goRicePar) | (bins & bitMask);
    encodeBinsEP(prefix, totalPrefixLength); //prefix
    encodeBinsEP(suffix, suffixLength); //separator, suffix, and rParam bits
  }
}

void BinEncoderBase::encodeBinTrm( unsigned bin )//编码终止位
{
  BinCounter::addTrm();
  m_Range -= 2;
  if( bin )
  {
    m_Low      += m_Range;
    m_Low     <<= 7;
    m_Range     = 2 << 7;
    m_bitsLeft -= 7;
  }
  else if( m_Range >= 256 )
  {
    return;
  }
  else
  {
    m_Low     <<= 1;
    m_Range   <<= 1;
    m_bitsLeft--;
  }
  if( m_bitsLeft < 12 )
  {
    writeOut();
  }
}


void BinEncoderBase::align()
{
  m_Range = 256;
}


void BinEncoderBase::encodeAlignedBinsEP( unsigned bins, unsigned numBins )
{
  unsigned remBins = numBins;
  while( remBins > 0 )
  {
    //The process of encoding an EP bin is the same as that of coding a normal
    //bin where the symbol ranges for 1 and 0 are both half the range:
    //
    //  low = (low + range/2) << 1       (to encode a 1)
    //  low =  low            << 1       (to encode a 0)
    //
    //  i.e.
    //  low = (low + (bin * range/2)) << 1
    //
    //  which is equivalent to:
    //
    //  low = (low << 1) + (bin * range)
    //
    //  this can be generalised for multiple bins, producing the following expression:
    //
    unsigned binsToCode = std::min<unsigned>( remBins, 8); //code bytes if able to take advantage of the system's byte-write function
    unsigned binMask    = ( 1 << binsToCode ) - 1;
    unsigned newBins    = ( bins >> ( remBins - binsToCode ) ) & binMask;
    m_Low               = ( m_Low << binsToCode ) + ( newBins << 8 ); //range is known to be 256
    remBins            -= binsToCode;
    m_bitsLeft         -= binsToCode;
    if( m_bitsLeft < 12 )
    {
      writeOut();
    }
  }
}
/*
即每编码完一个symbol都会 对m bitsLeft进行检测，如果满足如果满足m bitsLeft < 12，即寄存器高位中剩余比特数小于初始值23
的一半时，则调用输出到比特流的函数WriteOut()
当检测到寄存器中剩余比特数小于12时，会进行比特输出。将存储low的32位寄存器中存有数
值的高8位先存入一个8位的缓存器中，并将存储low的32位寄存器中的这8位清空，寄存器剩余比特 数m bitsLeft+ =
8。当下一次调用writeOut()函数时，将8位缓存器中的内容输出到比特流中，并存入新的8位数据。当所有需要编码的symbol都编码结束后，会调用finish()函数，将存储low的32位寄存器中剩
下的所有数据全部输出到比特流中。
*/
void BinEncoderBase::writeOut()
{
  unsigned leadByte = m_Low >> ( 24 - m_bitsLeft );
  m_bitsLeft       += 8;
  m_Low            &= 0xffffffffu >> m_bitsLeft;
  if( leadByte == 0xff )
  {
    m_numBufferedBytes++;
  }
  else
  {
    if( m_numBufferedBytes > 0 )
    {
      unsigned carry  = leadByte >> 8;
      unsigned byte   = m_bufferedByte + carry;
      m_bufferedByte  = leadByte & 0xff;
      m_Bitstream->write( byte, 8 );
      byte            = ( 0xff + carry ) & 0xff;
      while( m_numBufferedBytes > 1 )
      {
        m_Bitstream->write( byte, 8 );
        m_numBufferedBytes--;
      }
    }
    else
    {
      m_numBufferedBytes  = 1;
      m_bufferedByte      = leadByte;
    }
  }
}



template <class BinProbModel>
TBinEncoder<BinProbModel>::TBinEncoder()
  : BinEncoderBase( static_cast<const BinProbModel*>    ( nullptr ) )
  , m_Ctx         ( static_cast<CtxStore<BinProbModel>&>( *this   ) )
{}

/*
常规编码器，CABAC的编码核心，编码一个比特位，输入待编码的symbol(0或1) 和所选的概率模型。
首先进行symbol0所占区间长度的确定，根据所选概率模型中的概率P和接收到的区间宽度，
以查表的方式代替乘法运算，查出0的区间长度。
同时将1的区间长度赋值给range。之后根据待编码比特位symbol的取值，
更新概率模型、区间起始点(low) 和区间宽度(range)。

*/
      template <class BinProbModel>
void TBinEncoder<BinProbModel>::encodeBin( unsigned bin, unsigned ctxId )
{
  BinCounter::addCtx( ctxId );
  BinProbModel& rcProbModel = m_Ctx[ctxId];
  uint32_t      LPS         = rcProbModel.getLPS( m_Range );

  DTRACE( g_trace_ctx, D_CABAC, "%d" " %d " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  " "  -  " "%d" "\n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), ctxId, m_Range, m_Range - LPS, LPS, ( unsigned int ) ( rcProbModel.state() ), bin == rcProbModel.mps(), bin );

  m_Range   -=  LPS;
  if( bin != rcProbModel.mps() )//如果传过来的symbol是LPS,range肯定会小于256，因此要进行归一化操作
  {
    int numBits   = rcProbModel.getRenormBitsLPS( LPS );//归一化，得到需要左移的位数
    m_bitsLeft   -= numBits;//寄存器中剩余的供Low左移用的位数
    m_Low        += m_Range;
    m_Low         = m_Low << numBits;//Low左移numBits位进行比特输出
    m_Range       = LPS   << numBits;//range左移numBits位进行区间扩增
    if( m_bitsLeft < 12 )
    {
      writeOut();
    }
  }
  else //如果传过来的symbol是MPS
  {
    if( m_Range < 256 )//LPS首先要判断range是否小于一半的区间长度
    {
      int numBits   = rcProbModel.getRenormBitsRange( m_Range );
      m_bitsLeft   -= numBits;
      m_Low       <<= numBits;
      m_Range     <<= numBits;
      if( m_bitsLeft < 12 )
      {
        writeOut();
      }
    }
  }
  rcProbModel.update( bin );//更新概率模型
  BinEncoderBase::m_BinStore.addBin( bin, ctxId );//重归一化
}

template <class BinProbModel>
BinEncIf* TBinEncoder<BinProbModel>::getTestBinEncoder() const
{
  BinEncIf* testBinEncoder = 0;
  if( m_BinStore.inUse() )
  {
    testBinEncoder = new TBinEncoder<BinProbModel>();
  }
  return testBinEncoder;
}





template <class BinProbModel>
BitEstimatorBase::BitEstimatorBase( const BinProbModel* dummy )
  : BinEncIf      ( dummy )
{
  m_EstFracBits = 0;
}

void BitEstimatorBase::encodeRemAbsEP(unsigned bins, unsigned goRicePar, unsigned cutoff, int maxLog2TrDynamicRange)
{
  const unsigned threshold = cutoff << goRicePar;
  if (bins < threshold)
  {
    m_EstFracBits += BinProbModelBase::estFracBitsEP((bins >> goRicePar) + 1 + goRicePar);
  }
  else
  {
    const unsigned  maxPrefixLength = 32 - cutoff - maxLog2TrDynamicRange;
    unsigned        prefixLength = 0;
    unsigned        codeValue = (bins >> goRicePar) - cutoff;
    unsigned        suffixLength;
    if (codeValue >= ((1 << maxPrefixLength) - 1))
    {
      prefixLength = maxPrefixLength;
      suffixLength = maxLog2TrDynamicRange;
    }
    else
    {
      while (codeValue > ((2 << prefixLength) - 2))
      {
        prefixLength++;
      }
      suffixLength = prefixLength + goRicePar + 1; //+1 for the separator bit
    }
    m_EstFracBits += BinProbModelBase::estFracBitsEP(cutoff + prefixLength + suffixLength);
  }
}

void BitEstimatorBase::align()
{
  static const uint64_t add   = BinProbModelBase::estFracBitsEP() - 1;
  static const uint64_t mask  = ~add;
  m_EstFracBits += add;
  m_EstFracBits &= mask;
}






template <class BinProbModel>
TBitEstimator<BinProbModel>::TBitEstimator()
  : BitEstimatorBase  ( static_cast<const BinProbModel*>    ( nullptr) )
  , m_Ctx             ( static_cast<CtxStore<BinProbModel>&>( *this  ) )
{}



template class TBinEncoder<BinProbModel_Std>;

template class TBitEstimator<BinProbModel_Std>;

