/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (context) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *  be used to endorse or promote products derived from this software without
 *  specific prior written permission.
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
#if defined( USE_FFMPEG_VIDEO_CODEC )
#include "image.hpp"
#include "ffmpegLibVideoEncoder.hpp"
#include "ffmpegLibVideoDecoder.hpp"

#include <functional>
#include <set>
namespace vmesh {

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

template <typename T>
ffmpegLibVideoEncoder<T>::ffmpegLibVideoEncoder() {}
template <typename T>
ffmpegLibVideoEncoder<T>::~ffmpegLibVideoEncoder() {}

const std::string encoderName = "libx265";  // libx265 H.265 / HEVC (codec hevc)
// const std::string encoderName = "hevc_vaapi";     // H.265/HEVC (VAAPI) (codec hevc)

typedef std::function<void( uint8_t* outPtr, int size )> encodeCallback;

#define x265_PRESET "medium"  // from "ultrafast" to "placebo", "veryslow"
//#define ENABLE_DEBUG

#define CHECK_AV_ERROR( func )                             \
  result = func;                                           \
  if ( result < 0 ) {                                      \
    char errbuf[80];                                       \
    av_strerror( result, errbuf, sizeof( errbuf ) );       \
    fprintf( stderr, "FFMPEG: %d %s \n", result, errbuf ); \
    fflush( stderr );                                      \
    exit( -1 );                                            \
  }

template <typename T>
static int encodeFrame( AVCodecContext*    context,
                        int32_t&           index,
                        FrameSequence<T>&    videoSrc,
                        std::vector<uint8_t>& bitstream ) {
  int      result = 0;
  AVPacket packet;
  av_init_packet( &packet );
  packet.data = NULL;
  packet.size = 0;
  if ( index < videoSrc.frameCount() ) {
    AVFrame* frame = av_frame_alloc();
    frame->width   = context->width;
    frame->height  = context->height;
    frame->format  = AV_PIX_FMT_YUV420P;
    CHECK_AV_ERROR( av_frame_get_buffer( frame, 32 ) );
    videoSrc[index].get( frame->data[0], frame->data[1], frame->data[2], context->width, context->height,
                         frame->linesize[0], context->width / 2, context->height / 2, frame->linesize[1], 0, false );
    index++;
    CHECK_AV_ERROR( avcodec_send_frame( context, frame ) );
    av_frame_free( &frame );
  } else {
    CHECK_AV_ERROR( avcodec_send_frame( context, nullptr ) );
  }
  while ( true ) {
    result = avcodec_receive_packet( context, &packet );
    if ( result != 0 ) { break; }
    size_t size = bitstream.size();
    bitstream.resize( size + packet.size );
    memcpy( bitstream.data() + size, packet.data, packet.size );
    av_packet_unref( &packet );
  }
  return 0;
}

template <typename T>
void encodeStream( FrameSequence<T>& videoSrc, int depth, int qp, bool lossless, std::vector<uint8_t>& bitstream ) {
  int result = 0;
#ifdef ENABLE_DEBUG
  av_log_set_level( AV_LOG_TRACE );
  av_log_set_callback( []( void* ptr, int level, const char* fmt, va_list vargs ) { vprintf( fmt, vargs ); } );
#endif
  avcodec_register_all();
  AVCodec* codec = avcodec_find_encoder_by_name( encoderName.c_str() );
  assert( codec );
  AVCodecContext* context = avcodec_alloc_context3( codec );
  assert( context );
  int fps                      = 25;
  context->width               = videoSrc.width();
  context->height              = videoSrc.height();
  context->time_base.num       = 1;
  context->time_base.den       = fps;
  context->framerate           = {fps, 1};
  context->sample_aspect_ratio = {1, 1};
  context->pix_fmt             = ( depth == 10 ) ? AV_PIX_FMT_YUV420P10LE : AV_PIX_FMT_YUV420P;
  AVDictionary* options        = nullptr;
  char          x265_params_str[256];
  sprintf( x265_params_str, "qp=%d:lossless=%d:fps=%d", qp, lossless, fps );
  av_dict_set( &options, "x265-params", x265_params_str, 0 );
  av_dict_set( &options, "preset", x265_PRESET, 0 );
#if HEVC
#else
  av_dict_set( &options, "g", "32", 0 );
  av_dict_set( &options, "bf", "7", 0 );
  av_dict_set( &options, "b_strategy", "2", 0 );
#endif
  printf( "x265-params: %s preset: %s \n", x265_params_str, x265_PRESET );
  fflush( stdout );
  CHECK_AV_ERROR( avcodec_open2( context, codec, &options ) );
  int index = 0;
  while ( index < videoSrc.frameCount() ) { encodeFrame( context, index, videoSrc, bitstream ); }
  encodeFrame( context, index, videoSrc, bitstream );
  avcodec_free_context( &context );
}

template <typename T>
void ffmpegLibVideoEncoder<T>::encode( FrameSequence<T>&            videoSrc,
                                          VideoEncoderParameters& params,
                                          std::vector<uint8_t>&         bitstream,
                                          FrameSequence<T>&            videoRec ) {
  int lossless = ( params.qp_ <= 8 );
  printf( "FFMPEG encode: bitdepth = %d %d QP = %d lossless = %d \n", params.inputBitDepth_, params.inputBitDepth_,
          params.qp_, lossless );
  fflush( stdout );
  encodeStream( videoSrc, params.inputBitDepth_, params.qp_, lossless, bitstream );
  ffmpegLibVideoDecoder<T> decoder;
  decoder.decode( bitstream, videoRec, params.outputBitDepth_ );
  printf( "End decoder num frame = %zu %zu x %zu \n", videoRec.frameCount(), videoRec.width(),
          videoRec.height() );
}

template class vmesh::ffmpegLibVideoEncoder<uint8_t>;
template class vmesh::ffmpegLibVideoEncoder<uint16_t>;

}  // namespace vmesh

#endif
