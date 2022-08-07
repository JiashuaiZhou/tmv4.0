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

#if defined(USE_FFMPEG_VIDEO_CODEC)

#  include "ffmpegLibVideoDecoder.hpp"

#  include <functional>
#  include <set>

namespace vmesh {

extern "C" {
#  include <libavcodec/avcodec.h>
#  include <libavutil/opt.h>
#  include <libavutil/imgutils.h>
#  include <libavformat/avformat.h>
#  include <libavcodec/avcodec.h>
#  include <libavutil/imgutils.h>
#  include <libavformat/avformat.h>
#  include <libswscale/swscale.h>
}

template<typename T>
ffmpegLibVideoDecoder<T>::ffmpegLibVideoDecoder() {}
template<typename T>
ffmpegLibVideoDecoder<T>::~ffmpegLibVideoDecoder() {}

const std::string decoderName = "hevc";  // HEVC (High Efficiency Video Coding)
// const std::string decoderName = "hevc_cuvid";  // Nvidia CUVID HEVC decoder (codec hevc)

#  define CHECK_AV_ERROR(func) \
    result = func; \
    if (result < 0) { \
      char errbuf[80]; \
      av_strerror(result, errbuf, sizeof(errbuf)); \
      fprintf(stderr, "FFMPEG; %d %s", result, errbuf); \
      exit(-1); \
    }

template<typename T>
static int
decodePacket(AVCodecContext*   avctx,
             AVPacket*         packet,
             FrameSequence<T>& video) {
  int result;
  CHECK_AV_ERROR(avcodec_send_packet(avctx, packet));
  while (true) {
    AVFrame* frame = av_frame_alloc();
    result         = avcodec_receive_frame(avctx, frame);
    if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
      av_frame_free(&frame);
      break;
    } else if (result < 0) {
      av_frame_free(&frame);
      fprintf(stderr, "FFMPEG; error: %d", result);
      exit(-1);
    }
    ColourSpace format =
      frame->format == AV_PIX_FMT_YUV420P   ? ColourSpace::YUV420p
      : frame->format == AV_PIX_FMT_YUV444P ? ColourSpace::YUV444p
                                            : ColourSpace::RGB444p;
    video.resize(frame->width, frame->height, format, video.frameCount() + 1);
    video.frame(video.frameCount() - 1)
      .set(frame->data[0],
           frame->data[1],
           frame->data[2],
           frame->width,
           frame->height,
           frame->linesize[0],
           frame->width / 2,
           frame->height / 2,
           frame->linesize[1],
           0,
           format,
           format == ColourSpace::RGB444p);
  }
  return 0;
}

template<typename T>
void
decodeStream(std::vector<uint8_t>& bitstream, FrameSequence<T>& video) {
#  ifdef ENABLE_DEBUG
  av_log_set_level(AV_LOG_TRACE);
  av_log_set_callback(
    [](void* ptr, int level, const char* fmt, va_list vargs) {
      vprintf(fmt, vargs);
    });
#  endif
  struct BufferData {
    uint8_t* inPtr;
    int      bytesRemaining;
  };
  BufferData videoDecoderData = {bitstream.data(),
                                 static_cast<int>(bitstream.size())};
  int        result           = 0;
  const int  io_buffer_size   = 4096;
  uint8_t*   io_buffer        = (uint8_t*)av_malloc(io_buffer_size);
  auto       readFunction = [](void* thiz, uint8_t* buf, int buf_size) -> int {
    BufferData* bufferData = (BufferData*)thiz;
    buf_size = std::min(bufferData->bytesRemaining, buf_size);
    if (buf_size == 0) return AVERROR_EOF;
    memcpy(buf, bufferData->inPtr, buf_size);
    bufferData->inPtr += buf_size;
    bufferData->bytesRemaining -= buf_size;
    return buf_size;
  };
  AVIOContext* ioContext = avio_alloc_context(io_buffer,
                                              io_buffer_size,
                                              0,
                                              &videoDecoderData,
                                              readFunction,
                                              nullptr,
                                              nullptr);
  assert(ioContext);
  AVFormatContext* inputContext = avformat_alloc_context();
  assert(inputContext);
  inputContext->pb = ioContext;
  av_register_all();
  CHECK_AV_ERROR(
    avformat_open_input(&inputContext, nullptr, nullptr, nullptr));
  CHECK_AV_ERROR(avformat_find_stream_info(inputContext, nullptr));
  AVCodec* codec = avcodec_find_decoder_by_name(decoderName.c_str());
  assert(codec);
  int videoStreamIndex =
    av_find_best_stream(inputContext, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
  assert(videoStreamIndex >= 0);
  AVCodecContext* decoderContext = avcodec_alloc_context3(codec);
  assert(decoderContext);
  decoderContext->codec_id = AV_CODEC_ID_HEVC;
  AVStream* videoStream    = inputContext->streams[videoStreamIndex];
  CHECK_AV_ERROR(
    avcodec_parameters_to_context(decoderContext, videoStream->codecpar));
  decoderContext->get_format =
    [](AVCodecContext* ctx, const AVPixelFormat* pix_fmts) -> AVPixelFormat {
    std::set<AVPixelFormat> pix_fmts_set;
    for (auto p = pix_fmts; *p != -1; p++) pix_fmts_set.insert(*p);
    if (pix_fmts_set.find(AV_PIX_FMT_YUV420P) != pix_fmts_set.end()) {
      return AV_PIX_FMT_YUV420P;
    } else if (pix_fmts_set.find(AV_PIX_FMT_YUV420P10LE)
               != pix_fmts_set.end()) {
      return AV_PIX_FMT_YUV420P10LE;
    }
    fprintf(stderr, "FFMPEG; error \n");
    exit(-1);
    return AV_PIX_FMT_NONE;
  };
#  ifdef ENABLE_HARDWARE_ACCEL
  printf("HW Device: %s \n", FFMPEG_HW_DEVICE_STR);
  AVBufferRef* hw_device_ctx    = nullptr;
  char*        ffmpeg_hw_device = FFMPEG_HW_DEVICE_STR;
  auto         hwDeviceType = av_hwdevice_find_type_by_name(ffmpeg_hw_device);
  assert(hwDeviceType != AV_HWDEVICE_TYPE_NONE);
  CHECK_AV_ERROR(
    av_hwdevice_ctx_create(&hw_device_ctx, hwDeviceType, nullptr, nullptr, 0));
  decoderContext->hw_device_ctx = av_buffer_ref(hw_device_ctx);
#  else
  printf("Using SW decoder \n");
#  endif
  CHECK_AV_ERROR(avcodec_open2(decoderContext, codec, NULL));
  AVPacket packet;
  while (true) {
    result = av_read_frame(inputContext, &packet);
    if (result < 0) break;
    if (videoStreamIndex == packet.stream_index) {
      result = decodePacket(decoderContext, &packet, video);
    }
    av_packet_unref(&packet);
  }
  // flush the decoder
  packet.data = NULL;
  packet.size = 0;
  result      = decodePacket(decoderContext, &packet, video);
  av_packet_unref(&packet);
  avcodec_free_context(&decoderContext);
  avio_context_free(&ioContext);
  avformat_close_input(&inputContext);
#  ifdef ENABLE_HARDWARE_ACCEL
  av_buffer_unref(&hw_device_ctx);
#  endif
}

template<typename T>
void
ffmpegLibVideoDecoder<T>::decode(std::vector<uint8_t>& bitstream,
                                 FrameSequence<T>&     video,
                                 size_t                outputBitDepth,
                                 const std::string&    decoderPath,
                                 const std::string&    parameters) {
  video.clear();
  decodeStream(bitstream, video);
}

template class ffmpegLibVideoDecoder<uint8_t>;
template class ffmpegLibVideoDecoder<uint16_t>;

}  // namespace vmesh

#endif