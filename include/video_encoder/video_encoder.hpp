#pragma once

#include <deque>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv4/opencv2/core.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

class VideoEncoder {

private:
  VideoEncoder()
      : codec_ctx_(nullptr), frame_(nullptr), packet_(nullptr),
        sws_ctx_(nullptr), frame_count_{0} {}

public:
  ~VideoEncoder();

  struct EncoderParams {
    enum class HardwareAccelerationType { SOFTWARE, NVIDIA, INTEL, AMD, APPLE };

    HardwareAccelerationType accel_type;

    int width;
    int height;
    int fps;

    bool constant_quality_enabled;
  };

  // Parameters that can be changed dynamically without reinitializing
  struct DynamicParams {
    std::optional<int64_t> bitrate; // Target bitrate in bits/sec
    std::optional<int> crf;      // Constant rate factor (0-51, lower = better)
    std::optional<int> gop_size; // Keyframe interval
    std::optional<int> max_b_frames; // Max B-frames
  };

  static std::unique_ptr<VideoEncoder>
  create_unique(const EncoderParams &params);

  bool encode_frame(const cv::Mat &frame);

  std::optional<std::vector<uint8_t>> get_last_compressed_frame();

  std::optional<std::vector<uint8_t>> get_latest_compressed_frame();

  std::optional<std::vector<uint8_t>> clone_latest_compressed_frame() const;

  std::optional<std::vector<std::vector<uint8_t>>>
  get_compressed_frames(uint32_t num_frames);

  const std::size_t get_buffer_size() const { return frame_buffer_.size(); }

  void force_keyframe();

  static std::string base64_encode(const std::vector<uint8_t> &data);

  // Change parameters dynamically (no reinitialization needed)
  bool set_dynamic_params(const DynamicParams &params);

  // Change parameters that require reinitialization (flushes encoder)
  bool set_static_params(const EncoderParams &params);

private:
  EncoderParams params_;

  AVCodecContext *codec_ctx_;
  AVFrame *frame_;
  AVPacket *packet_;
  SwsContext *sws_ctx_;

  std::size_t frame_count_;
  std::deque<std::vector<uint8_t>> frame_buffer_;

  bool force_next_keyframe_ = false;

  bool initialize_av();
  void cleanup_av();
  bool flush_encoder();
};
