

#include "video_encoder/video_encoder.hpp"
#include <memory>

VideoEncoder::~VideoEncoder() { cleanup_av(); }

void VideoEncoder::cleanup_av() {
  if (sws_ctx_) {
    sws_freeContext(sws_ctx_);
    sws_ctx_ = nullptr;
  }
  if (frame_) {
    av_frame_free(&frame_);
    frame_ = nullptr;
  }
  if (packet_) {
    av_packet_free(&packet_);
    packet_ = nullptr;
  }
  if (codec_ctx_) {
    avcodec_free_context(&codec_ctx_);
    codec_ctx_ = nullptr;
  }
}

bool VideoEncoder::flush_encoder() {
  // Send flush signal to encoder
  int ret = avcodec_send_frame(codec_ctx_, nullptr);
  if (ret < 0 && ret != AVERROR_EOF) {
    std::cerr << "Error flushing encoder" << std::endl;
    return false;
  }

  // Receive all remaining packets
  while (true) {
    ret = avcodec_receive_packet(codec_ctx_, packet_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      break;
    }
    if (ret < 0) {
      std::cerr << "Error receiving packet during flush" << std::endl;
      return false;
    }

    std::vector<uint8_t> packet_data(packet_->data,
                                     packet_->data + packet_->size);
    frame_buffer_.push_front(std::move(packet_data));
    av_packet_unref(packet_);
  }

  return true;
}

bool VideoEncoder::set_dynamic_params(const DynamicParams &params) {
  if (!codec_ctx_) {
    std::cerr << "Encoder not initialized" << std::endl;
    return false;
  }

  bool success = true;

  // Change bitrate dynamically
  if (params.bitrate.has_value()) {
    codec_ctx_->bit_rate = params.bitrate.value();
    std::cout << "Bitrate changed to: " << params.bitrate.value() << std::endl;
  }

  // Change CRF dynamically (for constant quality mode)
  if (params.crf.has_value()) {
    if (av_opt_set_int(codec_ctx_->priv_data, "crf", params.crf.value(), 0) <
        0) {
      std::cerr << "Failed to set CRF" << std::endl;
      success = false;
    } else {
      std::cout << "CRF changed to: " << params.crf.value() << std::endl;
    }
  }

  // Change GOP size dynamically
  if (params.gop_size.has_value()) {
    codec_ctx_->gop_size = params.gop_size.value();
    std::cout << "GOP size changed to: " << params.gop_size.value()
              << std::endl;
  }

  // Change max B-frames dynamically
  if (params.max_b_frames.has_value()) {
    codec_ctx_->max_b_frames = params.max_b_frames.value();
    std::cout << "Max B-frames changed to: " << params.max_b_frames.value()
              << std::endl;
  }

  return success;
}

bool VideoEncoder::set_static_params(const EncoderParams &params) {
  if (!codec_ctx_) {
    std::cerr << "Encoder not initialized" << std::endl;
    return false;
  }

  std::cout << "Reinitializing encoder with new parameters..." << std::endl;

  // Flush encoder to get all remaining packets
  if (!flush_encoder()) {
    std::cerr << "Failed to flush encoder" << std::endl;
    return false;
  }

  // Cleanup existing resources
  cleanup_av();

  // Update params
  params_ = params;

  // Reset frame counter
  frame_count_ = 0;

  // Reinitialize with new parameters
  if (!initialize_av()) {
    std::cerr << "Failed to reinitialize encoder" << std::endl;
    return false;
  }

  std::cout << "Encoder reinitialized successfully" << std::endl;
  return true;
}

std::string VideoEncoder::base64_encode(const std::vector<uint8_t> &data) {
  static constexpr char encoding_table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string encoded;
  encoded.reserve(((data.size() + 2) / 3) * 4);

  for (std::size_t i = 0; i < data.size(); i += 3) {
    uint32_t triple = (static_cast<uint32_t>(data[i]) << 16);

    if (i + 1 < data.size()) {
      triple |= (static_cast<uint32_t>(data[i + 1]) << 8);
    }
    if (i + 2 < data.size()) {
      triple |= static_cast<uint32_t>(data[i + 2]);
    }

    encoded.push_back(encoding_table[(triple >> 18) & 0x3F]);
    encoded.push_back(encoding_table[(triple >> 12) & 0x3F]);

    if (i + 1 < data.size()) {
      encoded.push_back(encoding_table[(triple >> 6) & 0x3F]);
    } else {
      encoded.push_back('=');
    }

    if (i + 2 < data.size()) {
      encoded.push_back(encoding_table[triple & 0x3F]);
    } else {
      encoded.push_back('=');
    }
  }

  return encoded;
}

std::unique_ptr<VideoEncoder>
VideoEncoder::create_unique(const VideoEncoder::EncoderParams &params) {

  auto encoder = std::unique_ptr<VideoEncoder>(new VideoEncoder());
  encoder->params_ = params;

  if (!encoder->initialize_av()) {

    return nullptr;
  }

  return std::move(encoder);
}

bool VideoEncoder::initialize_av() {
  av_log_set_level(AV_LOG_ERROR);

  const AVCodec *codec = nullptr;

  // Select encoder based on hardware acceleration type
  switch (params_.accel_type) {
  case EncoderParams::HardwareAccelerationType::NVIDIA:
    codec = avcodec_find_encoder_by_name("h264_nvenc");
    if (codec) {
      std::cout << "Using NVIDIA NVENC encoder" << std::endl;
    }
    break;

  case EncoderParams::HardwareAccelerationType::INTEL:
    codec = avcodec_find_encoder_by_name("h264_qsv");
    if (codec) {
      std::cout << "Using Intel Quick Sync encoder" << std::endl;
    }
    break;

  case EncoderParams::HardwareAccelerationType::AMD:
    codec = avcodec_find_encoder_by_name("h264_amf");
    if (codec) {
      std::cout << "Using AMD AMF encoder" << std::endl;
    }
    break;

  case EncoderParams::HardwareAccelerationType::APPLE:
    codec = avcodec_find_encoder_by_name("h264_videotoolbox");
    if (codec) {
      std::cout << "Using Apple VideoToolbox encoder" << std::endl;
    }
    break;

  case EncoderParams::HardwareAccelerationType::SOFTWARE:
  default:
    codec = avcodec_find_encoder_by_name("libx264");
    if (codec) {
      std::cout << "Using software encoder (libx264)" << std::endl;
    }
    break;
  }

  if (!codec) {
    std::cerr << "Failed to find encoder" << std::endl;
    return false;
  }

  // Allocate codec context
  codec_ctx_ = avcodec_alloc_context3(codec);
  if (!codec_ctx_) {
    std::cerr << "Could not allocate codec context" << std::endl;
    return false;
  }

  codec_ctx_->codec_id = AV_CODEC_ID_H264;
  codec_ctx_->codec_type = AVMEDIA_TYPE_VIDEO;
  codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
  codec_ctx_->width = params_.width;
  codec_ctx_->height = params_.height;
  codec_ctx_->time_base = {1, params_.fps};
  codec_ctx_->framerate = {params_.fps, 1};

  // Key(intra) frame rate
  codec_ctx_->gop_size = params_.fps * 2;

  /*
  Max B-Frames: Set to 2. For low motion content you can increase this to 4.
  B-Frames increase image quality but consume bitrate, which decrease image
  quality on movement. If you see pixelation or artifacting on your stream you
  may want to reduce this.
   */
  codec_ctx_->max_b_frames = 2;

  if (params_.constant_quality_enabled) {
    // Use constant quality instead of bitrate (18-28 range, lower = better
    // quality)
    av_opt_set_int(codec_ctx_->priv_data, "crf", 23, 0);
  }

  // Compression efficiency (slower -> better quality + higher cpu%)
  // [ultrafast, superfast, veryfast, faster, fast, medium, slow, slower,
  // veryslow] Set this option to "ultrafast" is critical for realtime encoding
  av_opt_set(codec_ctx_->priv_data, "preset", "ultrafast", 0);

  // [psnr, ssim, grain, zerolatency, fastdecode, animation]
  // This option is most critical for realtime encoding, because it removes
  // delay between 1th input frame and 1th output packet.
  av_opt_set(codec_ctx_->priv_data, "tune", "zerolatency", 0);

  // For streaming: repeat SPS/PPS with each keyframe so decoders can join
  // at any point in the stream
  av_opt_set(codec_ctx_->priv_data, "repeat-headers", "1", 0);

  // Open codec
  if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
    std::cerr << "Could not open codec" << std::endl;
    return false;
  }

  // Initialize SWS context for RGB to YUV conversion
  sws_ctx_ = sws_getContext(params_.width, params_.height, AV_PIX_FMT_RGB24,
                            params_.width, params_.height, AV_PIX_FMT_YUV420P,
                            SWS_BICUBIC, nullptr, nullptr, nullptr);
  if (!sws_ctx_) {
    std::cerr << "Could not initialize SWS context" << std::endl;
    return false;
  }

  // Allocate frame
  frame_ = av_frame_alloc();
  if (!frame_) {
    std::cerr << "Could not allocate frame" << std::endl;
    return false;
  }

  frame_->format = codec_ctx_->pix_fmt;
  frame_->width = codec_ctx_->width;
  frame_->height = codec_ctx_->height;

  if (av_frame_get_buffer(frame_, 0) < 0) {
    std::cerr << "Could not allocate frame buffer" << std::endl;
    return false;
  }

  // Allocate packet
  packet_ = av_packet_alloc();
  if (!packet_) {
    std::cerr << "Could not allocate packet" << std::endl;
    return false;
  }

  return true;
}

bool VideoEncoder::encode_frame(const cv::Mat &frame) {

  // Check if frame is empty or the color space is anything other than 8UC3
  if (frame.empty() || frame.type() != CV_8UC3) {
    std::cerr << "Invalid frame" << std::endl;
    return false;
  }

  // Ensure frame is writable
  if (av_frame_make_writable(frame_) < 0) {
    std::cerr << "Could not make frame writable" << std::endl;
    return false;
  }

  // RGB -> YUV420P conversion
  // Convert RGB to YUV420P
  const uint8_t *rgb_data[1] = {frame.data};
  int rgb_linesize[1] = {(int)(frame.step[0])};

  sws_scale(sws_ctx_, rgb_data, rgb_linesize, 0, codec_ctx_->height,
            frame_->data, frame_->linesize);

  frame_->pts = frame_count_++;

  // Check if we need to force a keyframe
  if (force_next_keyframe_) {
    frame_->pict_type = AV_PICTURE_TYPE_I;
    frame_->flags |= AV_FRAME_FLAG_KEY;
    force_next_keyframe_ = false;
  } else {
    frame_->pict_type = AV_PICTURE_TYPE_NONE; // Let encoder decide
  }

  // Send frame to encoder
  int ret = avcodec_send_frame(codec_ctx_, frame_);
  if (ret < 0) {
    std::cerr << "Error sending frame to encoder" << std::endl;
    return false;
  }

  switch (avcodec_receive_packet(codec_ctx_, packet_)) {
  case 0: {
    std::vector<uint8_t> packet_data(packet_->data,
                                     packet_->data + packet_->size);
    frame_buffer_.push_front(std::move(packet_data));
    av_packet_unref(packet_);

    return true;
  }
  case AVERROR(EAGAIN):
    av_packet_unref(packet_);
    return true;
  case AVERROR_EOF:
    return false;
  case AVERROR(EINVAL):
    return false;
  default:
    return false;
  }

  return true;
}

std::optional<std::vector<uint8_t>> VideoEncoder::get_last_compressed_frame() {

  if (frame_buffer_.empty()) {
    return std::nullopt;
  }

  auto cframe = std::move(frame_buffer_.back());
  frame_buffer_.pop_back();

  return cframe;
}

std::optional<std::vector<uint8_t>>
VideoEncoder::clone_latest_compressed_frame() const {

  if (frame_buffer_.empty()) {
    return std::nullopt;
  }

  return frame_buffer_.front();
}

std::optional<std::vector<uint8_t>>
VideoEncoder::get_latest_compressed_frame() {

  if (frame_buffer_.empty()) {
    return std::nullopt;
  }

  auto latest_frame = std::move(frame_buffer_.front());
  frame_buffer_.pop_front();

  return latest_frame;
}

std::optional<std::vector<std::vector<uint8_t>>>
VideoEncoder::get_compressed_frames(uint32_t num_frames) {

  if (frame_buffer_.empty()) {
    return std::nullopt;
  }

  std::vector<std::vector<uint8_t>> frames;
  frames.reserve(num_frames);

  for (uint32_t i = 0; i < num_frames; ++i) {
    if (frame_buffer_.empty()) {
      break;
    }
    frames.push_back(std::move(frame_buffer_.back()));
    frame_buffer_.pop_back();
  }

  return frames;
}

void VideoEncoder::force_keyframe() { force_next_keyframe_ = true; }