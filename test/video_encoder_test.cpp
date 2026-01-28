#include <gtest/gtest.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include "video_encoder/video_encoder.hpp"

// Helper function to create a test frame with specified color
cv::Mat create_test_frame(int width, int height, cv::Scalar color) {
  cv::Mat frame(height, width, CV_8UC3, color);
  return frame;
}

// Helper function to create default encoder params
VideoEncoder::EncoderParams create_default_params() {
  VideoEncoder::EncoderParams params;
  params.width = 640;
  params.height = 480;
  params.fps = 30;
  params.accel_type =
      VideoEncoder::EncoderParams::HardwareAccelerationType::SOFTWARE;
  params.constant_quality_enabled = true;
  return params;
}

// =============================================================================
// Encoder Creation Tests
// =============================================================================

class VideoEncoderCreationTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(VideoEncoderCreationTest, CreateWithValidParams) {
  auto params = create_default_params();
  auto encoder = VideoEncoder::create_unique(params);
  ASSERT_NE(encoder, nullptr);
}

TEST_F(VideoEncoderCreationTest, CreateWithDifferentResolutions) {
  std::vector<std::pair<int, int>> resolutions = {
      {320, 240}, {640, 480}, {1280, 720}, {1920, 1080}};

  for (const auto &[width, height] : resolutions) {
    auto params = create_default_params();
    params.width = width;
    params.height = height;

    auto encoder = VideoEncoder::create_unique(params);
    ASSERT_NE(encoder, nullptr)
        << "Failed to create encoder for resolution " << width << "x" << height;
  }
}

TEST_F(VideoEncoderCreationTest, CreateWithDifferentFPS) {
  std::vector<int> fps_values = {15, 24, 30, 60};

  for (int fps : fps_values) {
    auto params = create_default_params();
    params.fps = fps;

    auto encoder = VideoEncoder::create_unique(params);
    ASSERT_NE(encoder, nullptr) << "Failed to create encoder for FPS " << fps;
  }
}

TEST_F(VideoEncoderCreationTest, CreateWithConstantQualityDisabled) {
  auto params = create_default_params();
  params.constant_quality_enabled = false;

  auto encoder = VideoEncoder::create_unique(params);
  ASSERT_NE(encoder, nullptr);
}

// =============================================================================
// Frame Encoding Tests
// =============================================================================

class VideoEncoderEncodingTest : public ::testing::Test {
protected:
  std::unique_ptr<VideoEncoder> encoder;

  void SetUp() override {
    auto params = create_default_params();
    encoder = VideoEncoder::create_unique(params);
    ASSERT_NE(encoder, nullptr);
  }

  void TearDown() override { encoder.reset(); }
};

TEST_F(VideoEncoderEncodingTest, EncodeSingleFrame) {
  cv::Mat frame = create_test_frame(640, 480, cv::Scalar(128, 128, 128));
  EXPECT_TRUE(encoder->encode_frame(frame));
}

TEST_F(VideoEncoderEncodingTest, EncodeMultipleFrames) {
  const int num_frames = 30;

  for (int i = 0; i < num_frames; ++i) {
    // Create frames with varying colors
    cv::Mat frame =
        create_test_frame(640, 480, cv::Scalar(i * 8 % 256, 128, 128));
    EXPECT_TRUE(encoder->encode_frame(frame))
        << "Failed to encode frame " << i;
  }

  // Should have some compressed frames in buffer
  EXPECT_GT(encoder->get_buffer_size(), 0u);
}

TEST_F(VideoEncoderEncodingTest, EncodeEmptyFrame) {
  cv::Mat empty_frame;
  EXPECT_FALSE(encoder->encode_frame(empty_frame));
}

TEST_F(VideoEncoderEncodingTest, EncodeWrongColorSpace) {
  // Create a grayscale frame (CV_8UC1 instead of CV_8UC3)
  cv::Mat grayscale_frame(480, 640, CV_8UC1, cv::Scalar(128));
  EXPECT_FALSE(encoder->encode_frame(grayscale_frame));
}

TEST_F(VideoEncoderEncodingTest, EncodeFrameAndRetrieve) {
  // Encode several frames to ensure we get output
  for (int i = 0; i < 10; ++i) {
    cv::Mat frame = create_test_frame(640, 480, cv::Scalar(i * 25, 100, 200));
    ASSERT_TRUE(encoder->encode_frame(frame));
  }

  // Check buffer has frames
  ASSERT_GT(encoder->get_buffer_size(), 0u);

  // Retrieve a compressed frame
  auto compressed = encoder->get_last_compressed_frame();
  ASSERT_TRUE(compressed.has_value());
  EXPECT_GT(compressed->size(), 0u);
}

// =============================================================================
// Frame Buffer Tests
// =============================================================================

class VideoEncoderBufferTest : public ::testing::Test {
protected:
  std::unique_ptr<VideoEncoder> encoder;

  void SetUp() override {
    auto params = create_default_params();
    encoder = VideoEncoder::create_unique(params);
    ASSERT_NE(encoder, nullptr);
  }

  void TearDown() override { encoder.reset(); }

  void fill_buffer(int num_frames) {
    for (int i = 0; i < num_frames; ++i) {
      cv::Mat frame = create_test_frame(640, 480, cv::Scalar(i * 10, 128, 128));
      encoder->encode_frame(frame);
    }
  }
};

TEST_F(VideoEncoderBufferTest, EmptyBufferReturnsNullopt) {
  EXPECT_FALSE(encoder->get_last_compressed_frame().has_value());
  EXPECT_FALSE(encoder->get_latest_compressed_frame().has_value());
  EXPECT_FALSE(encoder->clone_latest_compressed_frame().has_value());
}

TEST_F(VideoEncoderBufferTest, GetLastCompressedFrame) {
  fill_buffer(15);
  size_t initial_size = encoder->get_buffer_size();
  ASSERT_GT(initial_size, 0u);

  auto frame = encoder->get_last_compressed_frame();
  ASSERT_TRUE(frame.has_value());
  EXPECT_GT(frame->size(), 0u);

  // Buffer size should decrease by 1
  EXPECT_EQ(encoder->get_buffer_size(), initial_size - 1);
}

TEST_F(VideoEncoderBufferTest, GetLatestCompressedFrame) {
  fill_buffer(15);
  size_t initial_size = encoder->get_buffer_size();
  ASSERT_GT(initial_size, 0u);

  auto frame = encoder->get_latest_compressed_frame();
  ASSERT_TRUE(frame.has_value());
  EXPECT_GT(frame->size(), 0u);

  // Buffer size should decrease by 1
  EXPECT_EQ(encoder->get_buffer_size(), initial_size - 1);
}

TEST_F(VideoEncoderBufferTest, CloneLatestCompressedFrame) {
  fill_buffer(15);
  size_t initial_size = encoder->get_buffer_size();
  ASSERT_GT(initial_size, 0u);

  auto frame = encoder->clone_latest_compressed_frame();
  ASSERT_TRUE(frame.has_value());
  EXPECT_GT(frame->size(), 0u);

  // Buffer size should NOT change (clone doesn't remove)
  EXPECT_EQ(encoder->get_buffer_size(), initial_size);
}

TEST_F(VideoEncoderBufferTest, GetCompressedFramesBatch) {
  fill_buffer(30);
  size_t initial_size = encoder->get_buffer_size();
  ASSERT_GE(initial_size, 5u);

  auto frames = encoder->get_compressed_frames(5);
  ASSERT_TRUE(frames.has_value());
  EXPECT_EQ(frames->size(), 5u);

  // Buffer should have 5 fewer frames
  EXPECT_EQ(encoder->get_buffer_size(), initial_size - 5);
}

TEST_F(VideoEncoderBufferTest, GetCompressedFramesMoreThanAvailable) {
  fill_buffer(10);
  size_t initial_size = encoder->get_buffer_size();

  // Request more frames than available
  auto frames = encoder->get_compressed_frames(100);
  ASSERT_TRUE(frames.has_value());

  // Should get all available frames
  EXPECT_EQ(frames->size(), initial_size);
  EXPECT_EQ(encoder->get_buffer_size(), 0u);
}

// =============================================================================
// Dynamic Parameters Tests
// =============================================================================

class VideoEncoderDynamicParamsTest : public ::testing::Test {
protected:
  std::unique_ptr<VideoEncoder> encoder;

  void SetUp() override {
    auto params = create_default_params();
    encoder = VideoEncoder::create_unique(params);
    ASSERT_NE(encoder, nullptr);
  }

  void TearDown() override { encoder.reset(); }
};

TEST_F(VideoEncoderDynamicParamsTest, SetBitrate) {
  VideoEncoder::DynamicParams dyn_params;
  dyn_params.bitrate = 2000000; // 2 Mbps

  EXPECT_TRUE(encoder->set_dynamic_params(dyn_params));
}

TEST_F(VideoEncoderDynamicParamsTest, SetCRF) {
  VideoEncoder::DynamicParams dyn_params;
  dyn_params.crf = 20;

  EXPECT_TRUE(encoder->set_dynamic_params(dyn_params));
}

TEST_F(VideoEncoderDynamicParamsTest, SetGOPSize) {
  VideoEncoder::DynamicParams dyn_params;
  dyn_params.gop_size = 60;

  EXPECT_TRUE(encoder->set_dynamic_params(dyn_params));
}

TEST_F(VideoEncoderDynamicParamsTest, SetMaxBFrames) {
  VideoEncoder::DynamicParams dyn_params;
  dyn_params.max_b_frames = 4;

  EXPECT_TRUE(encoder->set_dynamic_params(dyn_params));
}

TEST_F(VideoEncoderDynamicParamsTest, SetMultipleDynamicParams) {
  VideoEncoder::DynamicParams dyn_params;
  dyn_params.bitrate = 1500000;
  dyn_params.crf = 25;
  dyn_params.gop_size = 30;
  dyn_params.max_b_frames = 3;

  EXPECT_TRUE(encoder->set_dynamic_params(dyn_params));
}

TEST_F(VideoEncoderDynamicParamsTest, EncodingAfterDynamicParamChange) {
  VideoEncoder::DynamicParams dyn_params;
  dyn_params.bitrate = 1000000;
  ASSERT_TRUE(encoder->set_dynamic_params(dyn_params));

  // Encoding should still work after parameter change
  for (int i = 0; i < 10; ++i) {
    cv::Mat frame = create_test_frame(640, 480, cv::Scalar(i * 25, 100, 200));
    EXPECT_TRUE(encoder->encode_frame(frame));
  }

  EXPECT_GT(encoder->get_buffer_size(), 0u);
}

// =============================================================================
// Static Parameters Tests (Reinitialization)
// =============================================================================

class VideoEncoderStaticParamsTest : public ::testing::Test {
protected:
  std::unique_ptr<VideoEncoder> encoder;

  void SetUp() override {
    auto params = create_default_params();
    encoder = VideoEncoder::create_unique(params);
    ASSERT_NE(encoder, nullptr);
  }

  void TearDown() override { encoder.reset(); }
};

TEST_F(VideoEncoderStaticParamsTest, ChangeResolution) {
  // Encode some frames first
  for (int i = 0; i < 5; ++i) {
    cv::Mat frame = create_test_frame(640, 480, cv::Scalar(128, 128, 128));
    encoder->encode_frame(frame);
  }

  // Change resolution
  auto new_params = create_default_params();
  new_params.width = 1280;
  new_params.height = 720;

  EXPECT_TRUE(encoder->set_static_params(new_params));

  // Encoding with new resolution should work
  cv::Mat new_frame = create_test_frame(1280, 720, cv::Scalar(200, 100, 50));
  EXPECT_TRUE(encoder->encode_frame(new_frame));
}

TEST_F(VideoEncoderStaticParamsTest, ChangeFPS) {
  auto new_params = create_default_params();
  new_params.fps = 60;

  EXPECT_TRUE(encoder->set_static_params(new_params));

  // Encoding should work after FPS change
  cv::Mat frame = create_test_frame(640, 480, cv::Scalar(128, 128, 128));
  EXPECT_TRUE(encoder->encode_frame(frame));
}

TEST_F(VideoEncoderStaticParamsTest, FlushPreservesFrames) {
  // Encode several frames
  for (int i = 0; i < 20; ++i) {
    cv::Mat frame = create_test_frame(640, 480, cv::Scalar(i * 12, 100, 150));
    encoder->encode_frame(frame);
  }

  size_t frames_before = encoder->get_buffer_size();

  // Change static params (triggers flush)
  auto new_params = create_default_params();
  new_params.fps = 24;
  ASSERT_TRUE(encoder->set_static_params(new_params));

  // Buffer should have at least as many frames as before (flush may add more)
  EXPECT_GE(encoder->get_buffer_size(), frames_before);
}

// =============================================================================
// Base64 Encoding Tests
// =============================================================================

class Base64Test : public ::testing::Test {};

TEST_F(Base64Test, EncodeEmptyVector) {
  std::vector<uint8_t> empty;
  std::string result = VideoEncoder::base64_encode(empty);
  EXPECT_TRUE(result.empty());
}

TEST_F(Base64Test, EncodeSingleByte) {
  std::vector<uint8_t> data = {0x4D}; // 'M'
  std::string result = VideoEncoder::base64_encode(data);
  EXPECT_EQ(result, "TQ==");
}

TEST_F(Base64Test, EncodeTwoBytes) {
  std::vector<uint8_t> data = {0x4D, 0x61}; // "Ma"
  std::string result = VideoEncoder::base64_encode(data);
  EXPECT_EQ(result, "TWE=");
}

TEST_F(Base64Test, EncodeThreeBytes) {
  std::vector<uint8_t> data = {0x4D, 0x61, 0x6E}; // "Man"
  std::string result = VideoEncoder::base64_encode(data);
  EXPECT_EQ(result, "TWFu");
}

TEST_F(Base64Test, EncodeKnownString) {
  // "Hello" in ASCII
  std::vector<uint8_t> data = {0x48, 0x65, 0x6C, 0x6C, 0x6F};
  std::string result = VideoEncoder::base64_encode(data);
  EXPECT_EQ(result, "SGVsbG8=");
}

TEST_F(Base64Test, EncodeBinaryData) {
  std::vector<uint8_t> data = {0x00, 0xFF, 0x80, 0x7F};
  std::string result = VideoEncoder::base64_encode(data);
  // Should produce valid base64 output
  EXPECT_FALSE(result.empty());
  // Base64 output should only contain valid characters
  for (char c : result) {
    EXPECT_TRUE((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '+' || c == '/' || c == '=');
  }
}

TEST_F(Base64Test, OutputLengthIsMultipleOfFour) {
  for (size_t len = 1; len <= 20; ++len) {
    std::vector<uint8_t> data(len, 0x42);
    std::string result = VideoEncoder::base64_encode(data);
    EXPECT_EQ(result.size() % 4, 0u) << "Failed for input length " << len;
  }
}

// =============================================================================
// Integration Tests
// =============================================================================

class VideoEncoderIntegrationTest : public ::testing::Test {};

TEST_F(VideoEncoderIntegrationTest, FullEncodingPipeline) {
  // Create encoder
  auto params = create_default_params();
  auto encoder = VideoEncoder::create_unique(params);
  ASSERT_NE(encoder, nullptr);

  // Encode frames simulating a video stream
  const int total_frames = 60; // 2 seconds at 30fps
  for (int i = 0; i < total_frames; ++i) {
    // Create frames with moving gradient
    cv::Mat frame(480, 640, CV_8UC3);
    for (int y = 0; y < 480; ++y) {
      for (int x = 0; x < 640; ++x) {
        frame.at<cv::Vec3b>(y, x) =
            cv::Vec3b((x + i * 5) % 256, (y + i * 3) % 256, (x + y + i) % 256);
      }
    }
    ASSERT_TRUE(encoder->encode_frame(frame)) << "Failed at frame " << i;
  }

  // Retrieve and verify all compressed frames
  size_t total_compressed = encoder->get_buffer_size();
  EXPECT_GT(total_compressed, 0u);

  size_t total_bytes = 0;
  while (auto frame = encoder->get_last_compressed_frame()) {
    EXPECT_GT(frame->size(), 0u);
    total_bytes += frame->size();

    // Verify base64 encoding works on compressed data
    std::string b64 = VideoEncoder::base64_encode(*frame);
    EXPECT_FALSE(b64.empty());
  }

  EXPECT_GT(total_bytes, 0u);
  EXPECT_EQ(encoder->get_buffer_size(), 0u);
}

TEST_F(VideoEncoderIntegrationTest, DynamicQualityAdjustment) {
  auto params = create_default_params();
  auto encoder = VideoEncoder::create_unique(params);
  ASSERT_NE(encoder, nullptr);

  // Encode with initial quality
  for (int i = 0; i < 15; ++i) {
    cv::Mat frame = create_test_frame(640, 480, cv::Scalar(100, 150, 200));
    encoder->encode_frame(frame);
  }

  // Change quality mid-stream
  VideoEncoder::DynamicParams dyn;
  dyn.crf = 18; // Higher quality
  ASSERT_TRUE(encoder->set_dynamic_params(dyn));

  // Continue encoding
  for (int i = 0; i < 15; ++i) {
    cv::Mat frame = create_test_frame(640, 480, cv::Scalar(100, 150, 200));
    EXPECT_TRUE(encoder->encode_frame(frame));
  }

  EXPECT_GT(encoder->get_buffer_size(), 0u);
}

TEST_F(VideoEncoderIntegrationTest, ResolutionChangeWorkflow) {
  auto params = create_default_params();
  params.width = 320;
  params.height = 240;

  auto encoder = VideoEncoder::create_unique(params);
  ASSERT_NE(encoder, nullptr);

  // Encode at low resolution
  for (int i = 0; i < 10; ++i) {
    cv::Mat frame = create_test_frame(320, 240, cv::Scalar(128, 128, 128));
    encoder->encode_frame(frame);
  }

  // Switch to higher resolution
  auto new_params = params;
  new_params.width = 1280;
  new_params.height = 720;
  ASSERT_TRUE(encoder->set_static_params(new_params));

  // Encode at new resolution
  for (int i = 0; i < 10; ++i) {
    cv::Mat frame = create_test_frame(1280, 720, cv::Scalar(128, 128, 128));
    EXPECT_TRUE(encoder->encode_frame(frame));
  }

  EXPECT_GT(encoder->get_buffer_size(), 0u);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
