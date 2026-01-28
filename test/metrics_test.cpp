#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include <opencv4/opencv2/opencv.hpp>

#include "video_encoder/video_encoder.hpp"

// Test configuration
struct TestConfig {
  std::string name;
  int crf;
  std::string preset;
  int gop_size;
};

// Metrics collected during test
struct EncodingMetrics {
  std::string config_name;
  int crf;
  std::string preset;
  int gop_size;

  // Frame size metrics (bytes)
  std::vector<size_t> frame_sizes;
  double avg_frame_size;
  double min_frame_size;
  double max_frame_size;
  double std_dev_frame_size;
  double total_bytes;

  // Timing metrics (microseconds)
  std::vector<double> encode_times;
  double avg_encode_time;
  double min_encode_time;
  double max_encode_time;
  double std_dev_encode_time;

  // Derived metrics
  double avg_bitrate_kbps; // At 30 fps
  double frames_per_second_capability;

  void calculate() {
    // Frame size statistics
    if (!frame_sizes.empty()) {
      total_bytes =
          std::accumulate(frame_sizes.begin(), frame_sizes.end(), 0.0);
      avg_frame_size = total_bytes / frame_sizes.size();
      min_frame_size =
          *std::min_element(frame_sizes.begin(), frame_sizes.end());
      max_frame_size =
          *std::max_element(frame_sizes.begin(), frame_sizes.end());

      double sq_sum = 0;
      for (auto &s : frame_sizes) {
        sq_sum += (s - avg_frame_size) * (s - avg_frame_size);
      }
      std_dev_frame_size = std::sqrt(sq_sum / frame_sizes.size());
    }

    // Timing statistics
    if (!encode_times.empty()) {
      double total_time =
          std::accumulate(encode_times.begin(), encode_times.end(), 0.0);
      avg_encode_time = total_time / encode_times.size();
      min_encode_time =
          *std::min_element(encode_times.begin(), encode_times.end());
      max_encode_time =
          *std::max_element(encode_times.begin(), encode_times.end());

      double sq_sum = 0;
      for (auto &t : encode_times) {
        sq_sum += (t - avg_encode_time) * (t - avg_encode_time);
      }
      std_dev_encode_time = std::sqrt(sq_sum / encode_times.size());

      // Derived metrics
      frames_per_second_capability = 1000000.0 / avg_encode_time;
      avg_bitrate_kbps = (avg_frame_size * 8.0 * 30.0) / 1000.0; // 30 fps
    }
  }
};

// Generate test frames with varying complexity
std::vector<cv::Mat> generate_test_frames(int width, int height,
                                          int num_frames) {
  std::vector<cv::Mat> frames;
  frames.reserve(num_frames);

  for (int i = 0; i < num_frames; ++i) {
    cv::Mat frame(height, width, CV_8UC3);

    // Create varying content to simulate real video
    // Phase 1: Simple gradient (low complexity)
    // Phase 2: Moving shapes (medium complexity)
    // Phase 3: Noise + shapes (high complexity)

    int phase = (i * 3) / num_frames;

    if (phase == 0) {
      // Low complexity: gradient background
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          frame.at<cv::Vec3b>(y, x) =
              cv::Vec3b((x * 255 / width + i) % 256, (y * 255 / height) % 256,
                        128 + (i % 64));
        }
      }
    } else if (phase == 1) {
      // Medium complexity: moving shapes on gradient
      frame.setTo(cv::Scalar(50, 50, 80));

      // Draw moving circles
      for (int c = 0; c < 5; ++c) {
        int cx = (width / 6) * (c + 1) + (i * 3) % 50;
        int cy = height / 2 + static_cast<int>(50 * sin(i * 0.1 + c));
        cv::circle(frame, cv::Point(cx, cy), 30 + c * 10,
                   cv::Scalar(200 - c * 30, 100 + c * 20, 50 + c * 40), -1);
      }

      // Draw moving rectangles
      for (int r = 0; r < 3; ++r) {
        int rx = (i * 5 + r * 100) % (width - 60);
        int ry = 50 + r * 100;
        cv::rectangle(frame, cv::Point(rx, ry), cv::Point(rx + 60, ry + 40),
                      cv::Scalar(100 + r * 50, 150, 200 - r * 30), -1);
      }
    } else {
      // High complexity: noise + detailed patterns
      cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(100, 100, 100));

      // Add structured elements on top of noise
      for (int y = 0; y < height; y += 20) {
        cv::line(frame, cv::Point(0, y + (i % 20)),
                 cv::Point(width, y + (i % 20)), cv::Scalar(200, 200, 200), 1);
      }

      // Moving detailed shapes
      for (int s = 0; s < 8; ++s) {
        int sx = (s * width / 8 + i * 2) % width;
        int sy = (s * height / 8 + i) % height;
        cv::circle(frame, cv::Point(sx, sy), 15, cv::Scalar(255, 200, 100), 2);
        cv::putText(frame, std::to_string(s), cv::Point(sx - 5, sy + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255),
                    1);
      }
    }

    frames.push_back(frame);
  }

  return frames;
}

// Run encoding test with specific configuration
EncodingMetrics run_encoding_test(const TestConfig &config,
                                  const std::vector<cv::Mat> &frames, int width,
                                  int height, int fps) {
  EncodingMetrics metrics;
  metrics.config_name = config.name;
  metrics.crf = config.crf;
  metrics.preset = config.preset;
  metrics.gop_size = config.gop_size;

  // Create encoder with base parameters
  VideoEncoder::EncoderParams params;
  params.width = width;
  params.height = height;
  params.fps = fps;
  params.accel_type =
      VideoEncoder::EncoderParams::HardwareAccelerationType::SOFTWARE;
  params.constant_quality_enabled = true;

  auto encoder = VideoEncoder::create_unique(params);
  if (!encoder) {
    std::cerr << "Failed to create encoder for config: " << config.name
              << std::endl;
    return metrics;
  }

  // Apply test-specific dynamic parameters
  VideoEncoder::DynamicParams dyn_params;
  dyn_params.crf = config.crf;
  dyn_params.gop_size = config.gop_size;
  encoder->set_dynamic_params(dyn_params);

  // Encode all frames and collect metrics
  for (const auto &frame : frames) {
    auto start = std::chrono::high_resolution_clock::now();

    bool success = encoder->encode_frame(frame);

    auto end = std::chrono::high_resolution_clock::now();
    double encode_time_us =
        std::chrono::duration<double, std::micro>(end - start).count();

    if (success) {
      metrics.encode_times.push_back(encode_time_us);

      // Get all encoded packets
      while (auto packet = encoder->get_latest_compressed_frame()) {
        metrics.frame_sizes.push_back(packet->size());
      }
    }
  }

  metrics.calculate();
  return metrics;
}

void print_metrics_table(const std::vector<EncodingMetrics> &all_metrics) {
  std::cout << "\n";
  std::cout << std::string(120, '=') << "\n";
  std::cout << "ENCODING METRICS SUMMARY\n";
  std::cout << std::string(120, '=') << "\n\n";

  // Frame Size Table
  std::cout << "FRAME SIZE METRICS\n";
  std::cout << std::string(100, '-') << "\n";
  std::cout << std::left << std::setw(20) << "Config" << std::setw(8) << "CRF"
            << std::setw(12) << "Preset" << std::setw(12) << "Avg (KB)"
            << std::setw(12) << "Min (KB)" << std::setw(12) << "Max (KB)"
            << std::setw(12) << "StdDev" << std::setw(14) << "Bitrate (kbps)"
            << "\n";
  std::cout << std::string(100, '-') << "\n";

  for (const auto &m : all_metrics) {
    std::cout << std::left << std::setw(20) << m.config_name << std::setw(8)
              << m.crf << std::setw(12) << m.preset << std::fixed
              << std::setprecision(2) << std::setw(12)
              << (m.avg_frame_size / 1024.0) << std::setw(12)
              << (m.min_frame_size / 1024.0) << std::setw(12)
              << (m.max_frame_size / 1024.0) << std::setw(12)
              << (m.std_dev_frame_size / 1024.0) << std::setw(14)
              << m.avg_bitrate_kbps << "\n";
  }

  std::cout << "\n";

  // Timing Table
  std::cout << "ENCODING TIME METRICS\n";
  std::cout << std::string(100, '-') << "\n";
  std::cout << std::left << std::setw(20) << "Config" << std::setw(8) << "CRF"
            << std::setw(12) << "Preset" << std::setw(14) << "Avg (ms)"
            << std::setw(14) << "Min (ms)" << std::setw(14) << "Max (ms)"
            << std::setw(12) << "StdDev" << std::setw(12) << "Max FPS" << "\n";
  std::cout << std::string(100, '-') << "\n";

  for (const auto &m : all_metrics) {
    std::cout << std::left << std::setw(20) << m.config_name << std::setw(8)
              << m.crf << std::setw(12) << m.preset << std::fixed
              << std::setprecision(3) << std::setw(14)
              << (m.avg_encode_time / 1000.0) << std::setw(14)
              << (m.min_encode_time / 1000.0) << std::setw(14)
              << (m.max_encode_time / 1000.0) << std::setw(12)
              << (m.std_dev_encode_time / 1000.0) << std::setprecision(1)
              << std::setw(12) << m.frames_per_second_capability << "\n";
  }

  std::cout << "\n";

  // Summary comparison
  std::cout << "QUALITY vs SIZE COMPARISON (relative to highest quality)\n";
  std::cout << std::string(80, '-') << "\n";

  if (!all_metrics.empty()) {
    // Find best quality config (lowest CRF)
    auto best_it = std::min_element(
        all_metrics.begin(), all_metrics.end(),
        [](const auto &a, const auto &b) {
          return a.crf < b.crf || (a.crf == b.crf && a.preset == "veryslow");
        });

    double ref_size = best_it->avg_frame_size;
    double ref_time = best_it->avg_encode_time;

    std::cout << std::left << std::setw(20) << "Config" << std::setw(15)
              << "Size Ratio" << std::setw(15) << "Speed Ratio" << std::setw(20)
              << "Efficiency Score" << "\n";
    std::cout << std::string(80, '-') << "\n";

    for (const auto &m : all_metrics) {
      double size_ratio = m.avg_frame_size / ref_size;
      double speed_ratio = ref_time / m.avg_encode_time;
      double efficiency =
          speed_ratio / size_ratio; // Higher is better (faster with less size)

      std::cout << std::left << std::setw(20) << m.config_name << std::fixed
                << std::setprecision(2) << std::setw(15) << size_ratio
                << std::setw(15) << speed_ratio << std::setw(20) << efficiency
                << "\n";
    }
  }

  std::cout << "\n" << std::string(120, '=') << "\n";
}

void print_detailed_analysis(const std::vector<EncodingMetrics> &all_metrics) {
  std::cout << "\nDETAILED ANALYSIS\n";
  std::cout << std::string(80, '-') << "\n\n";

  // CRF impact analysis
  std::cout << "1. CRF IMPACT ON FILE SIZE:\n";
  std::cout << "   Lower CRF = Higher quality = Larger files\n\n";

  std::vector<const EncodingMetrics *> crf_sorted;
  for (const auto &m : all_metrics) {
    if (m.preset == "ultrafast") {
      crf_sorted.push_back(&m);
    }
  }
  std::sort(crf_sorted.begin(), crf_sorted.end(),
            [](const auto *a, const auto *b) { return a->crf < b->crf; });

  if (crf_sorted.size() >= 2) {
    double size_change = ((crf_sorted.back()->avg_frame_size /
                           crf_sorted.front()->avg_frame_size) -
                          1.0) *
                         100;
    std::cout << "   CRF " << crf_sorted.front()->crf << " → CRF "
              << crf_sorted.back()->crf << ": " << std::fixed
              << std::setprecision(1) << size_change << "% size reduction\n\n";
  }

  // Preset impact analysis
  std::cout << "2. PRESET IMPACT ON ENCODING SPEED:\n";
  std::cout
      << "   Faster preset = Less CPU time = Potentially larger files\n\n";

  std::vector<const EncodingMetrics *> preset_sorted;
  for (const auto &m : all_metrics) {
    if (m.crf == 23) {
      preset_sorted.push_back(&m);
    }
  }

  if (preset_sorted.size() >= 2) {
    auto fastest =
        std::min_element(preset_sorted.begin(), preset_sorted.end(),
                         [](const auto *a, const auto *b) {
                           return a->avg_encode_time < b->avg_encode_time;
                         });
    auto slowest =
        std::max_element(preset_sorted.begin(), preset_sorted.end(),
                         [](const auto *a, const auto *b) {
                           return a->avg_encode_time < b->avg_encode_time;
                         });

    double speed_ratio =
        (*slowest)->avg_encode_time / (*fastest)->avg_encode_time;
    std::cout << "   " << (*fastest)->preset << " is " << std::fixed
              << std::setprecision(1) << speed_ratio << "x faster than "
              << (*slowest)->preset << "\n\n";
  }

  // Real-time capability
  std::cout
      << "3. REAL-TIME STREAMING CAPABILITY (30 FPS = 33.3ms per frame):\n\n";
  for (const auto &m : all_metrics) {
    double frame_time_ms = m.avg_encode_time / 1000.0;
    bool realtime_capable = frame_time_ms < 33.3;
    double margin = 33.3 - frame_time_ms;

    std::cout << "   " << std::left << std::setw(20) << m.config_name << ": "
              << std::fixed << std::setprecision(2) << frame_time_ms
              << " ms/frame " << (realtime_capable ? "✓ OK" : "✗ TOO SLOW");
    if (realtime_capable) {
      std::cout << " (margin: " << margin << " ms)";
    }
    std::cout << "\n";
  }

  std::cout << "\n";
}

int main(int argc, char *argv[]) {
  // Test parameters
  const int WIDTH = 640;
  const int HEIGHT = 480;
  const int FPS = 30;
  const int NUM_FRAMES = 300; // 10 seconds of video

  std::cout << "Video Encoder Metrics Test\n";
  std::cout << std::string(50, '=') << "\n";
  std::cout << "Resolution: " << WIDTH << "x" << HEIGHT << "\n";
  std::cout << "Target FPS: " << FPS << "\n";
  std::cout << "Test frames: " << NUM_FRAMES << "\n\n";

  // Generate test frames
  std::cout << "Generating " << NUM_FRAMES << " test frames..." << std::flush;
  auto frames = generate_test_frames(WIDTH, HEIGHT, NUM_FRAMES);
  std::cout << " Done.\n\n";

  // Define test configurations
  std::vector<TestConfig> configs = {
      // CRF variations (with ultrafast preset)
      {"CRF-18-ultrafast", 18, "ultrafast", 60},
      {"CRF-23-ultrafast", 23, "ultrafast", 60},
      {"CRF-28-ultrafast", 28, "ultrafast", 60},
      {"CRF-35-ultrafast", 35, "ultrafast", 60},
      {"CRF-45-ultrafast", 45, "ultrafast", 60},

      // Preset variations (with CRF 23)
      {"CRF-23-superfast", 23, "superfast", 60},
      {"CRF-23-veryfast", 23, "veryfast", 60},
      {"CRF-23-faster", 23, "faster", 60},
      {"CRF-23-fast", 23, "fast", 60},
      {"CRF-23-medium", 23, "medium", 60},

      // GOP size variations
      {"CRF-23-GOP15", 23, "ultrafast", 15},
      {"CRF-23-GOP30", 23, "ultrafast", 30},
      {"CRF-23-GOP120", 23, "ultrafast", 120},
  };

  // Run tests
  std::vector<EncodingMetrics> all_metrics;

  for (const auto &config : configs) {
    std::cout << "Testing: " << std::left << std::setw(25) << config.name
              << "..." << std::flush;

    auto metrics = run_encoding_test(config, frames, WIDTH, HEIGHT, FPS);

    std::cout << " Avg: " << std::fixed << std::setprecision(2)
              << (metrics.avg_encode_time / 1000.0) << " ms, "
              << (metrics.avg_frame_size / 1024.0) << " KB\n";

    all_metrics.push_back(metrics);
  }

  // Print results
  print_metrics_table(all_metrics);
  print_detailed_analysis(all_metrics);

  return 0;
}
