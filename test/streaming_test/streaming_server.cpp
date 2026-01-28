#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

#include <opencv4/opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

// WebSocket++ headers
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "video_encoder/video_encoder.hpp"

using WebSocketServer = websocketpp::server<websocketpp::config::asio>;
using ConnectionHandle = websocketpp::connection_hdl;

// Global state
bool new_conn = true;
std::atomic<bool> g_running{true};
std::mutex g_connections_mutex;
std::set<ConnectionHandle, std::owner_less<ConnectionHandle>> g_connections;
WebSocketServer g_server;

// Global encoder pointer for quality control
std::unique_ptr<VideoEncoder> *g_encoder_ptr = nullptr;
std::mutex g_encoder_mutex;

// Configuration structure
struct Config {
  // Encoder params
  int width = 640;
  int height = 480;
  int fps = 30;
  std::string accel_type = "SOFTWARE";
  bool constant_quality_enabled = true;

  // Media params
  std::string input_file;
  bool loop = true;

  // Server params
  int port = 8765;
  std::string host = "0.0.0.0";
};

Config load_config(const std::string &config_path) {
  Config config;

  try {
    YAML::Node yaml = YAML::LoadFile(config_path);

    // Encoder settings
    if (yaml["encoder"]) {
      auto enc = yaml["encoder"];
      if (enc["width"])
        config.width = enc["width"].as<int>();
      if (enc["height"])
        config.height = enc["height"].as<int>();
      if (enc["fps"])
        config.fps = enc["fps"].as<int>();
      if (enc["accel_type"])
        config.accel_type = enc["accel_type"].as<std::string>();
      if (enc["constant_quality_enabled"])
        config.constant_quality_enabled =
            enc["constant_quality_enabled"].as<bool>();
    }

    // Media settings
    if (yaml["media"]) {
      auto media = yaml["media"];
      if (media["input_file"])
        config.input_file = media["input_file"].as<std::string>();
      if (media["loop"])
        config.loop = media["loop"].as<bool>();
    }

    // Server settings
    if (yaml["server"]) {
      auto server = yaml["server"];
      if (server["port"])
        config.port = server["port"].as<int>();
      if (server["host"])
        config.host = server["host"].as<std::string>();
    }

  } catch (const YAML::Exception &e) {
    std::cerr << "Error loading config: " << e.what() << std::endl;
    throw;
  }

  return config;
}

VideoEncoder::EncoderParams::HardwareAccelerationType
parse_accel_type(const std::string &type) {
  if (type == "NVIDIA")
    return VideoEncoder::EncoderParams::HardwareAccelerationType::NVIDIA;
  if (type == "INTEL")
    return VideoEncoder::EncoderParams::HardwareAccelerationType::INTEL;
  if (type == "AMD")
    return VideoEncoder::EncoderParams::HardwareAccelerationType::AMD;
  if (type == "APPLE")
    return VideoEncoder::EncoderParams::HardwareAccelerationType::APPLE;
  return VideoEncoder::EncoderParams::HardwareAccelerationType::SOFTWARE;
}

void on_open(ConnectionHandle hdl) {
  std::lock_guard<std::mutex> lock(g_connections_mutex);
  g_connections.insert(hdl);
  // Request immediate keyframe for the new client

  std::cout << "Client connected. Total clients: " << g_connections.size()
            << std::endl;

  new_conn = true;
}

void on_close(ConnectionHandle hdl) {
  std::lock_guard<std::mutex> lock(g_connections_mutex);
  g_connections.erase(hdl);
  std::cout << "Client disconnected. Total clients: " << g_connections.size()
            << std::endl;
}

// Parse simple key=value format: "crf=23" or "bitrate=2000000"
void handle_control_message(const std::string &msg) {
  std::cout << "Received control message: " << msg << std::endl;

  std::lock_guard<std::mutex> lock(g_encoder_mutex);
  if (!g_encoder_ptr || !*g_encoder_ptr) {
    std::cerr << "Encoder not available" << std::endl;
    return;
  }

  VideoEncoder::DynamicParams params;

  // Parse the message (format: "param=value")
  std::istringstream iss(msg);
  std::string token;

  while (std::getline(iss, token, '&')) {
    size_t eq_pos = token.find('=');
    if (eq_pos == std::string::npos)
      continue;

    std::string key = token.substr(0, eq_pos);
    std::string value = token.substr(eq_pos + 1);

    try {
      if (key == "crf") {
        int crf = std::stoi(value);
        if (crf >= 0 && crf <= 51) {
          params.crf = crf;
          std::cout << "Setting CRF to " << crf << std::endl;
        }
      } else if (key == "bitrate") {
        int64_t bitrate = std::stoll(value);
        if (bitrate > 0) {
          params.bitrate = bitrate;
          std::cout << "Setting bitrate to " << bitrate << std::endl;
        }
      } else if (key == "gop_size") {
        int gop = std::stoi(value);
        if (gop > 0) {
          params.gop_size = gop;
          std::cout << "Setting GOP size to " << gop << std::endl;
        }
      } else if (key == "keyframe") {
        // Force immediate keyframe
        (*g_encoder_ptr)->force_keyframe();
        std::cout << "Forcing keyframe" << std::endl;
      }
    } catch (const std::exception &e) {
      std::cerr << "Error parsing " << key << ": " << e.what() << std::endl;
    }
  }

  // Apply dynamic params if any were set
  if (params.crf.has_value() || params.bitrate.has_value() ||
      params.gop_size.has_value()) {
    if ((*g_encoder_ptr)->set_dynamic_params(params)) {
      // Force keyframe after quality change so clients see it immediately
      (*g_encoder_ptr)->force_keyframe();
    }
  }
}

void on_message(ConnectionHandle hdl, WebSocketServer::message_ptr msg) {
  // Handle text messages as control commands
  if (msg->get_opcode() == websocketpp::frame::opcode::text) {
    handle_control_message(msg->get_payload());
  }
}

void broadcast_frame(const std::vector<uint8_t> &frame_data) {
  std::lock_guard<std::mutex> lock(g_connections_mutex);

  for (auto &hdl : g_connections) {
    try {
      g_server.send(hdl, frame_data.data(), frame_data.size(),
                    websocketpp::frame::opcode::binary);
    } catch (const websocketpp::exception &e) {
      std::cerr << "Error sending frame: " << e.what() << std::endl;
    }
  }
}

void encoding_thread(const Config &config,
                     std::unique_ptr<VideoEncoder> &encoder) {
  // Open video file
  cv::VideoCapture cap(config.input_file);
  if (!cap.isOpened()) {
    std::cerr << "Failed to open video file: " << config.input_file
              << std::endl;
    g_running = false;
    return;
  }

  std::cout << "Opened video file: " << config.input_file << std::endl;

  // Calculate frame interval based on FPS
  auto frame_interval = std::chrono::microseconds(1000000 / config.fps);

  cv::Mat frame;
  cv::Mat resized_frame;

  while (g_running) {
    auto frame_start = std::chrono::steady_clock::now();

    // Read frame
    if (!cap.read(frame)) {
      if (config.loop) {
        // Reset to beginning
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        continue;
      } else {
        std::cout << "End of video reached" << std::endl;
        break;
      }
    }

    // Resize if necessary
    if (frame.cols != config.width || frame.rows != config.height) {
      cv::resize(frame, resized_frame, cv::Size(config.width, config.height));
    } else {
      resized_frame = frame;
    }

    // Convert BGR to RGB (OpenCV uses BGR, encoder expects RGB)
    cv::Mat rgb_frame;
    cv::cvtColor(resized_frame, rgb_frame, cv::COLOR_BGR2RGB);
    if (new_conn) {
      encoder->force_keyframe();
      new_conn = false;
    }
    // Encode frame
    if (!encoder->encode_frame(rgb_frame)) {
      std::cerr << "Failed to encode frame" << std::endl;
      continue;
    }

    // Get compressed frame and broadcast
    while (auto compressed = encoder->get_latest_compressed_frame()) {
      broadcast_frame(*compressed);
    }

    // Sleep to maintain frame rate
    auto frame_end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        frame_end - frame_start);
    if (elapsed < frame_interval) {
      std::this_thread::sleep_for(frame_interval - elapsed);
    }
  }

  cap.release();
}

void signal_handler(int signal) {
  std::cout << "\nReceived signal " << signal << ", shutting down..."
            << std::endl;
  g_running = false;
  g_server.stop_listening();
  g_server.stop();
}

int main(int argc, char *argv[]) {
  // Default config path
  std::string config_path = "config.yaml";

  if (argc > 1) {
    config_path = argv[1];
  }

  // Load configuration
  std::cout << "Loading config from: " << config_path << std::endl;
  Config config;
  try {
    config = load_config(config_path);
  } catch (...) {
    std::cerr << "Failed to load configuration" << std::endl;
    return 1;
  }

  std::cout << "Configuration loaded:" << std::endl;
  std::cout << "  Resolution: " << config.width << "x" << config.height
            << std::endl;
  std::cout << "  FPS: " << config.fps << std::endl;
  std::cout << "  Accel Type: " << config.accel_type << std::endl;
  std::cout << "  Input File: " << config.input_file << std::endl;
  std::cout << "  Server Port: " << config.port << std::endl;

  // Create encoder
  VideoEncoder::EncoderParams enc_params;
  enc_params.width = config.width;
  enc_params.height = config.height;
  enc_params.fps = config.fps;
  enc_params.accel_type = parse_accel_type(config.accel_type);
  enc_params.constant_quality_enabled = config.constant_quality_enabled;

  auto encoder = VideoEncoder::create_unique(enc_params);
  if (!encoder) {
    std::cerr << "Failed to create encoder" << std::endl;
    return 1;
  }

  // Set global encoder pointer for quality control
  g_encoder_ptr = &encoder;

  std::cout << "Encoder initialized successfully" << std::endl;

  // Setup signal handler
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Configure WebSocket server
  try {
    g_server.set_access_channels(websocketpp::log::alevel::none);
    g_server.set_error_channels(websocketpp::log::elevel::all);

    g_server.init_asio();
    g_server.set_reuse_addr(true);

    g_server.set_open_handler(&on_open);
    g_server.set_close_handler(&on_close);
    g_server.set_message_handler(&on_message);

    g_server.listen(config.port);
    g_server.start_accept();

    std::cout << "WebSocket server listening on port " << config.port
              << std::endl;

  } catch (const websocketpp::exception &e) {
    std::cerr << "WebSocket error: " << e.what() << std::endl;
    return 1;
  }

  // Start encoding thread
  std::thread encoder_thread(encoding_thread, std::ref(config),
                             std::ref(encoder));

  // Run server (blocking)
  try {
    g_server.run();
  } catch (const std::exception &e) {
    std::cerr << "Server error: " << e.what() << std::endl;
  }

  // Cleanup
  g_running = false;
  if (encoder_thread.joinable()) {
    encoder_thread.join();
  }

  std::cout << "Server shutdown complete" << std::endl;
  return 0;
}
