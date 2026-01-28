cmake_minimum_required(VERSION 3.10)
project(video_encoder)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)
find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)
find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)

add_library(video_encoder STATIC src/video_encoder.cpp)

target_include_directories(video_encoder PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${AVCODEC_INCLUDE_DIR}
    ${AVFORMAT_INCLUDE_DIR}
    ${AVUTIL_INCLUDE_DIR}
    ${SWSCALE_INCLUDE_DIR}
)

target_link_libraries(video_encoder PUBLIC
    ${OpenCV_LIBS}
    ${AVCODEC_LIBRARY}
    ${AVFORMAT_LIBRARY}
    ${AVUTIL_LIBRARY}
    ${SWSCALE_LIBRARY}
)

install(TARGETS video_encoder
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
)

# =============================================================================
# Testing
# =============================================================================
option(BUILD_TESTS "Build the test suite" ON)

if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    
    enable_testing()
    
    add_executable(video_encoder_test test/video_encoder_test.cpp)
    
    target_link_libraries(video_encoder_test
        video_encoder
        GTest::gtest
        GTest::gtest_main
    )
    
    include(GoogleTest)
    gtest_discover_tests(video_encoder_test)
endif()

# =============================================================================
# Streaming Test Server
# =============================================================================
option(BUILD_STREAMING_TEST "Build the streaming test server" OFF)

if(BUILD_STREAMING_TEST)
    find_package(yaml-cpp REQUIRED)
    find_package(Boost REQUIRED COMPONENTS system)
    
    find_path(WEBSOCKETPP_INCLUDE_DIR websocketpp/server.hpp
        HINTS /usr/include /usr/local/include
    )
    
    if(NOT WEBSOCKETPP_INCLUDE_DIR)
        message(FATAL_ERROR "websocketpp not found. Install with: sudo apt install libwebsocketpp-dev")
    endif()
    
    add_executable(streaming_server 
        test/streaming_test/streaming_server.cpp
    )
    
    target_include_directories(streaming_server PRIVATE
        ${YAML_CPP_INCLUDE_DIRS}
        ${WEBSOCKETPP_INCLUDE_DIR}
    )
    
    target_link_libraries(streaming_server
        video_encoder
        yaml-cpp
        Boost::system
        pthread
    )
    
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/test/streaming_test/config.yaml
        ${CMAKE_CURRENT_BINARY_DIR}/config.yaml
        COPYONLY
    )
endif()

# =============================================================================
# Metrics Test
# =============================================================================
option(BUILD_METRICS_TEST "Build the encoder metrics test" ON)

if(BUILD_METRICS_TEST)
    add_executable(metrics_test test/metrics_test.cpp)
    
    target_link_libraries(metrics_test
        video_encoder
    )
endif()
