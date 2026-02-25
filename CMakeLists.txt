cmake_minimum_required(VERSION 3.10)
project(video_encoder VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

# Include GNUInstallDirs for standard installation paths
include(GNUInstallDirs)

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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
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

# Installation rules
install(TARGETS video_encoder
    EXPORT video_encoder-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Export targets for find_package support
install(EXPORT video_encoder-targets
    FILE video_encoder-targets.cmake
    NAMESPACE video_encoder::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/video_encoder
)

# Create package config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/video_encoder-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/video_encoder-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/video_encoder-config.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/video_encoder-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/video_encoder-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/video_encoder
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
