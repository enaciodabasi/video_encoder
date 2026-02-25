#!/bin/bash

# =============================================================================
# Video Encoder Installation Script
# =============================================================================
# This script builds all components of the video encoder project:
# - Main library (libvideo_encoder.a)
# - Unit tests (video_encoder_test)
# - Metrics test (metrics_test)
# - Streaming server (streaming_server)
# - Go client (video_client)
# =============================================================================

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
GO_CLIENT_DIR="${SCRIPT_DIR}/test/streaming_test/client"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "\n${BLUE}=============================================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}=============================================================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# =============================================================================
# Check Dependencies
# =============================================================================
check_dependencies() {
    print_header "Checking Dependencies"
    
    local missing_deps=()
    
    # Check cmake
    if ! command -v cmake &> /dev/null; then
        missing_deps+=("cmake")
    else
        print_success "cmake found: $(cmake --version | head -n1)"
    fi
    
    # Check g++
    if ! command -v g++ &> /dev/null; then
        missing_deps+=("g++")
    else
        print_success "g++ found: $(g++ --version | head -n1)"
    fi
    
    # Check pkg-config
    if ! command -v pkg-config &> /dev/null; then
        missing_deps+=("pkg-config")
    else
        print_success "pkg-config found"
    fi
    
    # Check OpenCV
    if ! pkg-config --exists opencv4 2>/dev/null; then
        missing_deps+=("libopencv-dev")
    else
        print_success "OpenCV found: $(pkg-config --modversion opencv4)"
    fi
    
    # Check FFmpeg libraries
    for lib in libavcodec libavformat libavutil libswscale; do
        if ! pkg-config --exists $lib 2>/dev/null; then
            missing_deps+=("$lib-dev")
        else
            print_success "$lib found: $(pkg-config --modversion $lib)"
        fi
    done
    
    # Check GTest
    if ! pkg-config --exists gtest 2>/dev/null; then
        missing_deps+=("libgtest-dev")
    else
        print_success "GTest found"
    fi
    
    # Check yaml-cpp
    if ! pkg-config --exists yaml-cpp 2>/dev/null; then
        missing_deps+=("libyaml-cpp-dev")
    else
        print_success "yaml-cpp found"
    fi
    
    # Check Boost
    if [ ! -f /usr/include/boost/version.hpp ] && [ ! -f /usr/local/include/boost/version.hpp ]; then
        missing_deps+=("libboost-system-dev")
    else
        print_success "Boost found"
    fi
    
    # Check websocketpp
    if [ ! -f /usr/include/websocketpp/server.hpp ] && [ ! -f /usr/local/include/websocketpp/server.hpp ]; then
        missing_deps+=("libwebsocketpp-dev")
    else
        print_success "websocketpp found"
    fi
    
    # Check Go (optional)
    if ! command -v go &> /dev/null; then
        print_warning "Go not found (optional, needed for Go client)"
    else
        print_success "Go found: $(go version)"
    fi
    
    # Report missing dependencies
    if [ ${#missing_deps[@]} -ne 0 ]; then
        echo ""
        print_error "Missing dependencies:"
        for dep in "${missing_deps[@]}"; do
            echo "  - $dep"
        done
        echo ""
        echo "Install with:"
        echo "  sudo apt update && sudo apt install -y ${missing_deps[*]}"
        echo ""
        exit 1
    fi
    
    print_success "All dependencies satisfied"
}

# =============================================================================
# Build C++ Components
# =============================================================================
build_cpp() {
    print_header "Building C++ Components"
    
    # Create build directory
    mkdir -p "${BUILD_DIR}"
    cd "${BUILD_DIR}"
    
    # Configure with CMake
    echo "Configuring with CMake..."
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DBUILD_STREAMING_TEST=OFF \
        -DBUILD_METRICS_TEST=OFF
    
    # Get number of CPU cores
    NPROC=$(nproc 2>/dev/null || echo 4)
    
    # Build
    echo "Building with ${NPROC} parallel jobs..."
    make -j${NPROC}
    
    print_success "Built video_encoder library"
    print_success "Built video_encoder_test"
    print_success "Built metrics_test"
    print_success "Built streaming_server"
}

# =============================================================================
# Build Go Client
# =============================================================================
build_go_client() {
    print_header "Building Go Client"
    
    if ! command -v go &> /dev/null; then
        print_warning "Go not found, skipping Go client build"
        return 0
    fi
    
    cd "${GO_CLIENT_DIR}"
    
    # Download dependencies
    echo "Downloading Go dependencies..."
    go mod tidy 2>/dev/null || go mod init streaming_client 2>/dev/null
    go get github.com/gorilla/websocket
    
    # Build
    echo "Building Go client..."
    CGO_ENABLED=1 go build -o video_client
    
    print_success "Built video_client (Go client)"
}

# =============================================================================
# Install to System
# =============================================================================
install_system() {
    print_header "Installing to System"
    
    cd "${BUILD_DIR}"
    
    echo "Installing to: ${INSTALL_PREFIX}"
    echo "  Library:  ${INSTALL_PREFIX}/lib/libvideo_encoder.a"
    echo "  Headers:  ${INSTALL_PREFIX}/include/video_encoder/"
    echo ""
    
    # Check if we need sudo
    if [ -w "${INSTALL_PREFIX}" ]; then
        cmake --install . --prefix "${INSTALL_PREFIX}"
    else
        echo "Root privileges required for installation to ${INSTALL_PREFIX}"
        sudo cmake --install . --prefix "${INSTALL_PREFIX}"
    fi
    
    print_success "Library installed to ${INSTALL_PREFIX}/lib/"
    print_success "Headers installed to ${INSTALL_PREFIX}/include/video_encoder/"
    
    # Update library cache
    if command -v ldconfig &> /dev/null; then
        if [ -w "/etc/ld.so.conf.d" ] || [ "$(id -u)" -eq 0 ]; then
            sudo ldconfig 2>/dev/null || true
        fi
    fi
}

# =============================================================================
# Run Tests
# =============================================================================
run_tests() {
    print_header "Running Tests"
    
    cd "${BUILD_DIR}"
    
    # Run unit tests
    echo "Running unit tests..."
    if ./video_encoder_test; then
        print_success "All unit tests passed"
    else
        print_error "Some unit tests failed"
        return 1
    fi
}

# =============================================================================
# Print Summary
# =============================================================================
print_summary() {
    print_header "Build Summary"
    
    echo -e "Build directory: ${GREEN}${BUILD_DIR}${NC}"
    echo ""
    echo "Built executables:"
    
    if [ -f "${BUILD_DIR}/video_encoder_test" ]; then
        echo -e "  ${GREEN}✓${NC} ${BUILD_DIR}/video_encoder_test"
    fi
    
    if [ -f "${BUILD_DIR}/metrics_test" ]; then
        echo -e "  ${GREEN}✓${NC} ${BUILD_DIR}/metrics_test"
    fi
    
    if [ -f "${BUILD_DIR}/streaming_server" ]; then
        echo -e "  ${GREEN}✓${NC} ${BUILD_DIR}/streaming_server"
    fi
    
    if [ -f "${GO_CLIENT_DIR}/video_client" ]; then
        echo -e "  ${GREEN}✓${NC} ${GO_CLIENT_DIR}/video_client"
    fi
    
    echo ""
    echo "Usage:"
    echo "  # Run unit tests"
    echo "  ${BUILD_DIR}/video_encoder_test"
    echo ""
    echo "  # Run metrics benchmark"
    echo "  ${BUILD_DIR}/metrics_test"
    echo ""
    echo "  # Start streaming server"
    echo "  ${BUILD_DIR}/streaming_server ${SCRIPT_DIR}/test/streaming_test/config.yaml"
    echo ""
    echo "  # Start Go client (connect to streaming server)"
    echo "  ${GO_CLIENT_DIR}/video_client --server localhost:8765 --web-port 8080"
    echo ""
    echo "  # Web client (serve with Python)"
    echo "  cd ${SCRIPT_DIR}/test/streaming_test/web_client && python3 -m http.server 8080"
}

# =============================================================================
# Main
# =============================================================================
main() {
    print_header "Video Encoder Installation"
    
    echo "Project directory: ${SCRIPT_DIR}"
    echo "Build directory: ${BUILD_DIR}"
    
    # Parse arguments
    SKIP_DEPS=false
    SKIP_TESTS=false
    INSTALL_SYSTEM=false
    INSTALL_PREFIX="/usr/local"
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-deps)
                SKIP_DEPS=true
                shift
                ;;
            --skip-tests)
                SKIP_TESTS=true
                shift
                ;;
            --install)
                INSTALL_SYSTEM=true
                shift
                ;;
            --prefix)
                INSTALL_PREFIX="$2"
                shift 2
                ;;
            --help|-h)
                echo "Usage: $0 [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  --skip-deps   Skip dependency checking"
                echo "  --skip-tests  Skip running tests after build"
                echo "  --install     Install library to system (default: /usr/local)"
                echo "  --prefix DIR  Set installation prefix (default: /usr/local)"
                echo "  --help, -h    Show this help message"
                echo ""
                echo "Examples:"
                echo "  $0                      # Build only"
                echo "  $0 --install            # Build and install to /usr/local"
                echo "  $0 --install --prefix /opt/video_encoder"
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done
    
    # Check dependencies
    if [ "$SKIP_DEPS" = false ]; then
        check_dependencies
    fi
    
    # Build C++ components
    build_cpp
    
    # Build Go client
    build_go_client
    
    # Run tests
    if [ "$SKIP_TESTS" = false ]; then
        run_tests
    fi
    
    # Install to system if requested
    if [ "$INSTALL_SYSTEM" = true ]; then
        install_system
    fi
    
    # Print summary
    print_summary
    
    echo ""
    print_success "Installation complete!"
}

main "$@"
