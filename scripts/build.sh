#!/bin/bash

# Interactive Build Script for vctracker
# Provides menu-based selection of build targets and optional flashing

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Build targets
declare -A TARGETS=(
    [1]="main"
    [2]="test_gps"
    [3]="test_leds"
    [4]="test_gimbal"
    [5]="test_ros"
    [6]="system_test"
    [7]="all"
    [8]="help"
)

declare -A DESCRIPTIONS=(
    [1]="Main antenna tracker firmware with ROS2 integration"
    [2]="Test GPS module reception and NMEA parsing"
    [3]="Test status LED control and patterns"
    [4]="Test gimbal servo movement and positioning"
    [5]="Test micro-ROS agent connection and message reception"
    [6]="Complete system integration test with simulation"
    [7]="Build all targets (main + all tests)"
    [8]="Show this help message"
)

show_header() {
    clear
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║         vctracker - Interactive Build System                ║${NC}"
    echo -e "${CYAN}║    Raspberry Pi Pico Antenna Tracker with micro-ROS        ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

show_menu() {
    show_header
    echo -e "${BLUE}Available build targets:${NC}"
    echo ""
    
    for i in {1..8}; do
        printf "  ${GREEN}%-1d${NC}. %-35s %s\n" "$i" "${DESCRIPTIONS[$i]}" ""
    done
    
    echo ""
    echo -e "${CYAN}Usage: bash build.sh [1-8]${NC}"
    echo -e "  Example: ${GREEN}bash build.sh 1${NC}  (builds main)"
    echo ""
}

show_help() {
    show_header
    echo -e "${BLUE}Build Targets:${NC}"
    echo ""
    echo "  1. main          - Main antenna tracker firmware"
    echo "  2. test_gps      - GPS module test"
    echo "  3. test_leds     - LED control test"
    echo "  4. test_gimbal   - Gimbal servo test"
    echo "  5. test_ros      - micro-ROS integration test"
    echo "  6. system_test   - Full system integration test"
    echo "  7. all           - Build everything"
    echo "  8. help          - Show this help"
    echo ""
    echo -e "${BLUE}Command-line usage:${NC}"
    echo "  bash build.sh              - Show interactive menu"
    echo "  bash build.sh 1            - Build main target"
    echo "  bash build.sh --help       - Show this help"
    echo ""
}

check_dependencies() {
    echo -e "${BLUE}Checking dependencies...${NC}"
    
    if [ -d "$PROJECT_DIR/vcgimbal/lib/pico-sdk" ]; then
        echo -e "${GREEN}✓${NC} pico-sdk found"
    else
        echo -e "${RED}✗${NC} pico-sdk not found"
        echo "  Please run: bash scripts/init-submodules.sh"
        exit 1
    fi
    
    if [ -d "$PROJECT_DIR/lib/micro_ros_raspberrypi_pico_sdk" ]; then
        echo -e "${GREEN}✓${NC} micro-ROS SDK found"
    else
        echo -e "${RED}✗${NC} micro-ROS SDK not found"
        echo "  Please run: bash scripts/init-submodules.sh"
        exit 1
    fi
    
    echo ""
}

build_target() {
    local target=$1
    
    if [ -z "${TARGETS[$target]}" ]; then
        echo -e "${RED}✗${NC} Invalid target: $target"
        show_menu
        exit 1
    fi
    
    local cmake_target="${TARGETS[$target]}"
    
    if [ "$cmake_target" = "help" ]; then
        show_help
        exit 0
    fi
    
    show_header
    echo -e "${BLUE}Building Project${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    check_dependencies
    
    # Create build directory
    echo -e "${BLUE}Creating build directory...${NC}"
    mkdir -p "$BUILD_DIR"
    
    # Clean CMake cache if BUILD_TARGET changed (prevents cached config)
    if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
        cached_target=$(grep "^BUILD_TARGET:" "$BUILD_DIR/CMakeCache.txt" | cut -d'=' -f2)
        if [ "$cached_target" != "$cmake_target" ]; then
            echo -e "${YELLOW}⚠${NC} BUILD_TARGET changed, clearing cache"
            rm -rf "$BUILD_DIR/CMakeFiles" "$BUILD_DIR/CMakeCache.txt"
        fi
    fi
    
    cd "$BUILD_DIR"
    
    # Configure CMake
    echo -e "${BLUE}Configuring CMake...${NC}"
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TARGET="$cmake_target" \
        .. || {
        echo -e "${RED}✗${NC} CMake configuration failed"
        exit 1
    }
    
    echo ""
    
    # Build
    echo -e "${BLUE}Compiling (target: $cmake_target)...${NC}"
    make -j"$(nproc)" || {
        echo -e "${RED}✗${NC} Build failed"
        exit 1
    }
    
    echo ""
    echo -e "${GREEN}✓${NC} Build completed successfully!"
    echo ""
    
    # List outputs
    if [ -d "bin" ]; then
        echo -e "${BLUE}Output files:${NC}"
        ls -lh bin/ | grep -E "\.uf2|\.bin|\.elf" | awk '{printf "  %s  %8s  %s\n", $6, $5, $9}'
        echo ""
    fi
    
    # Ask about flashing
    echo -e "${YELLOW}Detecting Pico device...${NC}"
    
    # Check for RPI-RP2 mount
    if mount | grep -q "RPI-RP2"; then
        RPI_RP2=$(mount | grep "RPI-RP2" | awk '{print $3}')
        echo -e "${GREEN}✓${NC} Found Pico at: $RPI_RP2"
        echo ""
        
        read -p "Flash firmware to Pico? (y/n) " -n 1 -r
        echo
        
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo -e "${BLUE}Flashing...${NC}"
            
            # Find the .uf2 file
            uf2_file=$(find . -maxdepth 2 -name "*.uf2" ! -path "./lib/*" ! -path "./vcgimbal/*" | head -1)
            
            if [ -z "$uf2_file" ]; then
                echo -e "${RED}✗${NC} No .uf2 file found"
                exit 1
            fi
            
            cp "$uf2_file" "$RPI_RP2/"
            echo -e "${GREEN}✓${NC} Firmware flashed successfully!"
            sleep 2
        fi
    else
        echo -e "${YELLOW}⚠${NC} Pico device not detected"
        echo "  Connect your Pico in bootloader mode (hold BOOTSEL during power-on)"
        echo ""
    fi
}

# Main entry point
if [ $# -eq 0 ]; then
    # Interactive mode
    show_menu
    
    read -p "Select target (1-8): " -n 1 selection
    echo ""
    
    if ! [[ "$selection" =~ ^[1-8]$ ]]; then
        echo -e "${RED}✗${NC} Invalid selection"
        exit 1
    fi
    
    build_target "$selection"
elif [ "$1" = "--help" ] || [ "$1" = "-h" ] || [ "$1" = "help" ]; then
    show_help
elif [[ "$1" =~ ^[1-8]$ ]]; then
    build_target "$1"
else
    echo -e "${RED}✗${NC} Invalid argument: $1"
    show_menu
    exit 1
fi
