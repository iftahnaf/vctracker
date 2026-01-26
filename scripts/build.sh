#!/bin/bash

# Interactive Build Script for vctracker
# Provides menu-based selection of build targets and optional flashing

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

# Color codes for UI
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Menu options
declare -a TARGETS=(
    "Main Antenna Tracker"
    "GPS Module Test"
    "Status LED Test"
    "Gimbal Servo Test"
    "micro-ROS Integration Test"
    "System Integration Test"
    "Build All Targets"
    "Exit"
)

# Build target CMake values
declare -a CMAKE_TARGETS=(
    "main"
    "test_gps"
    "test_leds"
    "test_gimbal"
    "test_ros"
    "system_test"
    "all"
    "exit"
)

# Descriptions
declare -a DESCRIPTIONS=(
    "Main antenna tracker firmware with ROS2 integration"
    "Test GPS module reception and NMEA parsing"
    "Test status LED control and patterns"
    "Test gimbal servo movement and positioning"
    "Test micro-ROS agent connection and message reception"
    "Complete system integration test with simulation"
    "Build all targets (main + all tests)"
    "Exit without building"
)

# Function to clear screen and show header
show_header() {
    clear
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║         vctracker - Interactive Build System                ║${NC}"
    echo -e "${CYAN}║    Raspberry Pi Pico Antenna Tracker with micro-ROS        ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# Function to show menu with arrow selection
show_menu() {
    local selection=$1
    
    show_header
    
    echo -e "${BLUE}Select build target:${NC}"
    echo ""
    
    for i in "${!TARGETS[@]}"; do
        local target="${TARGETS[$i]}"
        local desc="${DESCRIPTIONS[$i]}"
        
        if [ $i -eq $selection ]; then
            echo -e "${GREEN}▶ ${target}${NC}"
            echo -e "  ${YELLOW}→ ${desc}${NC}"
        else
            echo -e "  ${target}"
            echo -e "    ${desc}"
        fi
        
        if [ $i -lt $((${#TARGETS[@]} - 1)) ]; then
            echo ""
        fi
    done
    
    echo ""
    echo -e "${CYAN}Use ↑↓ arrow keys to navigate, Enter to select${NC}"
}

# Function to get keyboard input
get_input() {
    local key
    
    # Use bash read with timeout for arrow keys
    IFS= read -rsn1 key
    
    case "$key" in
        $'\x1b')  # ESC sequence
            read -rsn2 key  # Read two more bytes for arrow keys
            case "$key" in
                '[A')  echo "up" ;;      # Up arrow
                '[B')  echo "down" ;;    # Down arrow
                *)     echo "unknown" ;;
            esac
            ;;
        '')        echo "enter" ;;       # Enter key
        *)         echo "unknown" ;;
    esac
}

# Function to build the selected target
build_target() {
    local target="$1"
    
    clear
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║                    Building Project                         ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    # Check submodules
    echo -e "${BLUE}Checking dependencies...${NC}"
    
    if [ ! -d "$PROJECT_DIR/vcgimbal/lib/pico-sdk" ]; then
        echo -e "${RED}✗ Error: pico-sdk not found${NC}"
        echo "  Please run: git submodule update --init --recursive"
        return 1
    fi
    echo -e "${GREEN}✓ pico-sdk found${NC}"
    
    if [ ! -d "$PROJECT_DIR/lib/micro_ros_raspberrypi_pico_sdk" ]; then
        echo -e "${RED}✗ Error: micro-ROS SDK not found${NC}"
        echo "  Please run: git submodule update --init --recursive"
        return 1
    fi
    echo -e "${GREEN}✓ micro-ROS SDK found${NC}"
    
    echo ""
    echo -e "${BLUE}Creating build directory...${NC}"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    echo -e "${BLUE}Configuring CMake...${NC}"
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TARGET="$target" .. || {
        echo -e "${RED}✗ CMake configuration failed${NC}"
        return 1
    }
    
    echo ""
    echo -e "${BLUE}Building target: ${CYAN}${target}${NC}"
    echo -e "${BLUE}This may take a few minutes...${NC}"
    echo ""
    
    if make -j$(nproc); then
        echo ""
        echo -e "${GREEN}✓ Build completed successfully!${NC}"
        echo ""
        
        # Show output files
        echo -e "${CYAN}Output files:${NC}"
        for uf2_file in bin/*.uf2; do
            if [ -f "$uf2_file" ]; then
                size=$(du -h "$uf2_file" | cut -f1)
                echo -e "  ${GREEN}✓${NC} $(basename "$uf2_file") (${size})"
            fi
        done
        
        return 0
    else
        echo ""
        echo -e "${RED}✗ Build failed${NC}"
        return 1
    fi
}

# Function to ask about flashing
ask_flash() {
    local uf2_files=( build/bin/*.uf2 )
    
    if [ ! -e "${uf2_files[0]}" ]; then
        return 1
    fi
    
    echo ""
    echo -e "${YELLOW}Flash firmware to Raspberry Pi Pico?${NC}"
    echo ""
    echo "Instructions:"
    echo "  1. Hold BOOTSEL button on Pico"
    echo "  2. Connect USB cable (or reset while holding BOOTSEL)"
    echo "  3. When 'RPI-RP2' drive appears, answer 'yes' below"
    echo ""
    
    read -p "Ready to flash? (yes/no): " response
    
    if [ "$response" = "yes" ] || [ "$response" = "y" ]; then
        echo ""
        echo -e "${BLUE}Searching for RPI-RP2 drive...${NC}"
        
        # Check for mount point
        local mount_point=""
        
        if [ -d "/media/$USER/RPI-RP2" ]; then
            mount_point="/media/$USER/RPI-RP2"
        elif [ -d "/mnt/RPI-RP2" ]; then
            mount_point="/mnt/RPI-RP2"
        elif [ -d "/Volumes/RPI-RP2" ]; then
            mount_point="/Volumes/RPI-RP2"
        fi
        
        if [ -z "$mount_point" ]; then
            echo -e "${RED}✗ RPI-RP2 drive not found${NC}"
            echo ""
            echo "  Manual flashing:"
            echo "    1. Put Pico in BOOTSEL mode"
            echo "    2. Copy firmware to RPI-RP2 drive:"
            for uf2_file in "${uf2_files[@]}"; do
                echo "      cp $uf2_file /path/to/RPI-RP2/"
            done
            return 1
        fi
        
        echo -e "${GREEN}✓ Found RPI-RP2 at ${mount_point}${NC}"
        echo ""
        echo -e "${BLUE}Flashing firmware...${NC}"
        
        for uf2_file in "${uf2_files[@]}"; do
            if [ -f "$uf2_file" ]; then
                filename=$(basename "$uf2_file")
                echo "  Copying $filename..."
                
                if cp "$uf2_file" "$mount_point/"; then
                    echo -e "  ${GREEN}✓ ${filename} flashed${NC}"
                else
                    echo -e "  ${RED}✗ Failed to copy ${filename}${NC}"
                    return 1
                fi
            fi
        done
        
        echo ""
        echo -e "${GREEN}✓ Flash completed successfully!${NC}"
        echo ""
        echo "Pico should reboot automatically. Check serial output:"
        echo "  screen /dev/ttyACM0 115200"
        echo ""
        
        return 0
    else
        echo "Skipped flashing"
        return 1
    fi
}

# Main menu loop
main() {
    local selection=0
    local running=true
    
    while $running; do
        show_menu $selection
        
        read -t 0.1 input || input=$(get_input)
        
        case "$input" in
            "up")
                ((selection--))
                if [ $selection -lt 0 ]; then
                    selection=$((${#TARGETS[@]} - 1))
                fi
                ;;
            "down")
                ((selection++))
                if [ $selection -ge ${#TARGETS[@]} ]; then
                    selection=0
                fi
                ;;
            "enter")
                local target="${CMAKE_TARGETS[$selection]}"
                
                if [ "$target" = "exit" ]; then
                    clear
                    echo "Exiting..."
                    running=false
                else
                    if build_target "$target"; then
                        ask_flash
                    fi
                    
                    read -p "Press Enter to continue..."
                fi
                ;;
        esac
    done
}

# Show help if requested
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "Usage: bash $0 [options]"
    echo ""
    echo "Options:"
    echo "  (no args)      Interactive menu mode"
    echo "  main           Build main firmware only"
    echo "  test_gps       Build GPS test"
    echo "  test_leds      Build LED test"
    echo "  test_gimbal    Build gimbal test"
    echo "  test_ros       Build ROS test"
    echo "  system_test    Build system test"
    echo "  all            Build all targets"
    echo "  -h, --help     Show this help"
    exit 0
fi

# Command-line mode (if argument provided)
if [ -n "$1" ]; then
    if build_target "$1"; then
        ask_flash
    fi
else
    # Interactive menu mode
    main
fi
