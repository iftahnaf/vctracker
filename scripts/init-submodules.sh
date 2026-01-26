#!/bin/bash

# Submodule initialization script
# This script ensures all necessary submodules are properly initialized

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "========================================="
echo "  vctracker - Submodule Setup"
echo "========================================="
echo ""

# Check if we're in a git repository
if [ ! -d "$PROJECT_DIR/.git" ]; then
    echo "Error: Not in a git repository!"
    echo "Please run this from the project root directory."
    exit 1
fi

echo "Initializing git submodules..."
echo ""

cd "$PROJECT_DIR"

# Initialize all submodules recursively
git submodule update --init --recursive

echo ""
echo "Verifying submodules..."

# Check pico-sdk
if [ -d "vcgimbal/lib/pico-sdk" ]; then
    echo "✓ pico-sdk initialized"
else
    echo "✗ pico-sdk not found"
    exit 1
fi

# Check micro-ROS SDK
if [ -d "lib/micro_ros_raspberrypi_pico_sdk" ]; then
    echo "✓ micro-ROS SDK initialized"
else
    echo "✗ micro-ROS SDK not found"
    exit 1
fi

# Check vcgimbal
if [ -d "vcgimbal" ] && [ -f "vcgimbal/include/Gimbal.h" ]; then
    echo "✓ vcgimbal library initialized"
else
    echo "✗ vcgimbal library not found"
    exit 1
fi

echo ""
echo "========================================="
echo "  All submodules initialized successfully!"
echo "========================================="
echo ""
echo "Next steps:"
echo "  1. Build the project:"
echo "     bash scripts/build.sh"
echo ""
echo "  2. Or run tests:"
echo "     bash scripts/build.sh test_gps"
echo "     bash scripts/build.sh test_leds"
echo "     bash scripts/build.sh test_gimbal"
echo ""
