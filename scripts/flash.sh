#!/bin/bash

# Flash script for Raspberry Pi Pico
# Copies .uf2 file to mounted Pico drive

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

UF2_FILE="$BUILD_DIR/bin/antenna_tracker.uf2"

echo "========================================="
echo "  Flashing Raspberry Pi Pico"
echo "========================================="

# Check if UF2 file exists
if [ ! -f "$UF2_FILE" ]; then
    echo "Error: $UF2_FILE not found"
    echo "Please build the project first: bash scripts/build.sh"
    exit 1
fi

# Find Pico mount point
PICO_MOUNT=""

# Common mount points
MOUNT_POINTS=(
    "/media/$USER/RPI-RP2"
    "/media/RPI-RP2"
    "/mnt/RPI-RP2"
    "/run/media/$USER/RPI-RP2"
)

for mount_point in "${MOUNT_POINTS[@]}"; do
    if [ -d "$mount_point" ]; then
        PICO_MOUNT="$mount_point"
        break
    fi
done

if [ -z "$PICO_MOUNT" ]; then
    echo "Error: Raspberry Pi Pico not found"
    echo ""
    echo "Please:"
    echo "  1. Hold BOOTSEL button on Pico"
    echo "  2. Connect USB cable (or press reset while holding BOOTSEL)"
    echo "  3. Release BOOTSEL"
    echo "  4. Pico should appear as RPI-RP2 drive"
    echo ""
    echo "Searched locations:"
    for mount_point in "${MOUNT_POINTS[@]}"; do
        echo "  - $mount_point"
    done
    exit 1
fi

echo "Found Pico at: $PICO_MOUNT"
echo "Copying $UF2_FILE..."

cp "$UF2_FILE" "$PICO_MOUNT/"

echo ""
echo "========================================="
echo "  Flash completed successfully!"
echo "========================================="
echo "The Pico will automatically reboot and run the program."
echo ""
