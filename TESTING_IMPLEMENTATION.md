# Testing & Build System Implementation Summary

## Overview

Complete testing framework and interactive build system has been implemented for the vctracker antenna tracker project. This enables thorough component-level testing and easy firmware selection/flashing.

## What's New

### 1. Component Tests (5 new test applications)

#### [examples/test_gps.cpp](examples/test_gps.cpp) - GPS Module Test
- **Tests**: UART0 initialization, NMEA parsing, position updates
- **Duration**: 60 seconds of continuous GPS monitoring
- **Output**: Real GPS fixes with latitude, longitude, altitude, satellite count
- **Success Criteria**: At least 1 GPS fix obtained

#### [examples/test_leds.cpp](examples/test_leds.cpp) - Status LED Test
- **Tests**: GPIO 20/21 initialization, on/off control, blinking patterns
- **Duration**: 6 different test phases (2-5 seconds each)
- **Patterns**: Steady on, 2Hz blink, 5Hz rapid, alternating, pulse, off
- **Success Criteria**: Visual observation of correct LED behavior

#### [examples/test_gimbal.cpp](examples/test_gimbal.cpp) - Gimbal Servo Test
- **Tests**: PWM on GPIO 16/17, 50Hz frequency, servo movement
- **Duration**: ~2 minutes with sweeps and patterns
- **Movements**: 8 predefined positions, pan sweep, tilt sweep, circular motion
- **Success Criteria**: Smooth motion to all positions

#### [examples/test_ros.cpp](examples/test_ros.cpp) - micro-ROS Integration Test
- **Tests**: USB CDC, micro-ROS transport, agent connection, subscriptions
- **Duration**: 60 seconds (30s timeout for agent)
- **Connection**: Waits for micro-ROS agent on host
- **Message**: Receives sensor_msgs/NavSatFix messages
- **Success Criteria**: Connection to agent and message reception

#### [examples/system_test.cpp](examples/system_test.cpp) - System Integration Test
- **Tests**: All components working together end-to-end
- **Duration**: ~2-3 minutes total
- **Scenarios**: 
  - Component hardware check (LEDs, gimbal response)
  - GPS simulation with position data
  - Geodetic calculations (azimuth, elevation, distance)
  - Gimbal movement to calculated angles for 6 target positions
  - 30-second 10Hz control loop with simulated tracking
- **Success Criteria**: All components respond correctly

### 2. Interactive Build System

#### [scripts/build.sh](scripts/build.sh) - Rewritten with Interactive Menu
**Features**:
- Arrow-key navigation menu (↑↓ keys)
- Enter to select build target
- Real-time build output
- Automatic submodule verification
- Post-build flash prompt
- Color-coded status indicators

**Build Targets**:
```
1. Main Antenna Tracker       - Full application with ROS2
2. GPS Module Test            - GPS UART and NMEA parsing
3. Status LED Test            - LED GPIO control
4. Gimbal Servo Test          - PWM servo control
5. micro-ROS Integration Test - ROS2 agent connection
6. System Integration Test    - Complete system simulation
7. Build All Targets          - All of above
8. Exit                       - Back to terminal
```

**Usage**:
```bash
bash scripts/build.sh
# Arrow keys to select, Enter to build
```

**Command-line mode** (non-interactive):
```bash
bash scripts/build.sh main          # Build main only
bash scripts/build.sh test_gps      # Build GPS test
bash scripts/build.sh all           # Build all targets
```

#### [scripts/init-submodules.sh](scripts/init-submodules.sh) - New Helper Script
- Initializes all git submodules automatically
- Verifies pico-sdk, micro-ROS SDK, vcgimbal
- Provides setup instructions
- Run once after cloning:
  ```bash
  bash scripts/init-submodules.sh
  ```

### 3. Build System Updates

#### [CMakeLists.txt](CMakeLists.txt) - New Build Target Selection
**Features**:
- `BUILD_TARGET` CMake variable for test selection
- Macro `create_executable()` for easy test addition
- Conditional compilation of tests
- All tests link to common library

**Example**:
```cmake
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TARGET=test_gps
```

### 4. Comprehensive Documentation

#### [docs/testing.md](docs/testing.md) - Complete Testing Guide
- Detailed test-by-test instructions
- Hardware prerequisites for each test
- Expected output and behavior
- Troubleshooting guide for each component
- Serial monitoring instructions
- Test execution order recommendations
- Performance metrics and expected results

### 5. Configuration Files

#### [.gitmodules](.gitmodules) - Updated Submodule Configuration
- `vcgimbal` - Gimbal control library
- `lib/micro_ros_raspberrypi_pico_sdk` - micro-ROS SDK (NEW)

**Automatic initialization** when cloning:
```bash
git clone --recursive https://github.com/iftahnaf/vctracker.git
# Or manually:
git submodule update --init --recursive
```

### 6. Documentation Updates

#### [README.md](README.md) - Updated Quick Start
- Interactive build system instructions
- Available build targets listed
- Automatic flash feature explained
- Testing section with reference to docs/testing.md

#### [docs/setup.md](docs/setup.md) - Updated Prerequisites
- ROS2 and micro-ROS agent installation
- Alternative build-from-source instructions
- Updated submodule verification
- New "Starting micro-ROS Agent" section

## File Structure

### Examples Directory
```
examples/
├── main.cpp              # Main antenna tracker application
├── test_gps.cpp          # GPS module test
├── test_leds.cpp         # Status LED test
├── test_gimbal.cpp       # Gimbal servo test
├── test_ros.cpp          # micro-ROS integration test
└── system_test.cpp       # System integration test
```

### Scripts Directory
```
scripts/
├── build.sh              # Interactive build with menu and flash
├── init-submodules.sh    # Git submodule initialization
└── flash.sh              # (Legacy, functionality moved to build.sh)
```

### Documentation
```
docs/
├── hardware.md           # Wiring diagrams
├── setup.md              # Development environment setup
├── ros2-integration.md   # ROS2 and micro-ROS guide
├── testing.md            # Testing guide (NEW)
├── architecture.md       # System architecture
└── quickstart.md         # Quick reference
```

## Build Workflow

### 1. Initial Setup (One-time)
```bash
git clone --recursive https://github.com/iftahnaf/vctracker.git
cd vctracker
bash scripts/init-submodules.sh  # Verifies submodules
```

### 2. Build & Flash
```bash
bash scripts/build.sh
# Interactive menu appears:
#   Select test/app with ↑↓ keys
#   Press Enter to build
#   After build completes, asks:
#     "Flash firmware to Raspberry Pi Pico? (yes/no)"
#   Follow on-screen instructions
```

### 3. Test Individual Components
```bash
bash scripts/build.sh test_gps      # Build GPS test
# Wait for RPI-RP2 drive
# Answer "yes" to flash
# Watch serial output
```

## Testing Workflow

### Complete Test Sequence
1. **test_leds** (5 min) - Quickest, verify GPIO
2. **test_gimbal** (10 min) - Verify servo response
3. **test_gps** (60+ sec) - Depends on satellite acquisition
4. **test_ros** (with agent on host) - Verify ROS2 connection
5. **system_test** (2-3 min) - Full integration
6. **main** (continuous) - Actual antenna tracking

### Monitor Test Output
```bash
screen /dev/ttyACM0 115200
# Exit: Ctrl+A, K
```

## Key Features

### Interactive Build Menu
- ✅ Arrow key navigation
- ✅ Color-coded status (RED=error, GREEN=success, BLUE=info)
- ✅ Test descriptions for each option
- ✅ Automatic RPI-RP2 drive detection
- ✅ Post-flash automatic reboot
- ✅ Multiple mount point support (Linux, macOS, Windows)

### Comprehensive Testing
- ✅ 5 individual component tests
- ✅ 1 full system integration test
- ✅ Hardware simulation for offline testing
- ✅ 10 Hz control loop verification
- ✅ Geodetic calculations validation

### Submodule Management
- ✅ Automatic initialization
- ✅ Verification script
- ✅ Clear error messages if missing
- ✅ Support for recursive clone

## Usage Examples

### Build Main Application
```bash
bash scripts/build.sh
# Select "Main Antenna Tracker"
# Select "Ready to flash? yes"
```

### Test GPS Module
```bash
bash scripts/build.sh test_gps
# Automatically builds GPS test
# Automatically flashes to Pico (with prompt)
# Monitor output with: screen /dev/ttyACM0 115200
```

### Build Everything
```bash
bash scripts/build.sh
# Select "Build All Targets"
# Creates 6 different .uf2 files in build/bin/
```

### Non-interactive Mode
```bash
bash scripts/build.sh main           # Build main
bash scripts/build.sh all            # Build all
bash scripts/build.sh test_gimbal    # Build gimbal test
```

### Get Help
```bash
bash scripts/build.sh -h
# Shows all available options
```

## Flash Output Format

After successful build:
```
✓ Build completed successfully!

Output files:
  ✓ antenna_tracker.uf2 (120K)

Flash firmware to Raspberry Pi Pico?

Instructions:
  1. Hold BOOTSEL button on Pico
  2. Connect USB cable (or reset while holding BOOTSEL)
  3. When 'RPI-RP2' drive appears, answer 'yes' below

Ready to flash? (yes/no):
```

## Troubleshooting

### Build Issues
```bash
# Missing submodules?
bash scripts/init-submodules.sh

# Build errors?
rm -rf build/
bash scripts/build.sh
```

### Test Failures
```bash
# GPS no fix - wait 30-60 seconds (cold start)
# LED not lit - check GPIO connections
# Servo not moving - check 5V power supply
# ROS not connecting - start agent first:
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### Flash Issues
```bash
# RPI-RP2 drive not found?
# 1. Put Pico in BOOTSEL mode
# 2. Connect USB
# 3. Check: ls /media/$USER/RPI-RP2 or /Volumes/RPI-RP2
# 4. Manual flash: cp build/bin/*.uf2 /media/$USER/RPI-RP2/
```

## Files Modified Summary

### New Files (11)
- `examples/test_gps.cpp` - GPS test (450+ lines)
- `examples/test_leds.cpp` - LED test (350+ lines)
- `examples/test_gimbal.cpp` - Gimbal test (400+ lines)
- `examples/test_ros.cpp` - ROS test (400+ lines)
- `examples/system_test.cpp` - System test (700+ lines)
- `scripts/build.sh` - Interactive build (400+ lines)
- `scripts/init-submodules.sh` - Submodule init (70+ lines)
- `docs/testing.md` - Testing guide (600+ lines)
- `MIGRATION_NOTES.md` - Already created
- `.gitmodules` - Updated
- `CMakeLists.txt` - Updated with test targets
- `README.md` - Updated
- `docs/setup.md` - Updated
- `docs/ros2-integration.md` - Already updated

### Total New Code
- ~2,800 lines of test code
- ~400 lines of build system
- ~600 lines of testing documentation
- ~30 lines of configuration

## Next Steps for Users

1. **First Time Setup**
   ```bash
   git clone --recursive https://github.com/iftahnaf/vctracker.git
   cd vctracker
   bash scripts/init-submodules.sh
   ```

2. **Start Testing**
   ```bash
   bash scripts/build.sh
   # Choose first test: test_leds (simplest)
   ```

3. **Progress Through Tests**
   - LED test → Gimbal test → GPS test → ROS test → System test → Main app

4. **Deploy in Production**
   ```bash
   bash scripts/build.sh main
   ```

## Testing Complete!

The vctracker project now has:
- ✅ 5 individual component tests
- ✅ 1 complete system integration test
- ✅ Interactive build and flash system
- ✅ Automatic submodule management
- ✅ Comprehensive testing documentation
- ✅ Easy-to-use command-line interface

Users can now easily verify that all hardware components work correctly before deploying the antenna tracker application!
