# Software Setup Guide

## Development Environment

### Supported Platforms
- **Linux** (Ubuntu 20.04+, Debian 11+, Raspberry Pi OS)
- **macOS** (with Homebrew)
- **Windows** (WSL2 recommended)

### Prerequisites

#### Linux (Ubuntu/Debian)
```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    python3 \
    python3-pip
```

#### ROS2 & micro-ROS Agent (Required)
```bash
# Install ROS2 Humble (or your preferred distribution)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Install micro-ROS agent
sudo apt-get install -OS-micro-ros-agent

# Or from source if not available in repositories:
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

#### macOS
```bash
brew install cmake gcc-arm-embedded python3
# Note: ROS2 & micro-ROS agent installation on macOS requires Docker or manual build
```

#### Windows (WSL2)
```bash
# In WSL2 Ubuntu:
sudo apt-get update
sudo apt-get install -y build-essential cmake git gcc-arm-none-eabi python3
# Follow Linux ROS2 installation instructions above
```

## Project Setup

### 1. Clone Repository
```bash
git clone --recursive https://github.com/iftahnaf/vctracker.git
cd vctracker
```

**Important**: Use `--recursive` flag to fetch all submodules:
- `vcgimbal` (gimbal control library)
- `vcgimbal/lib/pico-sdk` (Raspberry Pi Pico SDK)
- `lib/micro_ros_raspberrypi_pico_sdk` (micro-ROS for Pico)

If already cloned without `--recursive`:
```bash
git submodule update --init --recursive
```

### 2. Verify Submodules
```bash
# Check pico-sdk
ls vcgimbal/lib/pico-sdk
# Should show pico-sdk files

# Check micro-ROS SDK
ls lib/micro_ros_raspberrypi_pico_sdk
# Should show micro-ROS files, CMakeLists.txt, etc.
```

### 3. Build Project
```bash
bash scripts/build.sh
```

Build options:
```bash
bash scripts/build.sh Release  # Optimized build (default)
bash scripts/build.sh Debug    # Debug build with symbols
```

### 4. Verify Build
```bash
ls build/bin/
# Should show:
# - antenna_tracker.uf2
# - antenna_tracker.bin
# - antenna_tracker.elf
```

## Flashing Firmware

### Method 1: Using Flash Script (Recommended)
```bash
# 1. Hold BOOTSEL button on Pico
# 2. Connect USB cable (or press reset while holding BOOTSEL)
# 3. Release BOOTSEL (Pico appears as RPI-RP2 drive)
# 4. Run flash script
bash scripts/flash.sh
```

### Method 2: Manual Copy
```bash
# 1. Put Pico in BOOTSEL mode (see above)
# 2. Copy UF2 file
cp build/bin/antenna_tracker.uf2 /media/$USER/RPI-RP2/

# On macOS:
cp build/bin/antenna_tracker.uf2 /Volumes/RPI-RP2/

# On Windows:
# Copy to RPI-RP2 drive in File Explorer
```

### Method 3: Using picotool
```bash
# Install picotool
sudo apt install picotool

# Flash firmware
picotool load build/bin/antenna_tracker.uf2
picotool reboot
```

## Serial Monitor & micro-ROS Agent

### Starting the micro-ROS Agent

The Pico firmware communicates with ROS2 via a micro-ROS agent running on the host computer. This must be started before the Pico can connect.

```bash
# Start the agent (in a dedicated terminal)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# On macOS:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/tty.usbmodem*

# Optional: Specify baud rate (default 115200)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

**Agent Output** (when Pico connects):
```
[1234567890.123456] (micro_ros_agent): Executing micro-ROS agent...
[1234567890.234567] (micro_ros_agent): UDP agent initialization... OK
[1234567890.345678] (micro_ros_agent): Running micro-ROS agent...
[1234567890.456789] (micro_ros_agent): Subscriber matched from client 0xdeadbeef
```

### Viewing Serial Debug Output

To see the Pico's debug output **while** the agent is running, use a secondary serial connection or check agent logs.

#### Linux (Debug Mode - Without Agent)
```bash
# Using screen (for debugging without agent)
screen /dev/ttyACM0 115200

# Using minicom
sudo minicom -D /dev/ttyACM0 -b 115200

# Using picocom
picocom /dev/ttyACM0 -b 115200

# Exit screen: Ctrl+A, then K
# Exit minicom: Ctrl+A, then X
# Exit picocom: Ctrl+A, then Ctrl+X
```

#### macOS
```bash
screen /dev/tty.usbmodem* 115200
```

#### Windows
```bash
# Use PuTTY, TeraTerm, or Arduino Serial Monitor
# Port: COM3 (or check Device Manager)
# Baud: 115200
# Note: Close serial monitor before starting agent
```

### Expected Serial Output
```
=========================================
  Raspberry Pi Pico Antenna Tracker
=========================================

Initializing Antenna Tracker...
GPS module initialized
ROS parser initialized
Waiting for micro-ROS agent...
PWMControllerPico: Initialized pin 16 at 50 Hz
PWMControllerPico: Initialized pin 17 at 50 Hz
Gimbal initialized successfully
Status LEDs initialized

=========================================
  Tracker initialized successfully!
=========================================

Hardware Configuration:
  Pan Servo:    GPIO 16
  Tilt Servo:   GPIO 17
  GPS UART:     UART0 (TX=0, RX=1, 9600 baud)
  micro-ROS:    USB Serial (automatic)
  GPS LED:      GPIO 20
  ROS LED:      GPIO 21

Please start micro-ROS agent on host:
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

Waiting for GPS fix and agent connection...
=========================================

--- Status Update ---
Tracker GPS: Lat=37.7749, Lon=-122.4194, Alt=10.5m, Sats=8, Fix=1
micro-ROS: Connected to agent
Target: Lat=37.8044, Lon=-122.2712, Alt=100.0m
Gimbal: Pan=15.3°, Tilt=5.2°
--------------------
```

## Configuration

### Customizing Pin Assignments

Edit [examples/main.cpp](../examples/main.cpp):

```cpp
// Pin definitions
constexpr uint PAN_SERVO_PIN = 16;    // Change pan servo pin
constexpr uint TILT_SERVO_PIN = 17;   // Change tilt servo pin

constexpr uint GPS_UART_ID = 0;       // UART0 or UART1
constexpr uint GPS_TX_PIN = 0;        // GPS UART TX
constexpr uint GPS_RX_PIN = 1;        // GPS UART RX
constexpr uint32_t GPS_BAUD = 9600;   // GPS baud rate

constexpr uint ROS_UART_ID = 1;       // UART0 or UART1
constexpr uint ROS_TX_PIN = 4;        // ROS UART TX
constexpr uint ROS_RX_PIN = 5;        // ROS UART RX
constexpr uint32_t ROS_BAUD = 115200; // ROS baud rate

constexpr uint GPS_LED_PIN = 20;      // GPS status LED
constexpr uint ROS_LED_PIN = 21;      // ROS status LED
```

### Adjusting Update Rate

```cpp
// Update rate
constexpr uint32_t UPDATE_INTERVAL_MS = 100; // 10 Hz (100ms)
```

Common rates:
- 100ms (10 Hz) - Default, good balance
- 50ms (20 Hz) - Higher update rate
- 200ms (5 Hz) - Lower update rate, less CPU

### Setting Minimum Elevation

```cpp
// In main() function:
tracker.setMinElevation(-5.0f); // Minimum tilt angle (degrees)
```

Prevents pointing below horizon (adjust for mounting angle).

### Enabling/Disabling Auto-Tracking

```cpp
tracker.setAutoTracking(true);  // Enable automatic tracking
tracker.setAutoTracking(false); // Manual mode only
```

## Debugging

### Debug Build
```bash
bash scripts/build.sh Debug
```

Debug build includes:
- Verbose logging
- Symbol information
- No optimizations
- Assertions enabled

### Serial Debug Output

Enable verbose output in source files:
```cpp
// Add to main.cpp for extra logging
#define DEBUG_GPS 1
#define DEBUG_ROS 1
#define DEBUG_TRACKING 1
```

### Common Issues

#### Build Fails: pico-sdk Not Found
```bash
# Solution: Initialize submodules
git submodule update --init --recursive
```

#### Build Fails: ARM Toolchain Not Found
```bash
# Solution: Install ARM GCC
sudo apt-get install gcc-arm-none-eabi
```

#### Pico Not Detected in BOOTSEL Mode
```bash
# Solution 1: Check USB cable (must support data, not just power)
# Solution 2: Try different USB port
# Solution 3: Hold BOOTSEL before connecting USB

# Verify with lsusb:
lsusb | grep "2e8a:0003"
# Should show: "Raspberry Pi RP2 Boot"
```

#### Serial Port Permission Denied
```bash
# Solution: Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in
```

#### No Serial Output
```bash
# Check USB connection
ls /dev/ttyACM*

# If no device, check with dmesg:
dmesg | grep tty

# Verify USB serial is enabled in code (should be default):
# In CMakeLists.txt:
# pico_enable_stdio_usb(antenna_tracker 1)
```

## Advanced Configuration

### Changing GPS Baud Rate

If your GPS module uses different baud rate:

1. Update `GPS_BAUD` in main.cpp
2. Rebuild and reflash

### Using Different GPS Sentences

Currently supports GPGGA. To add more:

Edit [src/GPSModule.cpp](../src/GPSModule.cpp):
```cpp
bool GPSModule::parseNMEA(const std::string& sentence) {
    // Add new sentence types
    if (sentence_type == "$GPRMC") {
        return parseGPRMC(fields);
    }
    // ...
}
```

### Custom ROS Message Format

Default: CSV format `lat,lon,alt,status\n`

To change, edit [src/ROSParser.cpp](../src/ROSParser.cpp):
```cpp
bool ROSParser::parseTextMessage(const std::string& line) {
    // Implement custom parsing
}
```

## Performance Optimization

### Reducing Memory Usage
- Disable debug logging
- Reduce NMEA buffer size
- Limit status update frequency

### Improving Tracking Accuracy
- Increase update rate (reduce UPDATE_INTERVAL_MS)
- Use DGPS/RTK GPS module
- Add Kalman filtering (future enhancement)

### Power Optimization
- Reduce LED brightness (PWM)
- Lower GPS update rate
- Use sleep modes between updates

## Clean Build

```bash
# Remove build directory
rm -rf build

# Rebuild from scratch
bash scripts/build.sh
```

## Testing Without Hardware

### GPS Simulation
Connect GPS RX to a USB-UART adapter and send NMEA:
```bash
echo '$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47' > /dev/ttyUSB0
```

### ROS Data Simulation
Send CSV data via UART:
```bash
echo "37.7749,-122.4194,10.5,1" > /dev/ttyUSB1
```

## Next Steps

- [Hardware Setup Guide](hardware.md)
- [ROS2 Integration Guide](ros2-integration.md)
- Calibrate servos if needed
- Mount hardware securely
- Test in field conditions
