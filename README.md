# vctracker - Raspberry Pi Pico Antenna Tracker

[![Release](https://img.shields.io/github/v/release/iftahnaf/vctracker?label=release&color=0e8a16)](https://github.com/iftahnaf/vctracker/releases)
[![CI](https://img.shields.io/github/actions/workflow/status/iftahnaf/vctracker/ci.yml?branch=main&label=CI)](https://github.com/iftahnaf/vctracker/actions/workflows/ci.yml)
[![Tests](https://img.shields.io/github/actions/workflow/status/iftahnaf/vctracker/test-builds.yml?branch=main&label=Test%20Builds)](https://github.com/iftahnaf/vctracker/actions/workflows/test-builds.yml)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%20Pico-c51a4a)](https://www.raspberrypi.com/products/raspberry-pi-pico/)

An intelligent antenna tracker system for Raspberry Pi Pico that automatically points a camera gimbal toward a moving target using GPS positioning and ROS2 integration.

## Features

- **Dual GPS System**: Onboard GPS for tracker position + ROS2 NavSatFix messages for target position
- **Automatic Tracking**: Real-time pan/tilt calculation using geodetic algorithms
- **Servo Gimbal Control**: 2-axis MG90S servo gimbal via [vcgimbal library](vcgimbal/)
- **Visual Status Indicators**: 
  - GPS Fix LED (off/slow/fast blink based on fix quality)
  - ROS Data LED (indicates message reception freshness)
- **Safety Features**:
  - Minimum elevation angle limits
  - Servo range protection
  - Message timeout detection
- **UART Communication**:
  - UART0: GPS module (9600 baud, NMEA protocol)
  - UART1: ROS2 messages (115200 baud, CSV format)
- **USB Serial Output**: Debug logging via USB CDC

## Hardware Requirements

### Components
- **Raspberry Pi Pico** (RP2040)
- **2x MG90S Servo Motors** (pan and tilt)
- **GPS Module** (UART, NMEA 0183 compatible, e.g., NEO-6M, NEO-7M)
- **2x LEDs** (with current-limiting resistors)
- **Power Supply** (5V for servos, USB for Pico)

### Wiring

| Component | Pico GPIO | Notes |
|-----------|-----------|-------|
| **Pan Servo Signal** | GPIO 16 | PWM output |
| **Tilt Servo Signal** | GPIO 17 | PWM output |
| **GPS TX** | GPIO 1 (UART0 RX) | GPS → Pico |
| **GPS RX** | GPIO 0 (UART0 TX) | Pico → GPS (optional) |
| **GPS Status LED** | GPIO 20 | Via 220Ω resistor |
| **ROS Status LED** | GPIO 21 | Via 220Ω resistor |
| **Servo Power** | 5V External | Shared ground with Pico |

See [docs/hardware.md](docs/hardware.md) for detailed wiring diagrams.

## Quick Start

### Prerequisites
- Ubuntu 20.04+ (or similar Linux distribution)
- ROS2 Humble (or compatible version)
- micro-ROS agent: `sudo apt install ros-humble-micro-ros-agent`

**Development Tools:**
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git gcc-arm-none-eabi python3
```

### Clone and Build

1. **Clone repository with submodules:**
   ```bash
   git clone --recursive https://github.com/iftahnaf/vctracker.git
   cd vctracker
   ```

2. **Initialize submodules (if needed):**
   ```bash
   bash scripts/init-submodules.sh
   ```

3. **Build using interactive menu:**
  ```bash
  bash scripts/build.sh
  # Enter 1-8 to select build target, then press Enter
  ```

### Available Build Targets

- **Main Antenna Tracker** - Full antenna tracking application
- **GPS Module Test** - Test GPS reception and parsing
- **Status LED Test** - Test LED control patterns
- **Gimbal Servo Test** - Test servo movement
- **micro-ROS Integration Test** - Test ROS2 connection
- **System Integration Test** - Complete system test
- **Build All Targets** - Build everything

### CI / Test Builds
- `ci.yml` builds release firmware and uploads artifacts
- `test-builds.yml` builds all primary targets (main, GPS, LEDs, gimbal, ROS) on every push/PR

### Flash to Pico

After build completes:
```
Flash firmware to Raspberry Pi Pico? (yes/no)
```

Automatic flashing works for **any target**:
1. Hold BOOTSEL button on Pico
2. Connect USB cable
3. Answer `yes` when prompted
4. Script auto-detects the built .uf2 file (test_leds, test_gimbal, etc.)
5. Script auto-detects RPI-RP2 drive and flashes

The flash script automatically finds whichever .uf2 file was just built, so flashing works seamlessly with all build targets.

### Run ROS2 Agent (in separate terminal)

The Pico communicates with ROS2 over **USB serial**:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

The USB cable handles both firmware flashing and ROS2 communication - no additional UART connection needed.

### Publish Target Positions
```bash
ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix \
  "header: {frame_id: 'gps'}
   latitude: 37.7749
   longitude: -122.4194
   altitude: 10.5
   position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
   position_covariance_type: 2" -r 1
```

## Testing

Complete testing guide: [docs/testing.md](docs/testing.md)

Test each component individually:
- **GPS** - UART reception, NMEA parsing
- **LEDs** - GPIO control, patterns
- **Gimbal** - Servo movement and positioning
- **micro-ROS** - Agent connection and messages
- **System** - All components integrated

## ROS2 Integration

The tracker uses **micro-ROS** for native ROS2 communication:
- **Subscribes to**: `/target/gps/fix` (sensor_msgs/NavSatFix)
- **Transport**: USB Serial with micro-ROS agent
- **Features**: Proper DDS messages, QoS policies, node management

See [docs/ros2-integration.md](docs/ros2-integration.md) for complete setup and examples.

## Project Structure

```
vctracker/
├── include/              # Header files
├── src/                  # Implementation files
├── examples/             # Example applications
├── vcgimbal/             # Gimbal control library (submodule)
├── docs/                 # Documentation
├── scripts/              # Build and flash scripts
└── CMakeLists.txt        # Build configuration
```

## Documentation

- [Hardware Setup Guide](docs/hardware.md)
- [Software Setup Guide](docs/setup.md)
- [ROS2 Integration Guide](docs/ros2-integration.md)

## License

MIT License
