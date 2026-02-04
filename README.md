# vctracker - Raspberry Pi Pico Antenna Tracker

[![Release](https://img.shields.io/github/v/release/iftahnaf/vctracker?label=release&color=0e8a16)](https://github.com/iftahnaf/vctracker/releases)
[![CI](https://img.shields.io/github/actions/workflow/status/iftahnaf/vctracker/ci.yml?branch=main&label=CI)](https://github.com/iftahnaf/vctracker/actions/workflows/ci.yml)
[![Tests](https://img.shields.io/github/actions/workflow/status/iftahnaf/vctracker/test-builds.yml?branch=main&label=Test%20Builds)](https://github.com/iftahnaf/vctracker/actions/workflows/test-builds.yml)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%20Pico-c51a4a)](https://www.raspberrypi.com/products/raspberry-pi-pico/)

An intelligent antenna tracker system for Raspberry Pi Pico that automatically points a camera gimbal toward a moving target using GPS positioning and USB serial communication for target updates.

## Features

- **Dual GPS System**: Onboard GPS module for tracker position + USB serial protocol for target position
- **Automatic Tracking**: Real-time pan/tilt calculation using geodetic algorithms
- **Servo Gimbal Control**: 2-axis MG90S servo gimbal via [vcgimbal library](vcgimbal/)
- **Visual Status Indicators**: 
  - GPS Fix LED (off/slow/fast blink based on fix quality)
  - Target Data LED (indicates successful target data reception)
- **Safety Features**:
  - Minimum elevation angle limits
  - Servo range protection
  - Message timeout detection
- **UART Communication**:
  - UART0: GPS module (38400 baud, UBX binary protocol)
  - UART1: Available for expansion
- **USB Serial Communication**:
  - Simple binary struct protocol for target data (lat/lon/alt)
  - Debug logging via USB CDC (115200 baud)

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
- Python3 with pyserial: `pip3 install pyserial`

**Development Tools:**
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git gcc-arm-none-eabi python3 python3-pip
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
- **GPS Module Test** - Test GPS reception and UBX parsing
- **Status LED Test** - Test LED control patterns
- **Gimbal Servo Test** - Test servo movement
- **Serial USB Test** - Test USB target data protocol
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

## USB Serial Protocol

The Pico receives target position updates over USB using a simple binary protocol:

**Target Data Packet (14 bytes):**
```
Byte 0:    Message Type (0x01 for TARGET_DATA)
Bytes 1-4: Latitude (float32, IEEE 754)
Bytes 5-8: Longitude (float32, IEEE 754)
Bytes 9-12: Altitude (float32, IEEE 754)
Byte 13:   Checksum (XOR of bytes 0-12)
```

**Response Messages:**
- `0x02` - ACK (data received successfully)
- `0x03 0xXX` - ERROR (error code in second byte)

**Example Python Usage:**
```python
import serial
import struct

# Create packet
packet = struct.pack('<B', 0x01)  # TARGET_DATA
packet += struct.pack('<f', 37.7749)   # latitude
packet += struct.pack('<f', -122.4194)  # longitude
packet += struct.pack('<f', 52.0)       # altitude

# Calculate XOR checksum
checksum = 0
for byte in packet:
    checksum ^= byte
packet += struct.pack('<B', checksum)

# Send to Pico
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.write(packet)
```

See [scripts/test_serial_usb.py](scripts/test_serial_usb.py) for a complete working example.

## Testing

Complete testing guide: [docs/testing.md](docs/testing.md)

Test each component individually:
- **GPS** - UART reception, UBX parsing
- **LEDs** - GPIO control, patterns
- **Gimbal** - Servo movement and positioning
- **USB Serial** - Target data protocol
- **System** - All components integrated

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
