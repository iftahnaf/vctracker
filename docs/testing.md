# Testing Guide - vctracker

Complete guide for testing the antenna tracker system and its individual components.

## Quick Start

### 1. Initialize Submodules
```bash
bash scripts/init-submodules.sh
```

### 2. Build and Flash Tests

```bash
bash scripts/build.sh
# Interactive menu will appear - enter 1-8 to pick a target
# When prompted, select yes to automatically flash to Pico
```

**Auto-Detection:** The flash script automatically detects which .uf2 file was built (main, test_leds, test_gimbal, etc.) and flashes that specific target to your Pico. This works for any build target.

CI note: GitHub Actions (`test-builds.yml`) builds all primary targets (main, GPS, LEDs, gimbal, serial) on every push/PR.

## Component Tests

### GPS Module Test

**What it tests:**
- UART0 initialization (GPIO 0/1, 9600 baud)
- GPS NMEA sentence reception
- Latitude, longitude, altitude parsing
- Fix quality and satellite count
- 60-second continuous operation

**Prerequisites:**
- GPS module connected to UART0
- GPS module powered and configured for NMEA output
- GPS module with clear sky view for satellite acquisition

**Run:**
```bash
bash scripts/build.sh
# Select "GPS Module Test"
```

**Expected Output:**
```
GPS Fix #1:
  Latitude:  37.7749°
  Longitude: -122.4194°
  Altitude:  10.5 m
  Satellites: 8
  Fix Type: GPS Fix
```

**Troubleshooting:**
- No fixes obtained?
  - Check GPS power (usually 3.3V or 5V)
  - Verify UART0 connections (GPIO 0=TX, GPIO 1=RX)
  - Check that GPS module TX is connected to GPIO 1 (RX)
  - Ensure clear sky view (at least 5-10 satellites needed)
  - Wait 30-60 seconds for first fix (cold start)

---

### Status LED Test

**What it tests:**
- GPIO 20 and 21 initialization
- Digital output control
- LED on/off state
- Blinking patterns (various frequencies)
- Pulse effects

**Prerequisites:**
- LED 1 on GPIO 20 with 220Ω current-limiting resistor
- LED 2 on GPIO 21 with 220Ω current-limiting resistor
- Both LEDs connected to ground

**Run:**
```bash
bash scripts/build.sh
# Select "Status LED Test"
```

**Expected Visual Behavior:**
1. Both LEDs light steadily (2 seconds)
2. LED1 blinks 2 Hz on/off pattern
3. LED2 blinks 5 Hz rapid pattern
4. LEDs alternate on/off (simulating GPS/Serial states)
5. Both pulse together at 1 Hz

**Troubleshooting:**
- LEDs don't light?
  - Verify GPIO connections
  - Check LED polarity (longer leg = anode = +, shorter = cathode = -)
  - Verify resistor connection
  - Test with multimeter to check voltage on GPIO
- LED very dim?
  - Resistor value too high (should be 220Ω)
  - Reduce resistor value or check LED specifications

---

### Gimbal Servo Test

**What it tests:**
- PWM initialization on GPIO 16 (pan) and GPIO 17 (tilt)
- 50 Hz PWM frequency (20ms period)
- 1-2 ms pulse width modulation (90° range)
- Pan servo movement ±90°
- Tilt servo movement ±90°
- Circular sweep pattern
- Smooth motion without jerking

**Prerequisites:**
- Pan servo (MG90S) on GPIO 16 with 5V external power
- Tilt servo (MG90S) on GPIO 17 with 5V external power
- Common ground between Pico and servo power supply

**Run:**
```bash
bash scripts/build.sh
# Select "Gimbal Servo Test"
```

**Expected Motion:**
1. Move to 8 predefined positions (center, left, right, up, down, diagonals)
2. Smooth pan sweep from -90° to +90°
3. Smooth tilt sweep from -90° to +90°
4. Circular motion pattern (simulating target tracking)
5. Return to center

**Troubleshooting:**
- Servos don't respond?
  - Check PWM GPIO connections (16=pan, 17=tilt)
  - Verify 5V power supply with at least 2A capacity
  - Test servo with dedicated servo tester or Arduino
  - Check if servos are stalled (won't move if fully locked)
- Servos jitter or stutter?
  - Increase external power supply capacity
  - Check ground connection quality
  - Reduce other USB-powered devices for interference
- Servo range limited?
  - MG90S typical range: ±90° from center
  - Check servo mechanical stops
  - Verify 50 Hz frequency in CMakeLists.txt

---
### USB Serial Protocol Test

**What it tests:**
- USB CDC Serial initialization
- Binary protocol packet reception
- Checksum validation
- Target data parsing (latitude, longitude, altitude)
- ACK/Error response sending

**Prerequisites:**
- Raspberry Pi Pico connected to host via USB
- Python3 with pyserial: `pip3 install pyserial`

**Run on Pico:**
```bash
bash scripts/build.sh
# Select "Serial USB Test" (option 5)
```

**Run on Host Computer:**
```bash
python3 scripts/test_serial_usb.py
```

**Expected Output on Host:**
```
Connected to /dev/ttyACM0 at 115200 baud

==================================================
TEST 1: Valid target data
==================================================

Sending target data:
  Latitude:  37.774900°
  Longitude: -122.419400°
  Altitude:  52.00m
  Packet: 017f191742bcd6f4c2000050427c
✓ ACK received - target data accepted!
```

**Expected Behavior on Pico:**
- Blue LED blinks when valid target data is received
- Green LED blinks if checksum validation fails
- USB output shows received coordinates

**Protocol Details:**

Target Data Packet (14 bytes):
```
Byte 0:    0x01 (TARGET_DATA message type)
Bytes 1-4: Latitude (float32, little-endian)
Bytes 5-8: Longitude (float32, little-endian)
Bytes 9-12: Altitude (float32, little-endian)
Byte 13:   XOR Checksum (of bytes 0-12)
```

Response Messages:
- `0x02` - ACK (success)
- `0x03 0xXX` - ERROR (error code in second byte)

**Troubleshooting:**
- No data received on Pico?
  - Check USB connection: `ls -la /dev/ttyACM*`
  - Verify firmware flashed: Pico should print "=== USB Target Data Receiver ===" on startup
  - Check serial port permissions: `sudo usermod -a -G dialout $USER`
  - Verify python script can connect: `python3 -c "import serial; print(serial.tools.list_ports.comports())"`
- Checksum errors?
  - Verify packet construction in your code
  - Ensure float32 is little-endian
  - Check XOR calculation logic
- Blue LED not blinking?
  - Test with `scripts/test_serial_usb.py` first
  - Verify LED wiring (GPIO 20)

---

### System Integration Test

**What it tests:**
- All components working together
- GPS position reception (simulated or real)
- ROS2 target position reception
- Geodetic calculations (azimuth, elevation, distance)
- Gimbal angle calculation and movement
- Status LED indication
- 10 Hz control loop stability
- 30-second continuous operation

**Prerequisites:**
- All hardware connected (GPS, servos, LEDs, USB)
- micro-ROS agent optional (can run with simulated GPS)
- GPS module recommended for full test

**Run:**
```bash
bash scripts/build.sh
# Select "System Integration Test"
```

**Expected Output:**
```
========================================
  System Integration Test
========================================

Initializing components...
  1. PWM Controller
  ✓ PWM ready

  2. Gimbal (Pan/Tilt Servos)
  ✓ Gimbal ready (centered)

  ... (component initialization)

Test 1: Component Hardware Check
LED Test: Blink pattern...
✓ LEDs responsive

Gimbal Test: Center position...
✓ Gimbal responsive

Test 2: GPS Simulation
Simulating GPS position updates...

Test 3: ROS2 Message Processing
Waiting for micro-ROS agent connection...
Note: This test simulates ROS2 messages

... (5 target tracking cycles)

Test 5: Control Loop Simulation
Running 10 Hz control loop for 30 seconds...
```

**Test Scenarios:**
1. **GPS Simulation** - Simulates 5 GPS fix positions
2. **Position Calculation** - Tests geodetic calculations with 6 targets
3. **Gimbal Movement** - Verifies gimbal reaches calculated angles
4. **Control Loop** - 30-second 10 Hz loop with target tracking
5. **LED Status** - Verifies LED indicators update correctly

---

## Building All Tests

```bash
# Build all targets at once
bash scripts/build.sh
# Select "Build All Targets"
```

This creates:
- `bin/antenna_tracker.uf2` - Main application
- `bin/test_gps.uf2` - GPS test
- `bin/test_leds.uf2` - LED test
- `bin/test_gimbal.uf2` - Gimbal test
- `bin/test_ros.uf2` - ROS test
- `bin/system_test.uf2` - System integration test

---

## Flashing Tests

### Interactive Flash
After build completes, script asks:
```
Flash firmware to Raspberry Pi Pico?
```

Steps:
1. Hold BOOTSEL button on Pico
2. Connect USB cable (or reset while holding BOOTSEL)
3. When RPI-RP2 drive appears, answer `yes`
4. Script auto-detects mount point and copies firmware

### Manual Flash
```bash
# 1. Put Pico in BOOTSEL mode
# 2. Copy .uf2 file to RPI-RP2 drive
cp build/bin/test_gps.uf2 /media/$USER/RPI-RP2/
# Pico reboots automatically
```

---

## Serial Monitoring

### View Test Output

**Linux:**
```bash
screen /dev/ttyACM0 115200
# Exit: Ctrl+A, then K
```

**macOS:**
```bash
screen /dev/tty.usbmodem* 115200
```

**Alternative (all platforms):**
```bash
picocom /dev/ttyACM0 -b 115200
# Exit: Ctrl+A, Ctrl+X
```

---

## Test Execution Order (Recommended)

1. **test_leds** (5 min) - Easiest, no external components
2. **test_gimbal** (10 min) - Verify servo movement
3. **test_gps** (60 sec for startup, requires satellites)
4. **test_ros** (requires agent on host computer)
5. **test_main** (antenna_tracker) - Full application
6. **system_test** (2-3 min) - Complete integration

---

## Troubleshooting Common Issues

### Build Failures

**"pico-sdk not found"**
```bash
git submodule update --init --recursive
```

**"micro-ROS SDK not found"**
```bash
git submodule update --init --recursive
```

**CMake configuration error**
```bash
rm -rf build/
bash scripts/build.sh
```

### Runtime Issues

**Serial output not appearing**
- Check USB cable is connected
- Verify /dev/ttyACM0 exists: `ls /dev/ttyACM*`
- Try different baud rate
- Ensure no other program is using serial port

**GPS no fix**
- Wait 30-60 seconds for cold start
- Verify antenna has clear sky view
- Check GPS module power
- Verify UART connections

**Servo not responding**
- Check 5V power supply
- Verify GPIO connections
- Test servo with dedicated tester
- Reduce USB devices to eliminate interference

**ROS connection failed**
- Verify agent running: `ros2 node list`
- Check USB connection
- Try: `ros2 daemon stop && ros2 daemon start`
- Check firewall settings

---

## Test Performance Metrics

Expected results:

| Component | Metric | Expected | Notes |
|-----------|--------|----------|-------|
| GPS | Time to fix | <60s | Cold start; warm ~5-10s |
| GPS | Accuracy | ±5-10m | Depends on satellite count |
| LEDs | Response | <1ms | Digital IO, very fast |
| Gimbal | Movement | <0.5s | Smooth, no jitter |
| Gimbal | Range | ±90° | From center position |
| ROS | Connection | <5s | After agent starts |
| ROS | Message latency | <50ms | Typical DDS latency |
| Control loop | Frequency | 10 Hz | 100ms update interval |

---

## Next Steps After Testing

1. **Integrate into main application**
   ```bash
   bash scripts/build.sh
   # Select "Main Antenna Tracker"
   ```

2. **Start micro-ROS agent** (in separate terminal)
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   ```

3. **Publish target positions** (in another terminal)
   ```bash
   ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix ...
   ```

4. **Monitor gimbal tracking**
   ```bash
   screen /dev/ttyACM0 115200
   ```

---

## Support & Debugging

For detailed information:
- See [docs/ros2-integration.md](../docs/ros2-integration.md) for ROS2 setup
- See [docs/hardware.md](../docs/hardware.md) for wiring diagrams
- See [docs/setup.md](../docs/setup.md) for development environment
- Check [MIGRATION_NOTES.md](../MIGRATION_NOTES.md) for architecture details

For issues:
1. Check build output for errors
2. Review serial monitor output
3. Verify hardware connections with multimeter
4. Test individual components in isolation
5. Consult hardware documentation
