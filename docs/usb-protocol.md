# USB Serial Protocol - vctracker

Simple binary protocol for sending target position data to the Pico via USB.

## Overview

The vctracker antenna tracker receives target position updates (latitude, longitude, altitude) over USB using a lightweight binary protocol. This eliminates the need for ROS2/micro-ROS dependencies while maintaining reliable communication.

## Protocol Specification

### Target Data Packet

**Size:** 14 bytes

| Offset | Size | Field | Type | Description |
|--------|------|-------|------|-------------|
| 0 | 1 | msg_type | uint8_t | Message type: 0x01 for TARGET_DATA |
| 1-4 | 4 | latitude | float32 | Target latitude in degrees (signed) |
| 5-8 | 4 | longitude | float32 | Target longitude in degrees (signed) |
| 9-12 | 4 | altitude | float32 | Target altitude in meters |
| 13 | 1 | checksum | uint8_t | XOR checksum of bytes 0-12 |

**Byte Order:** Little-endian (Intel format)

**Examples:**
- San Francisco: lat=37.7749, lon=-122.4194, alt=52.0
- New York: lat=40.7128, lon=-74.0060, alt=10.5
- Sydney: lat=-33.8688, lon=151.2093, alt=0.0
- Equator: lat=0.0, lon=0.0, alt=5000.0

### Response Messages

**ACK Message (1 byte):**
```
Byte 0: 0x02
```
Sent when target data is successfully received and validated.

**Error Message (2 bytes):**
```
Byte 0: 0x03 (ERROR)
Byte 1: Error code
```

Error codes:
- `0x01` - Incomplete data (fewer than 14 bytes received before timeout)
- `0x02` - Checksum validation failed
- `0x03` - Invalid message type

## Checksum Calculation

XOR checksum covers bytes 0-12 (everything except the checksum byte):

```c
uint8_t calculate_checksum(const uint8_t* data, int length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
```

**C example:**
```c
uint8_t packet[14];
packet[0] = 0x01;  // msg_type
memcpy(&packet[1], &latitude, 4);
memcpy(&packet[5], &longitude, 4);
memcpy(&packet[9], &altitude, 4);
// Calculate checksum
uint8_t checksum = 0;
for (int i = 0; i < 13; i++) {
    checksum ^= packet[i];
}
packet[13] = checksum;
```

## USB Serial Configuration

- **Port:** `/dev/ttyACM0` (Linux/Mac) or `COM3` (Windows)
- **Baud Rate:** 115200 bps
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
- **Flow Control:** None
- **Timeout:** 100ms per byte (configurable in firmware)

## Python Implementation

See [scripts/test_serial_usb.py](../scripts/test_serial_usb.py) for a complete working example.

**Basic usage:**
```python
import serial
import struct
import time

def create_target_packet(lat, lon, alt):
    """Create a 14-byte target data packet"""
    packet = struct.pack('<B', 0x01)  # msg_type
    packet += struct.pack('<f', lat)  # latitude
    packet += struct.pack('<f', lon)  # longitude
    packet += struct.pack('<f', alt)  # altitude
    
    # Calculate XOR checksum
    checksum = 0
    for byte in packet:
        checksum ^= byte
    packet += struct.pack('<B', checksum)
    
    return packet

# Send to Pico
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
packet = create_target_packet(37.7749, -122.4194, 52.0)
ser.write(packet)

# Wait for response
response = ser.read(1)
if response[0] == 0x02:
    print("✓ ACK received")
elif response[0] == 0x03:
    print("✗ Error response")
```

## Testing

### Manual Test with Python

```bash
# Build and flash test_serial firmware
bash scripts/build.sh
# Select option 5 for "Serial USB Test"

# Run test script in new terminal
python3 scripts/test_serial_usb.py
```

### Expected Output

**Pico Serial Output:**
```
=== USB Target Data Receiver ===
Waiting for target data...
✓ TARGET DATA RECEIVED:
  Latitude:  37.774900°
  Longitude: -122.419400°
  Altitude:  52.00m
```

**Host Python Output:**
```
Sending target data:
  Latitude:  37.774900°
  Longitude: -122.419400°
  Altitude:  52.00m
  Packet: 017f191742bcd6f4c2000050427c
✓ ACK received - target data accepted!
```

## LED Indicators

When running test_serial firmware:
- **Blue LED (GPIO 20):** Blinks when valid target data is received
- **Green LED (GPIO 21):** Blinks when validation fails (bad checksum, incomplete data)

## Integration with Main Application

To integrate the USB protocol into your main tracker application:

1. Include the header:
   ```c
   #include "USBProtocol.h"
   ```

2. Initialize USB stdio:
   ```c
   stdio_init_all();
   sleep_ms(2000);  // Wait for USB to be ready
   ```

3. Receive target data:
   ```c
   USBTargetData target;
   int result = usb_receive_target_data(&target);
   
   if (result > 0 && usb_validate_target_data(&target)) {
       // Use target.latitude, target.longitude, target.altitude
       float lat = target.latitude;
       float lon = target.longitude;
       float alt = target.altitude;
   }
   ```

## Advantages Over ROS2

- **No Dependencies:** No ROS2, micro-ROS, or agent software required
- **Lightweight:** 14 bytes per message, simple binary protocol
- **Fast:** Direct USB communication, no middleware overhead
- **Reliable:** Checksum validation ensures data integrity
- **Cross-Platform:** Works with any OS that supports USB CDC
- **Simple:** Easy to understand and implement

## Future Enhancements

Potential protocol extensions (reserved for future use):
- Additional message types (0x02-0xFF)
- Command messages (camera control, gimbal calibration)
- Telemetry responses (current position, servo angles)
- Confidence/accuracy data in target packets
- Multiple target support

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No data received | USB not connected | Check USB cable and port |
| Checksum errors | Wrong byte order | Ensure little-endian (use `<f` in struct.pack) |
| Timeout errors | Incomplete data | Verify all 14 bytes are sent |
| Blue LED not blinking | Data not reaching Pico | Check USB driver, permissions, port settings |
| Green LED blinking | Checksum mismatch | Recalculate XOR on correct bytes |

## See Also

- [testing.md](testing.md) - Full testing guide
- [USBProtocol.h](../include/USBProtocol.h) - C header with struct definitions
- [test_serial.cpp](../examples/test_serial.cpp) - Firmware implementation
- [test_serial_usb.py](../scripts/test_serial_usb.py) - Python test script
