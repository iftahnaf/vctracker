# Migration from UART CSV to micro-ROS

## Summary of Changes

This document outlines the complete migration from a custom UART CSV protocol to proper micro-ROS integration for the vctracker antenna tracker project.

## Architecture Change

### Before: Custom UART CSV Protocol
```
ROS2 Host → Custom Bridge → UART1 → CSV Parser → Pico
Protocol: latitude,longitude,altitude,status\n
Baudrate: 115200
Issues: Non-standard, custom bridge needed, no QoS
```

### After: micro-ROS
```
ROS2 Host → micro-ROS Agent → USB Serial → micro-ROS Client → Pico
Protocol: Native sensor_msgs/NavSatFix (DDS serialized)
Transport: USB Serial CDC (same cable as programming)
Benefits: Standard ROS2, native messages, QoS support
```

## Code Changes

### 1. **include/ROSParser.h** - Complete Refactor
**Old**: UART-based message parser with CSV parsing
```cpp
// OLD: UART parameters, CSV parsing functions
ROSParser(uart_id_t uart_id, uint tx_pin, uint rx_pin, uint32_t baud);
bool parse_csv_message(const std::string& line);
bool update();
```

**New**: micro-ROS subscriber with callback-based architecture
```cpp
// NEW: micro-ROS entities, proper message handling
ROSParser(const char* node_name, const char* topic_name);
void spin();
bool isConnected();
static void subscriptionCallback(const void* msg);
```

**Headers Added**:
- `#include <rcl/rcl.h>` - ROS Client Library
- `#include <rclc/rclc.h>` - ROS Client Library for C
- `#include <rclc/executor.h>` - Event executor
- `#include <sensor_msgs/msg/nav_sat_fix.h>` - Native ROS2 message

### 2. **src/ROSParser.cpp** - Completely Rewritten
**Old**: UART read, string buffer, CSV string splitting
```cpp
// OLD approach
while (uart_is_readable()) {
    char c = uart_getc();
    // Parse CSV...
}
```

**New**: micro-ROS executor with callback
```cpp
// NEW approach
rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));
// Callbacks invoked automatically when messages received
```

**Key Implementation Details**:
- **init()**: Sets up micro-ROS transport, waits for agent (10s timeout), creates node/subscription/executor
- **spin()**: Calls `rclc_executor_spin_some()` for non-blocking message processing
- **subscriptionCallback()**: Static wrapper for instance callbacks
- **handleMessage()**: Converts `sensor_msgs__msg__NavSatFix` to internal `NavSatFixMsg` struct
- **Agent Detection**: Uses `rmw_uros_ping_agent()` to verify connection

### 3. **CMakeLists.txt** - micro-ROS Integration
**Added**:
```cmake
# Import micro-ROS SDK
include(lib/micro_ros_raspberrypi_pico_sdk/pico_sdk_import.cmake)

# Require C11 for micro-ROS
set(CMAKE_C_STANDARD 11)

# Link micro-ROS library
target_link_libraries(antenna_tracker
    microros
)

# Link message library
target_link_libraries(antenna_tracker
    microros
)

message(STATUS "micro-ROS: Enabled")
```

### 4. **examples/main.cpp** - Updated Initialization
**Old Pin Configuration**: Required UART1 (GPIO 4-5)
```cpp
constexpr uint ROS_UART_ID = 1;
constexpr uint ROS_TX_PIN = 4;
constexpr uint ROS_RX_PIN = 5;
```

**New Configuration**: USB-only (no extra pins needed)
```cpp
// Removed UART1 pins entirely
// micro-ROS uses USB CDC automatically
```

**Updated Output**:
```cpp
std::cout << "Hardware Configuration:" << std::endl;
std::cout << "  GPS UART:     UART0 (TX=0, RX=1, 9600 baud)" << std::endl;
std::cout << "  micro-ROS:    USB Serial (automatic)" << std::endl;
```

### 5. **src/AntennaTracker.cpp** - Method Update
**Old**: Synchronous UART update
```cpp
ros_parser_->update();  // Non-blocking UART read
```

**New**: Asynchronous micro-ROS spin
```cpp
ros_parser_->spin();    // Executor processes pending messages
```

## Documentation Changes

### 1. **docs/ros2-integration.md** - Complete Rewrite
- **Removed**: CSV format specification, UART1 instructions, custom bridge setup
- **Added**: 
  - micro-ROS architecture diagram
  - Agent installation instructions
  - Building micro-ROS firmware
  - Publishing target positions via ROS2 topic
  - Multiple examples (CLI, Python)
  - Monitoring tools (ros2 topic, ros2 node)
  - Troubleshooting guide for micro-ROS specific issues
  - References to micro-ROS documentation

**Key Sections**:
- Why micro-ROS (QoS, native messages, standard tools)
- Prerequisites (ROS2 + micro-ROS agent)
- Agent startup (single command)
- Publishing examples (ros2 topic pub)
- Monitoring (ros2 topic list/echo/info)

### 2. **docs/hardware.md** - Wiring Diagram Update
**Old**:
- GPIO 4-5: UART1 ROS connection
- Required external UART adapter
- UART adapter to host computer

**New**:
- USB cable handles all ROS2 communication
- Wiring simplified (no UART1)
- Connection table updated to remove ROS UART row
- Added note: "micro-ROS: USB Serial (native ROS2)"

**Updated Connection Table**:
```
| micro-ROS | USB | USB Serial | Native ROS2 via agent |
```

### 3. **docs/setup.md** - Build System Update
**Added Prerequisites**:
- ROS2 Humble installation
- micro-ROS agent package
- Alternative: Build micro-ROS agent from source

**Updated Project Setup**:
- Explain three submodules: vcgimbal, pico-sdk, micro_ros_raspberrypi_pico_sdk
- Verify micro-ROS SDK exists
- Build instructions unchanged (build scripts updated)

**New Sections**:
- Starting micro-ROS Agent (required before Pico connects)
- Agent output examples
- Combined serial monitor + agent workflow

### 4. **README.md** - Overview Update
**Changed Features**:
```
OLD: "UART1: ROS2 messages (115200 baud, CSV format)"
NEW: "micro-ROS: Native ROS2 over USB serial"
```

**Updated Quick Start**:
1. Add ROS2 + micro-ROS agent installation
2. Add agent startup command (separate terminal)
3. Add example of publishing target position
4. Simplify hardware requirements (remove UART adapter)

**Updated Wiring Table**: Removed UART1 rows

## Build System Changes

### scripts/build.sh - Submodule Verification
**Added**:
```bash
# Check if micro-ROS SDK exists
if [ ! -d "$PROJECT_DIR/lib/micro_ros_raspberrypi_pico_sdk" ]; then
    echo "Error: micro-ROS SDK not found at lib/micro_ros_raspberrypi_pico_sdk"
    echo "Please run: git submodule update --init --recursive"
    exit 1
fi
```

## Runtime Flow Changes

### Old Flow (UART CSV)
1. Pico: UART1 interrupt handler reads characters into buffer
2. Pico: Parse CSV string when complete message received
3. Host: Custom application sends formatted CSV over UART
4. Host: Bridge application required between ROS2 and UART

### New Flow (micro-ROS)
1. Pico: micro-ROS agent connected via USB CDC
2. Pico: Subscription callback invoked when NavSatFix message received
3. Pico: Message in native ROS2 format (pre-deserialized)
4. Host: Start agent with `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
5. Host: Publish to /target/gps/fix using standard ROS2 tools
6. No bridge needed - standard ROS2 ecosystem!

## Benefits of Migration

| Aspect | UART CSV | micro-ROS |
|--------|----------|-----------|
| **Message Format** | Custom CSV | Native ROS2/DDS |
| **Serialization** | Manual string parsing | Automatic |
| **Type Safety** | String-based | Struct-based |
| **QoS Support** | No | Yes (with agent) |
| **ROS2 Tools** | Incompatible | Full compatibility |
| **Bridge Needed** | Yes (custom) | No (standard agent) |
| **Hardware Pins** | UART1 (GPIO 4-5) | USB only |
| **Latency** | Variable (parsing) | Consistent |
| **Debugging** | Custom format | ros2 topic tools |

## Testing the Migration

### Verify Agent Connection
```bash
# Terminal 1: Start agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# Terminal 2: In another terminal, check if node exists
ros2 node list
# Should show: /antenna_tracker

# Terminal 3: Check subscription
ros2 topic list
# Should show: /target/gps/fix
```

### Publish Test Message
```bash
ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix \
  --once \
  "header: {frame_id: 'gps'}
   latitude: 37.7749
   longitude: -122.4194
   altitude: 10.5
   position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
   position_covariance_type: 2"
```

### Monitor Incoming Messages
```bash
# Check if messages are being received
ros2 topic echo /target/gps/fix

# Check node info
ros2 node info /antenna_tracker
```

## Rollback Instructions

If needed to revert to UART CSV approach:
1. Git revert commits related to micro-ROS
2. Restore old UART-based ROSParser from git history
3. Re-add GPIO 4-5 UART1 configuration
4. Rebuild with original CMakeLists.txt

## References

- [micro-ROS Documentation](https://micro.ros.org)
- [micro-ROS Raspberry Pi Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)
- [ROS2 micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent)
- [sensor_msgs/NavSatFix Message](https://docs.ros2.org/humble/api/sensor_msgs/msg/NavSatFix.html)

## Files Modified

### Code Files (5)
1. ✅ include/ROSParser.h - Refactored for micro-ROS
2. ✅ src/ROSParser.cpp - Completely rewritten
3. ✅ CMakeLists.txt - Added micro-ROS integration
4. ✅ examples/main.cpp - Updated initialization
5. ✅ src/AntennaTracker.cpp - Changed update() to spin()

### Documentation Files (4)
1. ✅ docs/ros2-integration.md - Complete rewrite for micro-ROS
2. ✅ docs/hardware.md - Updated wiring and connection table
3. ✅ docs/setup.md - Added ROS2 and agent setup
4. ✅ README.md - Updated features and quick start

### Build System (1)
1. ✅ scripts/build.sh - Added micro-ROS submodule check

**Total: 10 files updated**

## Next Steps

1. ✅ Add micro_ros_raspberrypi_pico_sdk submodule to repository
2. ✅ Test build with micro-ROS libraries
3. ✅ Flash firmware and verify agent connection
4. ✅ Test message reception and gimbal control
5. Consider: Additional ROS2 nodes for GPS forwarder, trajectory visualization

## Migration Completed

This migration successfully transitions vctracker from a custom UART CSV protocol to a proper, standards-based micro-ROS implementation. The project now benefits from full ROS2 ecosystem compatibility, native message types, and professional-grade robustness.
