# vctracker Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                  Antenna Tracker System                      │
│                  (Raspberry Pi Pico RP2040)                  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            AntennaTracker (Main Controller)          │  │
│  │                                                       │  │
│  │  • Coordinates all subsystems                        │  │
│  │  • Main update loop (10 Hz)                          │  │
│  │  • Auto-tracking logic                               │  │
│  │  • Safety limits enforcement                         │  │
│  └───┬────────┬────────┬──────────┬──────────┬─────────┘  │
│      │        │        │          │          │             │
│  ┌───▼───┐ ┌─▼───┐ ┌──▼───┐  ┌───▼────┐ ┌──▼─────┐       │
│  │  GPS  │ │ ROS │ │ Geo  │  │ Gimbal │ │  LEDs  │       │
│  │Module │ │Parse│ │ Calc │  │Control │ │ Status │       │
│  └───┬───┘ └─┬───┘ └──┬───┘  └───┬────┘ └───┬────┘       │
│      │       │        │          │          │             │
└──────┼───────┼────────┼──────────┼──────────┼─────────────┘
       │       │        │          │          │
       │       │        │          │          │
   ┌───▼───┐ ┌─▼───┐  │     ┌────▼────┐ ┌───▼────┐
   │  GPS  │ │ UART│  │     │ Servos  │ │  LEDs  │
   │ NEO-6M│ │USB  │  │     │ MG90S   │ │ GPIO   │
   │(UART0)│ │(UART│  │     │ Pan/Tilt│ │ 20/21  │
   └───────┘ │  1) │  │     └─────────┘ └────────┘
             └─────┘  │
                      │
             (Geodetic Math)
```

## Component Architecture

### Core Classes

```
AntennaTracker
    ├─> GPSModule          (GPS data acquisition)
    ├─> ROSParser          (ROS2 message reception)
    ├─> Gimbal             (Servo control via vcgimbal)
    │   └─> PWMControllerPico
    ├─> StatusLED (x2)     (Visual feedback)
    └─> GeoCalculations    (Static utility class)
```

## Data Flow

```
1. GPS Data Flow:
   GPS Module → UART0 → GPSModule.update()
                      ↓
                  Parse NMEA
                      ↓
                  GPSData struct
                      ↓
              AntennaTracker.getTrackerPosition()

2. Target Data Flow:
   ROS2 → UART Adapter → UART1 → ROSParser.update()
                                      ↓
                                  Parse CSV
                                      ↓
                              NavSatFixMsg struct
                                      ↓
                      AntennaTracker.getTargetPosition()

3. Tracking Calculation Flow:
   Tracker GPS + Target GPS
            ↓
   GeoCalculations::calculateTrackerAngles()
            ↓
   (Distance, Bearing, Elevation)
            ↓
   PanTiltAngles struct
            ↓
   Apply safety limits
            ↓
   Gimbal.setTipAngle()
            ↓
   PWM to servos

4. Status Indication Flow:
   GPS fix quality → LED state (OFF/SLOW/FAST/ON)
   ROS message age → LED state (OFF/SLOW/FAST/ON)
            ↓
   StatusLED.update() → GPIO PWM
```

## Memory Layout

### Stack Usage
- **Main Stack**: 8 KB (default Pico stack)
- **String Buffers**: 
  - NMEA: 200 bytes
  - ROS CSV: 256 bytes
- **Total RAM**: ~20 KB (RP2040 has 264 KB)

### Flash Usage
- **Firmware**: ~150 KB
- **vcgimbal Library**: ~30 KB
- **pico-sdk**: ~50 KB
- **Total Flash**: ~230 KB (RP2040 has 2 MB)

## Timing

### Update Rates
- **Main Loop**: 10 Hz (100ms period)
- **GPS Update**: 1 Hz (module dependent)
- **ROS Messages**: Variable (typically 1-10 Hz)
- **Servo Update**: As needed (50 Hz PWM continuous)
- **LED Update**: Every main loop (for animations)

### Latency Budget
```
GPS → Parse → Calculate → Servo Move
 1ms    5ms      2ms         50ms
                          (servo settling)
Total: ~60ms end-to-end latency
```

## Pin Assignment

```
GPIO  | Function        | Direction | Protocol
------|----------------|-----------|----------
  0   | GPS TX         | Output    | UART0
  1   | GPS RX         | Input     | UART0
  4   | ROS TX         | Output    | UART1
  5   | ROS RX         | Input     | UART1
  16  | Pan Servo      | Output    | PWM
  17  | Tilt Servo     | Output    | PWM
  20  | GPS Status LED | Output    | GPIO
  21  | ROS Status LED | Output    | GPIO
```

## Error Handling

### GPS Module
- **No Fix**: LED blinks fast, don't update tracking
- **Invalid Data**: Checksum verification, discard bad sentences
- **Timeout**: Continue with last known position (with warning)

### ROS Parser
- **No Messages**: LED blinks fast, don't update tracking
- **Invalid Format**: Log error, continue
- **Timeout (5s)**: Mark data invalid, LED indicates stale

### Gimbal Control
- **Out of Range**: Clamp angles to [-90°, 90°]
- **Min Elevation**: Enforce minimum tilt angle (default -5°)
- **Servo Failure**: Log error, attempt recovery

### Safety
- **Rapid Changes**: Rate limiting could be added
- **Invalid Coordinates**: Range checking (lat ±90, lon ±180)
- **Altitude Sanity**: Reasonable altitude range checking

## Build System

```
CMakeLists.txt (root)
    ├─> vcgimbal/CMakeLists.txt
    │   └─> Builds gimbal_lib
    │
    └─> antenna_tracker_lib (this project)
        ├─> Links: gimbal_lib
        ├─> Links: pico_stdlib, hardware_uart, hardware_pwm
        └─> Creates: antenna_tracker executable
```

## Testing Strategy

### Unit Testing (Future)
- GeoCalculations (pure math functions)
- NMEA parsing
- CSV parsing
- Coordinate validation

### Integration Testing
1. GPS loopback test
2. ROS UART test with echo
3. Servo sweep test
4. LED pattern test
5. End-to-end with simulated data

### Field Testing
1. Static position test (known coordinates)
2. Moving target test (vehicle, drone)
3. Range test (distance limits)
4. Accuracy measurement

## Performance Optimization

### CPU Usage
- Main loop: ~5% CPU (100ms period, ~5ms work)
- UART parsing: ~2% CPU
- PWM generation: Hardware (0% CPU)
- **Total**: <10% CPU usage

### Power Consumption
- Pico: ~30 mA
- GPS: ~40 mA
- Servos: 200-600 mA (movement dependent)
- LEDs: ~30 mA
- **Total**: ~300-700 mA @ 5V

### Optimization Opportunities
1. Reduce string allocations
2. Use fixed-size buffers
3. DMA for UART (reduce CPU)
4. Servo move only when needed (reduce power)
5. Adaptive update rates

## Future Enhancements

### Planned Features
- [ ] Kalman filtering for smoother tracking
- [ ] Predictive tracking (target velocity estimation)
- [ ] SD card logging
- [ ] Web interface (via WiFi module)
- [ ] Multiple target switching
- [ ] Magnetic compass integration
- [ ] IMU for tracker orientation
- [ ] Battery monitoring

### API Extensions
- [ ] Binary protocol (more efficient than CSV)
- [ ] MAVLink support
- [ ] Custom calibration routines
- [ ] Remote configuration via UART

## Dependencies

```
vctracker
  ├─> vcgimbal (submodule)
  │   └─> pico-sdk (submodule)
  │       ├─> hardware_pwm
  │       ├─> hardware_uart
  │       ├─> hardware_clocks
  │       └─> pico_stdlib
  │
  └─> Standard C++17 libraries
      ├─> <cmath>
      ├─> <string>
      ├─> <vector>
      └─> <memory>
```

## License

MIT License - See LICENSE file for details
