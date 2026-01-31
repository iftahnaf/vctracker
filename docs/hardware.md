# Hardware Setup Guide

## Component List

### Required Components
- **Raspberry Pi Pico** (RP2040) - $4
- **2x MG90S Micro Servo Motors** - $3 each
- **GPS Module** (NEO-6M/7M/M8N with UART) - $10-15
- **2x LEDs** (any color, 3mm or 5mm) - $0.10 each
- **2x 220Ω Resistors** (for LEDs) - $0.05 each
- **Jumper Wires** (female-to-female, male-to-female)
- **Breadboard** (optional, for prototyping)
- **5V Power Supply** (2A minimum for servos)
- **USB Cable** (Micro-USB for Pico) - for power, programming, and ROS2 communication

### Optional Components
- **Pan-Tilt Bracket** (for servo mounting)
- **Camera Mount**
- **Enclosure**
- **Level Shifter** (if GPS is 5V logic only)

**Note**: No additional UART adapter needed! micro-ROS communicates over the same USB cable used for programming.

## Wiring Diagram

### Complete Connections

```
Raspberry Pi Pico                    GPS Module (NEO-6M)
┌─────────────────┐                 ┌──────────────┐
│                 │                 │              │
│     GPIO 0 (TX) ├─────────────────┤ RX           │
│     GPIO 1 (RX) ├─────────────────┤ TX           │
│                 │                 │              │
│          VBUS   ├─────────────────┤ VCC (3.3-5V) │
│           GND   ├─────┬───────────┤ GND          │
└─────────────────┘     │           └──────────────┘
                        │
                        │           USB to ROS2 Host
                        │           ┌──────────────┐
    ┌───────────────────┤           │              │
    │      USB Cable    ├───────────┤ Computer     │
    │  (Micro-USB)      │           │ (ROS2 + Agent│
    │                   │           │  running)    │
    └───────────────────┘           └──────────────┘
                                │
                                │   Pan Servo (MG90S)
                                │   ┌──────────────┐
                                │   │              │
        GPIO 16 (PWM) ──────────┼───┤ Signal (Org) │
                                │   │              │
        5V (External) ──────────┼───┤ VCC (Red)    │
        GND ────────────────────┼───┤ GND (Brown)  │
                                │   └──────────────┘
                                │
                                │   Tilt Servo (MG90S)
                                │   ┌──────────────┐
                                │   │              │
        GPIO 17 (PWM) ──────────┼───┤ Signal (Org) │
                                │   │              │
        5V (External) ──────────┼───┤ VCC (Red)    │
        GND ────────────────────┼───┤ GND (Brown)  │
                                │   └──────────────┘
                                │
                                │   GPS Status LED
                                │   ┌──────────────┐
        GPIO 20 ────────────────┼───┤  Anode (+)   │
                                │   │   [220Ω]     │
        GND ────────────────────┼───┤  Cathode (-) │
                                │   └──────────────┘
                                │
                                │   ROS Status LED
                                │   ┌──────────────┐
        GPIO 21 ────────────────┼───┤  Anode (+)   │
                                │   │   [220Ω]     │
        GND ────────────────────┴───┤  Cathode (-) │
                                    └──────────────┘
```

**Key Point**: The USB cable serves triple duty:
1. Powers the Pico (can also use VSYS pin for external 5V)
2. Provides serial debug output
3. Runs micro-ROS communication with host computer

## Detailed Pin Connections

### Raspberry Pi Pico Pinout

```
          ┌─────────────────┐
          │   Raspberry Pi  │
          │      Pico       │
          │                 │
GPIO 0  ──┤ 1         40 ├── VBUS (5V)
GPIO 1  ──┤ 2         39 ├── VSYS
GND     ──┤ 3         38 ├── GND
GPIO 2    │ 4         37 │  3V3_EN
GPIO 3    │ 5         36 │  3V3(OUT)
GPIO 4  ──┤ 6         35 │  ADC_VREF
GPIO 5  ──┤ 7         34 │  GPIO 28
GND       │ 8         33 ├── GND
GPIO 6    │ 9         32 │  GPIO 27
GPIO 7    │ 10        31 │  GPIO 26
GPIO 8    │ 11        30 │  RUN
GPIO 9    │ 12        29 │  GPIO 22
GND       │ 13        28 ├── GND
GPIO 10   │ 14        27 │  GPIO 21 ── (ROS LED)
GPIO 11   │ 15        26 │  GPIO 20 ── (GPS LED)
GPIO 12   │ 16        25 │  GPIO 19
GPIO 13   │ 17        24 │  GPIO 18
GND       │ 18        23 ├── GND
GPIO 14   │ 19        22 │  GPIO 17 ── (Tilt Servo)
GPIO 15   │ 20        21 │  GPIO 16 ── (Pan Servo)
          └─────────────────┘
```

### Connection Table

| Component | Wire Color | Pico Pin | Pico GPIO | Notes |
|-----------|------------|----------|-----------|-------|
| **Pan Servo** |
| Signal | Orange/Yellow | Pin 21 | GPIO 16 | PWM output |
| Power | Red | VBUS (Pin 40) | 5V | External recommended |
| Ground | Brown/Black | GND (any) | GND | Common ground |
| **Tilt Servo** |
| Signal | Orange/Yellow | Pin 22 | GPIO 17 | PWM output |
| Power | Red | VBUS (Pin 40) | 5V | External recommended |
| Ground | Brown/Black | GND (any) | GND | Common ground |
| **GPS Module** |
| TX (GPS→Pico) | - | Pin 2 | GPIO 1 (RX) | UART0 |
| RX (Pico→GPS) | - | Pin 1 | GPIO 0 (TX) | UART0 (optional) |
| VCC | Red | Pin 36 or 40 | 3V3 or 5V | Check GPS voltage |
| GND | Black | GND (any) | GND | Common ground |
| **micro-ROS** |
| Data | USB | Micro-USB | USB Serial | Native ROS2 via agent |
| **GPS LED** |
| Anode (+) | - | Pin 26 | GPIO 20 | Through 220Ω resistor |
| Cathode (-) | - | GND (any) | GND | Common ground |
| **ROS LED** |
| Anode (+) | - | Pin 27 | GPIO 21 | Through 220Ω resistor |
| Cathode (-) | - | GND (any) | GND | Common ground |

## Power Considerations

### Servo Power
- **DO NOT** power servos directly from Pico if both servos operate simultaneously
- **Pico VBUS** can provide up to 500mA (USB limit)
- **Each MG90S** draws 100-300mA (idle to stall)
- **Recommendation**: Use external 5V 2A power supply
  - Connect 5V supply to servo red wires
  - Connect ground to common ground with Pico
  - **IMPORTANT**: Share ground between power supply and Pico

### GPS Module Power
- Most GPS modules work with **3.3V** (Pico Pin 36)
- Some can accept **5V** (Pico Pin 40 VBUS)
- Check your GPS module specifications
- Typical consumption: 20-50mA

### LED Power
- LEDs powered through GPIO pins
- Current limited by 220Ω resistors
- Each LED draws ~10-15mA

## Assembly Instructions

### Step 1: Prepare Components
1. Lay out all components on workspace
2. Identify servo wire colors (Signal, Power, Ground)
3. Prepare jumper wires for connections

### Step 2: Mount Servos (if using bracket)
1. Attach pan servo to base
2. Attach tilt servo to pan servo arm
3. Center servos before final assembly (90° position)

### Step 3: Wire Servos
1. Connect pan servo signal to GPIO 16
2. Connect tilt servo signal to GPIO 17
3. Connect both servo grounds to common ground
4. Connect servo power (see power section)

### Step 4: Wire GPS Module
1. Connect GPS TX to Pico GPIO 1 (RX)
2. Connect GPS RX to Pico GPIO 0 (TX) - optional
3. Connect GPS VCC to appropriate voltage (3.3V or 5V)
4. Connect GPS GND to common ground

### Step 5: Connect to ROS2 Host
1. Connect Pico to host computer via USB Micro cable
2. micro-ROS will communicate over this USB connection
3. No additional UART wiring needed for ROS2!

### Step 6: Wire Status LEDs
1. GPS LED:
   - Anode (longer leg) → 220Ω resistor → GPIO 20
   - Cathode (shorter leg) → GND
2. ROS LED:
   - Anode (longer leg) → 220Ω resistor → GPIO 21
   - Cathode (shorter leg) → GND

### Step 7: Verify Connections
- Check all power connections
- Verify common ground
- Ensure no shorts between power and ground
- Confirm UART RX/TX not swapped

## GPS Module Configuration

### NEO-6M/7M Setup
- Default baud rate: **9600** (matches code)
- Default output: NMEA sentences
- No configuration needed for basic operation

### U-Center Configuration (optional)
1. Download u-blox U-Center software
2. Connect GPS to PC via USB-UART adapter
3. Configure output messages:
   - Enable: GPGGA (position)
   - Disable unnecessary messages for reduced UART load
4. Set update rate: 1 Hz (default) or higher
5. Save configuration to GPS flash

## Testing

### LED Test (without GPS/ROS)
- Both LEDs should blink fast after power-on
- Indicates waiting for GPS fix and ROS data

### Servo Test
- Servos should center (90°) on initialization
- No jittering or unusual sounds

### GPS Test
- GPS LED should transition from fast blink → slow blink → solid
- May take 30-60 seconds for cold start
- Requires clear sky view

### ROS UART Test
- Send test data via UART: `echo "37.7749,-122.4194,10.5,1" > /dev/ttyUSB0`
- ROS LED should turn solid

## Troubleshooting Hardware

### Servos Not Responding
- Check 5V power supply connection and capacity
- Verify signal wires on correct GPIO pins
- Ensure common ground between Pico and servo power
- Test servos with multimeter (signal should show ~50Hz PWM)

### No GPS Fix
- Ensure GPS has clear view of sky (near window or outdoors)
- Check UART connections (TX→RX, RX→TX)
- Verify GPS power LED is on
- Wait at least 60 seconds for cold start
- Use GPS test apps to verify module works

### LEDs Not Working
- Check LED polarity (anode to GPIO, cathode to GND)
- Verify 220Ω resistors are in series
- Test LEDs with 3.3V and resistor separately
- Check GPIO pin numbers in code match hardware

### UART Issues
- Verify baud rates match (GPS=9600, ROS=115200)
- Check RX/TX aren't swapped
- Ensure common ground connection
- Test UART with loopback (TX→RX on same UART)

## Safety Notes

- **Never hot-plug servos** while powered
- Use **appropriate power supply** for servos (5V 2A minimum)
- **Protect Pico** from shorts with housing/enclosure
- **Secure GPS antenna** away from motors/servos (EMI)
- **Don't exceed** GPIO current limits (16mA per pin, 50mA total)

## Next Steps

- [Software Setup Guide](setup.md)
- [ROS2 Integration Guide](ros2-integration.md)
