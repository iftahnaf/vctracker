#!/usr/bin/env python3
"""
USB Serial Target Data Test Script
Sends target position (lat, lon, alt) to Pico and receives ACK/error responses
"""

import serial
import struct
import time
import sys

# USB Protocol definitions (must match USBProtocol.h)
USB_MSG_TARGET_DATA = 0x01
USB_MSG_ACK = 0x02
USB_MSG_ERROR = 0x03

def calculate_checksum(data):
    """Calculate XOR checksum"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def create_target_packet(lat, lon, alt):
    """Create a target data packet"""
    msg_type = USB_MSG_TARGET_DATA
    
    # Pack: msg_type(1) + lat(4) + lon(4) + alt(4) + checksum(1) = 14 bytes
    packet = struct.pack('<B', msg_type)
    packet += struct.pack('<f', lat)
    packet += struct.pack('<f', lon)
    packet += struct.pack('<f', alt)
    
    # Calculate checksum on all bytes except the checksum byte itself
    checksum = calculate_checksum(packet)
    packet += struct.pack('<B', checksum)
    
    return packet

def send_target_data(ser, lat, lon, alt):
    """Send target data and wait for response"""
    packet = create_target_packet(lat, lon, alt)
    
    print(f"\nSending target data:")
    print(f"  Latitude:  {lat:.6f}°")
    print(f"  Longitude: {lon:.6f}°")
    print(f"  Altitude:  {alt:.2f}m")
    print(f"  Packet: {packet.hex()}")
    
    # Send packet
    ser.write(packet)
    time.sleep(0.1)
    
    # Wait for response
    if ser.in_waiting > 0:
        response = ser.read(1)
        response_byte = response[0]
        
        if response_byte == USB_MSG_ACK:
            print("✓ ACK received - target data accepted!")
            return True
        elif response_byte == USB_MSG_ERROR:
            error_code = 0
            if ser.in_waiting > 0:
                error_code = ser.read(1)[0]
            print(f"✗ ERROR received (code: {error_code})")
            return False
    else:
        print("⚠ No response received")
        return False

def main():
    # Find Pico on /dev/ttyACM0
    port = "/dev/ttyACM0"
    baudrate = 115200
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud")
        time.sleep(2)  # Wait for Pico to initialize
        
        # Clear any existing data
        ser.reset_input_buffer()
        
        # Test 1: Valid target data
        print("\n" + "="*50)
        print("TEST 1: Valid target data")
        print("="*50)
        send_target_data(ser, 37.7749, -122.4194, 52.0)  # San Francisco
        
        time.sleep(1)
        
        # Test 2: Another valid target
        print("\n" + "="*50)
        print("TEST 2: Different target location")
        print("="*50)
        send_target_data(ser, 40.7128, -74.0060, 10.5)  # New York
        
        time.sleep(1)
        
        # Test 3: Negative coordinates
        print("\n" + "="*50)
        print("TEST 3: Negative coordinates")
        print("="*50)
        send_target_data(ser, -33.8688, 151.2093, 0.0)  # Sydney
        
        time.sleep(1)
        
        # Test 4: Large altitude
        print("\n" + "="*50)
        print("TEST 4: High altitude target")
        print("="*50)
        send_target_data(ser, 0.0, 0.0, 5000.0)  # Equator, 5km altitude
        
        print("\n" + "="*50)
        print("All tests completed!")
        print("="*50)
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Error: Could not open {port}")
        print(f"Details: {e}")
        print("\nMake sure:")
        print("  1. Pico is connected via USB")
        print("  2. test_serial firmware is flashed")
        print(f"  3. {port} exists (check with: ls -la /dev/ttyACM*)")
        sys.exit(1)

if __name__ == "__main__":
    main()
