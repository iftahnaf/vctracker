#include "../include/AntennaTracker.h"
#include "../vcgimbal/include/Gimbal.h"
#include "../vcgimbal/include/PWMControllerPico.h"
#include "../include/USBProtocol.h"
#include "pico/stdlib.h"
#include <iostream>
#include <memory>

/**
 * @brief Raspberry Pi Pico Antenna Tracker with USB Serial Target Control
 * 
 * Hardware Configuration:
 * - Pan Servo: GPIO 16
 * - Tilt Servo: GPIO 17
 * - GPS Module: UART0 (TX=GPIO 0, RX=GPIO 1)
 * - USB Serial: USB CDC (automatic)
 * - GPS Status LED: GPIO 20
 * - Target Data LED: GPIO 21
 */

// Pin definitions
constexpr uint PAN_SERVO_PIN = 16;
constexpr uint TILT_SERVO_PIN = 17;

constexpr uint GPS_UART_ID = 0;
constexpr uint GPS_TX_PIN = 0;
constexpr uint GPS_RX_PIN = 1;
constexpr uint32_t GPS_BAUD = 38400;  // M10G default

constexpr uint GPS_LED_PIN = 20;
constexpr uint TARGET_LED_PIN = 21;

// Update rate
constexpr uint32_t UPDATE_INTERVAL_MS = 100; // 10 Hz

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Wait for USB connection (optional, comment out for standalone operation)
    sleep_ms(2000);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Raspberry Pi Pico Antenna Tracker" << std::endl;
    std::cout << "  with USB Serial Target Control" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Create PWM controller for servos
    auto pwm_controller = std::make_shared<PWMControllerPico>();
    
    // Create gimbal
    auto gimbal = std::make_shared<Gimbal>(pwm_controller, PAN_SERVO_PIN, TILT_SERVO_PIN);
    
    // Create GPS module
    auto gps = std::make_shared<GPSModule>(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD);
    
    // Create USB target parser (receives target lat/lon/alt over USB serial)
    auto usb_parser = std::make_shared<USBTargetParser>();
    
    // Create status LEDs
    auto gps_led = std::make_shared<StatusLED>(GPS_LED_PIN);
    auto target_led = std::make_shared<StatusLED>(TARGET_LED_PIN);
    
    // Create antenna tracker
    AntennaTracker tracker(gimbal, gps, usb_parser, gps_led, target_led);
    
    // Initialize tracker
    if (!tracker.init()) {
        std::cerr << "Failed to initialize antenna tracker!" << std::endl;
        std::cerr << "System halted." << std::endl;
        while (true) {
            sleep_ms(1000);
        }
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Tracker initialized successfully!" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::cout << "Hardware Configuration:" << std::endl;
    std::cout << "  Pan Servo:    GPIO " << PAN_SERVO_PIN << std::endl;
    std::cout << "  Tilt Servo:   GPIO " << TILT_SERVO_PIN << std::endl;
    std::cout << "  GPS UART:     UART" << GPS_UART_ID << " (TX=" << GPS_TX_PIN 
              << ", RX=" << GPS_RX_PIN << ", " << GPS_BAUD << " baud)" << std::endl;
    std::cout << "  USB Serial:   USB CDC (115200 baud)" << std::endl;
    std::cout << "  GPS LED:      GPIO " << GPS_LED_PIN << std::endl;
    std::cout << "  Target LED:   GPIO " << TARGET_LED_PIN << std::endl;
    
    std::cout << "\nTarget Position Protocol:" << std::endl;
    std::cout << "  Send 14-byte packets over USB (/dev/ttyACM0, 115200 baud):" << std::endl;
    std::cout << "  Byte 0:    0x01 (TARGET_DATA)" << std::endl;
    std::cout << "  Bytes 1-4: Latitude (float32, little-endian)" << std::endl;
    std::cout << "  Bytes 5-8: Longitude (float32, little-endian)" << std::endl;
    std::cout << "  Bytes 9-12: Altitude (float32, little-endian)" << std::endl;
    std::cout << "  Byte 13:   XOR Checksum (of bytes 0-12)" << std::endl;
    std::cout << "\nExample Python: python3 scripts/test_serial_usb.py" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Set tracking parameters
    tracker.setMinElevation(-5.0f); // Don't point more than 5 degrees below horizon
    tracker.setAutoTracking(true);
    
    uint32_t loop_count = 0;
    uint32_t last_status_time = 0;
    
    // Main loop
    while (true) {
        // Update tracker (reads GPS, ROS, updates gimbal)
        tracker.update();
        
        // Print status periodically (every 5 seconds)
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_status_time >= 5000) {
            GPSData gps_pos = tracker.getTrackerPosition();
            NavSatFixMsg target_pos = tracker.getTargetPosition();
            PanTiltAngles angles = tracker.getCurrentAngles();
            
            std::cout << "\n--- Status Update ---" << std::endl;
            std::cout << "Tracker GPS: ";
            if (gps_pos.valid) {
                std::cout << "Lat=" << gps_pos.latitude 
                          << ", Lon=" << gps_pos.longitude
                          << ", Alt=" << gps_pos.altitude << "m"
                          << ", Sats=" << (int)gps_pos.satellites
                          << ", Fix=" << (int)gps_pos.fix_quality << std::endl;
            } else {
                std::cout << "No fix" << std::endl;
            }
            
            std::cout << "Target (USB): ";
            float target_lat, target_lon, target_alt;
            if (usb_parser->getTargetPosition(target_lat, target_lon, target_alt)) {
                std::cout << "Lat=" << target_lat 
                          << ", Lon=" << target_lon
                          << ", Alt=" << target_alt << "m" << std::endl;
            } else {
                std::cout << "No data" << std::endl;
            }
            
            std::cout << "USB Target: " << (usb_parser->isConnected() ? "Connected" : "Disconnected") << std::endl;
            std::cout << "Gimbal: Pan=" << angles.pan << "°, Tilt=" << angles.tilt << "°" << std::endl;
            std::cout << "--------------------\n" << std::endl;
            
            last_status_time = current_time;
        }
        
        // Sleep for update interval
        sleep_ms(UPDATE_INTERVAL_MS);
        loop_count++;
    }
    
    // Cleanup (unreachable in this infinite loop)
    tracker.shutdown();
    
    return 0;
}
