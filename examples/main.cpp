#include "../include/AntennaTracker.h"
#include "../vcgimbal/include/Gimbal.h"
#include "../vcgimbal/include/PWMControllerPico.h"
#include "pico/stdlib.h"
#include <iostream>
#include <memory>

/**
 * @brief Raspberry Pi Pico Antenna Tracker with micro-ROS
 * 
 * Hardware Configuration:
 * - Pan Servo: GPIO 16
 * - Tilt Servo: GPIO 17
 * - GPS Module: UART0 (TX=GPIO 0, RX=GPIO 1)
 * - micro-ROS: USB Serial (automatic)
 * - GPS Status LED: GPIO 20
 * - ROS Status LED: GPIO 21
 */

// Pin definitions
constexpr uint PAN_SERVO_PIN = 16;
constexpr uint TILT_SERVO_PIN = 17;

constexpr uint GPS_UART_ID = 0;
constexpr uint GPS_TX_PIN = 0;
constexpr uint GPS_RX_PIN = 1;
constexpr uint32_t GPS_BAUD = 38400;  // M10G default

constexpr uint GPS_LED_PIN = 20;
constexpr uint ROS_LED_PIN = 21;

// Update rate
constexpr uint32_t UPDATE_INTERVAL_MS = 100; // 10 Hz

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Wait for USB connection (optional, comment out for standalone operation)
    sleep_ms(2000);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Raspberry Pi Pico Antenna Tracker" << std::endl;
    std::cout << "  with micro-ROS Integration" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Create PWM controller for servos
    auto pwm_controller = std::make_shared<PWMControllerPico>();
    
    // Create gimbal
    auto gimbal = std::make_shared<Gimbal>(pwm_controller, PAN_SERVO_PIN, TILT_SERVO_PIN);
    
    // Create GPS module
    auto gps = std::make_shared<GPSModule>(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD);
    
    // Create micro-ROS parser (topic: /target/gps/fix)
    auto ros_parser = std::make_shared<ROSParser>("antenna_tracker", "/target/gps/fix");
    
    // Create status LEDs
    auto gps_led = std::make_shared<StatusLED>(GPS_LED_PIN);
    auto ros_led = std::make_shared<StatusLED>(ROS_LED_PIN);
    
    // Create antenna tracker
    AntennaTracker tracker(gimbal, gps, ros_parser, gps_led, ros_led);
    
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
    std::cout << "  micro-ROS:    USB Serial (automatic)" << std::endl;
    std::cout << "  GPS LED:      GPIO " << GPS_LED_PIN << std::endl;
    std::cout << "  ROS LED:      GPIO " << ROS_LED_PIN << std::endl;
    
    std::cout << "\nMake sure to start micro-ROS agent on host:" << std::endl;
    std::cout << "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0" << std::endl;
    
    std::cout << "\nThen publish target GPS to /target/gps/fix:" << std::endl;
    std::cout << "  ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix ..." << std::endl;
    
    std::cout << "\nWaiting for GPS fix and ROS messages..." << std::endl;
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
            
            std::cout << "Target (ROS): ";
            if (target_pos.valid) {
                std::cout << "Lat=" << target_pos.latitude 
                          << ", Lon=" << target_pos.longitude
                          << ", Alt=" << target_pos.altitude << "m"
                          << ", Status=" << (int)target_pos.status << std::endl;
            } else {
                std::cout << "No data" << std::endl;
            }
            
            std::cout << "micro-ROS: " << (ros_parser->isConnected() ? "Connected" : "Disconnected") << std::endl;
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
