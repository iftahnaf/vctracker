#include "../include/GPSModule.h"
#include "pico/stdlib.h"
#include <iostream>
#include <memory>

/**
 * @brief GPS Module Unit Test
 * 
 * This test verifies that the GPS module can:
 * 1. Initialize UART0 correctly
 * 2. Read NMEA sentences from GPS module
 * 3. Parse latitude, longitude, altitude, and fix quality
 * 4. Update internal state
 * 
 * Expected Output:
 * - UART0 initialized at 9600 baud
 * - GPS sentences displayed as received
 * - Parsed position data printed every update cycle
 * 
 * Hardware Required:
 * - GPIO 0 (TX), GPIO 1 (RX) connected to GPS module
 * - GPS module powered and configured for NMEA 0183 output
 */

constexpr uint GPS_UART_ID = 0;
constexpr uint GPS_TX_PIN = 0;
constexpr uint GPS_RX_PIN = 1;
constexpr uint32_t GPS_BAUD = 9600;

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  GPS Module Test" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Create GPS module
    std::cout << "Initializing GPS module on UART" << GPS_UART_ID << "..." << std::endl;
    auto gps = std::make_shared<GPSModule>(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD);
    
    if (!gps->init()) {
        std::cerr << "Failed to initialize GPS module!" << std::endl;
        return -1;
    }
    
    std::cout << "✓ GPS module initialized successfully" << std::endl;
    std::cout << "\nWaiting for GPS data..." << std::endl;
    std::cout << "Expected: NMEA sentences (e.g., $GPRMC, $GPGGA, $GPGSA)" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;
    
    uint32_t read_count = 0;
    uint32_t fix_count = 0;
    uint32_t last_fix_time = 0;
    
    // Read GPS data for up to 60 seconds
    uint32_t start_time = time_us_32();
    uint32_t timeout_us = 60 * 1000 * 1000; // 60 seconds
    
    while (time_us_32() - start_time < timeout_us) {
        // Update GPS module (reads and parses data)
        if (gps->update()) {
            read_count++;
            
            // Get current position
                auto data = gps->getData();
                double lat = data.latitude;
                double lon = data.longitude;
                double alt = data.altitude;
                int sats = gps->getSatelliteCount();
                int fix_type = data.fix_quality;
            
            // Only print when we have valid fix
            if (fix_type > 0) {
                if (time_us_32() - last_fix_time > 5 * 1000 * 1000) { // Print every 5s
                    std::cout << "GPS Fix #" << ++fix_count << ":" << std::endl;
                    std::cout << "  Latitude:  " << lat << "°" << std::endl;
                    std::cout << "  Longitude: " << lon << "°" << std::endl;
                    std::cout << "  Altitude:  " << alt << " m" << std::endl;
                    std::cout << "  Satellites: " << sats << std::endl;
                    std::cout << "  Fix Type: ";
                    
                    switch (fix_type) {
                        case 1: std::cout << "GPS Fix"; break;
                        case 2: std::cout << "DGPS Fix"; break;
                        case 3: std::cout << "PPS Fix"; break;
                        default: std::cout << "Unknown"; break;
                    }
                    std::cout << "\n" << std::endl;
                    
                    last_fix_time = time_us_32();
                }
            } else if (read_count % 100 == 0) {
                std::cout << "." << std::flush; // Progress indicator
            }
        }
        
        sleep_ms(10); // Update every 10ms
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Test Complete" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::cout << "Results:" << std::endl;
    std::cout << "  Total updates: " << read_count << std::endl;
    std::cout << "  Fixes obtained: " << fix_count << std::endl;
    
    if (fix_count > 0) {
        std::cout << "\n✓ GPS Test PASSED" << std::endl;
    } else {
        std::cout << "\n✗ GPS Test FAILED - No fixes obtained" << std::endl;
        std::cout << "  Check GPS module connection and power" << std::endl;
    }
    
    std::cout << std::endl;
    
    return fix_count > 0 ? 0 : -1;
}
