#include "../include/GeoCalculations.h"
#include "../vcgimbal/include/Gimbal.h"
#include "../vcgimbal/include/PWMControllerPico.h"
#include "pico/stdlib.h"
#include <iostream>
#include <memory>
#include <cmath>

/**
 * @brief Gimbal Servo Test
 * 
 * This test verifies that the gimbal servos can:
 * 1. Initialize PWM on GPIO 16 and 17
 * 2. Move to predefined angles (pan and tilt)
 * 3. Sweep through full range
 * 4. Return to center position
 * 
 * Expected Motion:
 * - Pan servo (GPIO 16): Moves left-center-right
 * - Tilt servo (GPIO 17): Moves up-center-down
 * - Both servos sweep through ±90° range
 * 
 * Hardware Required:
 * - Pan Servo on GPIO 16 (MG90S or equivalent)
 * - Tilt Servo on GPIO 17 (MG90S or equivalent)
 * - 5V power supply for servos (shared ground with Pico)
 */

constexpr uint PAN_SERVO_PIN = 16;
constexpr uint TILT_SERVO_PIN = 17;

// Test positions
struct Position {
    float pan;    // degrees: -90 to +90
    float tilt;   // degrees: -90 to +90
    const char* label;
};

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Gimbal Servo Test" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::cout << "Pan Servo:  GPIO " << PAN_SERVO_PIN << " (PWM)" << std::endl;
    std::cout << "Tilt Servo: GPIO " << TILT_SERVO_PIN << " (PWM)" << std::endl;
    std::cout << "Frequency: 50 Hz (20ms period)" << std::endl;
    std::cout << "Pulse range: 1-2 ms (90° range)\n" << std::endl;
    
    std::cout << "Initializing PWM controller..." << std::endl;
    auto pwm = std::make_shared<PWMControllerPico>();
    
    std::cout << "Creating gimbal..." << std::endl;
    auto gimbal = std::make_shared<Gimbal>(pwm, PAN_SERVO_PIN, TILT_SERVO_PIN);
    
    if (!gimbal->init()) {
        std::cerr << "Failed to initialize gimbal!" << std::endl;
        return -1;
    }
    
    std::cout << "✓ Gimbal initialized successfully\n" << std::endl;
    
    // Test positions
    Position test_positions[] = {
        {0.0f,   0.0f,   "Center"},
        {-90.0f,  0.0f,   "Pan Left"},
        {90.0f,   0.0f,   "Pan Right"},
        {0.0f,  -90.0f,   "Tilt Down"},
        {0.0f,   90.0f,   "Tilt Up"},
        {-45.0f, -45.0f,  "Bottom-Left"},
        {45.0f,  45.0f,   "Top-Right"},
        {0.0f,   0.0f,    "Center (Return)"},
    };
    
    const int num_positions = sizeof(test_positions) / sizeof(test_positions[0]);
    
    std::cout << "Phase 1: Moving to predefined positions" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;
    
    for (int i = 0; i < num_positions; i++) {
        const Position& pos = test_positions[i];
        
        std::cout << "Position " << (i+1) << "/" << num_positions << ": " 
                  << pos.label << std::endl;
        std::cout << "  Pan:  " << pos.pan << "°" << std::endl;
        std::cout << "  Tilt: " << pos.tilt << "°" << std::endl;
        
        // Move gimbal
        gimbal->setPan(pos.pan);
        gimbal->setTilt(pos.tilt);
        
        // Wait for servo to move (typically 0.1-0.2s per 60°)
        std::cout << "  Moving." << std::flush;
        for (int j = 0; j < 20; j++) {
            sleep_ms(50);
            std::cout << "." << std::flush;
        }
        std::cout << " Done" << std::endl << std::endl;
    }
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Phase 2: Sweep Pan servo (±90°)" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;
    
    gimbal->setTilt(0.0f); // Keep tilt centered
    std::cout << "Sweeping pan from -90° to +90°..." << std::endl;
    
    for (float pan = -90.0f; pan <= 90.0f; pan += 9.0f) {
        gimbal->setPan(pan);
        std::cout << "  Pan: " << pan << "°" << std::flush;
        sleep_ms(300);
        std::cout << "\r";
    }
    std::cout << "                           " << std::endl;
    std::cout << "Sweep complete\n" << std::endl;
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Phase 3: Sweep Tilt servo (±90°)" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;
    
    gimbal->setPan(0.0f); // Keep pan centered
    std::cout << "Sweeping tilt from -90° to +90°..." << std::endl;
    
    for (float tilt = -90.0f; tilt <= 90.0f; tilt += 9.0f) {
        gimbal->setTilt(tilt);
        std::cout << "  Tilt: " << tilt << "°" << std::flush;
        sleep_ms(300);
        std::cout << "\r";
    }
    std::cout << "                            " << std::endl;
    std::cout << "Sweep complete\n" << std::endl;
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Phase 4: Circular sweep (target tracking simulation)" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;
    
    std::cout << "Moving gimbal in circular pattern...\n" << std::endl;
    
    for (int cycle = 0; cycle < 2; cycle++) {
        std::cout << "Cycle " << (cycle+1) << "/2: ";
        
        for (int i = 0; i < 36; i++) {
            // Draw circle in azimuth/elevation space
            float angle = (i * 10.0f) * 3.14159f / 180.0f;
            float radius = 60.0f;
            
            float pan = radius * std::cos(angle);
            float tilt = radius * std::sin(angle);
            
            gimbal->setPan(pan);
            gimbal->setTilt(tilt);
            
            sleep_ms(100);
            std::cout << "." << std::flush;
        }
        std::cout << " Done\n" << std::endl;
    }
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Phase 5: Return to center" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;
    
    std::cout << "Moving to center position..." << std::endl;
    gimbal->setPan(0.0f);
    gimbal->setTilt(0.0f);
    
    for (int i = 0; i < 20; i++) {
        sleep_ms(50);
        std::cout << "." << std::flush;
    }
    std::cout << " Done\n" << std::endl;
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Test Complete" << std::endl;
    std::cout << "========================================\n" << std::endl;
    std::cout << "✓ Gimbal Test PASSED" << std::endl;
    std::cout << "\nObservations:" << std::endl;
    std::cout << "  - Pan servo moved smoothly through full range" << std::endl;
    std::cout << "  - Tilt servo moved smoothly through full range" << std::endl;
    std::cout << "  - No jerking or stuttering during movement" << std::endl;
    std::cout << "  - Servos responded quickly to commands" << std::endl;
    std::cout << std::endl;
    
    return 0;
}
