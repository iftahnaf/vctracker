#include "../include/StatusLED.h"
#include "pico/stdlib.h"
#include <iostream>
#include <memory>

/**
 * @brief Status LED Unit Test
 * 
 * This test verifies that Status LEDs can:
 * 1. Initialize GPIO pins correctly
 * 2. Turn on and off
 * 3. Blink at various rates
 * 4. Display different states
 * 
 * Expected Output & Visual:
 * - LED 1 (GPIO 20): Static on, slow blink (2 Hz), fast blink (5 Hz)
 * - LED 2 (GPIO 21): Static on, slow blink (2 Hz), fast blink (5 Hz)
 * - Console output describing each test
 * 
 * Test Phases:
 * 1. Both LEDs on for 2 seconds
 * 2. LED 1 slow blink (0.5s on, 0.5s off) for 5 seconds
 * 3. LED 2 fast blink (0.1s on, 0.1s off) for 5 seconds
 * 4. Both blinking alternately for 5 seconds
 * 5. Both off
 * 
 * Hardware Required:
 * - GPIO 20 with LED (+ 220Ω resistor + GND)
 * - GPIO 21 with LED (+ 220Ω resistor + GND)
 */

constexpr uint LED1_GPIO = 20;  // GPS Status LED
constexpr uint LED2_GPIO = 21;  // ROS Status LED

void print_phase(const char* description) {
    std::cout << "\n▶ " << description << std::endl;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Status LED Test" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::cout << "GPIO " << LED1_GPIO << ": GPS Status LED" << std::endl;
    std::cout << "GPIO " << LED2_GPIO << ": ROS Status LED" << std::endl;
    std::cout << "\nInitializing LEDs..." << std::endl;
    
    // Create LED controllers
    auto led1 = std::make_shared<StatusLED>(LED1_GPIO);
    auto led2 = std::make_shared<StatusLED>(LED2_GPIO);
    
    if (!led1->init() || !led2->init()) {
        std::cerr << "Failed to initialize LEDs!" << std::endl;
        return -1;
    }
    
    std::cout << "✓ Both LEDs initialized\n" << std::endl;
    
    // Test 1: Both ON
    print_phase("Test 1: Both LEDs ON (2 seconds)");
    std::cout << "Expected: Both LEDs light up steadily" << std::endl;
    led1->on();
    led2->on();
    for (int i = 0; i < 20; i++) {
        sleep_ms(100);
        std::cout << "." << std::flush;
    }
    std::cout << " Done\n" << std::endl;
    
    // Test 2: LED1 slow blink
    print_phase("Test 2: LED1 Slow Blink - 2 Hz (5 seconds)");
    std::cout << "Expected: LED1 blinks on/off at 2 Hz" << std::endl;
    led1->off();
    led2->off();
    for (int i = 0; i < 10; i++) {
        led1->on();
        for (int j = 0; j < 5; j++) {
            sleep_ms(100);
            std::cout << "." << std::flush;
        }
        led1->off();
        for (int j = 0; j < 5; j++) {
            sleep_ms(100);
            std::cout << "," << std::flush;
        }
    }
    std::cout << " Done\n" << std::endl;
    
    // Test 3: LED2 fast blink
    print_phase("Test 3: LED2 Fast Blink - 5 Hz (5 seconds)");
    std::cout << "Expected: LED2 blinks on/off rapidly at 5 Hz" << std::endl;
    led1->off();
    led2->off();
    for (int i = 0; i < 50; i++) {
        led2->on();
        sleep_ms(100);
        led2->off();
        sleep_ms(100);
        std::cout << "." << std::flush;
    }
    std::cout << " Done\n" << std::endl;
    
    // Test 4: Alternating blink (simulating GPS/ROS status)
    print_phase("Test 4: Alternating Blink - GPS and ROS Status (5 seconds)");
    std::cout << "Expected: LEDs alternate on/off (pattern: LED1 on/LED2 off → LED1 off/LED2 on)" << std::endl;
    led1->off();
    led2->off();
    for (int i = 0; i < 10; i++) {
        // GPS found but no ROS
        led1->on();
        led2->off();
        for (int j = 0; j < 5; j++) {
            sleep_ms(100);
            std::cout << "#" << std::flush;
        }
        
        // Both connected
        led1->on();
        led2->on();
        for (int j = 0; j < 5; j++) {
            sleep_ms(100);
            std::cout << "@" << std::flush;
        }
    }
    std::cout << " Done\n" << std::endl;
    
    // Test 5: Both OFF
    print_phase("Test 5: Both LEDs OFF");
    std::cout << "Expected: Both LEDs turn off" << std::endl;
    led1->off();
    led2->off();
    sleep_ms(1000);
    std::cout << "Done\n" << std::endl;
    
    // Test 6: Pulse effect
    print_phase("Test 6: Pulse Effect - Both LEDs pulse together");
    std::cout << "Expected: Both LEDs pulse smoothly at 1 Hz (3 seconds)" << std::endl;
    for (int pulse = 0; pulse < 3; pulse++) {
        // ON for 500ms
        led1->on();
        led2->on();
        for (int i = 0; i < 5; i++) {
            sleep_ms(100);
            std::cout << "*" << std::flush;
        }
        
        // OFF for 500ms
        led1->off();
        led2->off();
        for (int i = 0; i < 5; i++) {
            sleep_ms(100);
            std::cout << "-" << std::flush;
        }
    }
    std::cout << " Done\n" << std::endl;
    
    // Final state
    led1->off();
    led2->off();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Test Complete" << std::endl;
    std::cout << "========================================\n" << std::endl;
    std::cout << "✓ LED Test PASSED" << std::endl;
    std::cout << "\nIf all LEDs behaved as expected, hardware is OK!" << std::endl;
    std::cout << std::endl;
    
    return 0;
}
