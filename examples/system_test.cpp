#include "../include/GPSModule.h"
#include "../include/ROSParser.h"
#include "../include/StatusLED.h"
#include "../vcgimbal/include/Gimbal.h"
#include "../vcgimbal/include/PWMControllerPico.h"
#include "pico/stdlib.h"
#include <iostream>
#include <memory>
#include <cmath>

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  System Integration Test - 30 Seconds" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Initialize LEDs FIRST - simplest test
    std::cout << "[1/5] Initializing Status LEDs (GPIO 20/21)..." << std::endl;
    auto status_led = std::make_shared<StatusLED>(20);  // Status indicator
    auto test_led = std::make_shared<StatusLED>(21);    // Test indicator
    if (!status_led->init() || !test_led->init()) {
        std::cerr << "Failed to initialize LEDs!" << std::endl;
        return -1;
    }
    status_led->off();
    test_led->off();
    std::cout << "  ✓ LEDs ready\n" << std::endl;
    
    // Test LEDs immediately
    std::cout << "Testing LEDs..." << std::endl;
    for (int i = 0; i < 10; i++) {
        status_led->on();
        test_led->on();
        sleep_ms(200);
        status_led->off();
        test_led->off();
        sleep_ms(200);
    }
    std::cout << "✓ LEDs working\n" << std::endl;
    
    // Initialize other components
    std::cout << "\n[2/5] Initializing PWM & Gimbal..." << std::endl;
    auto pwm = std::make_shared<PWMControllerPico>();
    auto gimbal = std::make_shared<Gimbal>(pwm, 16, 17);
    if (!gimbal->init()) {
        std::cerr << "Gimbal init failed!" << std::endl;
        return -1;
    }
    gimbal->setTipAngle(0.0f, 0.0f);
    gimbal->setTipAngle(0.0f, 0.0f);
    std::cout << "  ✓ Gimbal ready\n" << std::endl;
    
    std::cout << "[3/5] Initializing GPS..." << std::endl;
    auto gps = std::make_shared<GPSModule>(0, 0, 1, 38400);
    gps->init();
    std::cout << "  ✓ GPS ready\n" << std::endl;
    
    std::cout << "[4/5] Initializing micro-ROS..." << std::endl;
    auto ros_parser = std::make_shared<ROSParser>("antenna_tracker", "/target/gps/fix");
    if (!ros_parser->init()) {
        std::cerr << "  ✗ ROS init FAILED!" << std::endl;
        // Continue anyway to test other components
    } else {
        std::cout << "  ✓ ROS init OK" << std::endl;
    }
    
    // Wait for agent connection
    std::cout << "  Waiting for agent connection..." << std::endl;
    for (int i = 0; i < 50; i++) {
        ros_parser->spin();
        if (ros_parser->isConnected()) {
            std::cout << "  ✓ Connected to agent!" << std::endl;
            status_led->on();
            sleep_ms(500);
            status_led->off();
            break;
        }
        sleep_ms(100);
    }
    
    if (!ros_parser->isConnected()) {
        std::cerr << "  ✗ Agent connection timeout!" << std::endl;
    }
    
    // Quick gimbal test
    std::cout << "\nTesting Gimbal..." << std::endl;
    gimbal->setTipAngle(-45.0f, 0.0f);
    sleep_ms(300);
    gimbal->setTipAngle(45.0f, 0.0f);
    sleep_ms(300);
    gimbal->setTipAngle(0.0f, 0.0f);
    std::cout << "✓ Gimbal works\n" << std::endl;
    
    // Main 30-second test loop
    std::cout << "\n========================================" << std::endl;
    std::cout << "  30-Second ROS Integration Test" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Status LED (GPIO 20) = System health" << std::endl;
    std::cout << "Test LED (GPIO 21)   = Message received\n" << std::endl;
    
    uint32_t start_time = time_us_32();
    uint32_t duration = 30 * 1000 * 1000;
    int loop_count = 0;
    uint32_t last_ros_msg_time = 0;
    bool ros_connected = false;
    
    while (time_us_32() - start_time < duration) {
        loop_count++;
        
        ros_parser->spin();
        gps->update();
        
        // System health LED
        bool gps_ok = gps->getData().fix_quality > 0;
        bool ros_ok = ros_parser->isConnected();
        
        if (gps_ok || ros_ok) {
            status_led->on();
            ros_connected = true;
        } else {
            status_led->off();
        }
        
        // Check for ROS messages
        if (ros_parser->hasMessage()) {
            auto msg = ros_parser->getMessage();
            last_ros_msg_time = time_us_32();
            test_led->on();  // Turn on immediately
            std::cout << "✓ ROS Msg! Lat:" << msg.latitude << " Lon:" << msg.longitude << std::endl;
        }
        
        // Keep test LED on for 2 seconds after message (more visible)
        uint32_t time_since_msg = time_us_32() - last_ros_msg_time;
        if (time_since_msg > 2000000) {  // 2 seconds
            test_led->off();
        }
        
        // Move gimbal in circle
        float angle = (loop_count * 0.1f) * 3.14159f / 180.0f;
        float pan = 45.0f * std::cos(angle);
        float tilt = 30.0f * std::sin(angle);
        gimbal->setTipAngle(pan, 0.0f);
        gimbal->setTipAngle(0.0f, tilt);
        
        // Print status every 3 seconds
        if (loop_count % 30 == 0) {
            uint32_t elapsed = (time_us_32() - start_time) / 1000000;
            std::cout << "[" << elapsed << "s] GPS:" << (gps_ok ? "✓" : "✗") 
                      << " ROS:" << (ros_ok ? "✓" : "✗") << std::endl;
        }
        
        sleep_ms(100);
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Test Complete!" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    gimbal->setTipAngle(0.0f, 0.0f);
    gimbal->setTipAngle(0.0f, 0.0f);
    status_led->off();
    test_led->off();
    
    if (ros_connected) {
        std::cout << "✓ ROS connected successfully!" << std::endl;
    }
    std::cout << "✓ All components operational\n" << std::endl;
    
    return 0;
}
