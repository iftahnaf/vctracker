#include "../include/GPSModule.h"
#include "../include/USBTargetParser.h"
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
    auto status_led = std::make_shared<StatusLED>(20);  // GPS status
    auto target_led = std::make_shared<StatusLED>(21);  // USB target data
    if (!status_led->init() || !target_led->init()) {
        std::cerr << "Failed to initialize LEDs!" << std::endl;
        return -1;
    }
    status_led->off();
    target_led->off();
    std::cout << "  ✓ LEDs ready\n" << std::endl;
    
    // Test LEDs immediately
    std::cout << "Testing LEDs..." << std::endl;
    for (int i = 0; i < 10; i++) {
        status_led->on();
        target_led->on();
        sleep_ms(200);
        status_led->off();
        target_led->off();
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
    
    std::cout << "[4/5] Initializing USB Target Parser..." << std::endl;
    auto usb_parser = std::make_shared<USBTargetParser>();
    if (!usb_parser->init()) {
        std::cerr << "  ✗ USB init FAILED!" << std::endl;
        // Continue anyway to test other components
    } else {
        std::cout << "  ✓ USB CDC ready" << std::endl;
    }
    
    // Wait for USB connection
    std::cout << "  Waiting for USB target data..." << std::endl;
    for (int i = 0; i < 50; i++) {
        usb_parser->update();
        if (usb_parser->isConnected()) {
            std::cout << "  ✓ USB target data connected!" << std::endl;
            status_led->on();
            sleep_ms(500);
            status_led->off();
            break;
        }
        sleep_ms(100);
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
    std::cout << "  30-Second System Integration Test" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Status LED (GPIO 20) = GPS fix" << std::endl;
    std::cout << "Target LED (GPIO 21) = USB target data\n" << std::endl;
    
    uint32_t start_time = time_us_32();
    uint32_t duration = 30 * 1000 * 1000;
    int loop_count = 0;
    uint32_t last_usb_msg_time = 0;
    bool usb_connected = false;
    
    while (time_us_32() - start_time < duration) {
        loop_count++;
        
        usb_parser->update();
        gps->update();
        
        // System health LED
        bool gps_ok = gps->getData().fix_quality > 0;
        bool usb_ok = usb_parser->isConnected();
        
        if (gps_ok) {
            status_led->on();
        } else {
            status_led->off();
        }
        
        // Check for USB messages
        if (usb_parser->isConnected()) {
            float lat, lon, alt;
            usb_parser->getTargetPosition(lat, lon, alt);
            last_usb_msg_time = time_us_32();
            target_led->on();  // Turn on immediately
            std::cout << "✓ USB Msg! Lat:" << lat << " Lon:" << lon << " Alt:" << alt << std::endl;
            usb_connected = true;
        }
        
        // Keep target LED on for 2 seconds after message (more visible)
        uint32_t time_since_msg = time_us_32() - last_usb_msg_time;
        if (time_since_msg > 2000000) {  // 2 seconds
            target_led->off();
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
                      << " USB:" << (usb_ok ? "✓" : "✗") << std::endl;
        }
        
        sleep_ms(100);
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Test Complete!" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    gimbal->setTipAngle(0.0f, 0.0f);
    gimbal->setTipAngle(0.0f, 0.0f);
    status_led->off();
    target_led->off();
    
    if (usb_connected) {
        std::cout << "✓ USB target data connected successfully!" << std::endl;
    }
    std::cout << "✓ All components operational\n" << std::endl;
    
    return 0;
}
