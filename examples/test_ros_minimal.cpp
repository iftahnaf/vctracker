#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <iostream>

// LED pins
#define STATUS_LED 20
#define MESSAGE_LED 21

sensor_msgs__msg__NavSatFix msg;
bool message_received = false;

void subscription_callback(const void * msgin) {
    const sensor_msgs__msg__NavSatFix * msg_in = (const sensor_msgs__msg__NavSatFix *)msgin;
    message_received = true;
    gpio_put(MESSAGE_LED, 1);  // Turn on LED
    std::cout << "MESSAGE RECEIVED! Lat: " << msg_in->latitude << std::endl;
}

int main() {
    stdio_init_all();
    
    // Init LEDs
    gpio_init(STATUS_LED);
    gpio_set_dir(STATUS_LED, GPIO_OUT);
    gpio_init(MESSAGE_LED);
    gpio_set_dir(MESSAGE_LED, GPIO_OUT);
    gpio_put(STATUS_LED, 0);
    gpio_put(MESSAGE_LED, 0);
    
    sleep_ms(3000);
    std::cout << "\n=== Minimal ROS Test ===" << std::endl;
    
    // Blink to show we're alive
    for (int i = 0; i < 5; i++) {
        gpio_put(STATUS_LED, 1);
        gpio_put(MESSAGE_LED, 1);
        sleep_ms(100);
        gpio_put(STATUS_LED, 0);
        gpio_put(MESSAGE_LED, 0);
        sleep_ms(100);
    }
    
    std::cout << "Initializing micro-ROS..." << std::endl;
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    // Wait for agent
    std::cout << "Waiting for agent..." << std::endl;
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    
    if (ret != RCL_RET_OK) {
        std::cerr << "FAILED to init support! Error: " << ret << std::endl;
        gpio_put(STATUS_LED, 1);  // Solid on = error
        while(1) { sleep_ms(1000); }
    }
    
    std::cout << "✓ Support initialized" << std::endl;
    
    // Create node
    rcl_node_t node;
    ret = rclc_node_init_default(&node, "test_node", "", &support);
    if (ret != RCL_RET_OK) {
        std::cerr << "FAILED to create node! Error: " << ret << std::endl;
        while(1) { sleep_ms(1000); }
    }
    std::cout << "✓ Node created" << std::endl;
    
    // Create subscription
    rcl_subscription_t subscription;
    ret = rclc_subscription_init_default(
        &subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "/target/gps/fix");
        
    if (ret != RCL_RET_OK) {
        std::cerr << "FAILED to create subscription! Error: " << ret << std::endl;
        while(1) { sleep_ms(1000); }
    }
    std::cout << "✓ Subscription created on /target/gps/fix" << std::endl;
    
    // Create executor
    rclc_executor_t executor;
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        std::cerr << "FAILED to create executor! Error: " << ret << std::endl;
        while(1) { sleep_ms(1000); }
    }
    
    ret = rclc_executor_add_subscription(&executor, &subscription, &msg, &subscription_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        std::cerr << "FAILED to add subscription! Error: " << ret << std::endl;
        while(1) { sleep_ms(1000); }
    }
    std::cout << "✓ Executor ready" << std::endl;
    std::cout << "\nWaiting for messages... (60 seconds)" << std::endl;
    std::cout << "Publish with: ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix ..." << std::endl;
    
    // Blink status LED to show connected
    gpio_put(STATUS_LED, 1);
    sleep_ms(500);
    gpio_put(STATUS_LED, 0);
    
    // Main loop
    uint32_t start = time_us_32();
    uint32_t last_msg_time = 0;
    int loop_count = 0;
    
    while ((time_us_32() - start) < 60000000) {  // 60 seconds
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        // Keep message LED on for 3 seconds after receiving
        if (message_received && (time_us_32() - last_msg_time < 3000000)) {
            gpio_put(MESSAGE_LED, 1);
        } else {
            gpio_put(MESSAGE_LED, 0);
            if (message_received) {
                message_received = false;
                last_msg_time = time_us_32();
            }
        }
        
        // Blink status LED once per second to show alive
        loop_count++;
        if (loop_count % 100 == 0) {
            gpio_put(STATUS_LED, 1);
            sleep_ms(50);
            gpio_put(STATUS_LED, 0);
        }
        
        sleep_ms(10);
    }
    
    std::cout << "\nTest finished!" << std::endl;
    
    // Cleanup
    rcl_subscription_fini(&subscription, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    
    gpio_put(STATUS_LED, 0);
    gpio_put(MESSAGE_LED, 0);
    
    return 0;
}
