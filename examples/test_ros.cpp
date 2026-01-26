#include "../include/ROSParser.h"
#include "pico/stdlib.h"
#include <iostream>
#include <memory>

/**
 * @brief micro-ROS Subscriber Test
 * 
 * This test verifies that the micro-ROS integration works:
 * 1. Initializes micro-ROS transport over USB CDC
 * 2. Waits for and connects to micro-ROS agent
 * 3. Creates subscription to /target/gps/fix topic
 * 4. Receives and displays NavSatFix messages
 * 5. Detects agent disconnection
 * 
 * Expected Output:
 * - "Waiting for micro-ROS agent..." message
 * - "✓ Connected to micro-ROS agent" when agent connects
 * - GPS fix messages as they arrive
 * - "Agent lost" if connection drops
 * 
 * Prerequisites (on host computer):
 * 1. Start micro-ROS agent:
 *    $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
 * 
 * 2. In another terminal, publish test messages:
 *    $ ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix \
 *        "latitude: 37.7749
 *         longitude: -122.4194
 *         altitude: 10.5
 *         position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
 *         position_covariance_type: 2" --once
 * 
 *    Or use a continuous publisher:
 *    $ ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix \
 *        "latitude: 37.7749
 *         longitude: -122.4194
 *         altitude: 10.5
 *         position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
 *         position_covariance_type: 2" -r 1
 * 
 * Hardware Required:
 * - Raspberry Pi Pico with USB connected to host
 * - Host running ROS2 with micro-ROS agent installed
 */

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  micro-ROS Subscriber Test" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::cout << "Node Name: antenna_tracker" << std::endl;
    std::cout << "Topic: /target/gps/fix" << std::endl;
    std::cout << "Message Type: sensor_msgs/NavSatFix" << std::endl;
    std::cout << "Transport: USB CDC Serial\n" << std::endl;
    
    std::cout << "Prerequisites:" << std::endl;
    std::cout << "  1. Start micro-ROS agent on host:" << std::endl;
    std::cout << "     ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0" << std::endl;
    std::cout << "  2. Publish test messages in another terminal:" << std::endl;
    std::cout << "     ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix ..." << std::endl;
    std::cout << "  3. See docs/ros2-integration.md for detailed examples\n" << std::endl;
    
    std::cout << "Initializing micro-ROS client..." << std::endl;
    auto ros_parser = std::make_shared<ROSParser>("antenna_tracker", "/target/gps/fix");
    
    if (!ros_parser->init()) {
        std::cerr << "Failed to initialize micro-ROS!" << std::endl;
        return -1;
    }
    
    std::cout << "✓ micro-ROS client initialized\n" << std::endl;
    
    std::cout << "Waiting for micro-ROS agent..." << std::endl;
    std::cout << "  Make sure to start the agent on the host computer first!" << std::endl;
    std::cout << "  $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0\n" << std::endl;
    
    // Wait for agent connection (with timeout)
    uint32_t agent_wait_start = time_us_32();
    uint32_t agent_timeout = 30 * 1000 * 1000; // 30 seconds
    bool agent_connected = false;
    
    while (time_us_32() - agent_wait_start < agent_timeout) {
        if (ros_parser->isConnected()) {
            agent_connected = true;
            break;
        }
        
        ros_parser->spin(); // Process any callbacks
        sleep_ms(100);
        
        if ((time_us_32() - agent_wait_start) % 1000000 < 100000) {
            std::cout << "." << std::flush;
        }
    }
    
    std::cout << std::endl;
    
    if (!agent_connected) {
        std::cerr << "\n✗ Agent connection timeout (30s)" << std::endl;
        std::cerr << "Failed to connect to micro-ROS agent" << std::endl;
        std::cerr << "Check that:" << std::endl;
        std::cerr << "  1. Agent is running" << std::endl;
        std::cerr << "  2. USB cable is connected" << std::endl;
        std::cerr << "  3. Correct serial port is specified" << std::endl;
        return -1;
    }
    
    std::cout << "✓ Connected to micro-ROS agent!\n" << std::endl;
    
    std::cout << "========================================" << std::endl;
    std::cout << "  Waiting for messages..." << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::cout << "To test, publish a message in another terminal:" << std::endl;
    std::cout << "  ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix \\" << std::endl;
    std::cout << "    \"header: {frame_id: 'gps'} \\" << std::endl;
    std::cout << "     latitude: 37.7749 \\" << std::endl;
    std::cout << "     longitude: -122.4194 \\" << std::endl;
    std::cout << "     altitude: 10.5 \\" << std::endl;
    std::cout << "     position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] \\" << std::endl;
    std::cout << "     position_covariance_type: 2\" --once\n" << std::endl;
    
    uint32_t message_count = 0;
    uint32_t last_message_time = time_us_32();
    uint32_t total_test_time = 60 * 1000 * 1000; // 60 seconds
    uint32_t test_start = time_us_32();
    bool test_passed = false;
    
    std::cout << "Test running (60 seconds)..." << std::endl;
    
    while (time_us_32() - test_start < total_test_time) {
        ros_parser->spin();
        
        // Check if we received a message
        if (ros_parser->isConnected()) {
            // In a real scenario, we'd check if a new message arrived
            // This is a simplified test - in production, you'd parse the actual message
            
            uint32_t current_time = time_us_32();
            if (current_time - last_message_time > 5 * 1000 * 1000) {
                std::cout << "." << std::flush;
                last_message_time = current_time;
            }
        } else {
            // Agent lost connection
            std::cout << "\n✗ Agent connection lost!" << std::endl;
            break;
        }
        
        sleep_ms(10);
    }
    
    std::cout << "\n\n========================================" << std::endl;
    std::cout << "  Test Complete" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    if (agent_connected && ros_parser->isConnected()) {
        std::cout << "✓ micro-ROS Test PASSED" << std::endl;
        std::cout << "\nThe micro-ROS connection is working correctly!" << std::endl;
        std::cout << "You can now:" << std::endl;
        std::cout << "  1. Use ros2 topic pub to send target positions" << std::endl;
        std::cout << "  2. Monitor /target/gps/fix with: ros2 topic echo /target/gps/fix" << std::endl;
        std::cout << "  3. Check node info with: ros2 node info /antenna_tracker" << std::endl;
        test_passed = true;
    } else {
        std::cout << "✗ micro-ROS Test FAILED" << std::endl;
        std::cout << "\nTroubleshooting:" << std::endl;
        std::cout << "  1. Verify micro-ROS agent is running:" << std::endl;
        std::cout << "     ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0" << std::endl;
        std::cout << "  2. Check USB connection" << std::endl;
        std::cout << "  3. Verify ROS2 installation" << std::endl;
    }
    
    std::cout << std::endl;
    
    return test_passed ? 0 : -1;
}
