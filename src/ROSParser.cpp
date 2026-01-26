#include "../include/ROSParser.h"
#include "../include/pico_uart_transport.h"
#include "pico/stdlib.h"
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <iostream>

// Static instance for callback
ROSParser* ROSParser::instance_ = nullptr;

ROSParser::ROSParser(const char* node_name, const char* topic_name)
    : node_name_(node_name),
      topic_name_(topic_name),
      timeout_ms_(5000),
      last_message_time_(0),
      initialized_(false),
      connected_(false) {
    
    // Set static instance for callback
    instance_ = this;
    
    // Initialize ROS message
    sensor_msgs__msg__NavSatFix__init(&ros_msg_);
}

ROSParser::~ROSParser() {
    if (initialized_) {
        // Cleanup micro-ROS entities
        rcl_subscription_fini(&subscription_, &node_);
        rcl_node_fini(&node_);
        rclc_executor_fini(&executor_);
        rclc_support_fini(&support_);
    }
    
    sensor_msgs__msg__NavSatFix__fini(&ros_msg_);
    instance_ = nullptr;
}

bool ROSParser::init() {
    if (initialized_) {
        std::cout << "ROSParser already initialized" << std::endl;
        return true;
    }
    
    std::cout << "Initializing micro-ROS..." << std::endl;
    
    // Set up micro-ROS transport (USB serial)
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    
    // Small delay for transport to stabilize
    sleep_ms(100);
    
    // Initialize allocator
    allocator_ = rcl_get_default_allocator();
    
    // Wait for agent connection
    std::cout << "Waiting for micro-ROS agent..." << std::endl;
    const int timeout_sec = 10;
    rmw_ret_t ret = rmw_uros_ping_agent(timeout_sec * 1000, 5);
    
    if (ret != RMW_RET_OK) {
        std::cerr << "Failed to connect to micro-ROS agent" << std::endl;
        return false;
    }
    
    std::cout << "Connected to micro-ROS agent" << std::endl;
    connected_ = true;
    
    // Initialize support
    if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) {
        std::cerr << "Failed to initialize micro-ROS support" << std::endl;
        return false;
    }
    
    // Create node
    if (rclc_node_init_default(&node_, node_name_, "", &support_) != RCL_RET_OK) {
        std::cerr << "Failed to create micro-ROS node" << std::endl;
        return false;
    }
    
    std::cout << "Created node: " << node_name_ << std::endl;
    
    // Create subscriber
    if (rclc_subscription_init_default(
            &subscription_,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
            topic_name_) != RCL_RET_OK) {
        std::cerr << "Failed to create subscriber for topic: " << topic_name_ << std::endl;
        return false;
    }
    
    std::cout << "Created subscriber for topic: " << topic_name_ << std::endl;
    
    // Create executor
    if (rclc_executor_init(&executor_, &support_.context, 1, &allocator_) != RCL_RET_OK) {
        std::cerr << "Failed to create executor" << std::endl;
        return false;
    }
    
    // Add subscription to executor
    if (rclc_executor_add_subscription(
            &executor_,
            &subscription_,
            &ros_msg_,
            &ROSParser::subscriptionCallback,
            ON_NEW_DATA) != RCL_RET_OK) {
        std::cerr << "Failed to add subscription to executor" << std::endl;
        return false;
    }
    
    std::cout << "micro-ROS initialized successfully" << std::endl;
    initialized_ = true;
    
    return true;
}

void ROSParser::spin() {
    if (!initialized_) {
        return;
    }
    
    // Spin executor to process callbacks
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));
    
    // Check for timeout
    if (isTimeout() && current_msg_.valid) {
        current_msg_.valid = false;
    }
    
    // Check connection status periodically
    static uint64_t last_ping_time = 0;
    uint64_t now = time_us_64();
    if (now - last_ping_time > 5000000) { // Check every 5 seconds
        rmw_ret_t ret = rmw_uros_ping_agent(100, 1);
        connected_ = (ret == RMW_RET_OK);
        last_ping_time = now;
    }
}

NavSatFixMsg ROSParser::getMessage() const {
    return current_msg_;
}

bool ROSParser::hasMessage() const {
    return current_msg_.valid && !isTimeout();
}

uint64_t ROSParser::timeSinceLastMessage() const {
    if (last_message_time_ == 0) {
        return UINT64_MAX;
    }
    return (time_us_64() - last_message_time_) / 1000; // Convert to milliseconds
}

void ROSParser::setTimeout(uint32_t timeout_ms) {
    timeout_ms_ = timeout_ms;
}

bool ROSParser::isConnected() const {
    return connected_ && initialized_;
}

void ROSParser::subscriptionCallback(const void* msgin) {
    // Static callback - forward to instance
    if (instance_) {
        const sensor_msgs__msg__NavSatFix* msg = 
            static_cast<const sensor_msgs__msg__NavSatFix*>(msgin);
        instance_->handleMessage(msg);
    }
}

void ROSParser::handleMessage(const sensor_msgs__msg__NavSatFix* msg) {
    // Convert ROS message to internal format
    NavSatFixMsg internal_msg;
    
    internal_msg.latitude = msg->latitude;
    internal_msg.longitude = msg->longitude;
    internal_msg.altitude = msg->altitude;
    internal_msg.status = msg->status.status;
    internal_msg.service = msg->status.service;
    internal_msg.timestamp_sec = msg->header.stamp.sec;
    internal_msg.timestamp_nsec = msg->header.stamp.nanosec;
    internal_msg.valid = true;
    
    // Validate ranges
    if (internal_msg.latitude < -90.0 || internal_msg.latitude > 90.0 ||
        internal_msg.longitude < -180.0 || internal_msg.longitude > 180.0) {
        std::cerr << "Received invalid coordinates" << std::endl;
        return;
    }
    
    current_msg_ = internal_msg;
    last_message_time_ = time_us_64();
    
    // Debug output
    std::cout << "Received NavSatFix: lat=" << internal_msg.latitude 
              << ", lon=" << internal_msg.longitude 
              << ", alt=" << internal_msg.altitude << std::endl;
}

bool ROSParser::isTimeout() const {
    if (last_message_time_ == 0) {
        return true;
    }
    
    uint64_t elapsed_ms = (time_us_64() - last_message_time_) / 1000;
    return elapsed_ms > timeout_ms_;
}
