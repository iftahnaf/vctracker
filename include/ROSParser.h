#ifndef ROS_PARSER_H
#define ROS_PARSER_H

#include <cstdint>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>

/**
 * @struct NavSatFixMsg
 * @brief ROS2 sensor_msgs/NavSatFix message data
 */
struct NavSatFixMsg {
    double latitude;         // Degrees
    double longitude;        // Degrees
    double altitude;         // Meters above WGS84 ellipsoid
    int8_t status;           // -1=no fix, 0=fix, 1=SBAS fix, 2=GBAS fix
    uint16_t service;        // GPS service type
    uint64_t timestamp_sec;  // Timestamp seconds
    uint32_t timestamp_nsec; // Timestamp nanoseconds
    bool valid;              // Whether message is valid
    
    NavSatFixMsg() 
        : latitude(0.0), longitude(0.0), altitude(0.0), 
          status(-1), service(0), timestamp_sec(0), timestamp_nsec(0), valid(false) {}
};

/**
 * @class ROSParser
 * @brief micro-ROS subscriber for ROS2 sensor_msgs/NavSatFix messages
 * 
 * Receives genuine ROS2 NavSatFix messages via micro-ROS over USB serial.
 * Communicates with ROS2 host using micro-ROS agent.
 */
class ROSParser {
public:
    /**
     * @brief Constructor
     * @param node_name Name for micro-ROS node
     * @param topic_name Topic to subscribe to (default: "/target/gps/fix")
     */
    ROSParser(const char* node_name = "antenna_tracker", 
              const char* topic_name = "/target/gps/fix");
    
    /**
     * @brief Destructor
     */
    ~ROSParser();

    /**
     * @brief Initialize micro-ROS and create subscriber
     * @return true if successful
     */
    bool init();

    /**
     * @brief Spin executor to process callbacks
     * Call this regularly in the main loop
     */
    void spin();

    /**
     * @brief Get the latest NavSatFix message
     * @return NavSatFix message data
     */
    NavSatFixMsg getMessage() const;

    /**
     * @brief Check if a message has been received
     * @return true if message is valid and recent
     */
    bool hasMessage() const;

    /**
     * @brief Get time since last message (milliseconds)
     * @return Time in milliseconds
     */
    uint64_t timeSinceLastMessage() const;

    /**
     * @brief Set message timeout (milliseconds)
     * @param timeout_ms Timeout in milliseconds
     */
    void setTimeout(uint32_t timeout_ms);

    /**
     * @brief Check if micro-ROS is connected to agent
     * @return true if connected
     */
    bool isConnected() const;

private:
    // micro-ROS entities
    rcl_node_t node_;
    rcl_subscription_t subscription_;
    rclc_executor_t executor_;
    rclc_support_t support_;
    rcl_allocator_t allocator_;
    sensor_msgs__msg__NavSatFix ros_msg_;
    
    // Configuration
    const char* node_name_;
    const char* topic_name_;
    uint32_t timeout_ms_;
    
    // State
    NavSatFixMsg current_msg_;
    uint64_t last_message_time_;
    bool initialized_;
    bool connected_;
    
    /**
     * @brief Subscription callback (static wrapper)
     * @param msg Received ROS message
     */
    static void subscriptionCallback(const void* msg);
    
    /**
     * @brief Handle received NavSatFix message
     * @param msg Received ROS message
     */
    void handleMessage(const sensor_msgs__msg__NavSatFix* msg);
    
    /**
     * @brief Check if message has timed out
     * @return true if message is too old
     */
    bool isTimeout() const;
    
    // Static instance for callback access
    static ROSParser* instance_;
};

#endif // ROS_PARSER_H
