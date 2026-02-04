#ifndef USB_TARGET_PARSER_H
#define USB_TARGET_PARSER_H

#include "USBProtocol.h"
#include <cstdint>
#include <ctime>

/**
 * @brief USB Serial Target Position Parser
 * 
 * Receives target position updates over USB using simple binary protocol.
 * No ROS dependencies - just USB CDC serial communication.
 */
class USBTargetParser {
public:
    USBTargetParser(uint32_t timeout_ms = 5000);
    ~USBTargetParser();
    
    /**
     * @brief Initialize USB serial interface
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Update - call this regularly to process incoming USB data
     * @return true if new target data received
     */
    bool update();
    
    /**
     * @brief Get latest target position
     * @param out_lat Latitude in degrees
     * @param out_lon Longitude in degrees
     * @param out_alt Altitude in meters
     * @return true if data is valid (not timed out)
     */
    bool getTargetPosition(float& out_lat, float& out_lon, float& out_alt) const;
    
    /**
     * @brief Check if we're receiving valid target data
     * @return true if data is recent and valid
     */
    bool isConnected() const;
    
    /**
     * @brief Get time since last valid target data received (milliseconds)
     */
    uint32_t getTimeSinceLastUpdate() const;
    
private:
    uint32_t timeout_ms_;           // Timeout for considering connection lost
    uint32_t last_update_time_;     // Timestamp of last valid data
    
    float target_latitude_;         // Latest target latitude
    float target_longitude_;        // Latest target longitude
    float target_altitude_;         // Latest target altitude
    
    bool has_valid_data_;           // Whether we've ever received valid data
    
    /**
     * @brief Try to read a USB target packet
     * @return 1 if packet received, 0 if no data, -1 if error
     */
    int receivePacket(USBTargetData* data);
};

#endif // USB_TARGET_PARSER_H
