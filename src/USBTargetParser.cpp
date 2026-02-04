#include "../include/USBTargetParser.h"
#include "pico/stdlib.h"
#include <iostream>

USBTargetParser::USBTargetParser(uint32_t timeout_ms)
    : timeout_ms_(timeout_ms),
      last_update_time_(0),
      target_latitude_(0.0f),
      target_longitude_(0.0f),
      target_altitude_(0.0f),
      has_valid_data_(false) {
}

USBTargetParser::~USBTargetParser() {
}

bool USBTargetParser::init() {
    // Initialize USB stdio for communication
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB to enumerate
    
    std::cout << "USB Target Parser initialized" << std::endl;
    return true;
}

int USBTargetParser::receivePacket(USBTargetData* data) {
    int bytes_read = 0;
    uint8_t* data_ptr = (uint8_t*)data;
    const int packet_size = sizeof(USBTargetData);
    
    // Try to read exactly packet_size bytes with timeout
    while (bytes_read < packet_size) {
        int c = getchar_timeout_us(100000);  // 100ms timeout per byte
        if (c == PICO_ERROR_TIMEOUT) {
            if (bytes_read == 0) {
                return 0;  // No data available
            }
            return -1;  // Incomplete packet
        }
        
        data_ptr[bytes_read] = (uint8_t)c;
        bytes_read++;
    }
    
    return 1;  // Complete packet received
}

bool USBTargetParser::update() {
    USBTargetData target;
    
    int result = receivePacket(&target);
    
    if (result <= 0) {
        return false;  // No data or incomplete packet
    }
    
    // Validate checksum
    if (!usb_validate_target_data(&target)) {
        return false;  // Checksum mismatch
    }
    
    // Validate message type
    if (target.msg_type != USB_MSG_TARGET_DATA) {
        return false;  // Invalid message type
    }
    
    // Store the data
    target_latitude_ = target.latitude;
    target_longitude_ = target.longitude;
    target_altitude_ = target.altitude;
    last_update_time_ = to_ms_since_boot(get_absolute_time());
    has_valid_data_ = true;
    
    // Send ACK
    putchar(USB_MSG_ACK);
    
    return true;
}

bool USBTargetParser::getTargetPosition(float& out_lat, float& out_lon, float& out_alt) const {
    if (!has_valid_data_) {
        return false;
    }
    
    // Check if data has timed out
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_update_time_ > timeout_ms_) {
        return false;
    }
    
    out_lat = target_latitude_;
    out_lon = target_longitude_;
    out_alt = target_altitude_;
    
    return true;
}

bool USBTargetParser::isConnected() const {
    if (!has_valid_data_) {
        return false;
    }
    
    // Consider connected if we've received data within timeout period
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    return (current_time - last_update_time_) <= timeout_ms_;
}

uint32_t USBTargetParser::getTimeSinceLastUpdate() const {
    if (!has_valid_data_) {
        return UINT32_MAX;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    return current_time - last_update_time_;
}
