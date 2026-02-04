#ifndef USB_PROTOCOL_H
#define USB_PROTOCOL_H

#include <stdint.h>

// Simple struct for target position (lat, lon, alt)
typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // 0x01 = TARGET_DATA
    float latitude;         // Target latitude in degrees
    float longitude;        // Target longitude in degrees
    float altitude;         // Target altitude in meters
    uint8_t checksum;       // Simple XOR checksum
} USBTargetData;

// Message types
#define USB_MSG_TARGET_DATA 0x01
#define USB_MSG_ACK         0x02
#define USB_MSG_ERROR       0x03

// Calculate XOR checksum for data
static inline uint8_t usb_calculate_checksum(const uint8_t* data, uint16_t len) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Validate incoming data
static inline int usb_validate_target_data(const USBTargetData* data) {
    uint8_t calc_checksum = usb_calculate_checksum((uint8_t*)data, sizeof(USBTargetData) - 1);
    return calc_checksum == data->checksum;
}

#endif
