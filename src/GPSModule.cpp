#include "../include/GPSModule.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <cstring>
#include <cstdlib>

/**
 * UBX NAV-PVT message structure (0x01 0x07)
 * Size: 92 bytes payload
 */
struct UBXNavPVT {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t numSV;
    int32_t lon, lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc, vAcc;
    int32_t velN, velE, velD;
    int32_t gSpeed;
    int32_t heading;
};

static void ubx_checksum(const uint8_t* data, size_t len, uint8_t* ck_a, uint8_t* ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

static bool parse_ubx_navpvt(const uint8_t* payload, size_t len, UBXNavPVT* pvt) {
    if (len < 92) return false;
    
    size_t offset = 0;
    pvt->iTOW = *(uint32_t*)(payload + offset); offset += 4;
    pvt->year = *(uint16_t*)(payload + offset); offset += 2;
    pvt->month = payload[offset++];
    pvt->day = payload[offset++];
    pvt->hour = payload[offset++];
    pvt->min = payload[offset++];
    pvt->sec = payload[offset++];
    pvt->valid = payload[offset++];
    pvt->tAcc = *(uint32_t*)(payload + offset); offset += 4;
    pvt->nano = *(int32_t*)(payload + offset); offset += 4;
    pvt->fixType = payload[offset++];
    pvt->flags = payload[offset++];
    offset++;
    pvt->numSV = payload[offset++];
    pvt->lon = *(int32_t*)(payload + offset); offset += 4;
    pvt->lat = *(int32_t*)(payload + offset); offset += 4;
    pvt->height = *(int32_t*)(payload + offset); offset += 4;
    pvt->hMSL = *(int32_t*)(payload + offset); offset += 4;
    pvt->hAcc = *(uint32_t*)(payload + offset); offset += 4;
    pvt->vAcc = *(uint32_t*)(payload + offset); offset += 4;
    
    return true;
}

GPSModule::GPSModule(unsigned int uart_id, unsigned int tx_pin, unsigned int rx_pin, uint32_t baud_rate)
    : uart_id_(uart_id),
      tx_pin_(tx_pin),
      rx_pin_(rx_pin),
      baud_rate_(baud_rate) {
}

GPSModule::~GPSModule() {
    // UART cleanup handled by pico-sdk
}

bool GPSModule::init() {
    // Get UART instance
    uart_inst_t* uart = (uart_id_ == 0) ? uart0 : uart1;
    
    // Initialize UART
    uart_init(uart, baud_rate_);
    
    // Set GPIO functions
    gpio_set_function(tx_pin_, GPIO_FUNC_UART);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);
    
    // Set UART format: 8 data bits, 1 stop bit, no parity
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    
    // Enable UART FIFOs
    uart_set_fifo_enabled(uart, true);
    
    return true;
}

bool GPSModule::update() {
    uart_inst_t* uart = (uart_id_ == 0) ? uart0 : uart1;
    
    bool new_data = false;
    
    // Read available bytes and look for UBX messages
    while (uart_is_readable(uart)) {
        uint8_t c = uart_getc(uart);
        ubx_buffer_[ubx_buffer_idx_] = c;
        ubx_buffer_idx_++;
        
        // Look for UBX sync sequence
        if (ubx_buffer_idx_ >= 2) {
            if (ubx_buffer_[ubx_buffer_idx_ - 2] == 0xB5 && ubx_buffer_[ubx_buffer_idx_ - 1] == 0x62) {
                // Found sync, reset to start of message
                ubx_buffer_[0] = 0xB5;
                ubx_buffer_[1] = 0x62;
                ubx_buffer_idx_ = 2;
                continue;
            }
        }
        
        // Need at least header + length to know payload size
        if (ubx_buffer_idx_ >= 6) {
            uint16_t payload_len = ubx_buffer_[4] | (ubx_buffer_[5] << 8);
            uint16_t total_len = payload_len + 8;  // header(2) + class/id(2) + length(2) + payload + checksum(2)
            
            if (ubx_buffer_idx_ >= total_len) {
                // Have complete message, verify checksum
                uint8_t ck_a, ck_b;
                ubx_checksum(ubx_buffer_ + 2, payload_len + 4, &ck_a, &ck_b);
                
                if (ck_a == ubx_buffer_[total_len - 2] && ck_b == ubx_buffer_[total_len - 1]) {
                    // Valid message
                    uint8_t msg_class = ubx_buffer_[2];
                    uint8_t msg_id = ubx_buffer_[3];
                    
                    // NAV-PVT = class 0x01, id 0x07
                    if (msg_class == 0x01 && msg_id == 0x07) {
                        UBXNavPVT pvt;
                        if (parse_ubx_navpvt(ubx_buffer_ + 6, payload_len, &pvt)) {
                            current_data_.latitude = pvt.lat / 1e7;
                            current_data_.longitude = pvt.lon / 1e7;
                            current_data_.altitude = pvt.hMSL / 1000.0;
                            current_data_.satellites = pvt.numSV;
                            current_data_.fix_quality = (pvt.fixType > 0) ? pvt.fixType : 0;
                            current_data_.valid = (pvt.fixType > 0);
                            new_data = true;
                        }
                    }
                }
                
                // Reset for next message
                ubx_buffer_idx_ = 0;
            }
        }
        
        // Safety: reset if buffer gets too full
        if (ubx_buffer_idx_ >= sizeof(ubx_buffer_)) {
            ubx_buffer_idx_ = 0;
        }
    }
    
    return new_data;
}

GPSData GPSModule::getData() const {
    return current_data_;
}

bool GPSModule::hasFix() const {
    return current_data_.valid && current_data_.fix_quality > 0;
}

uint8_t GPSModule::getSatelliteCount() const {
    return current_data_.satellites;
}
