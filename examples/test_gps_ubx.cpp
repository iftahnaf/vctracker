#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <iostream>
#include <cstring>

/**
 * @brief Parse UBX binary protocol from GPS
 * 
 * The M10G outputs UBX by default. Parse NAV-PVT messages directly.
 */

constexpr uint GPS_UART_ID = 0;
constexpr uint GPS_TX_PIN = 0;
constexpr uint GPS_RX_PIN = 1;
constexpr uint32_t GPS_BAUD = 38400;

struct UBXNavPVT {
    uint32_t iTOW;        // GPS time of week
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    uint8_t valid;        // Validity flags
    uint32_t tAcc;        // Time accuracy
    int32_t nano;         // Nanoseconds
    uint8_t fixType;      // 0=no fix, 1=2D, 2=3D, 3=GNSS+RTK
    uint8_t flags;
    uint8_t numSV;        // Number of satellites
    int32_t lon, lat;     // Degrees * 1e-7
    int32_t height;       // Height above ellipsoid (mm)
    int32_t hMSL;         // Height above sea level (mm)
    uint32_t hAcc, vAcc;  // Accuracy
    int32_t velN, velE, velD;  // NED velocity (mm/s)
    int32_t gSpeed;       // Ground speed (mm/s)
    int32_t heading;      // Heading (degrees * 1e-5)
};

uart_inst_t* get_uart(uint uart_id) {
    return uart_id == 0 ? uart0 : uart1;
}

void ubx_checksum(const uint8_t* data, size_t len, uint8_t* ck_a, uint8_t* ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

bool parse_ubx_navpvt(const uint8_t* payload, size_t len, UBXNavPVT* pvt) {
    if (len < 92) return false;  // NAV-PVT minimum payload size
    
    size_t offset = 0;
    
    // Read fields (little endian)
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
    offset++;  // reserved
    pvt->numSV = payload[offset++];
    pvt->lon = *(int32_t*)(payload + offset); offset += 4;
    pvt->lat = *(int32_t*)(payload + offset); offset += 4;
    pvt->height = *(int32_t*)(payload + offset); offset += 4;
    pvt->hMSL = *(int32_t*)(payload + offset); offset += 4;
    pvt->hAcc = *(uint32_t*)(payload + offset); offset += 4;
    pvt->vAcc = *(uint32_t*)(payload + offset); offset += 4;
    
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  GPS UBX Protocol Parser Test" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    uart_inst_t* uart = get_uart(GPS_UART_ID);
    uart_init(uart, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart, true);
    
    std::cout << "Listening for UBX NAV-PVT messages..." << std::endl;
    std::cout << "Waiting for GPS fix...\n" << std::endl;
    
    uint8_t buffer[256];
    size_t buffer_idx = 0;
    uint32_t msg_count = 0;
    uint32_t start_time = time_us_32();
    
    while (time_us_32() - start_time < 60000000) {  // 60 seconds
        if (uart_is_readable(uart)) {
            uint8_t c = uart_getc(uart);
            
            // Look for UBX sync sequence
            if (buffer_idx == 0 && c == 0xB5) {
                buffer[buffer_idx++] = c;
            } else if (buffer_idx == 1 && c == 0x62) {
                buffer[buffer_idx++] = c;
            } else if (buffer_idx >= 2) {
                buffer[buffer_idx++] = c;
                
                // Check if we have header + length
                if (buffer_idx >= 6) {
                    uint16_t payload_len = buffer[4] | (buffer[5] << 8);
                    uint16_t total_len = payload_len + 8;  // header + payload + checksum
                    
                    if (buffer_idx >= total_len) {
                        // Verify checksum
                        uint8_t ck_a, ck_b;
                        ubx_checksum(buffer + 2, payload_len + 4, &ck_a, &ck_b);
                        
                        uint8_t msg_ck_a = buffer[total_len - 2];
                        uint8_t msg_ck_b = buffer[total_len - 1];
                        
                        if (ck_a == msg_ck_a && ck_b == msg_ck_b) {
                            uint8_t msg_class = buffer[2];
                            uint8_t msg_id = buffer[3];
                            
                            // NAV-PVT = class 0x01, id 0x07
                            if (msg_class == 0x01 && msg_id == 0x07) {
                                UBXNavPVT pvt;
                                if (parse_ubx_navpvt(buffer + 6, payload_len, &pvt)) {
                                    msg_count++;
                                    
                                    if (pvt.fixType > 0) {
                                        double lat = pvt.lat / 1e7;
                                        double lon = pvt.lon / 1e7;
                                        double alt = pvt.hMSL / 1000.0;
                                        
                                        std::cout << "Fix #" << msg_count << ":" << std::endl;
                                        std::cout << "  Lat: " << lat << "°" << std::endl;
                                        std::cout << "  Lon: " << lon << "°" << std::endl;
                                        std::cout << "  Alt: " << alt << " m" << std::endl;
                                        std::cout << "  Sats: " << (int)pvt.numSV << std::endl;
                                        std::cout << "  Fix: ";
                                        
                                        switch (pvt.fixType) {
                                            case 2: std::cout << "2D"; break;
                                            case 3: std::cout << "3D"; break;
                                            default: std::cout << "No fix";
                                        }
                                        std::cout << "\n" << std::endl;
                                    } else if (msg_count % 10 == 0) {
                                        std::cout << "." << std::flush;
                                    }
                                }
                            }
                        }
                        
                        buffer_idx = 0;  // Start looking for next message
                    }
                }
                
                if (buffer_idx >= sizeof(buffer)) {
                    buffer_idx = 0;  // Safety: reset if buffer full
                }
            } else {
                buffer_idx = 0;  // Invalid sequence
            }
        }
        
        sleep_ms(1);
    }
    
    std::cout << "\n\n========================================" << std::endl;
    std::cout << "  Test Complete" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    if (msg_count > 0) {
        std::cout << "✓ SUCCESS! Parsed " << msg_count << " UBX messages" << std::endl;
    } else {
        std::cout << "✗ No valid UBX messages found" << std::endl;
    }
    
    return msg_count > 0 ? 0 : -1;
}
