#include "../include/GPSModule.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <cstring>
#include <cstdlib>
#include <vector>
#include <sstream>

GPSModule::GPSModule(uint uart_id, uint tx_pin, uint rx_pin, uint32_t baud_rate)
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
    
    // Read available characters
    while (uart_is_readable(uart)) {
        char c = uart_getc(uart);
        
        if (c == '\n' || c == '\r') {
            if (nmea_buffer_.length() > 0) {
                // Process complete NMEA sentence
                if (parseNMEA(nmea_buffer_)) {
                    new_data = true;
                }
                nmea_buffer_.clear();
            }
        } else if (c == '$') {
            // Start of new NMEA sentence
            nmea_buffer_.clear();
            nmea_buffer_ += c;
        } else {
            // Accumulate sentence
            if (nmea_buffer_.length() < 200) { // Limit buffer size
                nmea_buffer_ += c;
            }
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

bool GPSModule::parseNMEA(const std::string& sentence) {
    // Verify checksum
    if (!verifyChecksum(sentence)) {
        return false;
    }
    
    // Split sentence into fields
    std::vector<std::string> fields = split(sentence, ',');
    
    if (fields.empty()) {
        return false;
    }
    
    // Check sentence type
    std::string sentence_type = fields[0];
    
    if (sentence_type == "$GPGGA" || sentence_type == "$GNGGA") {
        return parseGPGGA(fields);
    }
    
    return false;
}

bool GPSModule::parseGPGGA(const std::vector<std::string>& fields) {
    // GPGGA format:
    // 0: $GPGGA
    // 1: Time (hhmmss.ss)
    // 2: Latitude (ddmm.mmmm)
    // 3: N/S
    // 4: Longitude (dddmm.mmmm)
    // 5: E/W
    // 6: Fix quality (0=no fix, 1=GPS, 2=DGPS)
    // 7: Number of satellites
    // 8: HDOP
    // 9: Altitude
    // 10: M (meters)
    // ... checksum
    
    if (fields.size() < 11) {
        return false;
    }
    
    try {
        // Parse latitude
        if (!fields[2].empty() && !fields[3].empty()) {
            current_data_.latitude = nmeaToDecimal(fields[2], fields[3][0]);
        }
        
        // Parse longitude
        if (!fields[4].empty() && !fields[5].empty()) {
            current_data_.longitude = nmeaToDecimal(fields[4], fields[5][0]);
        }
        
        // Parse fix quality
        if (!fields[6].empty()) {
            current_data_.fix_quality = std::atoi(fields[6].c_str());
        }
        
        // Parse satellite count
        if (!fields[7].empty()) {
            current_data_.satellites = std::atoi(fields[7].c_str());
        }
        
        // Parse altitude
        if (!fields[9].empty()) {
            current_data_.altitude = std::atof(fields[9].c_str());
        }
        
        // Mark as valid if we have a fix
        current_data_.valid = (current_data_.fix_quality > 0);
        
        return true;
        
    } catch (...) {
        return false;
    }
}

double GPSModule::nmeaToDecimal(const std::string& nmea_coord, char direction) {
    if (nmea_coord.empty()) {
        return 0.0;
    }
    
    // Find decimal point
    size_t dot_pos = nmea_coord.find('.');
    if (dot_pos == std::string::npos) {
        return 0.0;
    }
    
    // Degrees are before last 2 digits of integer part
    // e.g., "4807.038" = 48 degrees, 07.038 minutes
    std::string degrees_str;
    std::string minutes_str;
    
    if (dot_pos >= 2) {
        degrees_str = nmea_coord.substr(0, dot_pos - 2);
        minutes_str = nmea_coord.substr(dot_pos - 2);
    } else {
        return 0.0;
    }
    
    double degrees = std::atof(degrees_str.c_str());
    double minutes = std::atof(minutes_str.c_str());
    
    double decimal = degrees + (minutes / 60.0);
    
    // Apply direction
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

bool GPSModule::verifyChecksum(const std::string& sentence) {
    // Find checksum separator
    size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos) {
        return false; // No checksum
    }
    
    // Calculate checksum (XOR of all characters between $ and *)
    uint8_t calc_checksum = 0;
    for (size_t i = 1; i < star_pos; i++) {
        calc_checksum ^= sentence[i];
    }
    
    // Extract provided checksum
    if (star_pos + 2 >= sentence.length()) {
        return false;
    }
    
    std::string checksum_str = sentence.substr(star_pos + 1, 2);
    uint8_t provided_checksum = std::strtol(checksum_str.c_str(), nullptr, 16);
    
    return (calc_checksum == provided_checksum);
}

std::vector<std::string> GPSModule::split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream token_stream(str);
    
    while (std::getline(token_stream, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}
