#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <cstdint>
#include <cstddef>

/**
 * @struct GPSData
 * @brief GPS position data
 */
struct GPSData {
    double latitude;      // Degrees (-90 to 90)
    double longitude;     // Degrees (-180 to 180)
    double altitude;      // Meters above sea level
    uint8_t fix_quality;  // 0=no fix, 1=GPS fix, 2=DGPS fix
    uint8_t satellites;   // Number of satellites
    bool valid;           // Whether data is valid
    
    GPSData() 
        : latitude(0.0), longitude(0.0), altitude(0.0), 
          fix_quality(0), satellites(0), valid(false) {}
};

/**
 * @class GPSModule
 * @brief GPS module interface for UBX-based receivers (u-blox M10G, etc.)
 * 
 * Parses UBX binary protocol NAV-PVT messages (0x01 0x07) to extract 
 * position, fix quality, and satellite count.
 * Works with M10G modules at their default 38400 baud rate.
 */
class GPSModule {
public:
    /**
     * @brief Constructor
     * @param uart_id UART peripheral ID (0 or 1 on Pico)
     * @param tx_pin TX GPIO pin
     * @param rx_pin RX GPIO pin
     * @param baud_rate Baud rate (default 38400 for M10G)
     */
    GPSModule(unsigned int uart_id, unsigned int tx_pin, unsigned int rx_pin, uint32_t baud_rate = 38400);
    
    /**
     * @brief Destructor
     */
    ~GPSModule();

    /**
     * @brief Initialize the GPS module
     * @return true if successful
     */
    bool init();

    /**
     * @brief Update GPS data by reading and parsing UBX messages
     * Call this regularly in the main loop
     * @return true if new data was parsed
     */
    bool update();

    /**
     * @brief Get the latest GPS data
     * @return GPS data structure
     */
    GPSData getData() const;

    /**
     * @brief Check if GPS has a valid fix
     * @return true if GPS has valid fix
     */
    bool hasFix() const;

    /**
     * @brief Get number of satellites in view
     * @return Satellite count
     */
    uint8_t getSatelliteCount() const;

private:
    unsigned int uart_id_;
    unsigned int tx_pin_;
    unsigned int rx_pin_;
    uint32_t baud_rate_;
    
    GPSData current_data_;
    
    // UBX message buffer (max NAV-PVT payload is 92 bytes + 8 header/checksum)
    static constexpr size_t MAX_UBX_BUFFER = 256;
    uint8_t ubx_buffer_[MAX_UBX_BUFFER];
    size_t ubx_buffer_idx_ = 0;
};

#endif // GPS_MODULE_H
