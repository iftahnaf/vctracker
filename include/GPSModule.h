#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <cstdint>
#include <string>
#include <vector>

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
 * @brief GPS module interface for UART-based GPS receivers
 * 
 * Supports NMEA 0183 protocol parsing (specifically GPGGA and GPRMC sentences)
 * for extracting position, fix quality, and satellite count.
 */
class GPSModule {
public:
    /**
     * @brief Constructor
     * @param uart_id UART peripheral ID (0 or 1 on Pico)
     * @param tx_pin TX GPIO pin
     * @param rx_pin RX GPIO pin
     * @param baud_rate Baud rate (default 9600 for most GPS modules)
     */
    GPSModule(unsigned int uart_id, unsigned int tx_pin, unsigned int rx_pin, uint32_t baud_rate = 9600);
    
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
     * @brief Update GPS data by reading and parsing UART data
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
    std::string nmea_buffer_;
    
    /**
     * @brief Parse NMEA sentence
     * @param sentence NMEA sentence string
     * @return true if successfully parsed
     */
    bool parseNMEA(const std::string& sentence);
    
    /**
     * @brief Parse GPGGA sentence (Global Positioning System Fix Data)
     * @param fields Array of comma-separated fields
     * @return true if successfully parsed
     */
    bool parseGPGGA(const std::vector<std::string>& fields);
    
    /**
     * @brief Convert NMEA coordinate format to decimal degrees
     * @param nmea_coord NMEA coordinate (DDMM.MMMM or DDDMM.MMMM)
     * @param direction Direction character (N/S/E/W)
     * @return Decimal degrees
     */
    double nmeaToDecimal(const std::string& nmea_coord, char direction);
    
    /**
     * @brief Verify NMEA checksum
     * @param sentence NMEA sentence with checksum
     * @return true if checksum is valid
     */
    bool verifyChecksum(const std::string& sentence);
    
    /**
     * @brief Split string by delimiter
     * @param str String to split
     * @param delimiter Delimiter character
     * @return Vector of substrings
     */
    std::vector<std::string> split(const std::string& str, char delimiter);
};

#endif // GPS_MODULE_H
