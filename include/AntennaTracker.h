#ifndef ANTENNA_TRACKER_H
#define ANTENNA_TRACKER_H

#include "../vcgimbal/include/Gimbal.h"
#include "GPSModule.h"
#include "ROSParser.h"
#include "GeoCalculations.h"
#include "StatusLED.h"
#include <memory>

/**
 * @class AntennaTracker
 * @brief Main antenna tracker controller
 * 
 * Integrates GPS module, ROS2 parser, gimbal controller, and status LEDs
 * to create a complete antenna tracking system that points toward a moving target.
 * 
 * Features:
 * - Onboard GPS for tracker position
 * - ROS2 NavSatFix message reception for target position
 * - Automatic pan/tilt calculation and gimbal control
 * - Visual status indicators (GPS fix, ROS data)
 */
class AntennaTracker {
public:
    /**
     * @brief Constructor
     * @param gimbal Shared pointer to initialized Gimbal controller
     * @param gps Shared pointer to GPS module
     * @param ros_parser Shared pointer to ROS parser
     * @param gps_led Shared pointer to GPS status LED
     * @param ros_led Shared pointer to ROS status LED
     */
    AntennaTracker(
        std::shared_ptr<Gimbal> gimbal,
        std::shared_ptr<GPSModule> gps,
        std::shared_ptr<ROSParser> ros_parser,
        std::shared_ptr<StatusLED> gps_led,
        std::shared_ptr<StatusLED> ros_led
    );
    
    /**
     * @brief Destructor
     */
    ~AntennaTracker();

    /**
     * @brief Initialize the antenna tracker
     * @return true if successful
     */
    bool init();

    /**
     * @brief Main update loop
     * Call this regularly to update GPS, ROS data, and gimbal
     */
    void update();

    /**
     * @brief Shutdown the tracker
     */
    void shutdown();

    /**
     * @brief Check if tracker is initialized
     * @return true if initialized
     */
    bool isInitialized() const;

    /**
     * @brief Get current tracker GPS position
     * @return GPS data
     */
    GPSData getTrackerPosition() const;

    /**
     * @brief Get current target position
     * @return NavSatFix message data
     */
    NavSatFixMsg getTargetPosition() const;

    /**
     * @brief Get current gimbal angles
     * @return Pan/tilt angles
     */
    PanTiltAngles getCurrentAngles() const;

    /**
     * @brief Enable/disable auto-tracking
     * @param enable true to enable tracking
     */
    void setAutoTracking(bool enable);

    /**
     * @brief Check if auto-tracking is enabled
     * @return true if enabled
     */
    bool isAutoTracking() const;

    /**
     * @brief Manually set gimbal angles (overrides auto-tracking temporarily)
     * @param pan Pan angle (-90 to 90 degrees)
     * @param tilt Tilt angle (-90 to 90 degrees)
     * @return true if successful
     */
    bool setManualAngles(float pan, float tilt);

    /**
     * @brief Set minimum elevation angle (prevent pointing below horizon)
     * @param min_elevation Minimum elevation in degrees
     */
    void setMinElevation(float min_elevation);

private:
    std::shared_ptr<Gimbal> gimbal_;
    std::shared_ptr<GPSModule> gps_;
    std::shared_ptr<ROSParser> ros_parser_;
    std::shared_ptr<StatusLED> gps_led_;
    std::shared_ptr<StatusLED> ros_led_;
    
    bool initialized_;
    bool auto_tracking_;
    float min_elevation_;
    
    PanTiltAngles current_angles_;
    uint64_t last_update_time_;
    
    /**
     * @brief Update GPS status and LED
     */
    void updateGPSStatus();
    
    /**
     * @brief Update ROS message status and LED
     */
    void updateROSStatus();
    
    /**
     * @brief Update gimbal tracking
     * Calculates and applies new angles if both GPS and ROS data are valid
     */
    void updateTracking();
    
    /**
     * @brief Apply safety limits to angles
     * @param angles Input angles
     * @return Limited angles
     */
    PanTiltAngles applySafetyLimits(const PanTiltAngles& angles);
};

#endif // ANTENNA_TRACKER_H
