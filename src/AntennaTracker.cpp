#include "../include/AntennaTracker.h"
#include <iostream>
#include "pico/time.h"

AntennaTracker::AntennaTracker(
    std::shared_ptr<Gimbal> gimbal,
    std::shared_ptr<GPSModule> gps,
    std::shared_ptr<USBTargetParser> usb_parser,
    std::shared_ptr<StatusLED> gps_led,
    std::shared_ptr<StatusLED> target_led)
    : gimbal_(gimbal),
      gps_(gps),
      usb_parser_(usb_parser),
      gps_led_(gps_led),
      target_led_(target_led),
      initialized_(false),
      auto_tracking_(true),
      min_elevation_(-10.0f),
      last_update_time_(0) {
}

AntennaTracker::~AntennaTracker() {
    if (initialized_) {
        shutdown();
    }
}

bool AntennaTracker::init() {
    if (initialized_) {
        std::cout << "AntennaTracker already initialized" << std::endl;
        return true;
    }
    
    std::cout << "Initializing Antenna Tracker..." << std::endl;
    
    // Initialize GPS module
    if (!gps_->init()) {
        std::cerr << "Failed to initialize GPS module" << std::endl;
        return false;
    }
    std::cout << "GPS module initialized" << std::endl;
    
    // Initialize USB target parser
    if (!usb_parser_->init()) {
        std::cerr << "Failed to initialize USB target parser" << std::endl;
        return false;
    }
    std::cout << "USB target parser initialized" << std::endl;
    
    // Initialize gimbal
    if (!gimbal_->init()) {
        std::cerr << "Failed to initialize gimbal" << std::endl;
        return false;
    }
    std::cout << "Gimbal initialized" << std::endl;
    
    // Initialize status LEDs
    if (!gps_led_->init()) {
        std::cerr << "Failed to initialize GPS LED" << std::endl;
        return false;
    }
    
    if (!target_led_->init()) {
        std::cerr << "Failed to initialize Target LED" << std::endl;
        return false;
    }
    std::cout << "Status LEDs initialized" << std::endl;
    
    // Set initial LED states
    gps_led_->setState(LEDState::FAST); // Blinking fast = no fix yet
    target_led_->setState(LEDState::FAST); // Blinking fast = no data yet
    
    initialized_ = true;
    std::cout << "Antenna Tracker initialized successfully" << std::endl;
    
    return true;
}

void AntennaTracker::update() {
    if (!initialized_) {
        return;
    }
    
    // Update GPS data
    gps_->update();
    
    // Update USB target parser to receive new data
    usb_parser_->update();
    
    // Update status indicators
    updateGPSStatus();
    updateTargetStatus();
    
    // Update tracking if enabled
    if (auto_tracking_) {
        updateTracking();
    }
    
    // Update LED animations
    gps_led_->update();
    target_led_->update();
}

void AntennaTracker::shutdown() {
    if (!initialized_) {
        return;
    }
    
    std::cout << "Shutting down Antenna Tracker..." << std::endl;
    
    // Turn off LEDs
    gps_led_->setState(LEDState::OFF);
    target_led_->setState(LEDState::OFF);
    
    // Shutdown gimbal (centers servos)
    gimbal_->shutdown();
    
    initialized_ = false;
    std::cout << "Antenna Tracker shut down" << std::endl;
}

bool AntennaTracker::isInitialized() const {
    return initialized_;
}

GPSData AntennaTracker::getTrackerPosition() const {
    return gps_->getData();
}

NavSatFixMsg AntennaTracker::getTargetPosition() const {
    float lat, lon, alt;
    NavSatFixMsg msg;
    
    if (usb_parser_->getTargetPosition(lat, lon, alt)) {
        msg.latitude = lat;
        msg.longitude = lon;
        msg.altitude = alt;
        msg.status = 0;  // GPS fix
        msg.valid = true;
    } else {
        msg.valid = false;
    }
    
    return msg;
}

PanTiltAngles AntennaTracker::getCurrentAngles() const {
    return current_angles_;
}

void AntennaTracker::setAutoTracking(bool enable) {
    auto_tracking_ = enable;
    std::cout << "Auto-tracking " << (enable ? "enabled" : "disabled") << std::endl;
}

bool AntennaTracker::isAutoTracking() const {
    return auto_tracking_;
}

bool AntennaTracker::setManualAngles(float pan, float tilt) {
    if (!initialized_) {
        return false;
    }
    
    PanTiltAngles angles(pan, tilt);
    angles = applySafetyLimits(angles);
    
    bool success = gimbal_->setTipAngle(angles.pan, angles.tilt);
    if (success) {
        current_angles_ = angles;
    }
    
    return success;
}

void AntennaTracker::setMinElevation(float min_elevation) {
    min_elevation_ = min_elevation;
    std::cout << "Minimum elevation set to " << min_elevation << " degrees" << std::endl;
}

void AntennaTracker::updateGPSStatus() {
    GPSData gps_data = gps_->getData();
    
    if (!gps_data.valid || gps_data.fix_quality == 0) {
        // No fix - fast blink
        gps_led_->setState(LEDState::FAST);
    } else if (gps_data.fix_quality == 1) {
        // GPS fix - slow blink
        gps_led_->setState(LEDState::SLOW);
    } else {
        // DGPS fix or better - solid on
        gps_led_->setState(LEDState::ON);
    }
}

void AntennaTracker::updateTargetStatus() {
    if (usb_parser_->isConnected()) {
        uint32_t time_since = usb_parser_->getTimeSinceLastUpdate();
        
        if (time_since < 1000) {
            // Recent data (< 1 second) - solid on
            target_led_->setState(LEDState::ON);
        } else if (time_since < 3000) {
            // Somewhat recent (1-3 seconds) - slow blink
            target_led_->setState(LEDState::SLOW);
        } else {
            // Old data - fast blink
            target_led_->setState(LEDState::FAST);
        }
    } else {
        // No target data - fast blink
        target_led_->setState(LEDState::FAST);
    }
}

void AntennaTracker::updateTracking() {
    // Check if we have valid data from both GPS and USB target
    GPSData tracker_pos = gps_->getData();
    float target_lat, target_lon, target_alt;
    
    if (!tracker_pos.valid) {
        return; // Don't update tracking without valid tracker GPS
    }
    
    if (!usb_parser_->getTargetPosition(target_lat, target_lon, target_alt)) {
        return; // No valid target data available
    }
    
    // Calculate required angles
    PanTiltAngles angles = GeoCalculations::calculateTrackerAngles(
        tracker_pos.latitude, tracker_pos.longitude, tracker_pos.altitude,
        target_lat, target_lon, target_alt
    );
    
    // Apply safety limits
    angles = applySafetyLimits(angles);
    
    // Apply angles to gimbal
    if (gimbal_->setTipAngle(angles.pan, angles.tilt)) {
        current_angles_ = angles;
        
        // Log tracking update (throttled to avoid spam)
        uint64_t current_time = time_us_64();
        if (current_time - last_update_time_ > 1000000) { // Once per second
            std::cout << "Tracking: Pan=" << angles.pan 
                      << "°, Tilt=" << angles.tilt << "°" << std::endl;
            last_update_time_ = current_time;
        }
    }
}

PanTiltAngles AntennaTracker::applySafetyLimits(const PanTiltAngles& angles) {
    PanTiltAngles limited = angles;
    
    // Apply minimum elevation limit
    if (limited.tilt < min_elevation_) {
        limited.tilt = min_elevation_;
    }
    
    // Servo limits are already applied in GeoCalculations,
    // but we can add additional safety margins here if needed
    
    // Ensure within servo range [-90, 90]
    if (limited.pan > 90.0f) limited.pan = 90.0f;
    if (limited.pan < -90.0f) limited.pan = -90.0f;
    if (limited.tilt > 90.0f) limited.tilt = 90.0f;
    if (limited.tilt < -90.0f) limited.tilt = -90.0f;
    
    return limited;
}
