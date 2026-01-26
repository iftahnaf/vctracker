#include "../include/AntennaTracker.h"
#include <iostream>

AntennaTracker::AntennaTracker(
    std::shared_ptr<Gimbal> gimbal,
    std::shared_ptr<GPSModule> gps,
    std::shared_ptr<ROSParser> ros_parser,
    std::shared_ptr<StatusLED> gps_led,
    std::shared_ptr<StatusLED> ros_led)
    : gimbal_(gimbal),
      gps_(gps),
      ros_parser_(ros_parser),
      gps_led_(gps_led),
      ros_led_(ros_led),
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
    
    // Initialize ROS parser
    if (!ros_parser_->init()) {
        std::cerr << "Failed to initialize ROS parser" << std::endl;
        return false;
    }
    std::cout << "ROS parser initialized" << std::endl;
    
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
    
    if (!ros_led_->init()) {
        std::cerr << "Failed to initialize ROS LED" << std::endl;
        return false;
    }
    std::cout << "Status LEDs initialized" << std::endl;
    
    // Set initial LED states
    gps_led_->setState(LEDState::FAST); // Blinking fast = no fix yet
    ros_led_->setState(LEDState::FAST); // Blinking fast = no data yet
    
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
    
    // Spin micro-ROS executor to process callbacks
    ros_parser_->spin();
    
    // Update status indicators
    updateGPSStatus();
    updateROSStatus();
    
    // Update tracking if enabled
    if (auto_tracking_) {
        updateTracking();
    }
    
    // Update LED animations
    gps_led_->update();
    ros_led_->update();
}

void AntennaTracker::shutdown() {
    if (!initialized_) {
        return;
    }
    
    std::cout << "Shutting down Antenna Tracker..." << std::endl;
    
    // Turn off LEDs
    gps_led_->setState(LEDState::OFF);
    ros_led_->setState(LEDState::OFF);
    
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
    return ros_parser_->getMessage();
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

void AntennaTracker::updateROSStatus() {
    if (ros_parser_->hasMessage()) {
        uint64_t time_since = ros_parser_->timeSinceLastMessage();
        
        if (time_since < 1000) {
            // Recent message (< 1 second) - solid on
            ros_led_->setState(LEDState::ON);
        } else if (time_since < 3000) {
            // Somewhat recent (1-3 seconds) - slow blink
            ros_led_->setState(LEDState::SLOW);
        } else {
            // Old message - fast blink
            ros_led_->setState(LEDState::FAST);
        }
    } else {
        // No message - fast blink
        ros_led_->setState(LEDState::FAST);
    }
}

void AntennaTracker::updateTracking() {
    // Check if we have valid data from both GPS and ROS
    GPSData tracker_pos = gps_->getData();
    NavSatFixMsg target_pos = ros_parser_->getMessage();
    
    if (!tracker_pos.valid || !target_pos.valid) {
        return; // Don't update tracking without valid data
    }
    
    if (!ros_parser_->hasMessage()) {
        return; // ROS data is too old
    }
    
    // Calculate required angles
    PanTiltAngles angles = GeoCalculations::calculateTrackerAngles(
        tracker_pos.latitude, tracker_pos.longitude, tracker_pos.altitude,
        target_pos.latitude, target_pos.longitude, target_pos.altitude
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
