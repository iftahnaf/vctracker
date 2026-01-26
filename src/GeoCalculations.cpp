#include "../include/GeoCalculations.h"

double GeoCalculations::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Haversine formula
    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);
    
    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
               std::cos(toRadians(lat1)) * std::cos(toRadians(lat2)) *
               std::sin(dLon / 2.0) * std::sin(dLon / 2.0);
    
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    
    return EARTH_RADIUS * c;
}

double GeoCalculations::calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = toRadians(lat1);
    double lat2_rad = toRadians(lat2);
    double dLon = toRadians(lon2 - lon1);
    
    double y = std::sin(dLon) * std::cos(lat2_rad);
    double x = std::cos(lat1_rad) * std::sin(lat2_rad) -
               std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dLon);
    
    double bearing = toDegrees(std::atan2(y, x));
    
    // Normalize to 0-360
    bearing = std::fmod(bearing + 360.0, 360.0);
    
    return bearing;
}

double GeoCalculations::calculateElevation(double distance, double altitude_diff) {
    if (distance == 0.0) {
        return altitude_diff > 0 ? 90.0 : (altitude_diff < 0 ? -90.0 : 0.0);
    }
    
    double elevation = toDegrees(std::atan2(altitude_diff, distance));
    
    return elevation;
}

PanTiltAngles GeoCalculations::calculateTrackerAngles(
    double tracker_lat, double tracker_lon, double tracker_alt,
    double target_lat, double target_lon, double target_alt) {
    
    // Calculate horizontal distance
    double distance = calculateDistance(tracker_lat, tracker_lon, target_lat, target_lon);
    
    // Calculate bearing (azimuth/pan angle)
    double bearing = calculateBearing(tracker_lat, tracker_lon, target_lat, target_lon);
    
    // Calculate elevation (tilt angle)
    double altitude_diff = target_alt - tracker_alt;
    double elevation = calculateElevation(distance, altitude_diff);
    
    // Convert bearing from 0-360 (N=0, E=90) to -180 to 180 (N=0, E=90, W=-90)
    double pan = bearing;
    if (pan > 180.0) {
        pan -= 360.0;
    }
    
    // Limit to servo range [-90, 90]
    if (pan > 90.0) pan = 90.0;
    if (pan < -90.0) pan = -90.0;
    
    // Limit elevation to servo range [-90, 90]
    if (elevation > 90.0) elevation = 90.0;
    if (elevation < -90.0) elevation = -90.0;
    
    return PanTiltAngles(static_cast<float>(pan), static_cast<float>(elevation));
}

double GeoCalculations::normalizeAngle(double angle) {
    angle = std::fmod(angle + 180.0, 360.0);
    if (angle < 0) {
        angle += 360.0;
    }
    return angle - 180.0;
}

double GeoCalculations::toRadians(double degrees) {
    return degrees * PI / 180.0;
}

double GeoCalculations::toDegrees(double radians) {
    return radians * 180.0 / PI;
}
