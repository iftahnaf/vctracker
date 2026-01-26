#ifndef GEO_CALCULATIONS_H
#define GEO_CALCULATIONS_H

#include <cmath>

/**
 * @struct Vector3D
 * @brief 3D vector representation
 */
struct Vector3D {
    double x;
    double y;
    double z;
    
    Vector3D() : x(0.0), y(0.0), z(0.0) {}
    Vector3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @struct PanTiltAngles
 * @brief Pan and tilt angles for antenna tracker
 */
struct PanTiltAngles {
    float pan;    // Degrees (-90 to 90, 0 = North)
    float tilt;   // Degrees (-90 to 90, 0 = Horizon)
    
    PanTiltAngles() : pan(0.0f), tilt(0.0f) {}
    PanTiltAngles(float p, float t) : pan(p), tilt(t) {}
};

/**
 * @class GeoCalculations
 * @brief Geodetic calculations for antenna tracking
 * 
 * Provides calculations for:
 * - Distance and bearing between two GPS coordinates
 * - Elevation angle to target
 * - Pan/tilt angles for gimbal pointing
 */
class GeoCalculations {
public:
    /**
     * @brief Calculate distance between two GPS coordinates (Haversine formula)
     * @param lat1 Latitude of point 1 (degrees)
     * @param lon1 Longitude of point 1 (degrees)
     * @param lat2 Latitude of point 2 (degrees)
     * @param lon2 Longitude of point 2 (degrees)
     * @return Distance in meters
     */
    static double calculateDistance(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Calculate bearing from point 1 to point 2
     * @param lat1 Latitude of point 1 (degrees)
     * @param lon1 Longitude of point 1 (degrees)
     * @param lat2 Latitude of point 2 (degrees)
     * @param lon2 Longitude of point 2 (degrees)
     * @return Bearing in degrees (0-360, 0=North, 90=East)
     */
    static double calculateBearing(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Calculate elevation angle to target
     * @param distance Horizontal distance to target (meters)
     * @param altitude_diff Altitude difference (target - tracker) in meters
     * @return Elevation angle in degrees (-90 to 90)
     */
    static double calculateElevation(double distance, double altitude_diff);

    /**
     * @brief Calculate pan/tilt angles for antenna tracker
     * @param tracker_lat Tracker latitude (degrees)
     * @param tracker_lon Tracker longitude (degrees)
     * @param tracker_alt Tracker altitude (meters)
     * @param target_lat Target latitude (degrees)
     * @param target_lon Target longitude (degrees)
     * @param target_alt Target altitude (meters)
     * @return Pan and tilt angles
     */
    static PanTiltAngles calculateTrackerAngles(
        double tracker_lat, double tracker_lon, double tracker_alt,
        double target_lat, double target_lon, double target_alt);

    /**
     * @brief Normalize angle to range [-180, 180]
     * @param angle Angle in degrees
     * @return Normalized angle
     */
    static double normalizeAngle(double angle);

    /**
     * @brief Convert degrees to radians
     * @param degrees Angle in degrees
     * @return Angle in radians
     */
    static double toRadians(double degrees);

    /**
     * @brief Convert radians to degrees
     * @param radians Angle in radians
     * @return Angle in degrees
     */
    static double toDegrees(double radians);

private:
    static constexpr double EARTH_RADIUS = 6371000.0; // Earth radius in meters
    static constexpr double PI = 3.14159265358979323846;
};

#endif // GEO_CALCULATIONS_H
