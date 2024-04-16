#pragma once
#include <cmath>
#include "constants.hpp"

inline int sgn(double x) {
    if (x > 0) {
        return 1;
    }
    if (x < 0) {
        return -1;
    }
    return 0;
}

inline double rad_to_deg(double rad) {
    return rad * 180 / PI;
}

inline double deg_to_rad(double deg) {
    return deg * PI / 180;
}

//Positive is clockwise, negative is counterclockwise
//Ranges from -180 to 180
inline double get_heading_difference(double heading, double heading_goal) {
    double diff = heading_goal - heading;
    if (diff > 180) {
        diff -= 360;
    }
    if (diff < -180) {
        diff += 360;
    }
    return diff;
}