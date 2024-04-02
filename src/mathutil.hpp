#pragma once
#include <cmath>

inline int sgn(double x) {
    if (x > 0) {
        return 1;
    }
    if (x < 0) {
        return -1;
    }
    return 0;
}

//Positive is counterclockwise, negative is clockwise
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