#include <cmath>
#include "main.h"
#include "motionprofile.hpp"
#include "globals.hpp"
#include "drive.hpp"

//Currently only supports forwards movement, negative distances will be implemented later
void move_straight(double x_goal, double v_start, double v_end) {
    double v_reachable = std::sqrt(x_goal * MAX_ACCELERATION + 0.5 * v_start * v_start + 0.5 * v_end * v_end);
    double v_max = std::min(MAX_VELOCITY, v_reachable);

    double t_speedup = (v_max - v_start) / MAX_ACCELERATION;
    double x_speedup = (v_max + v_start) * 0.5 * t_speedup;

    double t_slowdown = -(v_end - v_max) / MAX_ACCELERATION; //max accel is magnitude so we add a negative
    double x_slowdown = (v_max + v_end) * 0.5 * t_slowdown; 

    double x_cruise = x_goal - x_speedup - x_slowdown;
    double t_cruise = x_cruise / v_max;
}