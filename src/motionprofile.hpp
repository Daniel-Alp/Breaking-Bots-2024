#pragma once
#include <vector>
#include "main.h"

struct Segment {
    double x;
    double v;
    double a;
    double heading;

    Segment(double _x, double _v, double _a, double _heading) {
        x = _x;
        v = _v;
        a = _a;
        heading = _heading;
    }
};

void move_straight(double x_goal, double v_start, double v_end, bool reverse = false);

void move_circular_arc(double x_goal, double v_start, double v_end, double heading_start, double heading_end, bool reverse = false);

std::vector<Segment> generate_trajectory(double x_goal, double v_start, double v_end, double heading_start, double heading_end, bool reverse = false);

void follow_trajectory(std::vector<Segment>& right_traj, std::vector<Segment>& left_traj);

double calculate_power(double error, double error_prev, double v, double a);