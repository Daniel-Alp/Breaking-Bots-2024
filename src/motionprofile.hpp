#pragma once
#include <vector>
#include "main.h"

struct Segment {
    double x;
    double v;
    double a;

    Segment(double _x, double _v, double _a) {
        x = _x;
        v = _v;
        a = _a;
    }
};

std::vector<Segment> generate_trajectory(double x_goal, double v_start, double v_end, bool reverse);

void move_straight(double x_goal, double v_start, double v_end, bool reverse);

double calculate_power(double error, double error_prev, double v, double a);