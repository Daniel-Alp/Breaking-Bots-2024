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

void move_straight(double x_goal, double v_start, double v_end, bool reverse = false);

std::vector<Segment> generate_trajectory(double x_goal, double v_start, double v_end, bool reverse = false);

void follow_trajectory(std::vector<Segment>& right_traj, std::vector<Segment>& left_traj);

double calculate_power(double error, double error_prev, double v, double a);

bool trajectory_finished(double right_error, double left_error);