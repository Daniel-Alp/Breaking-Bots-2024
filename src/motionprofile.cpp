#include <cmath>
#include "main.h"
#include "motionprofile.hpp"
#include "constants.hpp"
#include "globals.hpp"
#include "drive.hpp"

void move_straight(double x_goal, double v_start, double v_end, bool reverse = false) {
    std::vector<Segment> traj = generate_trajectory(x_goal, v_start, v_end, reverse);
    follow_trajectory(traj, traj);
}

std::vector<Segment> generate_trajectory(double x_goal, double v_start, double v_end, bool reverse = false) {
    double v_reachable = std::sqrt(x_goal * MAX_ACCELERATION + 0.5 * v_start * v_start + 0.5 * v_end * v_end);
    double v_max = std::min(MAX_VELOCITY, v_reachable);

    double t_speedup = (v_max - v_start) / MAX_ACCELERATION;
    double x_speedup = (v_max + v_start) * 0.5 * t_speedup;

    double t_slowdown = -(v_end - v_max) / MAX_ACCELERATION;
    double x_slowdown = (v_max + v_end) * 0.5 * t_slowdown; 

    double x_cruise = x_goal - x_speedup - x_slowdown;
    double t_cruise = x_cruise / v_max;

    double t_total = t_speedup + t_cruise + t_slowdown;


    std::vector<Segment> traj{};
    traj.reserve(t_total / LOOP_DELAY_SEC);

    double t = 0;
    double x = 0;
    double v = 0;
    double v_prev = 0;
    double a = 0;
    
    while (t <= t_total) {
        if (t <= t_speedup) {
            v = v_start + MAX_ACCELERATION * t;
        } else if (t <= t_speedup + t_cruise) {
            v = v_max;
        } else {
            v = v_max - MAX_ACCELERATION * (t - t_speedup - t_cruise);
        }

        a = (v - v_prev) / LOOP_DELAY_SEC;
        x += (v + v_prev) * 0.5 * LOOP_DELAY_SEC;

        if (reverse) {
            traj.emplace_back(-x, -v, -a);
        } else {
            traj.emplace_back(x, v, a);
        }

        v_prev = v;
        t += LOOP_DELAY_SEC;
    }
    traj.emplace_back(x_goal, v_end, (v_end - v_prev) / LOOP_DELAY_SEC);

    std::cout << "Finished generating trajectory!" << std::endl;

    return traj;
}

void follow_trajectory(std::vector<Segment>& right_traj, std::vector<Segment>& left_traj) {
    tare_position_drive();
    
    double right_error = 0;
    double left_error = 0;
    double right_error_prev = 0;
    double left_error_prev = 0;

    for (int i = 0; i < right_traj.size(); i++) {
        Segment right_seg = right_traj[i];
        Segment left_seg = left_traj[i];

        right_error = right_seg.x - get_right_position();
        left_error = right_seg.x - get_left_position();

        double right_power = calculate_power(right_error, right_error_prev, right_seg.v, right_seg.a);
        double left_power = calculate_power(left_error, left_error_prev, left_seg.v, left_seg.a);

        move_voltage_right_drive(right_power);
        move_voltage_left_drive(left_power);

        right_error_prev = right_error;
        left_error_prev = left_error;
    }
}

double calculate_power(double error, double error_prev, double v, double a) {
    double kV = 1/MAX_VELOCITY;    
    //Need to be tuned, there are procedures online for how to do this
    double kA = 0;
    double kP = 0;
    double kD = 0;

    double feedforward = kV * v + kA * a;
    double feedback = kP * error + kD * ((error - error_prev) / LOOP_DELAY_SEC);

    return (feedforward + feedback) * 12000;
}