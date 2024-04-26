#pragma once
#include "main.h"

//Need to be measured and tuned. 
const double MAX_VELOCITY = 85; // in/s (a max velocity of 79.26in/s)
const double MAX_ACCELERATION = 140; // in/s^2 (an acceleration of 12in/s and 18in/s was measured depending on how many points were used for regression)

const double DRIVE_WIDTH = 15; // in (distance between where middle wheels touch the floor). Needs to be measured.
const double WHEEL_DIAM = 3.25; // in
const double MOTOR_TO_WHEEL_RATIO = static_cast<double>(36)/48;

void move_voltage_left_drive(int voltage);

void move_voltage_right_drive(int voltage);

void set_drive_brake_mode(pros::motor_brake_mode_e_t mode);

void tare_position_drive();

double get_left_position();

double get_left_velocity();

double get_right_position();

double get_right_velocity();