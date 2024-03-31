#include "main.h"
#include "globals.hpp"
#include "constants.hpp"
#include "drive.hpp"

//From -12000mV to 12000mV
void move_voltage_right_drive(int voltage) {
    r1.move_voltage(voltage);
    r2.move_voltage(voltage);
    r3.move_voltage(voltage);
}

//From -12000mV to 12000mV
void move_voltage_left_drive(int voltage) {
    l1.move_voltage(voltage);
    l2.move_voltage(voltage);
    l3.move_voltage(voltage);
}

void set_drive_brake_mode(motor_brake_mode_e_t mode) {
    r1.set_brake_mode(mode);
    r2.set_brake_mode(mode);
    r3.set_brake_mode(mode);
    l1.set_brake_mode(mode);
    l2.set_brake_mode(mode);
    l3.set_brake_mode(mode);
}

double get_left_velocity() {
    return WHEEL_DIAM * PI * MOTOR_TO_WHEEL_RATIO * (l1.get_actual_velocity() / 60);
}

double get_right_velocity() {
    return WHEEL_DIAM * PI * MOTOR_TO_WHEEL_RATIO * (r1.get_actual_velocity() / 60);
}

double get_left_position() {
    return WHEEL_DIAM * PI * MOTOR_TO_WHEEL_RATIO * l1.get_position();
}

double get_right_position() {
    return WHEEL_DIAM * PI * MOTOR_TO_WHEEL_RATIO * r1.get_position();
}