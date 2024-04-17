#include "main.h"
#include "globals.hpp"
#include "auton.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "motionprofile.hpp"
#include "pid.hpp"

void near_side_safe_AWP() {
    set_drive_brake_mode(MOTOR_BRAKE_HOLD); //Do not want to accidentally overshoot
    ratchet.set_value(0);

    //Raise hang to release the intake
    hang1.move(-127);
    hang2.move(-127);
    pros::delay(100);

    //Lower hang in parallel with deploying wing
    hang1.move(127);
    hang2.move(127);
    leftWing.set_value(1);
    delay(350);
    hang1.move(0);
    hang2.move(0);

    //Descore triball from matchload zone
    move_straight(18, 0, 0, 0, true);
    turn_to_heading(180);
    leftWing.set_value(0);

    //Move backwards to score alliance triball
    move_straight(25, 0, 0, 0, true);
    turn_to_heading(225);
    move_straight(4, 0, MAX_VELOCITY, 0, true);

    //Move to touch hang bar
    move_voltage_left_drive(0);
    move_voltage_right_drive(0);
    move_straight(4, 0, MAX_VELOCITY, 0, false);
    turn_to_heading(180);
    move_straight(24, 0, 0, 0, false);
    turn_to_heading(135);
    move_straight(35, 0, 0, 0, false);
}

void test_auton() {
    
}