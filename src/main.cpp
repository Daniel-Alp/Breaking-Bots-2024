#include "main.h"
#include "globals.hpp"
#include "constants.hpp"

void initialize() {
    //Initialize drive motors
	r1.set_gearing(E_MOTOR_GEAR_600);
    r2.set_gearing(E_MOTOR_GEAR_600);
    r3.set_gearing(E_MOTOR_GEAR_600);
    l1.set_gearing(E_MOTOR_GEAR_600);
    l2.set_gearing(E_MOTOR_GEAR_600);
    l3.set_gearing(E_MOTOR_GEAR_600);

    r1.set_reversed(false); 
    r2.set_reversed(false);
    r3.set_reversed(false);
    l1.set_reversed(true);
    l2.set_reversed(true);
    l3.set_reversed(true);

    r1.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
    l1.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);

    //Initialize intake motor
    intake.set_gearing(E_MOTOR_GEAR_200);
    intake.set_reversed(true);

    //Initialize hang motors
    hang1.set_gearing(E_MOTOR_GEAR_100);
    hang2.set_gearing(E_MOTOR_GEAR_200);

    hang1.set_reversed(false);
    hang2.set_reversed(true);

    hang1.set_encoder_units(E_MOTOR_ENCODER_DEGREES);

    //Initialize IMU
    imu.reset();
    while (imu.is_calibrating()) {
        delay(10);
    }

    std::cout << "Done initializing!" << std::endl;
}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {
    
}

void opcontrol() {

}