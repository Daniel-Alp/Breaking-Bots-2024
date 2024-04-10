#include <cmath>
#include "main.h"
#include "globals.hpp"
#include "constants.hpp"
#include "mathutil.hpp"
#include "drive.hpp"
#include "auton.hpp"

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
    r2.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
    r3.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
    l1.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
    l2.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
    l3.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);

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
    // while (imu.is_calibrating()) {
    //     delay(10);
    // }

    tare_position_drive();

    std::cout << "Done initializing!" << std::endl;
}

void disabled() {
    
}

void competition_initialize() {

}

void autonomous() {
    test_auton();
}

int map_joystick_input_to_power(double input) {
    input /= 127; //Scale down to -1 to 1
    input = sgn(input) * (input * input); //Map linear input to square output
    input *= 12000; //Scale up to -12000mV to 12000mV
    return input;
}

void opcontrol() {
    set_drive_brake_mode(E_MOTOR_BRAKE_COAST);
    
    while(true) {
        // Intake controls 
        if (master.get_digital(E_CONTROLLER_DIGITAL_L1)){
            intake.move(-127); 
        } else if (master.get_digital(E_CONTROLLER_DIGITAL_L2)){
            intake.move(127); 
        } else {
            intake.move(0); 
        }

        // Hang controls 
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            hang1.move(127); 
            hang2.move(-127); 
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            hang1.move(-127); 
            hang2.move(127); 
        } else {
            hang1.move(0); 
            hang2.move(0); 
        }

        // Main driver code 
        int linear_input = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int turn_input = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        int right_power = map_joystick_input_to_power(linear_input - turn_input);
        int left_power = map_joystick_input_to_power(linear_input + turn_input);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			right_power *= 0.44;
			left_power *= 0.44;
		}

        move_voltage_right_drive(right_power);
        move_voltage_left_drive(left_power);

        delay(LOOP_DELAY_MS);
    }
}