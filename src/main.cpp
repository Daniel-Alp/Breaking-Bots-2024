#include <cmath>
#include "main.h"
#include "globals.hpp"
#include "constants.hpp"
#include "mathutil.hpp"
#include "drive.hpp"
#include "auton.hpp"
#include "pros/misc.h"
#include "hang.hpp"

const float PRESET_BICEP_ANGLE = -130; 
const float PRESET_SIDE_HANG_ANGLE = -20; 

void initialize() {
    //Initialize drive motors
	r1.set_gearing(pros::E_MOTOR_GEAR_600);
    r2.set_gearing(pros::E_MOTOR_GEAR_600);
    r3.set_gearing(pros::E_MOTOR_GEAR_600);
    l1.set_gearing(pros::E_MOTOR_GEAR_600);
    l2.set_gearing(pros::E_MOTOR_GEAR_600);
    l3.set_gearing(pros::E_MOTOR_GEAR_600);

    r1.set_reversed(false); 
    r2.set_reversed(false);
    r3.set_reversed(false);
    l1.set_reversed(true);
    l2.set_reversed(true);
    l3.set_reversed(true);

    r1.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    r2.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    r3.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    l1.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    l2.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    l3.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);

    //Initialize intake motor
    intake.set_gearing(pros::E_MOTOR_GEAR_200);
    intake.set_reversed(true);

    //Initialize hang motors
    hang1.set_gearing(E_MOTOR_GEAR_100);
    hang2.set_gearing(E_MOTOR_GEAR_200);

    hang1.set_reversed(false);
    hang2.set_reversed(false);


    //Initialize IMU
    imu.reset();
    while (imu.is_calibrating()) {
        delay(10);
    }

    tare_position_drive();

    hang1.set_encoder_units(E_MOTOR_ENCODER_DEGREES); 
    hang1.tare_position();
    hang2.set_encoder_units(E_MOTOR_ENCODER_DEGREES); 
    hang2.tare_position();  

    std::cout << "Done initializing!" << std::endl;
}

void disabled() {
    
}

void competition_initialize() {

}

void autonomous() {
    // near_side_safe_AWP();
    // five_ball_far_side_safe(); 
    // far_side_safe_awp(); 
    // near_side_safe_AWP(); 
}

int map_joystick_input_to_power(double input) {
    input /= 127; //Scale down to -1 to 1
    input = sgn(input) * (input * input); //Map linear input to square output
    input *= 12000; //Scale up to -12000mV to 12000mV
    return input;
}

//Negative inertia accumulator based on team 254F-Daniel Alp DOES NOT WORK
// double adjust_turn_power(double turn_power, double prev_turn_power, double& neg_inertia_accumulator) {
//     double neg_inertia = turn_power - prev_turn_power;

//     neg_inertia_accumulator += neg_inertia;
//     if (neg_inertia_accumulator > 0) {
//         neg_inertia_accumulator--;
//     } else if (neg_inertia_accumulator < 0) {
//         neg_inertia_accumulator++;
//     }

//     return turn_power + neg_inertia_accumulator;
// }

void opcontrol() {
    set_drive_brake_mode(E_MOTOR_BRAKE_COAST);
    
    bool ratchetActive = false; // If set to true then the ratchet is active, if set to false then the ratchet is not currently engaged 
    bool wingsActive = false; // If set to true then both wings will be triggered, otherwise both in
    float timeHang = 0; 

    double negative_inertia_accumulator = 0;
    double turn_power = 0;
    double prev_turn_power = 0;

    while(true) {
        // Intake controls 
        if (master.get_digital(E_CONTROLLER_DIGITAL_L1)){
            intake.move(-127); 
        } else if (master.get_digital(E_CONTROLLER_DIGITAL_L2)){
            intake.move(127); 
        } else {
            intake.move(0); 
        }

        // Ratchet Activation 
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            ratchetActive = !ratchetActive; 
        }        
        if (ratchetActive){
            ratchet.set_value(RATCHET_ACTIVE); 
        } else {
            ratchet.set_value(RATCHET_INACTIVE); 
        }
        
        // Wings Activation 
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            wingsActive = !wingsActive; 
        }
        if (wingsActive){
            leftWing.set_value(1); 
            rightWing.set_value(1); 
        } else {
            leftWing.set_value(0); 
            rightWing.set_value(0); 
        }

        // Hang controls 
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            hang1.move(127); 
            hang2.move(127); 
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            hang1.move(-127); 
            hang2.move(-127); 
        } else if (timeHang > 0){
            hang1.move(127); 
            hang2.move(127); 
            timeHang -= LOOP_DELAY_MS; 
        } else {
            timeHang = 0; 
            hang1.move(0); 
            hang2.move(0); 
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            ratchetActive = false; 
            ratchet.set_value(RATCHET_INACTIVE); 
            hang_pd(PRESET_BICEP_ANGLE);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            ratchetActive = false; 
            ratchet.set_value(RATCHET_INACTIVE); 
            hang_pd(PRESET_SIDE_HANG_ANGLE);
        } 
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            ratchetActive = true; 
            ratchet.set_value(RATCHET_ACTIVE);   
            timeHang = 2500; 
        }

        // Main driver code 
        int linear_input = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int turn_input = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        double linear_power = map_joystick_input_to_power(linear_input);
        turn_power = map_joystick_input_to_power(turn_input);
        // turn_power = adjust_turn_power(turn_power, prev_turn_power, negative_inertia_accumulator);
        prev_turn_power = turn_power;

        int right_power = linear_power - turn_power;
        int left_power = linear_power + turn_power;

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			right_power *= 0.44;
			left_power *= 0.44;
		}

        move_voltage_right_drive(right_power);
        move_voltage_left_drive(left_power);

        float gearRatio = 12.0f/84.0f; 
        master.print(0,0, "ANGLE: %f", gearRatio * hang1.get_position()); 
                                                                                                                                                                                                                                                                                                                                                                                             
        delay(LOOP_DELAY_MS);
    }
}