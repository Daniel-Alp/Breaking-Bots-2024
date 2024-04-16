#include "main.h"
#include "globals.hpp"
#include "auton.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "motionprofile.hpp"
#include "pid.hpp"

void test_auton() {
    FILE* log_file = fopen("/usd/motion-profile-data.txt", "w");
    fprintf(log_file, "Time, Heading\n");

    double time_elapsed_ms = 0;
    while (time_elapsed_ms < 5000) {
        move_voltage_left_drive(12000);
        move_voltage_right_drive(-12000);
        pros::delay(10);
        time_elapsed_ms += 10;

        fprintf(log_file, "%f, %f\n", time_elapsed_ms, imu.get_heading());
    }
    fclose(log_file);

    // //Specify is reversed or not, default is not reversed.
    // move_straight(72, 0, 0, 0);
    // move_straight(72, 0, 0, 0, true);
    // move_straight(24, 0, 0, 0);
    // move_straight(24, 0, 0, 0);
    // move_straight(48, 0, 0, 0, true);

    // turn_to_heading(90);
    // pros::delay(200);
    // turn_to_heading(0);
    // pros::delay(200);
    // turn_to_heading(270);
    // pros::delay(200);
    // turn_to_heading(0);
    // pros::delay(200);
    // turn_to_heading(45);
    // turn_to_heading(45);
    // turn_to_heading(135);
}