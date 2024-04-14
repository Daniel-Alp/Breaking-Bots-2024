#include "main.h"
#include "globals.hpp"
#include "auton.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "motionprofile.hpp"
#include "pid.hpp"

void test_auton() {
    // FILE* log_file = fopen("/usd/drive-data.txt", "w");
    // fprintf(log_file, "Time, Right Pos, Left Pos, Right Vel, Left Vel\n");

    // double time_elapsed_ms = 0;

    // while (time_elapsed_ms <= 3000) //It won't take longer than 5 seconds to reach max speed
    // {
    //     move_voltage_left_drive(12000);
    //     move_voltage_right_drive(12000);

    //     delay(LOOP_DELAY_MS);
    //     time_elapsed_ms += LOOP_DELAY_MS;
        
    //     fprintf(log_file, "%f, %f, %f, %f, %f\n", time_elapsed_ms, get_right_position(), get_left_position(), get_right_velocity(), get_left_velocity());
    // }
    // fclose(log_file);
    

    // //Specify is reversed or not, default is not reversed.
    move_straight(72, 0, 0);
    // move_straight(72, 0, 0, true);
    // move_straight(24, 0, 0);
    // move_straight(24, 0, 0);
    // move_straight(24, 0, 0);
    // move_straight(72, 0, 0, true);
    // move_circular_arc(37.70, 0, 0, 0, 90);
}