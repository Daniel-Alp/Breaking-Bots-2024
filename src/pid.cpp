#include <cmath>
#include "pid.hpp"
#include "globals.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "mathutil.hpp"

void turn_to_heading(double heading_goal) {
    const double ERROR_THRESHOLD = 1.5;

    //NEED TO BE TUNED
    const double MAX_VELOCITY_ANGULAR = 0.500;//Deg/ms

    const double kP = 0.080;
    const double kD = 0.0080;
    const double MIN_POWER = 0.001;

    double abs_start_heading_diff = std::abs(get_heading_difference(imu.get_heading(), heading_goal));

    double error = 0;
    double error_prev = 0;

    double power = 0;
    double power_prev = 0;

    double time_elapsed_ms = 0;

    // FILE* log_file = fopen("/usd/pid-turn-data.txt", "w");
    // fprintf(log_file, "Time, Target Heading, Heading\n");

    do {
        double heading = imu.get_heading();

        error = get_heading_difference(heading, heading_goal);

        //Technically 12000 and LOOP_DELAY_SEC can be baked into the kP and kD terms
        power = (kP * error + kD * (error - error_prev) / LOOP_DELAY_SEC  + MIN_POWER) * 12000;

        move_voltage_right_drive(-power);
        move_voltage_left_drive(power);

        error_prev = error;
        power_prev = power;

        delay(LOOP_DELAY_MS);
        time_elapsed_ms += LOOP_DELAY_MS;

        if (time_elapsed_ms > abs_start_heading_diff / MAX_VELOCITY_ANGULAR + 1000) {
            break;
        }

        // fprintf(log_file, "%f, %f, %f\n",
        //     time_elapsed_ms, 
        //     heading_goal, 
        //     heading);

    } while (std::abs(error) > ERROR_THRESHOLD 
            || get_left_velocity() > 10 
            || get_right_velocity() > 10);

    move_voltage_left_drive(0);
    move_voltage_right_drive(0);

    // fclose(log_file);
}

//Hold one side stationary while turning the other side
void swing_to_heading(double heading_goal, bool swing_right_side) {
    const double ERROR_THRESHOLD = 1.5;

    //NEED TO BE TUNED
    const double MAX_VELOCITY_ANGULAR = 0.500;//Deg/ms

    const double kP = 0.080;
    const double kD = 0.0080;
    const double MIN_POWER = 0.001;

    double abs_start_heading_diff = std::abs(get_heading_difference(imu.get_heading(), heading_goal));

    double error = 0;
    double error_prev = 0;

    double power = 0;
    double power_prev = 0;

    double time_elapsed_ms = 0;

    do {
        double heading = imu.get_heading();

        error = get_heading_difference(heading, heading_goal);

        power = (kP * error + kD * (error - error_prev) / LOOP_DELAY_SEC  + MIN_POWER) * 12000;

        if (swing_right_side) {
            move_voltage_right_drive(-power);
        } else {
            move_voltage_left_drive(power);
        }

        error_prev = error;
        power_prev = power;

        delay(LOOP_DELAY_MS);
        time_elapsed_ms += LOOP_DELAY_MS;

        if (time_elapsed_ms > abs_start_heading_diff / MAX_VELOCITY_ANGULAR + 1000) {
            break;
        }

    } while (std::abs(error) > ERROR_THRESHOLD 
            || get_left_velocity() > 10 
            || get_right_velocity() > 10);

    move_voltage_left_drive(0);
    move_voltage_right_drive(0);
}