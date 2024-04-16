#include <cmath>
#include "pid.hpp"
#include "globals.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "mathutil.hpp"

void turn_to_heading(double heading_goal) {
    const double ERROR_THRESHOLD = 1;

    //NEED TO BE TUNED
    const double MAX_VELOCITY_ANGULAR = 0.500;//Deg/ms
    const double MAX_ACCELERATION_ANGULAR = 1;//Deg/ms^2

    const double kP = 0.080;
    const double kD = 0.0085;
    const double MIN_POWER = 0.000;
    const double MAX_POWER_CHANGE = 12000 * (MAX_ACCELERATION_ANGULAR * LOOP_DELAY_MS);

    double abs_start_heading_diff = std::abs(get_heading_difference(imu.get_heading(), heading_goal));

    double error = 0;
    double error_prev = 0;

    double power = 0;
    double power_prev = 0;

    double time_elapsed_ms = 0;

    FILE* log_file = fopen("/usd/pid-turn-data.txt", "w");
    fprintf(log_file, "Time, Target Heading, Heading\n");

    do {
        double heading = imu.get_heading();

        error = get_heading_difference(heading, heading_goal);

        //Technically 12000 and LOOP_DELAY_SEC can be baked into the kP and kD terms
        power = (kP * error + kD * (error - error_prev) / LOOP_DELAY_SEC  + MIN_POWER) * 12000;
        power = std::clamp(power, power_prev - MAX_POWER_CHANGE, power_prev + MAX_POWER_CHANGE);
        power = std::clamp(power, static_cast<double>(-12000), static_cast<double>(12000));

        move_voltage_right_drive(-power);
        move_voltage_left_drive(power);

        error_prev = error;
        power_prev = power;

        delay(LOOP_DELAY_MS);
        time_elapsed_ms += LOOP_DELAY_MS;

        if (time_elapsed_ms > abs_start_heading_diff / MAX_VELOCITY_ANGULAR + 1000) {
            break;
        }

        fprintf(log_file, "%f, %f, %f\n",
            time_elapsed_ms, 
            heading_goal, 
            heading);

    } while (std::abs(error) > ERROR_THRESHOLD 
            || get_left_velocity() > 6 
            || get_right_velocity() > 6);

    move_voltage_left_drive(0);
    move_voltage_right_drive(0);

    fclose(log_file);
}