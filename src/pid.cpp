#include <cmath>
#include "pid.hpp"
#include "globals.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "mathutil.hpp"

void turn_to_heading(double heading_goal) {
    const double ERROR_THRESHOLD = 0.5;

    //NEED TO BE TUNED
    const double kP = 0.41;
    const double kD = 0.20;
    const double MIN_POWER = 0.05;
    const double MAX_POWER_CHANGE = 12000;

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
        power = std::clamp(power, power - MAX_POWER_CHANGE, power + MAX_POWER_CHANGE);
        power = std::clamp(power, static_cast<double>(-12000), static_cast<double>(12000));

        move_voltage_right_drive(-power);
        move_voltage_left_drive(power);

        error_prev = error;
        power_prev = power;

        delay(LOOP_DELAY_MS);
        time_elapsed_ms += LOOP_DELAY_MS;

        fprintf(log_file, "%f, %f, %f\n",
            time_elapsed_ms, 
            heading_goal, 
            heading);

    } while (std::abs(error) > ERROR_THRESHOLD 
            || get_left_velocity() > 5 
            || get_right_velocity() > 5);

    fclose(log_file);
}