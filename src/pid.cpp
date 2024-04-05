#include <cmath>
#include "pid.hpp"
#include "globals.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "mathutil.hpp"

void turn_to_heading(double heading_goal) {
    double error_threshold = 0.5;

    //NEED TO BE TUNED
    double kP = 0;
    double kD = 0;
    double min_power = 0;

    double error = 0;
    double error_prev = 0;

    double power = 0;

    do {
        error = get_heading_difference(imu.get_heading(), heading_goal);

        //Technically 12000 and LOOP_DELAY_SEC can be baked into the kP and kD terms
        //But I'm hoping we can reuse the values tuned for straight movements
        power = (kP * error + kD * (error - error_prev) / LOOP_DELAY_SEC  + min_power) * 12000;

        move_voltage_right_drive(-power);
        move_voltage_left_drive(power);

        error_prev = error;

        delay(LOOP_DELAY_MS);
    } while (std::abs(error) > error_threshold || get_left_velocity() > 5 || get_right_velocity() > 5);
}