#include "hang.hpp"
#include "globals.hpp"

#define HANG_P 10.0f
#define HANG_D 5.0f

float prev_error = 0.0f;
float prev_target = -9999.0f;

void hang_pd(float target) {
    
    float curr_hang_angle = (hang1.get_position() + hang2.get_position()) / 2.0f;
    float error = target - curr_hang_angle;
    if(target != prev_target) {
        prev_target = target;
        prev_error = error;
    }
    float error_delta = error - prev_error;
    prev_error = error;

    float power = error * HANG_P + prev_error * HANG_D;
    hang1.move(HANG_P);
    hang2.move(HANG_P);


}