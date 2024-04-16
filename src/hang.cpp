#include "hang.hpp"
#include "globals.hpp"

#define HANG_P 50.0f
#define HANG_D 5.0f

float prev_error = 0.0f;
float prev_target = -9999.0f;

void hang_pd(float target) {
    

    float gearRatio = 12.0f/84.0f; 
    master.print(0, 0, "ANGLE %f", gearRatio*hang1.get_position()); 

    float curr_hang_angle = gearRatio*(hang1.get_position());
    float error = target - curr_hang_angle;

    if(target != prev_target) {
        prev_target = target;
        prev_error = error;
    }
    float error_delta = error - prev_error;
    prev_error = error;

    float power = error * HANG_P + prev_error * HANG_D;
    hang1.move(power);
    hang2.move(power);


}