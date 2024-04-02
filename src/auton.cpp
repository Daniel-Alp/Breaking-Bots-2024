#include "main.h"
#include "auton.hpp"
#include "motionprofile.hpp"

void test_auton() {
    //Specify is reversed or not, default is not reversed.
    move_straight(72, 0, 0);
    move_straight(72, 0, 0, true);
    move_straight(24, 0, 0);
    move_straight(24, 0, 0);
    move_straight(24, 0, 0);
    move_circular_arc(37.70, 0, 0, 0, 90);
}