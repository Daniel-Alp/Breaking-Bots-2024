#include "main.h"
#include "globals.hpp"
#include "auton.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "motionprofile.hpp"
#include "pid.hpp"

void test_auton() {
    // //Specify is reversed or not, default is not reversed.
    move_straight(72, 0, 0, 0);
    move_straight(72, 0, 0, 0, true);
    move_straight(24, 0, 0, 0);
    move_straight(24, 0, 0, 0);
    move_straight(48, 0, 0, 0, true);
    move_circular_arc(68.33, 0, 0, 0, 90);
}