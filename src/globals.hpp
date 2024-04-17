#pragma once
#include "main.h"
#include "pros/adi.hpp"

using namespace pros;

extern Controller master;

extern Motor r1;
extern Motor r2;
extern Motor r3;
extern Motor l1;
extern Motor l2;
extern Motor l3;

extern Motor hang1;
extern Motor hang2;

extern Motor intake;

extern IMU imu;

extern pros::adi::DigitalOut leftWing; 
extern pros::adi::DigitalOut rightWing; 
extern pros::adi::DigitalOut ratchet; 

extern const int RATCHET_ACTIVE; 
extern const int RATCHET_INACTIVE; 