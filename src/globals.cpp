#include "main.h"
#include "pros/adi.hpp"
#include "globals.hpp"

Controller master(E_CONTROLLER_MASTER);

//11W motors
Motor r1(12);
Motor r2(1);
Motor r3(11);
Motor l1(20);
Motor l2(17);
Motor l3(10);

//5.5W motor
Motor intake(13); 

//11W motor
Motor hang1(15); 
//5.5W motor
Motor hang2(16);

IMU imu(14);

// Pneumatics objects
pros::adi::DigitalOut ratchet('G');
pros::adi::DigitalOut leftWing('A'); 
pros::adi::DigitalOut rightWing('F'); 
