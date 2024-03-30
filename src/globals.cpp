#include "main.h"
#include "globals.hpp"

Controller master(E_CONTROLLER_MASTER);

//11W motors
Motor r1(1);
Motor r2(2);
Motor r3(3);
Motor l1(4);
Motor l2(5);
Motor l3(6);

//5.5W motor
Motor intake(7); 

//11W motor
Motor hang1(8); 
//5.5W motor
Motor hang2(9);

IMU imu(10);