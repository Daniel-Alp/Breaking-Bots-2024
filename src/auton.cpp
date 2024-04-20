#include "main.h"
#include "globals.hpp"
#include "auton.hpp"
#include "constants.hpp"
#include "drive.hpp"
#include "motionprofile.hpp"
#include "pid.hpp"
#include <atomic>
#include <ios>

void release_intake(){
    hang1.move(-127); 
    hang2.move(-127); 
    pros::delay(177); 
}

void lower_hang(){
    hang1.move(127); 
    hang2.move(127); 
}

void stop_hang(){
    hang1.move(0); 
    hang2.move(0); 
}

/*
    Near side AWP route
    3 balls(1 in net, 2 on the other side)
*/
void near_side_safe_AWP() {
    set_drive_brake_mode(MOTOR_BRAKE_HOLD); //Do not want to accidentally overshoot
    ratchet.set_value(RATCHET_INACTIVE);

    //Raise hang to release the intake
    release_intake(); 
    intake.move(127);

    //Lower hang in parallel with deploying wing
    lower_hang(); 
    leftWing.set_value(1);
    pros::delay(500);
    stop_hang(); 
    intake.move(0);

    //Descore triball from matchload zone
    move_straight(18, 0, 0, 0, true);
    turn_to_heading(270);
    leftWing.set_value(0);

    pros::delay(500); 
    turn_to_heading(350); 

    //Move backwards to score alliance triball
    move_straight(23, 0, 0, 0, false);
    turn_to_heading(45);
    intake.move(-127);
    pros::delay(400);
    turn_to_heading(225);
    move_straight(8, 0, MAX_VELOCITY, 0, true);

    //Move to touch hang bar
    move_voltage_left_drive(0);
    move_voltage_right_drive(0);
    move_straight(4, 0, MAX_VELOCITY, 0, false);
    turn_to_heading(180);
    move_straight(25, 0, 0, 0, false);
    turn_to_heading(145);
    leftWing.set_value(1);

    intake.move(-127); 
    move_straight(35, 0, 0, 0, false);
    move_voltage_left_drive(0);
    move_voltage_right_drive(0);
}

/*
* This autonomous starts at the bar prepared to intake the triball in the middle of the field, starts with the 3 side triballs and moves to the middle at the end 
* Gets alliance triball, matchload triball, and bar triball into the net 
* If time permits this autonomous will run for the middle triballs after doing the first 3 safely
*/
void five_ball_far_side_safe(){
    set_drive_brake_mode(MOTOR_BRAKE_HOLD); //Do not want to accidentally overshoot
    ratchet.set_value(RATCHET_INACTIVE);

    //Raise hang to release the intake
    intake.move(127);
    release_intake(); 

    lower_hang(); 

    /*Move forward and back to intake for sure*/
    move_straight(0.9, 0, 0, 0, false); 

    /*Move back with the one triball on the back of the bot after intaking the alliance triball*/
    move_straight(34, 0, 0, 0, true); 
    swing_to_heading(320, false); // heading for descore position  
    stop_hang(); // Stop moving the hang 
    intake.move(0); 

    /*
    * Descoring the triball 
    */
    leftWing.set_value(1); 
    move_straight(14, 0, 0, 320, true); // Move back to descore 
    swing_to_heading(270, false, 400); 

    /*
    * This part of the motion shooots the triball to the net by swinging 
    */
    // turn_to_heading(225); 
    // turn_to_heading(270); 
    turn_to_heading(235);
    leftWing.set_value(0);
    pros::delay(300);   
    turn_to_heading(300, 750); 
    
    /*
    * This part of the motion is when we ram into the net with 2 triballs 
    */
    double rearPushAngle = 300.0f; 
    move_straight(18, 0, MAX_VELOCITY, rearPushAngle, true); // Move into the net at a slight angle with  rear of robot 

    // Move away from net 
    move_straight(9, 0, 0, rearPushAngle, false); 
    
    // Now ram triballl inside intake 
    turn_to_heading(110); 
    move_straight(18, 0, MAX_VELOCITY, 110, false); 

    // Move back for final triball grap 
    move_straight(17, 0, 0, 110, true);

    // FINAL SECTION 
    double finalTurnAngle = 25.0f;
    turn_to_heading(finalTurnAngle); // point to the last triball 
    intake.move(127); 

    // Head to triball 
    move_straight(48, 0, 0, finalTurnAngle, false); 

    // Turn to push last triball
    double finalPushAngle = 150; 
    turn_to_heading(finalPushAngle, 400); 
    leftWing.set_value(1); 
    rightWing.set_value(1); 
    move_voltage_left_drive(12000); 
    move_voltage_right_drive(12000); 
    // move_circular_arc(30, 0, MAX_VELOCITY, 135, 180); 
    // move_straight(18, 0, MAX_VELOCITY, 150, false); 



}


/*
* Safe side 3 ball autonomous 
* Starts on tile (4,0)
*/
void far_side_safe_awp(){
    set_drive_brake_mode(MOTOR_BRAKE_HOLD); //Do not want to accidentally overshoot
    ratchet.set_value(RATCHET_INACTIVE);

    //Raise hang to release the intake
    intake.move(127);
    release_intake(); 

    lower_hang(); 

    /*Move forward and back to intake for sure*/
    move_straight(0.9, 0, 0, 0, false); 

    /*Move back with the one triball on the back of the bot after intaking the alliance triball*/
    move_straight(34, 0, 0, 0, true); 
    swing_to_heading(320, false); // heading for descore position  
    stop_hang(); // Stop moving the hang 
    intake.move(0); 

    /*
    * Descoring the triball 
    */
    move_straight(14, 0, 0, 320, true); // Move back to descore 
    swing_to_heading(270, false, 400); 

    /*
    * This part of the motion shooots the triball to the net by swinging 
    */
    // turn_to_heading(225); 
    // turn_to_heading(270); 
    turn_to_heading(280, 200); 
    
    /*
    * This part of the motion is when we ram into the net with 2 triballs 
    */
    double rearPushAngle = 300.0f; 
    move_straight(18, 0, MAX_VELOCITY, rearPushAngle, true); // Move into the net at a slight angle with  rear of robot 

    // Move away from net 
    move_straight(9, 0, 0, rearPushAngle, false); 
    
    // Now ram triballl inside intake 
    turn_to_heading(110); 
    move_straight(18, 0, MAX_VELOCITY, 110, false); 

    // Move back for final triball grap 
    move_straight(18, 0, 0, 110, true);

    // FINAL SECTION 
    double finalTurnAngle = 25.0f;
    turn_to_heading(finalTurnAngle); // point to the last triball 
    intake.move(127); 

    // Head to triball 
    move_straight(53, 0, 0, finalTurnAngle, false); 

    turn_to_heading(150);
    move_straight(50, 0, MAX_VELOCITY/3, 150, false); 

    // Move back 
    swing_to_heading(165, false); 
    move_straight(43, 0, 0, 165, true); 
    turn_to_heading(280, 300);  
    move_straight(42, 0, 0, 280, false); 

    move_voltage_left_drive(1000); 
    move_voltage_right_drive(1000); 
    pros::delay(1000);  
    move_voltage_left_drive(0); 
    move_voltage_right_drive(0); 
}

void far_side_safe_awp_2(){
    set_drive_brake_mode(MOTOR_BRAKE_HOLD); //Do not want to accidentally overshoot
    ratchet.set_value(RATCHET_INACTIVE);

    //Raise hang to release the intake
    intake.move(127);
    release_intake(); 

    lower_hang(); 

    /*Move forward and back to intake for sure*/
    move_straight(0.9, 0, 0, 0, false); 

    /*Move back with the one triball on the back of the bot after intaking the alliance triball*/
    move_straight(34, 0, 0, 0, true); 
    swing_to_heading(320, false); // heading for descore position  
    stop_hang(); // Stop moving the hang 
    intake.move(0); 

    /*
    * Descoring the triball 
    */
    move_straight(14, 0, 0, 320, true); // Move back to descore 
    swing_to_heading(270, false, 400); 

    /*
    * This part of the motion shooots the triball to the net by swinging 
    */
    // turn_to_heading(225); 
    // turn_to_heading(270); 
    turn_to_heading(280, 200); 
    
    /*
    * This part of the motion is when we ram into the net with 2 triballs 
    */
    double rearPushAngle = 300.0f; 
    move_straight(18, 0, MAX_VELOCITY, rearPushAngle, true); // Move into the net at a slight angle with  rear of robot 

    // Move away from net 
    move_straight(9, 0, 0, rearPushAngle, false); 
    
    // Now ram triballl inside intake 
    turn_to_heading(100); 
    move_straight(18, 0, MAX_VELOCITY, 110, false); 

    // Move back for final triball grap 
    move_straight(17, 0, 0, 110, true);

    // FINAL SECTION 
    double finalTurnAngle = 25.0f;
    turn_to_heading(finalTurnAngle); // point to the last triball 
    intake.move(127); 

    // Head to triball 
    move_straight(48, 0, 0, finalTurnAngle, false); 

    // Turn to push last triball
    double finalPushAngle = 150; 
    turn_to_heading(finalPushAngle, 400); 
    intake.move(-127); 
    pros::delay(500); 

    // Second triball in the middle 
    turn_to_heading(90); 
    intake.move(127); 
    move_straight(24, 0, 0, 0, false);  
    
    turn_to_heading(0); 
    move_straight(30, 0, MAX_VELOCITY, 0, true); 
    

    // leftWing.set_value(1); 
    // rightWing.set_value(1); 
    // move_voltage_left_drive(12000); 
    // move_voltage_right_drive(12000); 
    // move_circular_arc(30, 0, MAX_VELOCITY, 135, 180); 
    // move_straight(18, 0, MAX_VELOCITY, 150, false); 

}

void far_side_five_ball(){
    set_drive_brake_mode(MOTOR_BRAKE_HOLD); //Do not want to accidentally overshoot
    ratchet.set_value(RATCHET_INACTIVE);

    //Raise hang to release the intake
    intake.move(127);
    release_intake(); 

    lower_hang(); 

    /*Move forward and back to intake for sure*/
    move_straight(0.9, 0, 0, 0, false); 

    /*Move back with the one triball on the back of the bot after intaking the alliance triball*/
    move_straight(34, 0, 0, 0, true); 
    swing_to_heading(320, false); // heading for descore position  
    stop_hang(); // Stop moving the hang 

    /*
    * Descoring the triball 
    */
    move_straight(14, 0, 0, 320, true); // Move back to descore 
    swing_to_heading(270, false, 400); 

    /*
    * This part of the motion shooots the triball to the net by swinging 
    */
    // turn_to_heading(225); 
    // turn_to_heading(270); 
    turn_to_heading(280, 200); 
    intake.move(0); 
    /*
    * This part of the motion is when we ram into the net with 2 triballs 
    */
    double rearPushAngle = 300.0f; 
    move_straight(18, 0, MAX_VELOCITY, rearPushAngle, true); // Move into the net at a slight angle with  rear of robot 

    // Move away from net 
    move_straight(9, 0, 0, rearPushAngle, false); 
    
    // Now ram triballl inside intake 
    turn_to_heading(110); 
    move_straight(18, 0, MAX_VELOCITY, 110, false); 

    // Move back for final triball grap 
    move_straight(18, 0, 0, 110, true);

    // Start to intake the 3 other gtriballs
    turn_to_heading(25, 200); 
    move_straight(50, 0, 0, 25); 

    turn_to_heading(150); 
    intake.move(-127); 
    pros::delay(750); 
    

}

void near_side_heavy(){
    ratchet.set_value(0); 
    rightWing.set_value(1);
    set_drive_brake_mode(MOTOR_BRAKE_HOLD); 
    release_intake();

    intake.move(127);      
    lower_hang(); 
    rightWing.set_value(0);

    move_straight(54, 0, 0, 0, false); 

    turn_to_heading(88-12); 
    move_straight(20, 0, 0, 88-10, false); 

    swing_to_heading(30, false);
    move_straight(48, 0, 0, 35, true);  

    leftWing.set_value(1); 
    swing_to_heading(300, false); 
    move_straight(10, 0, 0, 300, true); 
    swing_to_heading(255, false); 
    leftWing.set_value(0);
    turn_to_heading(165);
    pros::delay(750);
    turn_to_heading(80);

    intake.move(-127);
    move_straight(30, 0, 0, false);

    //Push descore and under-hang-pipe triball to the other side
    // turn_to_heading(258); 
    // move_straight(18, 0, 0, true);

    // move_voltage_left_drive(0); 
    // move_voltage_right_drive(0); 

    // //Turn around and spit out triball that was intaken
    // turn_to_heading(78);
    // intake.move(-127);
    // pros::delay(500);
}

void far_side_four_ball_rush() {
    release_intake();
    lower_hang();

    // Rush first triball
    intake.move(127);
    move_straight(90, 0, 0, 0, false);

    // Spit it out
    turn_to_heading(100);
    intake.move(-127);
    pros::delay(400);
    
    // Rush the second triball
    turn_to_heading(280);
    intake.move(127);
    move_straight(20, 0, 0, 280, false);

    // Ram the first triball
    move_straight(36, 0, 0, 280, true);

    // Ram second triball
    move_straight(10, 0, 0, 280, false);
    turn_to_heading(100);
    move_straight(12, 0, 0, 100, false);

    // :-)

}

// /*
//     Commnet
// */
// void new_far_side_5_balls() {
//     release_intake();
//     lower_hang();
//     intake.move(127);
//     move_straight(1, 0, 0, 0, false);

//     // Go to the net
//     move_straight(36, 0, 0, 0, true);
//     turn_to_heading(315);
//     move_straight(28, 0, 0, 225, true);
    
//     // Ram triballs
//     turn_to_heading(270);
//     move_straight(22, 0, 0, 270, true);
//     move_straight(10, 0, 0, 270, false);

//     // Ram intake triball
//     turn_to_heading(90);
//     intake.move(-127);
//     move_straight(14, 0, 0, 90, false);

//     // Descore triball   
//     // move_straight(22, 0, 0, 270, false);
//     // swing_to_heading(315, false);
//     // rightWing.set_value(1);

// }

void test_auton() {
    
}