/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\0702224                                          */
/*    Created:      Mon Nov 29 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// INTAKE               motor         10              
// LEFTMOTOR            motor         5               
// RIGHTMOTOR           motor         4               
// LEFTMOTORF           motor         6               
// RIGHTMOTORF          motor         7               
// PUSH                 motor_group   8, 9            
// PneumaticH           digital_out   A               
// PneumaticG           digital_out   B               
// INTERTIAL            inertial      16              
// LIFT                 motor         3               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// Initializing Robot Configuration. DO NOT REMOVE!

competition Competition;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  INTERTIAL.calibrate();
  PneumaticH.set(true);
  PneumaticG.set(true);
  
}

double absdbl(double v) {
  if (v < 0) {
    return -v;
  }
  return v;
}

void move(int left_speed, int right_speed) {
  RIGHTMOTOR.spin(directionType::fwd, right_speed, velocityUnits::pct);
  RIGHTMOTORF.spin(directionType::fwd, right_speed, velocityUnits::pct);

  LEFTMOTOR.spin(directionType::fwd, left_speed, velocityUnits::pct);
  LEFTMOTORF.spin(directionType::fwd, left_speed, velocityUnits::pct); 
}

void move_inches(double inches, int speed) {
  RIGHTMOTOR.resetPosition();
  move(speed, speed);
  waitUntil(absdbl(RIGHTMOTOR.position(degrees)) >= 15 * inches);
  RIGHTMOTOR.resetPosition();
}

double get_rotation() {
  double currentRotation = INTERTIAL.rotation(degrees);
  printf("Rotation %f\n", currentRotation);
  return currentRotation;
}

void rotate_degrees(double degree, int speed) {
  // INTERTIAL.setRotation(0, degrees);
  double currentRotation = get_rotation();

  if (degree > 0) {
    move(speed, -speed);
  } else {
    move(-speed, speed);
  }

  waitUntil(absdbl(get_rotation() - currentRotation) >= degree);
  // INTERTIAL.setRotation(0, degrees);
}

void orient_bot(double degree, int speed) {
  double currentRotation = get_rotation();
  double newDegree = absdbl(currentRotation - degree);

  if ((currentRotation - degree) > 0) {
    move(speed, -speed);
  } else {
    move(-speed, speed);
  }

  // // printf("Rotation %f\n", newDegree);
  waitUntil(absdbl(get_rotation() - currentRotation) >= newDegree);
}

void move_motors_timed(int leftSpeed, int rightSpeed, double sec) {
  move(leftSpeed, rightSpeed);
  wait(sec, seconds);
}

void stop_robot() {
  LEFTMOTOR.stop();
  LEFTMOTORF.stop();
  RIGHTMOTOR.stop();
  RIGHTMOTORF.stop();
}

void lock_bot() {
  LEFTMOTOR.setBrake(hold);
  LEFTMOTORF.setBrake(hold);
  RIGHTMOTOR.setBrake(hold);
  RIGHTMOTORF.setBrake(hold);
}

void autonomous(void) {
  // get currentRotation from intertial position

  while (INTERTIAL.isCalibrating()) {
    PneumaticH.set(true);
    PneumaticG.set(true);
    wait(200, msec);
  }
  
  INTERTIAL.setRotation(0, degrees);
  
  int globalSpeed = 100;
  int rotateSpeed = 30;

  // move bot forward
  move_inches(7, -globalSpeed);
  stop_robot();
  lock_bot();

  // rotate towards goal
  rotate_degrees(15, rotateSpeed);
  stop_robot();
  lock_bot();

  // move bot into goal
  move_inches(31, -globalSpeed);
  stop_robot();

  // rotate bot so it is flat next to goal
  move_motors_timed(-50, 0, 0.7);
  stop_robot();

  // make the bot move backwards and hit into the goal
  move_inches(2, globalSpeed);
  move_inches(3, -globalSpeed);
  stop_robot();

  // move towards launching area
  move_inches(3, globalSpeed);
  stop_robot();

  // align the bot against the launching area
  move_motors_timed(0, 50, 1.3);
  stop_robot();

  // launch for 30 seconds
  PUSH.spin(directionType::fwd, 60, velocityUnits::pct);
  lock_bot();
  PneumaticH.set(false);
  wait(30, seconds);
  PUSH.spin(directionType::fwd, 0, velocityUnits::pct);
  PneumaticH.set(true);

  // moving the robot forward 
  move_inches(4, -globalSpeed);
  stop_robot();

  // spinning bot around so intake faces forward
  orient_bot(90, rotateSpeed);
  stop_robot();
  lock_bot();

  // turn on intake
  INTAKE.spin(directionType::fwd, 127, velocityUnits::pct);
  move_inches(7, -globalSpeed);
  stop_robot();
  
  
  // move_inches(3, globalSpeed);
  // move_inches(4, -globalSpeed);
  
  // rotate_degrees(-90, 80);
  // stop_robot();

  // turns robot to the goal

//   LEFTMOTOR.spin(directionType::fwd, -80, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -80, velocityUnits::pct);
//   RIGHTMOTOR.spin(directionType::fwd, -80, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, -80, velocityUnits::pct); 

//   wait(1, seconds);

//   //moves robot to the goal

//   LEFTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct);

//   wait(0.3, seconds);

//     LEFTMOTOR.stop();
//     LEFTMOTORF.stop();
//     RIGHTMOTOR.stop();
//     RIGHTMOTORF.stop();


//   RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 

//   waitUntil((INTERTIAL.rotation(degrees)  <= -5.0));

//   //pulls back and turns to straighten with the goal

//   LEFTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct);
//   RIGHTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct);

//   wait(0.5, seconds);

//   //pushes all set triballs in the goal

//   LEFTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 

//   wait(0.25, seconds);

//   RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct); 


//   waitUntil((INTERTIAL.rotation(degrees)  <= -20));

    
//     LEFTMOTOR.stop();
//     LEFTMOTORF.stop();
//     RIGHTMOTOR.stop();
//     RIGHTMOTORF.stop();

//   LEFTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 

//     wait(0.25, seconds);

//     LEFTMOTOR.stop();
//     LEFTMOTORF.stop();
//     RIGHTMOTOR.stop();
//     RIGHTMOTORF.stop();

//   //pulls back to turn and sets up launching

//   //PUSH.spin(directionType::fwd, 50, velocityUnits::pct);
  
//   PneumaticH.set(false);
//   PneumaticG.set(false);

//  // wait (30, seconds);

//   PneumaticH.set(true);
//   PneumaticG.set(true);

//   //launching here!!!

//     LEFTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 

//     wait(0.5, seconds);

//   //foward to the post

//   //            RIGHTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//  // RIGHTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct); 
//  // LEFTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//  // LEFTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct);

//  // wait(0.1, seconds);

//           RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct); 


//  // waitUntil((INTERTIAL.rotation(degrees)  == 90.0));
//   waitUntil((INTERTIAL.rotation(degrees)  <= -170));

// //turning after launching

// wait(10, seconds);


//   RIGHTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct);

//   wait(0.1, seconds);



//   RIGHTMOTOR.spin(directionType::fwd, 100, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 100, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct); 


//  // waitUntil((INTERTIAL.rotation(degrees)  == 90.0));
//   waitUntil((INTERTIAL.rotation(degrees)  <= -210));

//   INTAKE.spin(directionType::fwd, -127, velocityUnits::pct);

//   RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct);

//   wait(0.5, seconds);



//   RIGHTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 


//  // waitUntil((INTERTIAL.rotation(degrees)  == 90.0));
//   waitUntil((INTERTIAL.rotation(degrees)  >= -250.0));


//     RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 

//   wait(0.5, seconds);

//     RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct); 


//  // waitUntil((INTERTIAL.rotation(degrees)  == 90.0));
//   waitUntil((INTERTIAL.rotation(degrees)  <= -200.0));

//   PneumaticH.set(false);
//   PneumaticG.set(false);

//     RIGHTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct); 

//   wait (1, sec);

//   RIGHTMOTOR.spin(directionType::fwd, 127, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 127, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, 127, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 127, velocityUnits::pct); 

//   wait (4, sec);

//     RIGHTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct); 

//   wait (0.5, sec);

//     RIGHTMOTOR.spin(directionType::fwd, 100, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 100, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, 100, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 100, velocityUnits::pct); 

//   wait (1, sec);

//       RIGHTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -100, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -100, velocityUnits::pct); 

//   wait (0.75, sec);


//       RIGHTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct); 


//  // waitUntil((INTERTIAL.rotation(degrees)  == 90.0));
//   waitUntil((INTERTIAL.rotation(degrees)  <= -290.0));

//     PneumaticH.set(true);
//   PneumaticG.set(true);

//         RIGHTMOTOR.spin(directionType::fwd, 100, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 100, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, 100, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 100, velocityUnits::pct); 

//     wait (1.75, sec);

//   RIGHTMOTOR.spin(directionType::fwd, -60, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, -60, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, 60, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 60, velocityUnits::pct); 

//   waitUntil((INTERTIAL.rotation(degrees)  >= -300.0));


//   RIGHTMOTOR.spin(directionType::fwd, 100, velocityUnits::pct);
//   RIGHTMOTORF.spin(directionType::fwd, 100, velocityUnits::pct); 
//   LEFTMOTOR.spin(directionType::fwd, 100, velocityUnits::pct);
//   LEFTMOTORF.spin(directionType::fwd, 100, velocityUnits::pct); 

//   wait(1, sec);

//      LEFTMOTOR.stop();
//     LEFTMOTORF.stop();
//     RIGHTMOTOR.stop();
//     RIGHTMOTORF.stop();



 // run(127, 127, 1.5);
  // Drives & Pushes Triballs into Goal

 // run(-127, -127, 0.5);
 //   run(-127, 127, 0.45);

  //INTAKE.spin(directionType::fwd, 0, velocityUnits::pct);
//run(10,10,1.7);
 
 
  // RIGHTMOTOR.spin(directionType::fwd, 0, velocityUnits::pct); 
  //  LEFTMOTOR.spin(directionType::fwd, 0, velocityUnits::pct);  

}
  

void usercontrol(void) {

  while (true) {

    LEFTMOTOR.spin(
        directionType::fwd, (Controller1.Axis3.value()),
        velocityUnits::pct); //+ Controller1.Axis4.value())/2,
                             //velocityUnits::pct); (Axis3+Axis4)/2;

    RIGHTMOTOR.spin(
        directionType::fwd, (Controller1.Axis2.value()),
        velocityUnits::pct); //- Controller1.Axis1.value())/2,
                            // velocityUnits::pct);(Axis3-Axis4)/2;
    LEFTMOTORF.spin(
        directionType::fwd, (Controller1.Axis3.value()),
        velocityUnits::pct); //+ Controller1.Axis4.value())/2,
                             //velocityUnits::pct); (Axis3+Axis4)/2;

    RIGHTMOTORF.spin(
        directionType::fwd, (Controller1.Axis2.value()),
        velocityUnits::pct); //- Controller1.Axis1.value())/2,
                            // velocityUnits::pct);(Axis3-Axis4)/2;

   if(Controller1.Axis3.value()>= -10 && Controller1.Axis3.value()<= 10)
      
   LEFTMOTOR.stop(brakeType::hold);

if(Controller1.Axis2.value()>= -10 && Controller1.Axis2.value()<= 10)    
  
   RIGHTMOTOR.stop(brakeType::hold);

    if (Controller1.ButtonR2.pressing())
      INTAKE.spin(directionType::fwd, -127, velocityUnits::pct);

          if (Controller1.ButtonR1.pressing())
      INTAKE.spin(directionType::fwd, 127, velocityUnits::pct);

          if (Controller1.ButtonB.pressing())
      LIFT.spin(directionType::fwd, -20, velocityUnits::pct);
      
     else
     LIFT.stop();

    if (Controller1.ButtonL1.pressing())
      INTAKE.spin(directionType::fwd, 0, velocityUnits::pct);
    

    if (Controller1.ButtonX.pressing())
      PUSH.spin(directionType::fwd, 50, velocityUnits::pct);

    

    else
      PUSH.stop(brakeType::hold);

      
          if (Controller1.ButtonY.pressing())
      PUSH.spin(directionType::rev, 50, velocityUnits::pct);





      PneumaticH.set(true);
      PneumaticG.set(true);

    if (Controller1.ButtonL2.pressing())
      PneumaticH.set(false);

    //else
     // PneumaticH.set(true);

        if (Controller1.ButtonL2.pressing())
      PneumaticG.set(false);

    //else
    //  PneumaticG.set(true);

      //  if (Controller1.ButtonRight.pressing())
     // PneumaticG.set(true);

      //        if (Controller1.ButtonRight.pressing())
     // PneumaticH.set(true);
      
   // if (Controller1.ButtonUp.pressing())
   //   Lift.set(true);

   // else
   //   Lift.set(false);

    wait(20, msec);
  }
}

int main() {

  Competition.autonomous(autonomous);

  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}