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
#include <iostream>

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


double map(double val, double inputMin, double inputMax, double outputMin, double outputMax) {
  return (val - inputMin) * outputMax / inputMax + outputMin;
}

double controlCurve(double val) {
  if (val == 0) return 0;

  if (val > 0)
    return 0.00787402 * val * val;
  
  return -0.00787402 * val * val;
}

void move(int left_speed, int right_speed) {
  // move right side
  RIGHTMOTOR.spin(directionType::fwd, right_speed, velocityUnits::pct);
  RIGHTMOTORF.spin(directionType::fwd, right_speed, velocityUnits::pct);
  // move left side
  LEFTMOTOR.spin(directionType::fwd, left_speed, velocityUnits::pct);
  LEFTMOTORF.spin(directionType::fwd, left_speed, velocityUnits::pct); 
}

void move_motors_timed(int leftSpeed, int rightSpeed, double sec) {
  move(leftSpeed, rightSpeed);
  wait(sec, seconds);
}

double seconds_to_rpm(double dist, double secs, double circumference, double gearRatio) {
  return (60.0 / (circumference * gearRatio)) * (dist / secs);
}

void move_inches_in_seconds(smartdrive smart_drivetrain, double dist, double secs, double circumference, double gearRatio, directionType dir) {
  smart_drivetrain.driveFor(dir, dist, inches, seconds_to_rpm(dist, secs, circumference, gearRatio), rpm);
}

void autonomous(void) {
  // get currentRotation from INTERTIAL position
  const double wheelTravel = 12.566370614359172;
  const double trackWidth = 13.0;
  const double wheelBase = 12.0;

  motor_group leftDriveGroup = motor_group(LEFTMOTOR, LEFTMOTORF);
  motor_group rightDriveGroup = motor_group(RIGHTMOTOR, RIGHTMOTORF);
  smartdrive smart_drivetrain = smartdrive(leftDriveGroup, rightDriveGroup, INTERTIAL, wheelTravel, trackWidth, wheelBase, inches);

  PneumaticH.set(true);
  PneumaticG.set(true);

  while (INTERTIAL.isCalibrating()) {
    wait(1, msec);
  }

  smart_drivetrain.setRotation(0, degrees);
  
  int rotateSpeed = 99999;

  double gearRatio = 36.0 / 60.0;

  // move bot forward
  move_inches_in_seconds(smart_drivetrain, 7, 1, wheelTravel, gearRatio, directionType::rev);
  smart_drivetrain.stop(brakeType::hold);

  // rotate towards goal
  smart_drivetrain.turnFor(right, 15, degrees, rotateSpeed);
  smart_drivetrain.stop(brakeType::hold);

  // move bot into goal
  move_inches_in_seconds(smart_drivetrain, 31, 1, wheelTravel, gearRatio, directionType::rev);
  smart_drivetrain.stop(brakeType::hold);

  // rotate bot so it is flat next to goal
  move_motors_timed(-50, 0, 0.7);
  smart_drivetrain.stop(brakeType::hold);

  // make the bot move backwards and hit into the goal
  move_inches_in_seconds(smart_drivetrain, 2, 1, wheelTravel, gearRatio, fwd);
  move_inches_in_seconds(smart_drivetrain, 3, 1, wheelTravel, gearRatio, directionType::rev);
  smart_drivetrain.stop(brakeType::hold);

  // move towards launching area
  move_inches_in_seconds(smart_drivetrain, 3, 1, wheelTravel, gearRatio, fwd);
  smart_drivetrain.stop(brakeType::hold);

  // align the bot against the launching area
  move_motors_timed(0, 50, 1.3);
  smart_drivetrain.stop(brakeType::hold);

  // launch for 30 seconds
  PUSH.spin(directionType::fwd, 60, velocityUnits::pct);
  smart_drivetrain.stop(brakeType::hold);
  PneumaticH.set(false);
  wait(30, seconds);
  PUSH.spin(directionType::fwd, 0, velocityUnits::pct);
  PneumaticH.set(true);

  // moving the robot forward 
  move_inches_in_seconds(smart_drivetrain, 4, 1, wheelTravel, gearRatio, directionType::rev);
  smart_drivetrain.stop(brakeType::hold);

  // spinning bot around so intake faces forward
  smart_drivetrain.turnToRotation(90, degrees, rotateSpeed);
  smart_drivetrain.stop(brakeType::hold);

  // turn on intake
  INTAKE.spin(directionType::fwd, 127, velocityUnits::pct);
  move_inches_in_seconds(smart_drivetrain, 7, 1, wheelTravel, gearRatio, directionType::rev);
  smart_drivetrain.stop(brakeType::hold);
}
  

void usercontrol(void) {
  while (true) {
    double leftMotorSpin = controlCurve(1.27 * Controller1.Axis3.value());
    double rightMotorSpin = controlCurve(1.27 * Controller1.Axis2.value());

    // Left side movement
    LEFTMOTOR.spin(
      directionType::fwd, leftMotorSpin,
      velocityUnits::pct);

    LEFTMOTORF.spin(
      directionType::fwd, leftMotorSpin,
      velocityUnits::pct);

    // Right side movement
    RIGHTMOTOR.spin(
      directionType::fwd, rightMotorSpin,
      velocityUnits::pct);

    RIGHTMOTORF.spin(
      directionType::fwd, rightMotorSpin,
      velocityUnits::pct);
  

    // left side deadzone
    if (abs(Controller1.Axis3.value()) < 1) {
      LEFTMOTOR.stop(brakeType::hold);
      LEFTMOTORF.stop(brakeType::hold);
    }

    // right side deadzone
    if (abs(Controller1.Axis2.value()) < 1) {
      RIGHTMOTOR.stop(brakeType::hold);
      RIGHTMOTORF.stop(brakeType::hold);
    }

    if (Controller1.ButtonA.pressing()) {
      LIFT.spin(directionType::fwd, 40, velocityUnits::pct);
    } else if (Controller1.ButtonB.pressing()) {
      LIFT.spin(directionType::fwd, -40, velocityUnits::pct);
    } else {
      LIFT.stop(brakeType::hold);
    }

    if (Controller1.ButtonX.pressing()) {
      PUSH.spin(directionType::fwd, 127, velocityUnits::pct);
    } else if (Controller1.ButtonY.pressing()) {
      PUSH.spin(directionType::rev, 50, velocityUnits::pct);
    } else {
      PUSH.stop(brakeType::hold);
    }

    // Top Buttons

    // start spinning intake forwards
    if (Controller1.ButtonR1.pressing()) {
      INTAKE.spin(directionType::fwd, 127, velocityUnits::pct);
    }

    // start spinning intake backwards
    if (Controller1.ButtonR2.pressing()) {
      INTAKE.spin(directionType::fwd, -127, velocityUnits::pct);
    }

    // stop spinning intake
    if (Controller1.ButtonL1.pressing()) {
      INTAKE.spin(directionType::fwd, 0, velocityUnits::pct);
    }

    // Always close flaps
    PneumaticH.set(true);
    PneumaticG.set(true);

    // unless L2 is pressed
    if (Controller1.ButtonL2.pressing()) {
      PneumaticH.set(false);
      PneumaticG.set(false);
    }

    wait(1, msec);
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