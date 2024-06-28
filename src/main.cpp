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
// INERTIAL            inertial      16              
// LIFT                 motor         3               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;

// Initializing Robot Configuration. DO NOT REMOVE!
competition Competition;

motor_group leftDriveGroup = motor_group(LEFTMOTOR, LEFTMOTORF);
motor_group rightDriveGroup = motor_group(RIGHTMOTOR, RIGHTMOTORF);

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  INERTIAL.calibrate();
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
  rightDriveGroup.spin(directionType::fwd, right_speed, velocityUnits::pct);
  leftDriveGroup.spin(directionType::fwd, left_speed, velocityUnits::pct);
}

void move_inches(double inches, int speed) {
  RIGHTMOTOR.resetPosition();
  move(speed, speed);
  waitUntil(absdbl(RIGHTMOTOR.position(degrees)) >= 15 * inches);
  RIGHTMOTOR.resetPosition();
}


double get_rotation() {
  double currentRotation = INTERTIAL.rotation(degrees);
  // printf("Rotation %f\n", currentRotation);
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

double seconds_to_rpm(double dist, double secs, double circumference, double gearRatio) {
  return (60.0 / (circumference * gearRatio)) * (dist / secs);
}

void move_inches_in_seconds(smartdrive smart_drivetrain, double dist, double secs, double circumference, double gearRatio, directionType dir) {
  smart_drivetrain.driveFor(dir, dist, inches, seconds_to_rpm(dist, secs, circumference, gearRatio), rpm);
}

void autonomous(void) {
  // get currentRotation from INERTIAL position
  const double wheelTravel = 12.566370614359172;
  const double trackWidth = 13.0;
  const double wheelBase = 12.0;

  motor_group leftDriveGroup = motor_group(LEFTMOTOR, LEFTMOTORF);
  motor_group rightDriveGroup = motor_group(RIGHTMOTOR, RIGHTMOTORF);
  smartdrive smart_drivetrain = smartdrive(leftDriveGroup, rightDriveGroup, INERTIAL, wheelTravel, trackWidth, wheelBase, inches);

  PneumaticH.set(true);
  PneumaticG.set(true);

  while (INERTIAL.isCalibrating()) {
    wait(1, msec);
  }

  smart_drivetrain.setRotation(0, degrees);
  
  INTERTIAL.setRotation(0, degrees);

  int globalSpeed = 100;
  int rotateSpeed = 30;

  // move bot forward
  move_inches(7, -globalSpeed);
  stop_robot();

  // rotate towards goal
  rotate_degrees(20, rotateSpeed);
  stop_robot();

  // move bot into goal
  move_inches_in_seconds(smart_drivetrain, 31, 1, wheelTravel, gearRatio, directionType::rev);
  smart_drivetrain.stop(brakeType::hold);

  // rotate bot so it is flat next to goal
  move_motors_timed(-50, 0, 0.7);
  stop_robot();
  lock_bot();

  // make the bot move backwards and hit into the goal
  move_inches(4, 127);
  move_inches(5, -127);
  stop_robot();
  lock_bot();

  // move towards launching area
  move_inches(3.6, globalSpeed);
  stop_robot();
  lock_bot();

  // align the bot against the launching area
  move_motors_timed(0, 50, 1.2);
  stop_robot();
  lock_bot();

  // launch for 30 seconds
  PUSH.spin(directionType::fwd, 50, velocityUnits::pct);
  lock_bot();
  PneumaticH.set(false);
  wait(30, seconds);
  PUSH.spin(directionType::fwd, 0, velocityUnits::pct);
  PneumaticH.set(true);

  // moving the robot forward 
  move_inches(4, -globalSpeed);
  stop_robot();
  lock_bot();

  // spinning bot around so intake faces forward
  orient_bot(30, -rotateSpeed);
  stop_robot();
  lock_bot();

  // turn on intake
  INTAKE.spin(directionType::rev, 127, velocityUnits::pct);

  move_inches(27, globalSpeed);
  stop_robot();
  lock_bot();

  // PUSH.resetPosition();
  // PUSH.spin(directionType::fwd, 127, velocityUnits::pct);
  // waitUntil((absdbl(PUSH.position(degrees)) >= 130));
  // PUSH.stop(brakeType::hold);

  move_motors_timed(50, 0, 0.2);
  stop_robot();
  lock_bot();

  move_inches(75, globalSpeed);
  stop_robot();
  lock_bot();

  // move_motors_timed(127, 127, 0.5);
  // move_motors_timed(-127, -127, 0.5);

  move_motors_timed(50, -50, 0.6);
  stop_robot();

  PneumaticG.set(false);
  
  move_inches(30, globalSpeed);
  stop_robot();

  move_inches(10, -globalSpeed);
  stop_robot();

  move_inches(30, globalSpeed);
  stop_robot();

  move_inches(30, -globalSpeed);
  stop_robot();
  lock_bot();

  rotate_degrees(30, rotateSpeed);
  PneumaticH.set(false);

  move_motors_timed(127, 127, 1);
  move_motors_timed(127, 0, 1);

  move_inches(10, -globalSpeed);
  stop_robot();
  lock_bot();
}

double map(double val, double inputMin, double inputMax, double outputMin, double outputMax) {
  return (val - inputMin) * outputMax / inputMax + outputMin;
}


void usercontrol(void) {
  while (true) {
    double leftMotorSpin = map(Controller1.Axis3.value(), 0, 100, 0, 127);
    double rightMotorSpin = map(Controller1.Axis2.value(), 0, 100, 0, 127);

    // Left side movement
    leftDriveGroup.spin(directionType::fwd, leftMotorSpin, velocityUnits::pct);

    // Right side movement
    rightDriveGroup.spin(directionType::fwd, rightMotorSpin, velocityUnits::pct);

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
      PUSH.spin(directionType::fwd, 50, velocityUnits::pct);
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