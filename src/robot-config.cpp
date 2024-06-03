#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor INTAKE = motor(PORT10, ratio18_1, false);
motor LEFTMOTOR = motor(PORT5, ratio18_1, true);
motor RIGHTMOTOR = motor(PORT4, ratio18_1, false);
motor LEFTMOTORF = motor(PORT6, ratio18_1, true);
motor RIGHTMOTORF = motor(PORT7, ratio18_1, false);
motor PUSHMotorA = motor(PORT8, ratio36_1, true);
motor PUSHMotorB = motor(PORT9, ratio36_1, false);
motor_group PUSH = motor_group(PUSHMotorA, PUSHMotorB);
digital_out PneumaticH = digital_out(Brain.ThreeWirePort.A);
digital_out PneumaticG = digital_out(Brain.ThreeWirePort.B);
inertial INTERTIAL = inertial(PORT16);
motor LIFT = motor(PORT3, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}