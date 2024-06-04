using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor INTAKE;
extern motor LEFTMOTOR;
extern motor RIGHTMOTOR;
extern motor LEFTMOTORF;
extern motor RIGHTMOTORF;
extern motor_group PUSH;
extern digital_out PneumaticH;
extern digital_out PneumaticG;
extern inertial INERTIAL;
extern motor LIFT;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );