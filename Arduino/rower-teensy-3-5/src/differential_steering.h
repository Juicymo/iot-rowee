#ifndef DIFFERENTIAL_STEERING_H
#define DIFFERENTIAL_STEERING_H

// INPUTS
extern int     nJoyX;              // Joystick X input                     (-128..+127)
extern int     nJoyY;              // Joystick Y input                     (-128..+127)

// OUTPUTS
extern int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
extern int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

// CONFIG
// - fPivYLimt  : The threshold at which the pivot action starts
//                This threshold is measured in units on the Y-axis
//                away from the X-axis (Y=0). A greater value will assign
//                more of the joystick's range to pivot actions.
//                Allowable range: (0..+127)
extern float fPivYLimit;

void computeDifferentialSteering();

#endif
