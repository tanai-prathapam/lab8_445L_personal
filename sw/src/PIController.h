// ----------------------------------------------------------------------
//
// PIControl.h
// 
// ----------------------------------------------------------------------
// Egan Wang, written 11/9/2026
// JONATHAN, READ THIS BEFORE ADDING CODE TO MAIN OR RUNNING
// There are #defines in this header file that you should use for setting 
// up the left and right motors. While we need to be able to change 
// values on the fly, the #defines should correspond to optimal behavior.
// 
// To use FF mode, treat the FFCONFIG defines as a single variable that 
// handles the multiple inputs into  PIController_changeparamsFF, like so:
//
// PIController_changeparamsFF(*leftController, kp, ki, speed_ref, L_MOTOR_FFCONFIG);
#ifndef PICONTROLLER_H
#define PICONTROLLER_H

#include <math.h>
#include <stdio.h>

// Defines the time step of the controller. 
// Convert the time step (in seconds) to floating point binary, then 
// store the significand (including the leading 1!) in T_S and the 
// exponent in T_S_SHIFT.  
#define T_S 0x1A
#define T_S_SHIFT 8

// The floating-point exponent for kp and ki (let's hope that it's the 
// same for all 4 parameters across 2 motors).
#define K_SHIFT 0

// EXPERIMENTAL: Enables feed-forward calculation to improve transient 
// behavior. MAY BE COMPUTATIONALLY EXPENSIVE AND/OR UNSTABLE.
#define FF_MODE 1

// "Magic Numbers" cooked up by MATLAB script for use in FF mode. 
#define L_MOTOR_FFCONFIG 2691, 88436, 2208822077
#define R_MOTOR_FFCONFIG 2854, 91556, 2418236177

// Gain for both motors in linear approximation is 18/5. Gain in FF motors is 1.

// Controller structure
typedef struct{
  // equal to kp + (Ts/2) * ki
  uint32_t kpi_plus;
  // equal to -kp + (Ts/2) * ki
  uint32_t kpi_minus;
  // Previous output. Used for bilinear transformation
  uint16_t prev_out;
  // Previous input. Used for bilinear transformation
  uint16_t prev_error;
  // The target speed. Keep it in parity with the speed measurement of the 
  // tachometer. 
  uint16_t speed_ref;
  #if FF_MODE
  // Feed-forward quadratic terms. Calculated via quadratic regression. These
  // values are used to solve a quadratic equation to estimate the nonlinear
  // control effort required by the controller. See MATLAB script for more 
  // details.
  uint32_t ff_a, ff_b, ff_c;
  #else
  // The offset duty cycle, roughly corresponding to the maximum duty cycle
  // resulting in no rotation. Useful for reducing the size of startup 
  // transient wackiness. 
  // According to testing, set this to 2500. 
  uint16_t offset_duty;
  #endif
} PIController;

// initialization function. Can be called again to change PI/reference speed parameters
void PIController_changeparams(PIController *controller, uint32_t kp, uint32_t ki, uint16_t speed_ref);
#if FF_MODE
// initialization function for FF mode
void PIController_changeparamsFF(PIController *controller, uint32_t kp, uint32_t ki, uint16_t speed_ref, uint32_t ff_a, uint32_t ff_b, uint32_t ff_c);
#endif
// update function and create output
uint16_t PIController_update(PIController *controller, uint32_t speed_in);
#endif