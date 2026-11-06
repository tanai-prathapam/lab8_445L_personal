// ----------------------------------------------------------------------
//
// PIControl.h
// 
// ----------------------------------------------------------------------
#ifndef PICONTROLLER_H
#define PICONTROLLER_H

#include <math.h>
#include <stdio.h>

// Defines the time step of the controller. 
// Convert the time step (in seconds) to floating point binary, then 
// store the significand (including the leading 1!) in T_S and the 
// exponent in T_S_SHIFT.  
#define T_S 67
#define T_S_SHIFT 0
// The target speed. Keep it in parity with the speed measurement of the tachometer. 
#define SPEED_REF 0

// The floating-point exponent for kp and ki (let's hope that it's the 
// same for all 4 parameters across 2 motors).
#define K_SHIFT 0

// The offset duty cycle, roughly corresponding to the maximum duty cycle
// resulting in no rotation. Useful for reducing the size of startup 
// transient wackiness. 
#define DUTY_BIAS 0

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
} PIController;

// initialization function
void PIController_init(PIController *controller, uint32_t kp, uint32_t ki);
// update function and create output
uint16_t PIController_update(PIController *controller, uint32_t speed_in);
#endif