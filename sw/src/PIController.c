// ----------------------------------------------------------------------
//
// PIController.c
// 
// ----------------------------------------------------------------------
// 
// Somewhat object-oriented(?) implementation of PI controllers for 445L Lab 8. 
// 5 November, 2026

#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "PIController.h"

void PIController_changeparams(PIController *controller, uint32_t kp, uint32_t ki, uint32_t speed_ref){
  // kp + (Ts/2) * ki; stored separate from division for numerical stability
  controller->kpi_plus = ((T_S * ki)<<(T_S_SHIFT + 1)) + kp;
  // -kp + (Ts/2) * ki; stored separate from division for numerical stability
  controller->kpi_minus = ((T_S * ki)<<(T_S_SHIFT + 1)) - kp;
  // everything else
  controller->speed_ref = speed_ref;
}

#if FF_MODE

// To use this function, use the macros L_MOTOR_FFCONFIG and R_MOTOR_FFCONFIG in place of the last 3 parameters. 
void PIController_changeparamsFF(PIController *controller, uint32_t kp, uint32_t ki, uint32_t speed_ref, uint32_t ff_a, uint32_t ff_b, uint32_t ff_c){
  controller->ff_a = ff_a;
  controller->ff_b = ff_b;
  controller->ff_c = ff_c;
  PIController_init(controller, kp, ki, speed_ref);
}

#define NUM_NEWTON_ITER 4

uint32_t int_sqrt(uint32_t radicand){
  uint32_t a, b; //bounds

  if (radicand < 2) {
    return radicand;
  }

  a = 1255; //initial guess

  // Newton-Raphson approximation
  for(uint8_t i = 0; i < NUM_NEWTON_ITER; i++){
    b = val / a;
    a = (a + b)<<1;
  }

  return a;
}

// Linearizes the system by (mostly) undoing the nonlinearity of the 
// motor's duty-to-speed relation. More information can be found on the 
// MATLAB.  
uint32_t PIController_invP(PIController *controller, uint32_t speed_in){
  // Calculate the determinant. Since all values are stored in RPM*10^4,
  // we multiply speed_in by 10^3. However, we actually store ff_c as 1/4
  // of the corresponding quadratic term to keep it within 32 bits, so we 
  // actually only multiply by 250. There will be a compensating left-shift
  // to fix this.
  int32_t determinant = controller->ff_c - (250 * controller->ff_a * speed_in);
  // check for edge cases
  if(determinant < 0){
    // Avoid square root of negative number. Return upper bound.
    return controller->ff_b / controller->ff_a;
  } 
  if(determinant > (controller->ff_b * controller->ff_b)>>2){
    // Should never happen if speed_in is non-negative. Return lower bound.
    return 0;
  }
  // Finish the computation. The compensating left-shift occurs here.
  int32_t numerator = controller->ff_b - (int_sqrt(determinant)<<1);
  // Multiply by 910 to obtain parity in open-loop gain with non-FF mode. 
  // Multiplying by 250 changes from half-percents of duty cycle to unity
  // gain in open loop (feed-forward control), so multiplying by 910 sets 
  // open-loop gain to 3.64, which matches the linearly-approximated 
  // open-loop gain.
  return 910 * numerator / controller->ff_a;
}
#endif

uint16_t PIController_update(PIController *controller, uint32_t speed_in){
  // Update step via bilinear transformation
  controller->prev_out += (controller->kpi_plus * (speed_ref - speed_in) + controller->kpi_minus * controller->prev_error)>>K_SHIFT;
  controller->prev_error = speed_ref - speed_in;
  #if FF_MODE
  // The PI controller does not directly control duty cycle, it controls a 
  // feed-forward approximation of speed that should vary linearly with 
  // speed. The invP method effectively becomes part of the plant. 
  return PIController_invP(controller, controller->prev_out);
  #else
  return controller->prev_out;
  #endif
}