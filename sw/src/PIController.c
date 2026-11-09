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

void PIController_changeparams(PIController *controller, uint32_t kp, uint32_t ki, uint16_t speed_ref){
  // kp + (Ts/2) * ki; stored separate from division for numerical stability
  controller->kpi_plus = ki + (T_S * 2 * kp);
  // -kp + (Ts/2) * ki; stored separate from division for numerical stability
  controller->kpi_minus = ki - (T_S * 2 * kp);
  // everything else
  controller->speed_ref = speed_ref;
}

#if FF_MODE

// To use this function, use the macros L_MOTOR_FFCONFIG and R_MOTOR_FFCONFIG in place of the last 3 parameters. 
void PIController_changeparamsFF(PIController *controller, uint32_t kp, uint32_t ki, uint16_t speed_ref, uint32_t ff_a, uint32_t ff_b, uint32_t ff_c){
  controller->ff_a = ff_a;
  controller->ff_b = ff_b;
  controller->ff_c = ff_c;
  PIController_changeparams(controller, kp, ki, speed_ref);
}

#define NUM_NEWTON_ITER 8

uint32_t int_sqrt(uint32_t n) {
  uint64_t left = 0, right = n, mid;
  while (left < right-1) {
      mid = (left>>1) + ((right + 1)>>1);
      if (mid * mid <= n) {
          left = mid;
      } else {
          right = mid - 1;
      }
  }
  return left;
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
  int32_t determinant = controller->ff_c - (2500 * controller->ff_a * speed_in);
  // check for edge cases
  if(determinant < 0){
    // Avoid square root of negative number. Return upper bound.
    return controller->ff_b / controller->ff_a;
  } 
  // Finish the computation. The compensating left-shift occurs here.
  int32_t numerator = controller->ff_b - (int_sqrt(determinant)<<1);
  if( numerator < 0 ) return 0;
  // Multiply to obtain parity in open-loop gain with non-FF mode. 
  // Multiplying by 250 changes from half-percents of duty cycle to unity
  // gain in open loop (feed-forward control), so multiplying by 29/16 sets 
  // DC gain of the plant to 0.0073, which matches the linearly-approximated 
  // open-loop gain.
  // return (29 * numerator / controller->ff_a)>>4;
  return (1000 * numerator/ controller->ff_a);
}
#endif

uint16_t PIController_update(PIController *controller, uint32_t speed_in){
  // Update step via bilinear transformation
  controller->prev_out += (((controller->kpi_plus * (controller->speed_ref - speed_in) / RPM_SCALE + controller->kpi_minus * controller->prev_error) / K_DENOM)>>(K_SHIFT+1)) / T_S;
  controller->prev_error = controller->speed_ref - speed_in;
  #if FF_MODE
  // The PI controller does not directly control duty cycle, it controls a 
  // feed-forward approximation of speed that should vary linearly with 
  // speed. The invP method effectively becomes part of the plant. 
  return PIController_invP(controller, controller->prev_out);
  #else
  return controller->prev_out;
  #endif
}