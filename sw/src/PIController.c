// ----------------------------------------------------------------------
//
// PIController.c
// 
// ----------------------------------------------------------------------
// 
// Somewhat object-oriented(?) implementation of PI controllers for 445L Lab 8. 
// 5 November, 2026

#include "dq_controller.h"

void PIController_init(PIController *controller, uint32_t kp, uint32_t ki){
  // kp + (Ts/2) * ki; stored separate from division for numerical stability
  controller->kpi_plus = ((T_S * ki)<<(T_S_SHIFT + 1)) + kp;
  // -kp + (Ts/2) * ki; stored separate from division for numerical stability
  controller->kpi_minus = ((T_S * ki)<<(T_S_SHIFT + 1)) - kp;
  // IVP initialization
  controller->prev_out = 0;
  controller->prev_error = 0;
}

uint16_t PIController_update(PIController *controller, uint32_t speed_in){
  controller->prev_out += (controller->kpi_plus * (SPEED_REF - speed_in) + controller->kpi_minus * controller->prev_error)>>K_SHIFT;
  controller->prev_error = SPEED_REF - speed_in;
  return controller->prev_out;
}