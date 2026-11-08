// ----------------------------------------------------------------------
//
// MotorControl.c
// 
// ----------------------------------------------------------------------
// 
//  This code runs the Motor Controller 
// RSLK v2.0.3
// July 13, 2026
// Motor
//   MSPM0
//   PB4  Motor_PWML, ML+, IN3, PWM TIMA1_C0
//   PB1  Motor_PWMR, MR+, IN1, PWM TIMA1_C1
//   PB0  Motor_DIR_L,ML-, IN4, GPIO 0 means forward, 1 means backward
//   PB16 Motor_DIR_R,MR-, IN2, GPIO 0 means forward, 1 means backward
// tachometer
//   PB8  ELA  TA0_C0
//   PB7  ERB  not connected
//   PB6  ELB  not connected
//   PB12 ERA  TA0_C1
// Negative Logic Bumper switches
//   PA27 Left, Bump 0, 
//   PB15 Center Left, Bump 1, 
//   PA28 Center Right, Bump 2
//   PA31 Right, Bump 3

#include <stdint.h>

#include "../inc/MotorControl.h"
#include "../inc/Tachometer.h"
#include <ti/devices/msp/msp.h>

// ****ECE445L write this ****

// Backing static state variables
static uint32_t desiredSpeed = 0;
static int32_t kp1 = 0;
static int32_t kp2 = 1; // Default to 1 to avoid division by zero if applicable
static int32_t ki1 = 0;
static int32_t ki2 = 1; // Default to 1 to avoid division by zero if applicable

static int32_t leftError = 0;
static int32_t rightError = 0;
static int32_t leftActuator = 0;
static int32_t rightActuator = 0;

static uint32_t controlTime = 0;

// Dump buffers (200 speed samples each)
static uint32_t leftSpeedDump[DUMPSIZE];
static uint32_t rightSpeedDump[DUMPSIZE];

// ----------------------------------------------------------------------------
// Speed Getters & Setters
// ----------------------------------------------------------------------------
// desired speeds in 0.1 rpm
void MC_SetDesiredSpeed(uint32_t newSpeed) {
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    desiredSpeed = newSpeed;
    controlTime = 0; // Reset time tracker on speed change
    __set_PRIMASK(saved);
}

uint32_t MC_GetDesiredSpeed(void) {
    return desiredSpeed;
}

uint32_t MC_GetLeftSpeed(void) {
    return Tachometer_GetLeftrpm();
}

uint32_t MC_GetRightSpeed(void) {
    return Tachometer_GetRightrpm();
}

// ----------------------------------------------------------------------------
// Controller Gain Getters & Setters (Kp1, Kp2, Ki1, Ki2)
// ----------------------------------------------------------------------------
void MC_SetKp1(int32_t val) {
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    kp1 = val;
    __set_PRIMASK(saved);
}

int32_t MC_GetKp1(void) {
    return kp1;
}

void MC_SetKp2(int32_t val) {
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    kp2 = val;
    __set_PRIMASK(saved);
}

int32_t MC_GetKp2(void) {
    return kp2;
}

void MC_SetKi1(int32_t val) {
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    ki1 = val;
    __set_PRIMASK(saved);
}

int32_t MC_GetKi1(void) {
    return ki1;
}

void MC_SetKi2(int32_t val) {
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    ki2 = val;
    __set_PRIMASK(saved);
}

int32_t MC_GetKi2(void) {
    return ki2;
}

// ----------------------------------------------------------------------------
// Controller Error and Actuator Output Getters (E and U)
// ----------------------------------------------------------------------------
int32_t MC_GetLeftE(void) {
    return leftError;
}

int32_t MC_GetRightE(void) {
    return rightError;
}

int32_t MC_GetLeftU(void) {
    return leftActuator;
}

int32_t MC_GetRightU(void) {
    return rightActuator;
}

// ----------------------------------------------------------------------------
// Timing and Data Dump Getters
// ----------------------------------------------------------------------------
uint32_t MC_Time(void) {
    return controlTime;
}

uint32_t *MC_DumpLeft(void) {
    return leftSpeedDump;
}

uint32_t *MC_DumpRight(void) {
    return rightSpeedDump;
}
   
// --------------------   MC_PIControlLoop    ----------------------------------
//
// This routine is the main PI control loop. It is run every 10ms. The Count10ms 
// flag indicates if the motor is running. If not, the rev/sec variable is cleared. 
//
void MC_PIControlLoop(void){
  
}

// --------------------- Initialize Motor Control -------------------
//
void MC_Init(void) {

  
  
}

