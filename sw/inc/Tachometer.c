// Tachmeter.c
// Runs on MSPM0
// Jonathan Valvano
// July 13, 2026

// tachometer
//   PB8  ELA  TA0_C0
//   PB7  ERB  not connected
//   PB6  ELB  not connected
//   PB12 ERA  TA0_C1

#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "../inc/InputCapture.h"
#include "../inc/Median.h"

#define STOP_TICKS 4U


// ****ECE445L write this ****

// Previous capture timestamps
static volatile uint32_t LeftLast;
static volatile uint32_t RightLast;

// Measured periods in microseconds
static volatile uint32_t LeftPeriod;
static volatile uint32_t RightPeriod;

// Number of periodic checks since the last edge
static volatile uint32_t LeftAge;
static volatile uint32_t RightAge;

// Set after the first capture timestamp is available
static volatile uint32_t LeftReady;
static volatile uint32_t RightReady;

// Published speeds in 0.1 RPM
static volatile uint32_t LeftRPM10;
static volatile uint32_t RightRPM10;

uint32_t Tachometer_GetRightrpm(void){
    // ****ECE445L write this ****

   return RightRPM10;
}
uint32_t Tachometer_GetLeftrpm(void){
// ****ECE445L write this ****

  return LeftRPM10;
}


void Tachometer_Init(void){
// ****ECE445L write this ****
  uint32_t mask = __get_PRIMASK();

  __disable_irq();

  LeftLast = 0;
  RightLast = 0;

  LeftPeriod = 0;
  RightPeriod = 0;

  LeftAge = STOP_TICKS;
  RightAge = STOP_TICKS;

  LeftReady = 0;
  RightReady = 0;

  LeftRPM10 = 0;
  RightRPM10 = 0;

  // 80 MHz / 80 = 1 MHz, giving 1 us ticks
  InputCapture_InitDual(INPUTCAPTUREUSEBUSCLK, 79, 1);

  __set_PRIMASK(mask);


}
// ****ECE445L write this ****
// Higher-level software must call this every 10 ms.
void Tachometer_CheckForStopped(void){
  uint32_t leftPeriod;
  uint32_t rightPeriod;
  uint32_t mask = __get_PRIMASK();

  __disable_irq();

  // Advance the time since each wheel's last edge
  if(LeftAge < STOP_TICKS){
    ++LeftAge;
  }

  if(RightAge < STOP_TICKS){
    ++RightAge;
  }

  // Discard stale periods and timestamps
  if(LeftAge >= STOP_TICKS){
    LeftPeriod = 0;
    LeftReady = 0;
  }

  if(RightAge >= STOP_TICKS){
    RightPeriod = 0;
    RightReady = 0;
  }

  // Copy the periods before restoring interrupts
  leftPeriod = LeftPeriod;
  rightPeriod = RightPeriod;

  __set_PRIMASK(mask);

  // RPM10 = 600000000 / (360 * period_us)
  if(leftPeriod != 0U){
    LeftRPM10 = 600000000U / (360U * leftPeriod);
  } else {
    LeftRPM10 = 0;
  }

  if(rightPeriod != 0U){
    RightRPM10 = 600000000U / (360U * rightPeriod);
  } else {
    RightRPM10 = 0;
  }
}


void TIMA0_IRQHandler(void){
  uint32_t iidx = TIMA0->CPU_INT.IIDX; // This acknowledges

  if(iidx == 5){ // CCD0, PB8 ELA, TA0_C0
    uint32_t now = TIMA0->COUNTERREGS.CC_01[0] & 0xFFFFU;

    if(LeftReady){
      LeftPeriod = (LeftLast - now) & 0xFFFFU;
    }

    LeftLast = now;
    LeftReady = 1;
    LeftAge = 0;
  }

  if(iidx == 6){ // CCD1, PB12 ERA, TA0_C1
    uint32_t now = TIMA0->COUNTERREGS.CC_01[1] & 0xFFFFU;

    if(RightReady){
      RightPeriod = (RightLast - now) & 0xFFFFU;
    }

    RightLast = now;
    RightReady = 1;
    RightAge = 0;
  }
}

