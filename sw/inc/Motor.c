/* Motor.c
 * Jonathan Valvano
 * v2.0.3
 * July 13, 2026
// Motor
//   MSPM0
//    PB4  Motor_PWML, ML+, IN3, PWM TIMA1_C0
//    PB1  Motor_PWMR, MR+, IN1, PWM TIMA1_C1
//    PB0  Motor_DIR_L,ML-, IN4, GPIO 0 means forward, 1 means backward
//    PB16 Motor_DIR_R,MR-, IN2, GPIO 0 means forward, 1 means backward

 */
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/PWM1.h"
#include "../inc/Motor.h"
#define DIR_L 1
#define DIR_R (1<<16)
#define PWM6_PERIOD 50000
/*
void PWMG6_Init(uint32_t prescale, uint32_t period, uint32_t duty){
// 32768Hz/256 = 256Hz, 7.8125
  TIMG6->COUNTERREGS.LOAD = period-1; // reload register sets period
  PWMG6_Period = period;
  TIMG6->COUNTERREGS.CTRCTL = 0x02;
// bits 5-4 CM =0, down
// bits 3-1 REPEAT =001, continue
// bit 0 EN enable (0 for disable, 1 for enable)
  TIMG6->COUNTERREGS.CCCTL_01[0] = 0; // no capture
// COC compare mode
  TIMG6->GEN_EVENT0.IMASK = 0x00; // no interrupts
  TIMG6->COMMONREGS.CCPD = 0x01; // output CCP0
  TIMG6->COMMONREGS.CCLKCTL = 1;
  TIMG6->COUNTERREGS.CC_01[0] = PWMG6_Period-duty;
  TIMG6->COUNTERREGS.OCTL_01[0] = 0x0000; // connected to PWM
  TIMG6->COUNTERREGS.CCACT_01[0] = 0x0088;
// bits 10-9 CUACT 0 for no action on up compare
// bits 7-6 CDACT 10 for make low on compare event down
// bits 4-3 LACT 01 for make high on load event
// bits 1-0 ZACT 00 for no action on zero event
  TIMG6->COUNTERREGS.CTRCTL |= 0x01;
}
*/
void Motor_Init(void){
  // ****ECE445L write this ****
  TIMA1->GPRCM.RSTCTL = (uint32_t)0xB1000003;
  TIMA1->GPRCM.PWREN = (uint32_t)0x26000001;
  Clock_Delay(24); // time to power up
  IOMUX->SECCFG.PINCM[PB1INDEX] = 0x00000084; // TIMA1 output CCP1
  IOMUX->SECCFG.PINCM[PB4INDEX] = 0x00000084; // TIMA1 output CCP0
  GPIOB->DOE31_0 |= (1<<1) | (1<<4);
  TIMG6->CLKSEL = 8; // 8=BUSCLK, 4= MFCLK, 2= LFCLK clock
  TIMG6->CLKDIV = 0x01; // divide by 1
  TIMG6->COMMONREGS.CPS = 15; // 1/16x prescale
  TIMG6->COUNTERREGS.LOAD = PWM6_PERIOD - 1; // 16 * 50 000 / 80 000 000 = 10 ms period
  TIMG6->COUNTERREGS.CTRCTL = 0x02;
  // bits 5-4 CM =0, down
  // bits 3-1 REPEAT =001, continue
  // bit 0 EN enable (0 for disable, 1 for enable)
  TIMG6->COUNTERREGS.CCCTL_01[0] = 0; // no capture
  TIMG6->GEN_EVENT0.IMASK = 0x00; // no interrupts
  TIMG6->COMMONREGS.CCPD = 0x03; // output CCP0, CCP1
  TIMG6->COMMONREGS.CCLKCTL = 1;
  TIMG6->COUNTERREGS.CC_01[0] = PWM6_PERIOD>>1; //50% nominal duty cycle for testing
  TIMG6->COUNTERREGS.CC_01[1] = (PWM6_PERIOD * 3)>>2; //25% nominal duty cycle for testing
  TIMG6->COUNTERREGS.OCTL_01[0] = 0x0000; // connected to PWM
  TIMG6->COUNTERREGS.CCACT_01[0] = 0x0088;
// bits 10-9 CUACT 0 for no action on up compare
// bits 7-6 CDACT 10 for make low on compare event down
// bits 4-3 LACT 01 for make high on load event
// bits 1-0 ZACT 00 for no action on zero event
  TIMG6->COUNTERREGS.CTRCTL |= 0x01; //enable
}

void Motor_Forward(uint32_t dutyLeft, uint32_t dutyRight){
  // ****ECE445L write this ****

  
}

void Motor_Backward(uint32_t dutyLeft, uint32_t dutyRight){
    // ****ECE445L write this ****

 
}
void Motor_Right(uint32_t dutyLeft, uint32_t dutyRight){
    // ****ECE445L write this ****

}
void Motor_Left(uint32_t dutyLeft, uint32_t dutyRight){
   // ****ECE445L write this ****


}
