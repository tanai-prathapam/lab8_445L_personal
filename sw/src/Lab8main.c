/* Lab8main.c
 * Jonathan Valvano
 * v2.0.3
 * July 13, 2026
 *
 */

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/SSD1306.h"
#include "../inc/ADC.h"
#include "../inc/Timer.h"
#include "../inc/Tachometer.h"
#include "PIController.h"
#include "../inc/CLI.h"

#define DEBUG_MODE 2

uint32_t Duty,Period,Change;
// Motor
//   PB4  Motor_PWML=PB.4/TIMA1_C0;
//   PB1  Motor_PWMR=PB.1/TIMA1_C1;
//   PB0  Motor_DIR_L  1 means forward, 0 means backward
//   PB16 Motor_DIR_R, ***change from 2.0.0***
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
// SSD1306 I2C OLED
//   PB2  SCL I2C clock
//   PB3  SDA I2C data
// IR analog distance sensors
//   PA26 Right ADC0_1
//   PB24 Center ADC0_5 (remove J9)
//   PA24 Left ADC0_3
// TF Luna TOF distance sensor
//   TM4C MSPM0
//   PB1  PA8 TxD microcontroller sensor RxD pin 2
//   PB0  PA9 RxD microcontroller sensor TxD pin 3
// LaunchPad pins
//   PA0  red LED1
//   PB22 BLUE LED2
//   PB26 RED LED2
//   PB27 GREEN LED2
//   PA18 S1 positive logic switch
//   PB21 S2 negative logic switch
// Timers
//   TIMA0 for tachometer input capture
//   TIMA1 for PWM to motors
//   TIMG0 100 Hz periodic interrupt 
//         to check for stopped motors
//         to run motor controller
// Remove +5V jumper on RSLK2 from debugger to target
// Remove J9: Disconnects thermistor from PB24
// Remove J16 J17 J18: Disconnects light sensor from the microcontroller PA22 PA27 PA26
// Insert J5: Connects PB22 to blue LED2
// Insert J6: Connects PB26 to red LED2
// Insert J7: Connects PB27 to green LED2
uint32_t Bump;
#if DEBUG_MODE == 1
int main(void){ // use main1 to test bump switches
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
  Bump_Init();
   __enable_irq();
  while(1){
    Bump = Bump_In();
  }
}
#elif DEBUG_MODE == 2
int main(void){ // use main2 to test motors and OLED
  uint32_t sw2,lasts2;

  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator

  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  SSD1306_OutString("RSLK 2");
  Bump_Init();
   __enable_irq();

  Motor_Init(); // 100Hz
  Duty = 4;
  Period = 10000;
  Change = 500;
  lasts2 = LaunchPad_InS2();
  SSD1306_SetCursor(0,1);
  SSD1306_OutString("Period= ");
  SSD1306_OutUDec(Period);
  SSD1306_SetCursor(0,2);
  SSD1306_OutString("Duty= ");
  SSD1306_SetCursor(6,2);
  SSD1306_OutUDec(Duty);
  while(1){
    Clock_Delay(100000); // debounce switch
    if(Bump_In()){
      Motor_Forward(4,4);
      while(Bump_In()){};
      Motor_Forward(Duty,Duty);
    }
    sw2 = LaunchPad_InS2();
    if(sw2 && (lasts2==0)){ // touch s2
      Duty = Duty+Change;
      if(Duty >= Period){ // 10 20 30 ... 90%
        Duty = 4;
      }
      Motor_Forward(Duty,Duty);
      SSD1306_SetCursor(6,2);
      SSD1306_OutUDec(Duty);
    }
    lasts2 = sw2;
  }
}
uint32_t Left,Center,Right;
#elif DEBUG_MODE == 3
int main(void){ // use main3 to test OLED and IR sensors
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  SSD1306_OutString("RSLK 2");
  SSD1306_SetCursor(0,1);
  SSD1306_OutString("Left  =");
  SSD1306_SetCursor(0,2);
  SSD1306_OutString("Center=");
  SSD1306_SetCursor(0,3);
  SSD1306_OutString("Right =");
  SSD1306_SetCursor(0,4);
  SSD1306_OutString("Bump  =");
  //PA26 IR analog distance sensor, Right ADC0_1
  //PB24 IR analog distance sensor, Center ADC0_5
  //PA22 IR analog distance sensor, Left ADC0_3
  //ADC0_Init(5,ADCVREF_VDDA);
  ADC_InitTriple(ADC0,1,5,3,ADCVREF_VDDA);
  Bump_Init();
   __enable_irq();

  while(1){
    Clock_Delay(100000); // slow down
    Bump = Bump_In();
    ADC_InTriple(ADC0,&Right,&Center,&Left);
    SSD1306_SetCursor(7,1);
    SSD1306_OutUDec(Left);
    SSD1306_SetCursor(7,2);
    SSD1306_OutUDec(Center);
    SSD1306_SetCursor(7,3);
    SSD1306_OutUDec(Right);
    SSD1306_SetCursor(7,4);
    SSD1306_OutUHex7(Bump);
  }
}
#elif DEBUG_MODE == 4
int main(void){ // use main5 to test CLI interface
  
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
 
   __enable_irq();
   CLI_Init();
  while(1){
    CLI_Process();
  }
}
#else
int main(void){ // use main4 to run Lab 8
  
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
 
 
   __enable_irq();
  while(1){
  }
}
#endif
// runs in real time at 100 Hz
void TIMG0_IRQHandler(void){
  if((TIMG0->CPU_INT.IIDX) == 1){ // this will acknowledge


  }
}
