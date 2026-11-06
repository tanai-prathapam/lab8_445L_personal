// -------------------------------------------------
// CLI.c
// 
// This UART port is for the Command Line Interface to
// Motor controller.
//
// Author:      Daniel Valvano
// Date:        May 23, 2014
// Modified by: Mark McDermott, Jonathan Valvano
// Date:        July 13, 2026
//
//-------------------------------------------------
//
// PA10 is UART0 Tx    
// PA11 is UART0 Rx    


#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "CLI.h"
#include "UART.h"
#include "MotorControl.h"

#define DUMPSIZE 200 // 200 speed samples per motor. At 100 Hz, that is approx 2 seconds of data

// Input capture setup without externing
typedef enum { SPEED, KP1, KP2, KI1, KI2, FIELD_COUNT } Field;

/*
* All in 0.1 RPM
*  target   - speed you want both motors to reach
*  left     - Actual measured left-motor speed
*  right    - Actual measured right-motor speed
*/
typedef struct {
    uint32_t target, left, right;
    int32_t kp1, kp2, ki1, ki2, leftE, rightE, leftU, rightU;
} Status;

static Field field;
static char entry[12]; // sign + 10 decimal digits + terminator
static uint32_t length;
static uint8_t tooLong, skipLF, captureRequested, dumping;
static uint32_t dumpIndex, dumpTarget;
// Save a consistent copy of the motor data so it stays unchanged while the CLI displays it
static uint32_t leftCopy[DUMPSIZE], rightCopy[DUMPSIZE];

static void ClearEntry(void) {
    length = 0;
    entry[0] = '\0';
    tooLong = 0;
}

static void RPM(uint32_t tenths) {
    UART_OutUDec(tenths/ 10);
    UART_OutChar('.');
    UART_OutChar((char)('0' + tenths % 10));
}

static void Snapshot(Status *s) {
    uint32_t saved = __get_PRIMASK();
    __disable_irq();

    s->target = 11; //MC_GetDesiredSpeed();
    // --------- These two currently do not exist in MotorControl.h--------
    s->left = 9; // MC_GetLeftSpeed();
    s->right = 10; // MC_GetRightSpeed();
    // --------------------------------------------------------------------
    s->kp1 = 1; // MC_GetKp1();
    s->kp2 = 2; // MC_GetKp2();

    s->ki1 = 3; // MC_GetKi1(); 
    s->ki2 = 4; // MC_GetKi2();

    s->leftE = 5; // MC_GetLeftE(); 
    s->rightE = 6; // MC_GetRightE();

    s->leftU = 7; // MC_GetLeftU(); 
    s->rightU = 8; // MC_GetRightU();

    __set_PRIMASK(saved);
}

static void Settings(const Status *s) {
    UART_OutString("New Speed = "); RPM(s->target); UART_OutString(" RPM\r\n");
    UART_OutString("Kp1 = "); UART_OutSDec(s->kp1); UART_OutString("\r\n");
    UART_OutString("Kp2 = "); UART_OutSDec(s->kp2); UART_OutString("\r\n");
    UART_OutString("Ki1 = "); UART_OutSDec(s->ki1); UART_OutString("\r\n");
    UART_OutString("Ki2 = "); UART_OutSDec(s->ki2); UART_OutString("\r\n");
}

// Outputs status to UART0
void CLI_SystemStatus(void) {
    Status s;
    Snapshot(&s);
    UART_OutString("\r\nStatus:\r\n");
    Settings(&s);
    UART_OutString("Left: RPM="); RPM(s.left);
    UART_OutString(" E(raw)="); UART_OutSDec(s.leftE);
    UART_OutString(" U(raw)="); UART_OutSDec(s.leftU); UART_OutString("\r\n");
    UART_OutString("Right: RPM="); RPM(s.right);
    UART_OutString(" E(raw)="); UART_OutSDec(s.rightE);
    UART_OutString(" U(raw)="); UART_OutSDec(s.rightU); UART_OutString("\r\n");
}

static void Prompt(void) {
    static const char *const names[FIELD_COUNT] = {
        "speed (whole RPM)", "Kp1", "Kp2 (>0)", "Ki1", "Ki2 (>0)"
    };
    UART_OutString("Enter new "); UART_OutString(names[field]); UART_OutString(": "); UART_OutString(entry);
}

//-------------------   CLI_Init  -------------------
// Configure UART0 for serial full duplex operation
// Inputs: none
// Outputs: none
void CLI_Init(void) {
    UART_Init();
    field = SPEED;
    skipLF = captureRequested = dumping = 0;
    ClearEntry();
    UART_OutString("\r\nLab 8 CLI\r\n");
    UART_OutString("Enter keeps an empty field; BS/DEL edits.\r\n");
    UART_OutString("c/Ctrl-C cancels; s/Ctrl-S shows status; d dumps.\r\n");
    CLI_SystemStatus();
    Prompt();
}

// -----------------    CLI_Process   ----------------------------
// This routine is called periodically. It is a command line interface
// to enter values to control the Motor 
// processed one character at a time, does not spin
void CLI_Process(void){
 
    // does not spin
}  





