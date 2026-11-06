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

#define DUMPSIZE 200 // 200 speed samples per motor. At 100 Hz, that is approx 2 seconds of data

// Input capture setup without externing
typedef enum { SPEED, KP1, KP2, KI1, KI2, FIELD_COUNT } Field;

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
    
//-------------------   CLI_Init  -------------------
// Configure UART0 for serial full duplex operation
// Inputs: none
// Outputs: none
void CLI_Init(void) {
    UART_Init();

}

// Outputs status to UART0
void CLI_SystemStatus(void){
    
}

// -----------------    CLI_Process   ----------------------------
// This routine is called periodically. It is a command line interface
// to enter values to control the Motor 
// processed one character at a time, does not spin
void CLI_Process(void){
 
    // does not spin
}  





