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
typedef enum { SPEED, KP1, KP2, KI1, KI2, DUTY } Field;

/*
* All in 0.1 RPM
*  target   - speed you want both motors to reach
*  left     - Actual measured left-motor speed
*  right    - Actual measured right-motor speed
*/
typedef struct {
    uint32_t target, left, right;
    int32_t dutyPct, kp1, kp2, ki1, ki2, leftE, rightE, leftU, rightU;
} Status;

static Field field;
static char entry[12]; // sign + 10 decimal digits + terminator
static uint32_t length;
static uint8_t tooLong, skipLF, dumping;
static uint32_t dumpIndex, dumpTarget;
static int32_t dumpDutyPct;
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

    s->target = MC_GetDesiredSpeed();
    s->dutyPct = MC_GetOpenLoopDuty();
    s->left = MC_GetLeftSpeed();
    s->right = MC_GetRightSpeed();
    s->kp1 = MC_GetKp1();
    s->kp2 = MC_GetKp2();

    s->ki1 = MC_GetKi1(); 
    s->ki2 = MC_GetKi2();

    s->leftE = MC_GetLeftE(); 
    s->rightE = MC_GetRightE();

    s->leftU = MC_GetLeftU(); 
    s->rightU = MC_GetRightU();

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

    if(s.dutyPct >= 0){
        UART_OutString("Open-loop PWM = ");
        UART_OutUDec((uint32_t)s.dutyPct);
        UART_OutString("% for both motors\r\n");
    }

    UART_OutString("Left: RPM="); RPM(s.left);
    UART_OutString(" E(raw)="); UART_OutSDec(s.leftE);
    UART_OutString(" U(raw)="); UART_OutSDec(s.leftU); UART_OutString("\r\n");
    UART_OutString("Right: RPM="); RPM(s.right);
    UART_OutString(" E(raw)="); UART_OutSDec(s.rightE);
    UART_OutString(" U(raw)="); UART_OutSDec(s.rightU); UART_OutString("\r\n");
}

static void Prompt(void) {
    static const char *const names[6] = {
        "speed (whole RPM)", "Kp1", "Kp2 (>0)", "Ki1", "Ki2 (>0)", "open-loop duty (whole %%)"
    };
    if(field == 0) UART_OutChar('\n');
    UART_OutString("Enter new "); UART_OutString(names[field]); UART_OutString(": "); UART_OutString(entry);
}

//-------------------   CLI_Init  -------------------
// Configure UART0 for serial full duplex operation
// Inputs: none
// Outputs: none
void CLI_Init(void) {
    UART_Init();
    field = SPEED;
    skipLF = dumping = 0;
    ClearEntry();
    UART_OutString("\r\n\nLab 8 CLI\r\n");
    UART_OutString("Enter keeps an empty field; BS/DEL edits.\r\n");
    UART_OutString("c/Ctrl-C cancels; s/Ctrl-S shows status; d dumps.\r\n");
    UART_OutString("o sets fixed PWM for both motors; o then 0 stops.\r\n");
    CLI_SystemStatus();
    Prompt();
}

/* Unsigned magnitude arithmetic handles INT32_MIN without signed overflow. */
static uint8_t Parse(int32_t *result){
    uint32_t i = 0, magnitude = 0, limit = INT32_MAX;
    uint8_t negative = (entry[0] == '-');
    if(tooLong || length == 0) return 0;
    if(negative){ i = 1; limit = (uint32_t)INT32_MAX + 1; }
    if(i == length) return 0;
    for(; i < length; ++i){
        uint32_t digit;
        if(entry[i] < '0' || entry[i] > '9') return 0;
        digit = (uint32_t)(entry[i] - '0');
        if(magnitude > (limit - digit) / 10) return 0;
        magnitude = magnitude * 10 + digit;
    }
    if(negative){
        *result = magnitude == (uint32_t)INT32_MAX + 1 ? INT32_MIN : -(int32_t)magnitude;
    }else{
        *result = (int32_t)magnitude;
    }
    return 1;
}

static uint8_t Commit(int32_t value){
    uint32_t saved;
    if(field == SPEED &&
       (value < 0 || (uint32_t)value > CLI_MAX_SPEED_RPM)){
        UART_OutString("Speed must be 0.."); UART_OutUDec(CLI_MAX_SPEED_RPM);
        UART_OutString(" whole RPM.\r\n");
        return 0;
    }
    if((field == KP2 || field == KI2) && value <= 0){
        UART_OutString("Divider must be greater than zero.\r\n");
        return 0;
    }
    if(field == DUTY){
        if(value < 0 || (uint32_t)value > MC_OPEN_LOOP_MAX_DUTY){
            UART_OutString("Duty must be 0.."); UART_OutUDec(MC_OPEN_LOOP_MAX_DUTY);
            UART_OutString(" whole percent.\r\n");
            return 0;
        }
        if(!MC_SetOpenLoopDuty((uint32_t)value)){
            UART_OutString("Bumper pressed; duty command ignored.\r\n");
            return 0;
        }
        return 1;
    }
    saved = __get_PRIMASK();
    __disable_irq();
    switch(field){
        case SPEED: MC_SetDesiredSpeed((uint32_t)value * 10); break;
        case KP1: MC_SetKp1(value); break;
        case KP2: MC_SetKp2(value); break;
        case KI1: MC_SetKi1(value); break;
        case KI2: MC_SetKi2(value); break;
        default: break;
    }
    __set_PRIMASK(saved);
    return 1;
}

static void ConfirmField(void){
    Status s;
    Snapshot(&s);
    switch(field){
        case SPEED: UART_OutString("Speed = "); RPM(s.target); UART_OutString(" RPM"); break;
        case KP1: UART_OutString("Kp1 = "); UART_OutSDec(s.kp1); break;
        case KP2: UART_OutString("Kp2 = "); UART_OutSDec(s.kp2); break;
        case KI1: UART_OutString("Ki1 = "); UART_OutSDec(s.ki1); break;
        case KI2: UART_OutString("Ki2 = "); UART_OutSDec(s.ki2); break;
        default: break;
    }
    UART_OutString("\r\n");
}

static void Enter(void){
    int32_t value;
    UART_OutString("\r\n");
    if(tooLong || (length != 0 && !Parse(&value))){
        UART_OutString("Invalid or overflowing integer; previous value retained.\r\n");
        ClearEntry(); Prompt(); return;
    }
    if(length != 0 && !Commit(value)){
        ClearEntry(); Prompt(); return;
    }

    if(field == DUTY){
        if(length != 0){
            UART_OutString("Open-loop duty = ");
            UART_OutUDec((uint32_t)value);
            UART_OutString("%%. Wait two seconds, then press d.\r\n");
        }
        field = SPEED;
    }else{
        ConfirmField();
        field = field == KI2 ? SPEED : (Field)(field + 1);
    }

    ClearEntry();
    Prompt();
}

static void Cancel(void){
    Status s;
    dumping = 0;
    ClearEntry(); field = SPEED;
    UART_OutString("\r\nEntry cancelled. Accepted values retained.\r\n");
    Snapshot(&s); Settings(&s); Prompt();
}

static void BeginDump(void){
    uint32_t i, saved, ticks;
    uint32_t *left, *right;
    
    saved = __get_PRIMASK();
    __disable_irq();
    ticks = MC_Time();
    if(ticks < DUMPSIZE){
        __set_PRIMASK(saved);
        UART_OutString("\r\nCapture in progress; try d after two seconds.\r\n");
        Prompt(); return;
    }
    left = MC_DumpLeft(); right = MC_DumpRight();
    if(left == 0 || right == 0){
        __set_PRIMASK(saved);
        UART_OutString("\r\nController log unavailable.\r\n"); Prompt(); return;
    }
    dumpTarget = MC_GetDesiredSpeed();
    dumpDutyPct = MC_GetOpenLoopDuty();
    // Foreground-only bounded copy, never UART output with IRQs masked
    // Measure this critical section on target when assessing latency
    for(i = 0U; i < DUMPSIZE; ++i){
        leftCopy[i] = left[i]; rightCopy[i] = right[i];
    }
    __set_PRIMASK(saved);
    dumpIndex = 0; dumping = 1;

    if(dumpDutyPct >= 0){
        UART_OutString("\r\ntime_s,duty_pct,left_rpm,right_rpm\r\n");
    }else{
        UART_OutString("\r\nXstar="); RPM(dumpTarget); UART_OutString(" RPM\r\n");
        UART_OutString("time_s,left_rpm,right_rpm\r\n");
    }
}

static void DumpRow(void){
    UART_OutUDec(dumpIndex / 100); UART_OutChar('.');
    UART_OutChar((char)('0' + (dumpIndex % 100) / 10));
    UART_OutChar((char)('0' + dumpIndex % 10));

    if(dumpDutyPct >= 0){
        UART_OutChar(','); UART_OutUDec((uint32_t)dumpDutyPct);
    }

    UART_OutChar(','); RPM(leftCopy[dumpIndex]);
    UART_OutChar(','); RPM(rightCopy[dumpIndex]); UART_OutString("\r\n");
    ++dumpIndex;
    if(dumpIndex == DUMPSIZE){
        dumping = 0;
        UART_OutString("End of capture.\r\n"); Prompt();
    }
}

// -----------------    CLI_Process   ----------------------------
// This routine is called periodically. It is a command line interface
// to enter values to control the Motor 
// processed one character at a time, does not spin
void CLI_Process(void){
    char c = UART_InCharNonBlock();
    if(c == '\0'){
        if(dumping) DumpRow();
        return;
    }
    if(skipLF){
        skipLF = 0;
        if(c == LF) return;
    }
    if(c == CR) skipLF = 1;
    if(c == 'c' || c == 'C' || c == 0x03){ Cancel(); return; }
    if(dumping){
        // Only cancellation while streaming, to keep CSV uninterrupted
        DumpRow(); return;
    }
    if(c == 's' || c == 'S' || c == 0x13){
        UART_OutChar('\n');
        CLI_SystemStatus(); Prompt(); return;
    }
    if(c == 'd' || c == 'D'){ BeginDump(); return; }
    if(c == 'o' || c == 'O'){
        ClearEntry(); field = DUTY;
        UART_OutString("\r\n"); Prompt(); return;
    }
    if(c == CR || c == LF){ Enter(); return; }
    if(c == BS || c == DEL){
        if(length != 0){
            UART_OutChar('\n');
            entry[--length] = '\0'; UART_OutString("\b \b");
        }
        return;
    }
    if((c >= '0' && c <= '9') || (c == '-' && length == 0)){
        if(length + 1 < sizeof(entry)){
            entry[length++] = c; entry[length] = '\0'; UART_OutChar(c);
        }else{
            tooLong = 1; UART_OutChar('\a');
        }
    }else{
        UART_OutChar('\a');
    }
}  





