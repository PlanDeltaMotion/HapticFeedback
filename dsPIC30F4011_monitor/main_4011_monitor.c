/*
 * File:   main.c
 * Author: PaoloDantes
 *
 * Created on April 17, 2015, 2:23 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "p30f4011.h"

#define PIC_ID 7
#define GET_POSDATA     positionData[0]
#define CHECK_RXID      positionData[1]
#define CHECK_TXDATA    positionData[2]
#define GET_HAPTICDATA  positionData[3]
#define VALID_SLAVEDATA PIC_ID-3
#define VALID_MASTERDATA PIC_ID-3
#define VALID_RXID      PIC_ID-1
#define SEND_POSDATA    C1TX0B1 = POSCNT
#define SEND_PIC_ID     C1TX0B2 = PIC_ID
#define UART_TX_LEN     36
#define MASTER1_ID  1
#define MASTER2_ID  2
#define MASTER3_ID  3
#define SLAVE1_ID   4
#define SLAVE2_ID   5
#define SLAVE3_ID   6

// LCD pin outs
#define RS      LATEbits.LATE0 // RED
#define RW      LATEbits.LATE1 // BLUE
#define EN      LATEbits.LATE2 // ORANGE
#define DATA    LATB
#define LEDRED  LATEbits.LATE3 // YELLOW
#define LEDGRN  LATEbits.LATE4 // GREEN
#define TRISRED  TRISEbits.TRISE3 // YELLOW
#define TRISGRN  TRISEbits.TRISE4 // GREEN

// Display commands for LCD Sreen (LMB162ABC)
#define CLEAR_DISPLAY       0x01 // Writes "20h" (ASCII for space
#define RETURN_HOME         0X02
#define ENTRY_MODE_SET      0x06 // No screen shifting, AC=AC+1
#define DISPLAY_ON          0x0F // Display on, cursor on, cursor blinking on
#define CURSOR_SHIFT        0x14 // Shift cursor to right side
#define FUNCTION_SET        0x28 // 4-bit interface, 2-line display, 5x8 dots font
#define SET_CGRAM_ADD       0x40 // OR this with AC0-AC5 CGRAM address
#define SET_DDRAM_ADD       0x80 // OR this with AC0-AC6 DDRAM address

// CAN bit timing
#define FOSC        7370000 // 7.37MHz
#define FCY         FOSC/4
#define BITRATE     100000  // 100kbps
#define NTQ         16      // Amount of Tq per bit time
#define BRP_VAL     (((4*FCY)/(2*NTQ*BITRATE))-1) // refer to pg. 693 of Family Reference

// CAN Operation Modes
#define CONFIG_MODE     4
#define LISTEN_ALL      7 // DO NOT USE
#define LISTEN_ONLY     3
#define LOOPBACK        2
#define DISABLE         1
#define NORMAL          0

// Define motorState state values
#define INITIALIZE          0
#define READ_INPUT_OUTPUT   1
#define CALCULATE_PID       2
#define UPDATE_PID          3
#define IDLE                4

// Define PID controls
#define PID_KP  1
#define PID_KD  0.1
#define PID_KI  0
#define PID_TI  0
#define PID_TD  0
#define PID_TS  10
#define PID_N   10

// Define PWM constants
#define PWM_FREQUENCY       16000
#define PWM_PRESCALER       0       // 0=1:1 1=1:4 2=1:16 3=1:64
#define PWM_COUNTS_PERIOD   (FCY/PWM_FREQUENCY)-1   // 16kHz for FRC


// pid_t type
typedef struct {
    float Kp, Kd, Ki, T;
    unsigned short N;       // derivative filter parameter (3-20)
    float i, ilast;         // integral --> NOT USED IN THIS VERSION. FOR FUTURE IMPLEMENTATION
    float y, ylast;         // output
    float d, dlast;         // derivative term
    float u;                // u=uc-y
    float e, elast;            // error
} pid_t;


//// CAN1 MODULE
unsigned int OutData0[4] = {0xAAAA, 0x5555, 0xAAAA, 0xAAAA};
unsigned int masterData[4] = {0, 0, 0, 0};
unsigned int slaveData[4] = {0, 0, 0, 0};
unsigned int motorState = 0;
unsigned char txInProgress, sendMsg = 1;

void InitCan(void) {
    // Initializing CAN Module Control Register
    C1CTRLbits.REQOP = CONFIG_MODE; // 4 = Configuration mode
    C1CTRLbits.CANCAP = 1; // Enable CAN capture
    C1CTRLbits.CSIDL = 0; // 0 = Continue CAN module op in idle mode
    C1CTRLbits.CANCKS = 0; // 1: Fcan=Fcy 0: Fcan=4Fcy
    C1CFG1bits.SJW = 0; // Synchronized jump width is 1xTq
    C1CFG1bits.BRP = 8; // Baud rate prescaler = 20 (CAN baud rate of 100kHz
    C1CFG2bits.SEG2PHTS = 1; // 1=Freely Programmable 0=Maximum of SEG1PH or 3Tq's whichever is greater
    C1CFG2bits.PRSEG = 1; // Propagation Segment = 2Tq
    C1CFG2bits.SEG1PH = 6; // Phase Buffer Segment 1 = 7Tq
    C1CFG2bits.SEG2PH = 5; // Phase Buffer Segment 2 = 6Tq
    C1CFG2bits.SAM = 1; // 1=Bus line sampled 3 times 0=Bus line sampled once

    // Initializing CAN interrupt
    C1INTF = 0; // Reset all CAN interrupts
    IFS1bits.C1IF = 0; // Reset Interrupt flag status register
    C1INTE = 0x00FF; // Enable all CAN interrupt sources
    IEC1bits.C1IE = 1; // Enable CAN1 Interrupt

    /*---------------------------------------------------------
     *  CONFIGURE RECEIVER REGISTERS, FILTERS AND MASKS
     *---------------------------------------------------------*/

    // Configure CAN Module Receive Buffer Register
    C1RX0CONbits.DBEN = 0; // Buffer 0 does not overflow in buffer 1

    // Initializing Acceptance Mask Register
//    C1RXM0SID = 0x1FFD;
    C1RXM0SID = 0x01;

    // Initializing Message Acceptance filter
    C1RXF0SID = 0x0AA8; // 0x0FA

    /*---------------------------------------------------------
     *  CONFIGURE RECEIVER REGISTERS, FILTERS AND MASKS
     *---------------------------------------------------------*/

    // Configure CAN Module Transmit Buffer Register
    C1TX0CONbits.TXPRI = 2; // 2 = High intermediate message priority

    // Initializing Transmit SID
    C1TX0SID = 0X50A8;
    C1TX0DLCbits.DLC = 8; // Data length is 8bytes

    // Data Field 1,Data Field 2, Data Field 3, Data Field 4 // 8 bytes selected by DLC

    C1TX0B1 = OutData0[0];
    C1TX0B2 = OutData0[1];
    C1TX0B3 = OutData0[2];
    C1TX0B4 = OutData0[3];
}

void InitInt(void){
    INTCON1bits.NSTDIS = 0;         // Interrupt nesting is disabled
    IEC0bits.INT0IE = 0;            // Disable external interrupt 0
    INTCON2bits.ALTIVT = 0;         // Default interrupt vector table
    INTCON2bits.INT0EP = 1;         // Detect on 1 = negative edge 0 = positive for interrupt 0
    IPC0bits.INT0IP = 6;            // External interrupt 0 is priority 5
    IFS0bits.INT0IF = 0;            // Clear external interrupt 0 flag
    IEC0bits.INT0IE = 1;            // Enable external interrupt 0
    return;
}

void InitQEI(void)
{
    ADPCFG |= 0x003B;           // RB3, RB4, RB5 configured to digital pin
    QEICONbits.QEIM = 0;        // Disable QEI module
    QEICONbits.CNTERR = 0;      // Clear any count errors
    QEICONbits.QEISIDL = 0;     // Continue operation during sleep
    QEICONbits.SWPAB = 0;       // QEA and QEB not swapped
    QEICONbits.PCDOUT = 0;      // Normal I/O pin operation
    QEICONbits.POSRES = 1;      // Index pulse resets position counter
    QEICONbits.TQCS = 0;        // Internal clock source (Fcy) = 2Mhz
    DFLTCONbits.CEID = 1;       // Count error interrupts disabled
    DFLTCONbits.QEOUT = 1;      // Digital filters output enabled for QEn pins
    DFLTCONbits.QECK = 2;       // 1:4 clock divide for digital filter for QEn
                                // FILTER_DIV = (MIPS*FILTERED_PULSE)/3
                                //  ==> 5MHz*5usec/3 = 3.33 --> 2
//    DFLTCONbits.IMV1 = 1;       // Required state of phase B input signal for match on index pulse
    POSCNT = 12000;                 // Reset position counter
    QEICONbits.QEIM = 7;        // X4 mode with position counter reset by
                                // 6 - index pulse
                                // 7 - match (MAXCNT)
    return;
}

void InitPwm(void)
{
    PTCONbits.PTEN = 0;                 // Disable PWM timerbase
    PTCONbits.PTCKPS = PWM_PRESCALER;   // prescaler
    PTCONbits.PTOPS = 0;                // 1:1 postscaler
    PTCONbits.PTMOD = 0;                // free running mode
    PWMCON1bits.PMOD1 = 1;              // PWM in independent mode
    PWMCON1bits.PMOD2 = 1;              // PWM in independent mode
    PWMCON1bits.PEN1L = 1;              // PWM1L (PIN 24) output controlled by PWM
    PWMCON1bits.PEN2L = 1;              // PWM2L (PIN 26) output controlled by PWM
    PTMR = 0;                           // PWM counter value start at 0
    PTPER = (unsigned int) PWM_COUNTS_PERIOD;          // Set PWM period
//    PTCONbits.PTEN = 1;                 // enable PWM timerbase
    return;
}

void InitUart(){
    U1MODEbits.UARTEN = 0;      // UART is disabled
    U1MODEbits.USIDL = 0;       // Continue operation in Idle Mode
    U1MODEbits.ALTIO = 1;       // UART communicates using U1ATX and U1ARX (pins 11&12)
    U1MODEbits.WAKE = 1;        // Enable wake-up on Start bit detec durign sleep mode
    U1MODEbits.PDSEL = 0;       // 8-bit data, no parity
    U1MODEbits.STSEL = 0;       // 2 stop bits

    U1STAbits.UTXISEL = 0;      // Interrupt when TX buffer has one character empty
    U1STAbits.UTXBRK = 0;       // U1TX pin operates normally
    U1STAbits.URXISEL = 0;      // Interrupt when word moves from REG to RX buffer
    U1STAbits.ADDEN = 0;        // Address detect mode disabled

    U1BRG = 11;                 // p.507 of family reference
                                // 9600 baud rate

    IFS0bits.U1TXIF = 0;        // Clear U1TX interrupt
    IFS0bits.U1RXIF = 0;        // Clear U1RX interrupt
    IPC2bits.U1TXIP = 5;        // U1TX interrupt 5 priority
    IPC2bits.U1RXIP = 5;        // U1RX interrupt 5 priority
    IEC0bits.U1TXIE = 1;        // Enable U1TX interrupt
    IEC0bits.U1RXIE = 1;        // Enable U1RX interrupt

    U1MODEbits.LPBACK = 0;      // Enable loopback mode
    U1MODEbits.UARTEN = 1;      // UART is enabled
    U1STAbits.UTXEN = 1;        // U1TX pin enabled

}

void msDelay(unsigned int mseconds) //For counting time in ms
//-----------------------------------------------------------------
{
    int i;
    int j;
    for (i = mseconds; i; i--) {
        for (j = 265; j; j--) {
            // 5667 for XT_PLL8 -> Fcy = 34MHz
            // 714 for 20MHz oscillator -> Fcy=4.3MHz
            // 265 for FRC
        }
    }
    return;
}
void InitPid(pid_t *p, float kp, float kd, float ki, float T, unsigned short N, float il, float yl, float dl, float el)
{
    p->Kp = kp;
    p->Kd = kd;
    p->Ki = ki;
    p->T = T;
    p->N = N;
    p->ilast = il;
    p->ylast = yl;
    p->dlast = dl;
    p->elast = el;
}

void UpdatePid(pid_t *mypid)
{
   // Update PD variables
    mypid->ylast = mypid->y;
    mypid->dlast = mypid->d;
    mypid->elast = mypid->e;
}

unsigned int * CalcPid(pid_t *mypid, float degPOT, float degMTR)
{
    float pidOutDutyCycle;
    static unsigned int pwmOUT[2] = {0, 0};

    mypid->y = degMTR;
    mypid->e = degPOT-degMTR;

    mypid->d = (mypid->T*mypid->dlast)/mypid->N + (mypid->Kd*mypid->T)*(mypid->y-mypid->ylast)/(mypid->T+(mypid->T/mypid->N));
    mypid->u = mypid->Kp*mypid->e;//+mypid->d;

    pidOutDutyCycle = (float) ((mypid->u/degPOT)*2*PWM_COUNTS_PERIOD);


    if (pidOutDutyCycle >= 2*PWM_COUNTS_PERIOD){
        pwmOUT[0] = (unsigned int) (2*PWM_COUNTS_PERIOD);
        pwmOUT[1] = 0;
    }
    else if (pidOutDutyCycle <= -(2*PWM_COUNTS_PERIOD)){
        pwmOUT[0] = 0;
        pwmOUT[1] = (unsigned int) (2*PWM_COUNTS_PERIOD);
    }
    else if (pidOutDutyCycle <0){
        pwmOUT[0] = 0;
        pwmOUT[1] = (unsigned int)((-1)*2*pidOutDutyCycle);
    }
    else{
        pwmOUT[0] = (unsigned int)(2*pidOutDutyCycle);
        pwmOUT[1] = 0;
    }
    return pwmOUT;
}

int main() {


    InitCan(); // Initialize CAN module
    InitInt(); // Initialize INT0
    InitUart(); // Initialize UART module

    // Misc. variables
    unsigned char i,j = 0;
    unsigned char txData[UART_TX_LEN] = {'\0'};



    while (1) {

        // Motor PWM control state machine
        switch (motorState) {
            case INITIALIZE:
                // Initialization to offset POSCNT to three turns (12000 counts)
                motorState = READ_INPUT_OUTPUT;
//                motorState = IDLE;
                C1CTRLbits.REQOP = NORMAL;
                while (C1CTRLbits.OPMODE != NORMAL);
                break;

            case IDLE :
                LEDRED ^= 1;
                break;

            case READ_INPUT_OUTPUT :
                LEDRED = 1;
                // check if transmit command is present ( 80 = capital P )
                if (sendMsg == 80) {
                    txInProgress = 0;
                    //                        // load CAN data
                    sprintf(txData, "%u %u %u %u %u %u\0", masterData[0], masterData[1], masterData[2], slaveData[0], slaveData[1], slaveData[2]);
//                    sprintf(txData, "%u %u %u\0", masterData[0], masterData[1], masterData[2]);
                    // transmit characters to computer
                    for (i = 0; i < UART_TX_LEN; i++) {
                        U1TXREG = txData[i];
                        while (!(U1STAbits.TRMT));
                    }
                    txInProgress = 1;
                    sendMsg = 0;
                }
                break;
        }


        msDelay(PID_TS);
    } //while
} // main

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    IFS1bits.C1IF = 0; // Clear interrupt flag
    if (C1INTFbits.TX0IF) {
        C1INTFbits.TX0IF = 0;
    }

    if (C1INTFbits.RX0IF) {
        C1INTFbits.RX0IF = 0;
        C1RX0CONbits.RXFUL = 0;
        if (txInProgress) {
            if (C1RX0B4 == MASTER1_ID) {
                masterData[0] = C1RX0B1;
            } else if (C1RX0B4 == MASTER2_ID) {
                masterData[1] = C1RX0B2;
            } else if (C1RX0B4 == MASTER3_ID) {
                masterData[2] = C1RX0B3;
            } else if (C1RX0B4 == SLAVE1_ID) {
                slaveData[0] = C1RX0B1;
            } else if (C1RX0B4 == SLAVE2_ID) {
                slaveData[1] = C1RX0B2;
            } else if (C1RX0B4 == SLAVE3_ID) {
                slaveData[2] = C1RX0B3;
            }
        }
    }
}

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {
    IFS0bits.INT0IF = 0;
    motorState = READ_INPUT_OUTPUT;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void){
    IFS0bits.U1TXIF = 0;        // Clear U1TX interrupt
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void){
    IFS0bits.U1RXIF = 0;        // Clear U1RX interrupt
    sendMsg = U1RXREG;
}
