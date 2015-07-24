/*
 * File:   main.c
 * Author: PaoloDantes
 *
 * Created on April 17, 2015, 2:23 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "p30FXXXX.h"


#define UART_TX_LEN     8

#define RS      LATEbits.LATE0 // RED WIRE
#define RW      LATEbits.LATE1 // BLUE WIRE
#define EN      LATEbits.LATE2 // GREEN WIRE
#define DATA    LATB
#define LEDRED  LATEbits.LATE3 // YELLOW
#define LEDGRN  LATEbits.LATE4 // GREEN
#define TRISRED  TRISEbits.TRISE3 // YELLOW
#define TRISGRN  TRISEbits.TRISE4 // GREEN

// Display commands for LMB162ABC
#define CLEAR_DISPLAY       0x01 // Writes "20h" (ASCII for space
#define RETURN_HOME         0X02
#define ENTRY_MODE_SET      0x06 // No screen shifting, AC=AC+1
#define DISPLAY_ON          0x0F // Display on, cursor on, cursor blinking on
#define CURSOR_SHIFT        0x14 // Shift cursor to right side
#define FUNCTION_SET        0x28 // 4-bit interface, 2-line display, 5x8 dots font
#define SET_CGRAM_ADD       0x40 // OR this with AC0-AC5 CGRAM address
#define SET_DDRAM_ADD       0x80 // OR this with AC0-AC6 DDRAM address

// CAN Operation Modes
#define CONFIG_MODE     4
#define LISTEN_ALL      7
#define LISTEN_ONLY     3
#define LOOPBACK        2
#define DISABLE         1
#define NORMAL          0

// CAN bit timing
#define FOSC        7370000 // 7.37MHz
#define FCY         FOSC/4
#define BITRATE     100000  // 100kbps
#define NTQ         16      // Amount of Tq per bit time
#define BRP_VAL     (((4*FCY)/(2*NTQ*BITRATE))-1) // refer to pg. 693 of Family Reference

// Define motorState state values
#define INITIALIZE          0
#define READ_INPUT_OUTPUT   1
#define CALCULATE_PID       2
#define UPDATE_PID          3
#define IDLE                4

// Define PID controls
#define PID_KP  10
#define PID_KD  0.1
#define PID_KI  0
#define PID_TI  0
#define PID_TD  0
#define PID_TS  10
#define PID_N   10

//ADC
#define INDEXSIZE 25

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

// LCD FUNCTIONS
void lcd_init(void);
void lcd_command(unsigned char cmd);
void lcd_data(unsigned char data);
void msDelay(unsigned int mseconds);

//// CAN1 MODULE
unsigned int OutData0[4] = {0x5555, 0xAAAA, 0xAAAA, 0xAAAA};
unsigned int InData0[4] = {0, 0, 0, 0};
void InitCan(void);
void InitInt(void);
void InitAdc(void);
void InitQEI(void);
void InitPwm(void);
void InitPid(pid_t *p, float kp, float kd, float ki, float T, unsigned short N, float il, float yl, float dl, float el);
void UpdatePid(pid_t *p);
unsigned int * CalcPid(pid_t *mypid, float degPOT, float degMTR);
unsigned int C1INTFtest, RX0IFtest, whileLooptest = 0;
unsigned int motorState = 0;
unsigned int ADCValue0, ADCValue1 = 0;
unsigned int ADCvaluebuffer[INDEXSIZE]={0};
unsigned int adcbufferindex = 0;
unsigned char motordirect = 0;
unsigned int ADCsum = 0;
unsigned char fullflag = 0;




void InitAdc(void) {
//    ADPCFG &= ~0x0030; // Sets  PB4 and PB5 as analog pin (clear bits)
//    ADPCFG = 0xFFF0; // Sets  PB0, PB1 and PB2 as analog pin (clear bits)
    ADPCFG &= ~0x0007;
//    TRISBbits.TRISB4 = 0; // 0=output 1=input
//    TRISBbits.TRISB5 = 0; // 0=output 1=input
//    TRISBbits.TRISB1 = 1; // 0=output 1=input
//    TRISBbits.TRISB2 = 1; // 0=output 1=input
//    TRISBbits.TRISB3 = 1; // 0=output 1=input
    ADCON1bits.ADON = 0; // A/D converter module off
    ADCON1bits.ADSIDL = 0; // Continue module operation in Idle mode
    ADCON1bits.SIMSAM = 0; // Samples CH0, CH1, CH2 and CH3 simultaneously
    ADCON1bits.FORM = 0; // Integer (DOUT = 0000 00dd dddd dddd)
    ADCON1bits.SSRC = 7; // Internal counter ends sampling and starts conversion (auto convert)
    ADCON1bits.ASAM = 1; // Sampling in a channel begins when the conversion finishes (Auto-sets SAMP bit)
//        ADCON1bits.SAMP = 1;        // At least one A/D sample/hold amplifier is sampling

//        ADCON2bits.VCFG = 3;        // Set external Vref+ and Vref- pins as A/D VrefH and VrefL
    ADCON2bits.VCFG = 0; // Set AVdd and AVss as A/D VrefH and VrefL
    //    ADCON2bits. CSCNA = 1;
    ADCON2bits.BUFM = 0; // Buffer configured as one 16-word buffer ADCBUF(15...0)
    ADCON2bits.CHPS = 0; // Convert CH0, CH1, CH2 and CH3
    ADCON2bits.SMPI = 0; // Interrupts at the completion of conversion for each sample/convert sequence
    //    ADCON2bits.ALTS = 0;        // Always use MUX A input multiplexer settings

    // Setting ADC clock source
    ADCON3bits.ADRC = 0; // Clock derived from system clock
    ADCON3bits.ADCS = 31; // A/D Conversion clock select = 32*Tcy
    ADCON3bits.SAMC = 31; // 31*Tad Auto-Sample time

    // Connecting AN4 and AN5 for input scan
//    ADCSSLbits.CSSL1 = 1; // Select AN4 and AN5 for input scan and skip the rest
////    ADCSSL = 0x0002;
//    ADCSSLbits.CSSL1 = 1;       // Select AN2 for input scan
    ADCHSbits.CH0SA = 1;        // Channel 0 positive input is AN2
    ADCHSbits.CH0NA = 0;        // Channel 0 negative input is Vref-

    // ADC Interrupt Initialization
    IFS0bits.ADIF = 0; // Clear timer 1 interrupt flag
    IPC2bits.ADIP = 5; // ADC interrupt priority 5
    IEC0bits.ADIE = 1; // Enable timer 1 interrupts
    ADCON1bits.ADON = 1; // A/D converter module on
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
int main() {
//    lcd_init();
    InitCan();
    InitInt();
    InitAdc();
    InitQEI();
    InitPwm();
//    InitUart();
    pid_t mypid;

    TRISRED = 0; // PORTE output
    TRISGRN = 0;

    // QEI Parameters
    MAXCNT = 65535; // 65,535

    // Misc. variables
    float degPOT = 0.0;
    float degMTR = 0.0;
    unsigned int *pwmOUT;
    unsigned char i,j = 0;
    unsigned char txData[UART_TX_LEN] = {'\0'};

    while (1) {

        // Motor PWM control state machine
        switch (motorState) {
            case INITIALIZE:
                // Initialization to offset POSCNT to three turns (12000 counts)
                motorState = IDLE;
                POSCNT = 12000; // This prevents under and overflow of the POSCNT register
                PTCONbits.PTEN = 1; // Enable PWM module
                InitPid(&mypid, PID_KP, PID_KD, PID_KI, PID_TS, PID_N, 0, 0, 0, 0);
                C1CTRLbits.REQOP = NORMAL;
                while (C1CTRLbits.OPMODE != NORMAL);
                break;

            case READ_INPUT_OUTPUT:

                // Read and convert ADC value and encoder position to degrees
                degMTR = (float) (POSCNT * 0.09); // motor position
                degPOT = (float) (InData0[0] * 0.09); // target position with motor encoder input
                //                degPOT = (float) ((InData0[4]+3084)*0.3);    // target position with ADC pot input
                // Calcaulte PWM duty cycle
                pwmOUT = CalcPid(&mypid, degPOT, degMTR);
                motorState = UPDATE_PID;
                break;

            case UPDATE_PID:
                // Update PID variables
                UpdatePid(&mypid);
                motorState = IDLE;
                break;

            case IDLE:
                if (InData0[3] == 2) {
                    motorState = READ_INPUT_OUTPUT;
                }
                if (InData0[3] == 2) {
                    C1TX0B4 = 1;
                    C1TX0B1 = POSCNT;
                    C1TX0B2 = ADCValue0;
                    C1TX0B3 = motordirect;
                    C1TX0CONbits.TXREQ = 1;
                    while (C1TX0CONbits.TXREQ != 0);
                    PDC1 = *(pwmOUT + 0); // 16-bit register
                    PDC2 = *(pwmOUT + 1); // 16-bit register


//                    sprintf(txData, "%u\r\n", ADCValue0);
//                    // transmit characters to computer
//                    for (i = 0; i < UART_TX_LEN; i++) {
//                        U1TXREG = txData[i];
//                        while (!(U1STAbits.TRMT));
//                    }
                }
                break;
        }

        if (motorState != INITIALIZE) {
            LEDRED = 0;
            LEDGRN = 1;
            PDC1 = *(pwmOUT + 0); // 16-bit register
            PDC2 = *(pwmOUT + 1); // 16-bit register
    }

    } // WHILE

    // LED management

} // main
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void){
    IFS0bits.U1TXIF = 0;        // Clear U1TX interrupt
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void){
    IFS0bits.U1RXIF = 0;        // Clear U1RX interrupt
//    sendMsg = U1RXREG;
}

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
    unsigned int k = 0;
    IFS0bits.ADIF = 0;
    if (adcbufferindex >= INDEXSIZE) {
        adcbufferindex = 0;
        fullflag = 1;
    }
    if (adcbufferindex == (INDEXSIZE - 1)){
        k = 0;
    }
//    ADCValue0 = ADCBUF0;
    //can set requirements to detect PWM before reading ADC
    if (ADCBUF0 < 200){
        ADCvaluebuffer[adcbufferindex] = ADCBUF0;
        if (fullflag == 0) {
//            ADCvaluebuffer[adcbufferindex] = ADCBUF0;
            ADCsum = ADCsum + ADCvaluebuffer[adcbufferindex];
            ADCValue0 = ADCsum / (adcbufferindex+1);
        }
        if(fullflag == 1){
            ADCsum = 0;
            for (k = 0; k < INDEXSIZE; k++) {
                ADCsum = ADCsum + ADCvaluebuffer[k];
//                ADCValue0 = ADCsum*0.1;
//                return ADCsum;
            }
            ADCValue0 = ADCsum/INDEXSIZE;
//        }
        //method 2 subtract the oldest value then add the newest (no need to sum each time)
//        if (fullflag == 1) {
//            ADCsum = ADCsum - ADCvaluebuffer[k];
//            ADCvaluebuffer[adcbufferindex] = ADCBUF0;
//            ADCsum = ADCsum + ADCvaluebuffer[adcbufferindex];
//            ADCValue0 = ADCsum / INDEXSIZE;
//            k++;
        }
    }

        adcbufferindex += 1;
}


//    if (ADCBUF0 < 250) {
//        ADCvaluebuffer[adcbufferindex] = ADCBUF0;
//        if (fullflag == 0) {
//            ADCsum = ADCsum + ADCvaluebuffer[adcbufferindex];
//            ADCValue0 = ADCsum / adcbufferindex;
//        }
//




void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {
    IFS0bits.INT0IF = 0;
//    motorState = IDLE;
}

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    IFS1bits.C1IF = 0; // Clear interrupt flag
    C1INTFtest = C1INTF;
    if (C1INTFbits.TX0IF) {
        C1INTFbits.TX0IF = 0;
    }

    if (C1INTFbits.RX0IF) {
        C1INTFbits.RX0IF = 0;

        InData0[0] = C1RX0B1;
        InData0[1] = C1RX0B2; //Move the recieve data from Buffers to InData
        InData0[2] = C1RX0B3;
        InData0[3] = C1RX0B4;
        C1RX0CONbits.RXFUL = 0;
    }
}

void lcd_init() {
    ADPCFG = 0xFF; // 0=analog 1=digital for PORTB
    TRISE = 0; // 0=output 1=input
//    TRISB = 0; // 0=output 1=input
    LATE = 0;
//    LATB = 0;

    msDelay(100);

    lcd_command(FUNCTION_SET);
    msDelay(10);
    lcd_command(ENTRY_MODE_SET);
    msDelay(10);
    lcd_command(DISPLAY_ON);
    msDelay(10);
    lcd_command(CLEAR_DISPLAY);
    msDelay(10);
    lcd_command(RETURN_HOME);
    msDelay(10);

}

void lcd_command(unsigned char cmd) {
    RS = 0;
    RW = 0;

    // Send high-byte of command
    DATA = ((cmd >> 4) & 0x0F);
    EN = 1;
    EN = 0;

    // Send low-byte of command
    DATA = ((cmd) & 0x0F);
    EN = 1;
    EN = 0;

    msDelay(1);
}

void lcd_data(unsigned char data) {
    RS = 1;
    RW = 0;

    // Send high-byte of data
    DATA = ((data >> 4) & 0x0F);
    EN = 1;
    EN = 0;

    // Send low-byte of data
    DATA = (data & 0x0F);
    EN = 1;
    EN = 0;

    msDelay(1);
}

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
    IPC6bits.C1IP = 7; // CAN 1 Module interrupt is priority 6
    IEC1bits.C1IE = 1; // Enable CAN1 Interrupt

    /*---------------------------------------------------------
     *  CONFIGURE RECEIVER REGISTERS, FILTERS AND MASKS
     *---------------------------------------------------------*/

    // Configure CAN Module Receive Buffer Register
    C1RX0CONbits.DBEN = 0; // Buffer 0 does not overflow in buffer 1

    // Initializing Acceptance Mask Register
    C1RXM0SID = 0x1FFD;

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

        C1CTRLbits.REQOP = NORMAL;
        while(C1CTRLbits.OPMODE != NORMAL);//Wait for CAN1 mode change from Configuration Mode to Loopback mode
}

void InitInt(void) {
    INTCON1bits.NSTDIS = 1; // Interrupt nesting is disabled
    IEC0bits.INT0IE = 0; // Disable external interrupt 0
    INTCON2bits.ALTIVT = 0; // Default interrupt vector table
    INTCON2bits.INT0EP = 1; // Detect on 1 = negative edge 0 = positive for interrupt 0
    IPC0bits.INT0IP = 6; // External interrupt 0 is priority 5
    IFS0bits.INT0IF = 0; // Clear external interrupt 0 flag
    IEC0bits.INT0IE = 1; // Enable external interrupt 0
    return;
}



void InitQEI(void)
{
    ADPCFG |= 0x0038;           // RB3, RB4, RB5 configured to digital pin
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
    PTPER = PWM_COUNTS_PERIOD;          // Set PWM period
    PTCONbits.PTEN = 1;                 // enable PWM timerbase
    return;
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
        } // 1ms/(250ns/instruction*4instructions/forloop)=1000 for loops
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
        motordirect = 1;
    }
    else if (pidOutDutyCycle <= -(2*PWM_COUNTS_PERIOD)){ //counter clock
        pwmOUT[0] = 0;
        pwmOUT[1] = (unsigned int) (2*PWM_COUNTS_PERIOD);
        motordirect = 0;
    }
    else if (pidOutDutyCycle <0){
        pwmOUT[0] = 0;
        pwmOUT[1] = (unsigned int)((-1)*pidOutDutyCycle);
        motordirect = 0;
    }
    else{
        pwmOUT[0] = (unsigned int)(pidOutDutyCycle);
        pwmOUT[1] = 0;
        motordirect = 1;
    }
    return pwmOUT;
}