/*
 * File:   main.c
 * Author: PaoloDantes
 *
 * The truth table for the motor driver is as follows:
 *   IN1   IN2   OUT1  OUT2  STATE
 * ---------------------------------
 *    H     L     H     L     FWD
 *    L     H     L     H     REV
 *    L     L     L     L    FW LOW
 *    H     H     H     H    FW HIGH
 *
 * Here are the control outputs:
 * PDC 1 = Duty cycle of IN1
 * PDC 2 = Duty cycle of IN2
 *
 *
 * Created on March 9, 2015, 4:56 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <p30F4012.h>
#include <dsp.h>
//#include <libpic30.h>
#include "config.h"

#define MAX_CNT_PER_REV (1000 * 4 - 1)
#define MAXSPEED (unsigned int)(((unsigned long)MAX_CNT_PER_REV*2048)/125)
#define HALFMAXSPEED (MAXSPEED>>1)
#define INITIALIZE          0
#define CLOCKWISE           1
#define COUNTERCLOCKWISE    2
#define CW_PAUSE            3
#define CCW_PAUSE           4

#define PWM_COUNTS_PERIOD   2056

#define RUNMTR50PERCENT     2056
#define RUNMTR5PERCENT      205
#define STOPMOTOR           0

#define INITIALIZE          0
#define INITIAL_STOP        1
#define READ_POSITION       2
#define CALCULATE_PID       3
#define UPDATE_PID          4
#define ACTUATE_MOTOR       5

#define PID_KP  1
#define PID_KD  0.1
#define PID_KI  0
#define PID_TI  0
#define PID_TD  0
#define PID_TS  10
#define PID_N   10

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void);

void InitPWM(void);
void InitQEI(void);
void InitInt(void);
void PositionCalculation(void);
void InitTMR1(void);
void InitOsc(void);
void InitAdc(void);
void GetAdcValue(void);
void msDelay(unsigned int mseconds);
void moveMotor(unsigned int direction, unsigned int degreePosition);

// Angular position and speed variables
int POSCNTcopy = 0;
int AngPos[2] = {0,0};
int Speed;

// ADC variables
unsigned int ADCValue = 0;

// PID Parameters
//tPID fooPID;
//fractional abcCoefficient[3] __attribute__ ((section (".xbss, bss, xmemory")));
//fractional controlHistory[3] __attribute__ ((section (".ybss, bss, ymemory")));
//fractional kCoeffs[] = {0,0,0};


/* PID Control
 *
 * Discretized algorithm:
 *  u[k] = P[k] + I[k] + D[k]
 *  P[k] = K(b*u[k]-y[k])
 *  I[k] = I[k-1] + K*T*e[k-1]/Ti
 *  D[k] = (Td)/(Td+N*T)*D[k-1]
 *          -(K*Td*N)/(Td+N*T)*(y[k]-y[k-1])
 */

typedef struct {
    float Kp, Kd, Ki, Ti, T, Td;
    unsigned short N;       // derivative filter parameter (3-20)
    float i, ilast;         // integral
    float y, ylast;         // output
    float d, dlast;         // derivative term
    float uc;               // input
    float u;                // u=uc-y
    float e, elast;            // error
    float ulow, uhigh;      // low/high inputs
} pid_t;

void InitPid(pid_t *p, float kp, float kd, float ki, float Ti, float Td, float T, unsigned short N, float il, float yl, float dl, float el);
void UpdatePid(pid_t *p);
void CalcPid(pid_t *p, unsigned int currentPOTPosition, unsigned int currentMTRPosition);


// Misc. variables
unsigned char direction = 0; // 0 = CW, 1 = CCW
unsigned char indexPulseCount = 0;
unsigned int revCount = 0;


int main( void) {
    // Initialize functions
    InitPWM();
    InitQEI();
    InitInt();
    InitOsc();
    InitTMR1();
    InitAdc();

    // Pin configuration
//    ADPCFG |= 0x0038;       // Set PB0, PB1, PB3, PB4 and PB5 to digital pins
    TRISEbits.TRISE1 = 0;   // Set PE1 as 0 = output, 1 = input
    TRISBbits.TRISB0 = 1;   // Set PB0 as input Vref+
    TRISBbits.TRISB1 = 1;   // Set PB1 as input Vref-
    TRISBbits.TRISB2 = 1;   // Set PB2 as input AN2
    LATEbits.LATE1 = 0;     // Initialize off PIN25
//    LATBbits.LATB0 = 0;     // Initialize off LED0
//    LATBbits.LATB1 = 0;     // Initialize off LED1
//    LATBbits.LATB2 = 0;     // Initialize off LED2

    // PWM Parameters
    PTPER = PWM_COUNTS_PERIOD; // period

    // QEI Parameters
    MAXCNT = 4000; // 65,535

//    // PID data structure initialization
//    fooPID.abcCoefficients = &abcCoefficient[0];    /*Set up pointer to derived coefficients */
//    fooPID.controlHistory = &controlHistory[0];     /*Set up pointer to controller history samples */
//    PIDInit(&fooPID);                               /*Clear the controler history and the controller output */
//    kCoeffs[0] = Q15(1); // Kp
//    kCoeffs[1] = Q15(0); // Ki
//    kCoeffs[2] = Q15(0); // Kd
//    PIDCoeffCalc(&kCoeffs[0], &fooPID);             /*Derive the a,b, & c coefficients from the Kp, Ki & Kd */

    // Use PID Controller
//    fooPID.controlReference = Q15(0.74);
//    fooPID.measuredOutput = Q15(0.453);

    // In-house PID
    pid_t mypid;
    InitPid(&mypid, PID_KP, PID_KD, PID_KI, PID_TI, PID_TD, PID_TS, PID_N, 0, 0, 0, 0);

    // Misc. variables
    unsigned int ADCdummy1 = 0;
    unsigned long ADCdummy2 = 0;
    unsigned int ADCdummy3 = 0;
    int i; // for loop index
    unsigned int motorState = 0;
    unsigned int encoderPosition = 0;
    float degPOT = 0.0;
    float degMTR = 0.0;
    double u = 0;
    float pidOutDutyCycle = 0;
    unsigned int pwmOUT1 = 0;
    unsigned int pwmOUT2 = 0;


    while(1)
    {
//        LATEbits.LATE1 = 1;
//        for(i=1;i;i--){}
//        LATEbits.LATE1 = 0;
//        for(i=1999;i;i--){}
/*
 * THIS CODE IS TESTED BUT NOT WORK
 * IMPLEMENTS POSITION PD CONTROL
 * KNOWN BUGS: Invalid region of 0-70 degrees on the pot
 */
        if (ADCValue <= 34){ // dead zone of 0-10 degrees
            degPOT = 0.0;
        }
        else {
            degPOT = (float) (ADCValue*0.3);    // target position
        }

        if (POSCNT <= 120){ // dead zone of 0-10 degrees
            degMTR = 0.0;
        }
        else {
            degMTR = (float) (POSCNT*0.09);    // motor position
        }


        mypid.e = degPOT-degMTR;

        mypid.d = (mypid.T*mypid.dlast)/mypid.N + (mypid.Kd*mypid.T)*(mypid.e-mypid.elast)/(mypid.T+(mypid.T/mypid.N));
        mypid.u = mypid.Kp*mypid.e+mypid.d;
        mypid.dlast = mypid.d;
        mypid.elast = mypid.e;

        pidOutDutyCycle = (float) ((mypid.u/degPOT)*2*PWM_COUNTS_PERIOD);
        if (pidOutDutyCycle >= 2*PWM_COUNTS_PERIOD){
            pwmOUT1 = 2*PWM_COUNTS_PERIOD;
            pwmOUT2 = 0;
        }
        else if (pidOutDutyCycle <= -(2*PWM_COUNTS_PERIOD)){
            pwmOUT1 = 0;
            pwmOUT2 = 2*PWM_COUNTS_PERIOD;
        }
        else if (pidOutDutyCycle <0){
            pwmOUT1 = 0;
            pwmOUT2 = (-1)*pidOutDutyCycle;
        }
        else{
            pwmOUT1 = pidOutDutyCycle;
            pwmOUT2 = 0;
        }

        PDC1 = pwmOUT1;
        PDC2 = pwmOUT2;


/*
 * THIS CODE IS TESTED AND WORKS
 * Controls the rotational speed of the motor using the pot
 * Known bugs: when ADCValue < 22 ---> unsigned int overflows and becomes 65K+
 */
//        ADCdummy1 = ADCValue-22;
//        ADCdummy2 = (unsigned long)ADCdummy1*4112;
//        ADCdummy3 = ADCdummy2/985;
//
//        if (ADCdummy1 >= 50000) // for the case when the unsigned int overlows and becomes negative
//        {
//            PDC1 = STOPMOTOR;
//            PDC2 = STOPMOTOR;
//        }
//        else
//        {
//            PDC1 = ADCdummy3;
//            PDC2 = STOPMOTOR;
//        }

/*
 * THIS CODE IS TESTED AND WORKING
 * IMPLEMENTS STATE MACHINE TO ALTERNATE SWITCH OF THE MOTOR DIRECTION
 */
//        switch (motorState){
//            case INITIALIZE:
//                PDC1 = STOPMOTOR;
//                PDC1 = STOPMOTOR;
//                motorState = CLOCKWISE;
//            case CLOCKWISE:
//                LATEbits.LATE1 = 0;
//                PDC1 = RUNMTR5PERCENT;
//                PDC2 = STOPMOTOR;
//                if (POSCNT >= 1000){
//                    motorState = CW_PAUSE;
//                }
//                break;
//            case COUNTERCLOCKWISE:
//                LATEbits.LATE1 = 0;
//                PDC1 = STOPMOTOR;
//                PDC2 = RUNMTR5PERCENT;
//                if (POSCNT <= 500){
//                    motorState = CCW_PAUSE;
//                }
//                break;
//            case CW_PAUSE:
//                while(i<1000){
//                    LATEbits.LATE1 = 1;
//                    PDC1 = STOPMOTOR;
//                    PDC2 = STOPMOTOR;
//                    i++;
//                }
////                Delay_ms(3000);
//                motorState = COUNTERCLOCKWISE;
//                break;
//            case CCW_PAUSE:
//                while(i<1000){
//                    LATEbits.LATE1 = 1;
//                    PDC1 = STOPMOTOR;
//                    PDC2 = STOPMOTOR;
//                    i++;
//                }
////                Delay_ms(3000);
//                motorState = CLOCKWISE;
//                break;
//        }




    }//while
}//main

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
//    LATEbits.LATE1 = ~PORTEbits.RE1; // toggle led on PIN25
    revCount = POSCNT;
    IFS0bits.INT0IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;   // Clear timer 1 interrupt flag
    PositionCalculation();
    Speed = AngPos[0] - AngPos[1];
    if (Speed >= 0)
    {
       if (Speed >= (HALFMAXSPEED))
          Speed = Speed - MAXSPEED;
    } else {
        if (Speed < -(HALFMAXSPEED))
          Speed = Speed + MAXSPEED;
    }
    Speed *= 2;
    return;
}

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void)
{
    ADCValue = ADCBUF0;     // Read from ADC buffer
    IFS0bits.ADIF = 0;      // Clear ADC interrupt flag
}

void InitPWM(void)
{
    PTCONbits.PTEN = 0;         // Disable PWM timerbase
    PTCONbits.PTCKPS = 0;       // 1:1 prescalerm
    PTCONbits.PTOPS = 0;        // 1:1 postscaler
    PTCONbits.PTMOD = 0;        // free running mode
    PWMCON1bits.PMOD1 = 1;      // PWM in independent mode
    PWMCON1bits.PMOD2 = 1;      // PWM in independent mode
    PWMCON1bits.PEN1L = 1;      // PWM1L (PIN 24) output controlled by PWM
    //PWMCON1bits.PEN1H = 1;      // PWM1H output controlled by PWM
    PWMCON1bits.PEN2L = 1;      // PWM2L (PIN 26) output controlled by PWM
    PTMR = 0;                   // PWM counter value start at 0
    PTCONbits.PTEN = 1;         // enable PWM timerbase
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
    POSCNT = 0;                 // Reset position counter
    QEICONbits.QEIM = 7;        // X4 mode with position counter reset by
                                // 6 - index pulse
                                // 7 - match (MAXCNT)
    return;
}

void InitInt(void){

    INTCON1bits.NSTDIS = 0;         // Interrupt nesting is disabled
    IEC0bits.INT0IE = 0;            // Disable external interrupt 0
    INTCON2bits.ALTIVT = 0;         // Default interrupt vector table
    INTCON2bits.INT0EP = 1;         // Detect on 1 = negative edge 0 = positive for interrupt 0
    IPC0bits.INT0IP = 5;            // External interrupt 0 is priority 5
    IFS0bits.INT0IF = 0;            // Clear external interrupt 0 flag
    IEC0bits.INT0IE = 1;            // Enable external interrupt 0
    return;
}

void PositionCalculation(void)
{
    POSCNTcopy = (int)POSCNT;
    if (POSCNTcopy < 0){
        POSCNTcopy = -POSCNTcopy;
    }
    AngPos[1] = AngPos[0];
    AngPos[0] = (unsigned int)(((unsigned long)POSCNTcopy*2048)/125);
    return;
}

void InitTMR1(void)
{
   TMR1 = 0;                // Reset timer counter
   T1CONbits.TON = 0;       // Turn off timer 1
   T1CONbits.TSIDL = 0;     // Continue operation during sleep
   T1CONbits.TGATE = 0;     // Gated timer accumulation disabled
   T1CONbits.TCS = 0;       // Use Tcy as source clock
   T1CONbits.TCKPS = 0;     // Tcy/1 as input clock
   PR1 = 34483;             // Interrupt period = 0.0075sec with 64 prescaler
   IFS0bits.T1IF = 0;       // Clear timer 1 interrupt flag
   IEC0bits.T1IE = 1;       // Enable timer 1 interrupts
   T1CONbits.TON = 1;       // Turn on timer 1
   return;
}

void InitOsc(void)
{
    // Initializing system postscaler
    // for Fcy = 5MHz
    OSCCONL = 0x46; // OSCCONL writing sequence refer to Section 7.4.1
    OSCCONL = 0x57; // in dsPIC30F Family Reference Manual
    OSCCONL |= 0x40; // Postscaler = divide by 4 --Fcy=Fosc*PLL/(Postscaler*4)
}

void InitAdc(void)
{
    ADPCFG &= ~0x0007;          // Sets  PB0 and PB1 as analog pin (clear bits)
    ADCON1bits.ADON = 0;        // A/D converter module off
    ADCON1bits.ADSIDL = 0;      // Continue module operation in Idle mode
    ADCON1bits.SIMSAM = 0;      // Samples multiple channels individually in sequence
    ADCON1bits.FORM = 0;        // Integer (DOUT = 0000 00dd dddd dddd)
    ADCON1bits.SSRC = 7;        // Internal counter ends sampling and starts conversion (auto convert)
    ADCON1bits.ASAM = 1;        // Sampling in a channel begins when the conversion finishes (Auto-sets SAMP bit)
//    ADCON1bits.SAMP = 1;        // At least one A/D sample/hold amplifier is sampling

    ADCON2bits.VCFG = 3;        // Set external Vref+ and Vref- pins as A/D VrefH and VrefL
//    ADCON2bits.VCFG = 7;        // Set AVss and AVdd as A/D VrefH and VrefL
//    ADCON2bits. CSCNA = 1;
    ADCON2bits.BUFM = 0;        // Buffer configured as one 16-word buffer ADCBUF(15...0)
    ADCON2bits.CHPS = 0;        // Convert CH0
    ADCON2bits.SMPI = 0;        // Interrupts at the completion of conversion for each sample/convert sequence
//    ADCON2bits.ALTS = 0;        // Always use MUX A input multiplexer settings

    ADCON3bits.ADRC = 0;        // Clock derived from system clock
    ADCON3bits.ADCS = 31;       // A/D Conversion clock select = 32*Tcy
    ADCON3bits.SAMC = 31;       // 31*Tad Auto-Sample time


    ADCSSLbits.CSSL2 = 1;       // Select AN2 for input scan
    ADCHSbits.CH0SA = 2;        // Channel 0 positive input is AN2
    ADCHSbits.CH0NA = 0;        // Channel 0 negative input is Vref-

    // ADC Interrupt Initialization
    IFS0bits.ADIF = 0;          // Clear timer 1 interrupt flag
    IEC0bits.ADIE = 1;          // Enable timer 1 interrupts
    IPC2bits.ADIP = 6;          // ADC interrupt priority 6
    ADCON1bits.ADON = 1;        // A/D converter module on
    return;
}

void GetAdcValue(void)
{
    ADCON1bits.SAMP = 1; // Manually start sampling
    while(!ADCON1bits.SAMP);
    ADCON1bits.SAMP = 0;                       // end sampling
    while(!ADCON1bits.DONE);
    ADCValue = ADCBUF0;
    msDelay(100);
}

void msDelay(unsigned int mseconds) //For counting time in ms
//-----------------------------------------------------------------
{
    int i;
    int j;
    for(i=mseconds; i; i--){}
        for(j=5000;j;j--){}
    return;
}

void InitPid(pid_t *p, float kp, float kd, float ki, float Ti, float Td, float T, unsigned short N, float il, float yl, float dl, float el)
{
    p->Kp = kp;
    p->Kd = kd;
    p->Ki = ki;
    p->Ti = Ti;
    p->Td = Td;
    p->T = T;
    p->N = N;
    p->ilast = il;
    p->ylast = yl;
    p->dlast = dl;
    p->elast = el;
}

void UpdatePid(pid_t *p)
{
    p->dlast = p->d;
    p->ylast = p->y;
    p->ilast = p->i;
}

void CalcPid(pid_t *p, unsigned int currentPOTPosition, unsigned int currentMTRPosition)
{
    p->y = currentMTRPosition;
    p->uc = currentPOTPosition;
    p->d = (1/p->N)*p->T*p->dlast + p->Kd*p->T*(p->y-p->ylast)/(p->T+(1/p->N)*p->T);
    p->u = p->Kp*(p->uc-p->y)+ p->d;
}