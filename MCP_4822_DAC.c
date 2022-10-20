/****************************************************************************************************
TARGET DEVICE : MCP_4822_DAC
INTERFACE : TM4c123gh6pm SPI1 interface.
PURPOSE : Setups up the TM4c123gh6pm to send 16-bit words of instructions to the device.
The target device accepts 12-bits of data that can be outputted on two ports with bit LDAC that
will sync the two. These 12-bits will be obtained from a LUT taken from the upper 32-bits of
phase variable; in the LUT there is values defining a sine wave form.
**************************************************************************************************
PINS USED :
SSI
    PD0 : SSI1CLK
    PD1 : SSI1FSS
    PD3 : SSI1TX
UART
    PA0: UART0_RX
    PA1: UART0_TX
    PA3: ~LDAC
DEV CONTROL
****************************************************************************************************/

//DIRECTIVES, MACROS, GLOBALS:
//Hello
#include "MCP_4822_DAC.h"
#include "spi1.h"
#include "nvic.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "uart_UI.h"
#include "uart0.h"
#include "math.h"
#include "adc0.h"
#include "adc1.h"
#include "MCP_4822_DAC_Config.h"
#include "wait.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//Debug section

//#define __debug


//REGISTER DEFINES
#define APINT_REG (* ( (volatile uint32_t *) (0xE000ED0C) ) )


//ARRAYS SIZES
#define LUT_SIZE 4096
#define TAPS 31
#define DATA_SIZE 4096
#define PREAMBLE_SIZE 2
#define PREAMBLE_FORM QPSK

//Shared Global Functions
uint32_t symbolRate = 100000;

bool runningM22 = false;
bool levelingFlag = false;

//For signal functions
struct Wave wavA;
struct Wave wavB;

uint16_t LUTA[LUT_SIZE];
uint16_t LUTB[LUT_SIZE];


uint32_t bits = 0;
uint32_t dataBytes = 0;
//First byte is the preamble
uint8_t data[DATA_SIZE] = {0x2C, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x11, 0x44, 0x22, 0x12, 0x83, 0x15, 0x21, 0x99, 0xAC, 0x22, 0xB1, 0x1C};
uint8_t preamble[PREAMBLE_SIZE] = {0x2C, 0x00};
//uint8_t data[DATA_SIZE] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
//uint8_t data[8] = {0xB4, 0xCD};

SEND_OPTION sendOption;
SYMBOL_FORMAT symOption = PSK8;
uint8_t upSampling = SAMPLING_INIT;

bool RRCFlag = false;
int16_t IPrev[TAPS];
int16_t QPrev[TAPS];

//a = 0
/*double RRCFilter[TAPS] =
{
           -0.0141,   -0.0216,   -0.0165,   0.0000,    0.0198,   0.0310,   0.0245,   -0.0000,
           -0.0317,   -0.0525,   -0.0447,    0.0000,    0.0748,    0.1590,    0.2250,    0.2500,
            0.2250,    0.1590,    0.0748,    0.0000,   -0.0447,   -0.0525,   -0.0317,   -0.0000,
            0.0245,    0.0310,    0.0198,    0.0000,  -0.0165,   -0.0216,   -0.0141
};*/

//a = 0.25
int32_t RRCFilter[TAPS] =
{
     0.0023*65536,   -0.0043*65536,   -0.0102*65536,   -0.0090*65536,    0.0015*65536,   0.0159*65536,   0.0230*65536,    0.0130*65536,
    -0.0136*65536,   -0.0422*65536,   -0.0493*65536,   -0.0160*65536,    0.0593*65536,    0.1553*65536,    0.2357*65536,    0.2671*65536,
     0.2357*65536,    0.1553*65536,    0.0593*65536,   -0.0160*65536,   -0.0493*65536,   -0.0422*65536,   -0.0136*65536,   0.0130*65536,
     0.0230*65536,    0.0159*65536,    0.0015*65536,   -0.0090*65536,   -0.0102*65536,   -0.0043*65536,    0.0023*65536
};

//a = 0.50
/*double RRCFilter[TAPS] =
{
      -0.0009,    0.0025,    0.0039,    0.0007,   -0.0040,   -0.0037,    0.0038,    0.0104,
       0.0038,   -0.0186,   -0.0389,   -0.0264,    0.0391,    0.1445,    0.2436,    0.2842,
       0.2436,    0.1445,    0.0391,   -0.0264,   -0.0389,   -0.0186,    0.0038,    0.0104,
       0.0038,   -0.0037,   -0.0040,    0.0007,    0.0039,    0.0025,   -0.0009
};*/



//QPSK
const uint16_t QPSK_ARR[4] = {
    (M22_90deg >> 1 ),                          //00
    (M22_90deg >> 1) + M22_90deg,               //01
    (M22_90deg >> 1) + (M22_90deg)*3,      //10
    (M22_90deg >> 1) + (M22_90deg)*2      //11
};

//8PSK
const uint16_t PSK8_ARR[8] = {
    (M22_90deg >> 2),                       //000
    (M22_90deg >> 2) + (M22_90deg >> 1),    //001
    (M22_90deg >> 2) + (M22_90deg >> 1)*3,  //010
    (M22_90deg >> 2) + (M22_90deg >> 1)*2,  //011
    (M22_90deg >> 2) + (M22_90deg >> 1)*7,  //100
    (M22_90deg >> 2) + (M22_90deg >> 1)*6,  //101
    (M22_90deg >> 2) + (M22_90deg >> 1)*4,  //110
    (M22_90deg >> 2) + (M22_90deg >> 1)*5  //111
};

//ASide 16QAM
const uint16_t QAM16_ARR_A[4] = {
    DAC_MAX + OFFSET_A,
    DAC_MAX - (DAC_MAX/3) + OFFSET_A,
    DAC_MAX - (DAC_MAX/3)*3 + OFFSET_A,
    DAC_MAX - (DAC_MAX/3)*2 + OFFSET_A
};

//Bside 16QAM
const uint16_t QAM16_ARR_B[4] = {
    DAC_MAX + OFFSET_B,
    DAC_MAX - (DAC_MAX/3) + OFFSET_B,
    DAC_MAX - (DAC_MAX/3)*3 + OFFSET_B,
    DAC_MAX - (DAC_MAX/3)*2 + OFFSET_B
};

//BSide 16QAM

//ASide 64QAM
const uint16_t QAM64_ARR_A[8] = {
   DAC_MAX + OFFSET_A,
   DAC_MAX - (DAC_MAX/7) + OFFSET_A,
   DAC_MAX - (DAC_MAX/7)*3 + OFFSET_A,
   DAC_MAX - (DAC_MAX/7)*2 + OFFSET_A,
   DAC_MAX - (DAC_MAX/7)*7 + OFFSET_A,
   DAC_MAX - (DAC_MAX/7)*6 + OFFSET_A,
   DAC_MAX - (DAC_MAX/7)*4 + OFFSET_A,
   DAC_MAX - (DAC_MAX/7)*5 + OFFSET_A
};

//BSide 64QAM
const uint16_t QAM64_ARR_B[8] = {
   DAC_MAX + OFFSET_B,
   DAC_MAX - (DAC_MAX/7) + OFFSET_B,
   DAC_MAX - (DAC_MAX/7)*3 + OFFSET_B,
   DAC_MAX - (DAC_MAX/7)*2 + OFFSET_B,
   DAC_MAX - (DAC_MAX/7)*7 + OFFSET_B,
   DAC_MAX - (DAC_MAX/7)*6 + OFFSET_B,
   DAC_MAX - (DAC_MAX/7)*4 + OFFSET_B,
   DAC_MAX - (DAC_MAX/7)*5 + OFFSET_B
};

//**************************************************************************************************

//*******SENDING IRSs********
void sendDACM22()
{
    if(sendOption == WAVE)
    {
        sendWaveDACM22();
    }
    else if(sendOption == STREAM)
    {
        sendStreamDACM22();
    }
    WTIMER1_ICR_R = TIMER_ICR_TATOCINT;

}

//Sends sinusoidal waves
void sendWaveDACM22()
{
    uint16_t signalA, signalB;

    setPinValue(LDAC_PIN, 0);
    __asm("    NOP\n    NOP\n   NOP\n   NOP\n");
    setPinValue(LDAC_PIN, 1);

    if(wavA.endless | wavA.cycles)
        signalA = LUTA[wavA.phase >> 20];
    else
        signalA = 2048;

    #ifndef __debug
    SSI1_DR_R = ( SEND_DACA + signalA );
    while(SSI1_SR_R & SSI_SR_BSY);
    #endif

    if(wavB.endless | wavB.cycles)
        signalB = LUTB[wavB.phase >> 20];
    else
        signalB = 2048;

    #ifndef __debug
    SSI1_DR_R = ( SEND_DACB + signalB );
    while(SSI1_SR_R & SSI_SR_BSY);
    #endif

    #ifdef __debug
    putsUart0("\nSignalA:");
    printUInt32ToUart0(signalA);
    putsUart0("\nSignalB:");
    printUInt32ToUart0(signalB);
    #endif

    if(wavA.cycles && wavA.phase + wavA.taPhase < wavA.phase)
        (wavA.cycles)--;
    if(wavB.cycles && wavB.phase + wavB.taPhase < wavB.phase)
        (wavB.cycles)--;

    wavA.phase += wavA.taPhase;
    wavB.phase += wavB.taPhase;


}


//******INITIALIZATION FUNCTIONS******

//Initializes all needed peripherals needed to interface with DAC.
void initDACM22()
{
    //Setups clock for WTIMER
    SYSCTL_RCGCWTIMER_R = SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R2;
    //Initializes a 16-bit transfer SPI interface
    initSpi1(USE_SSI_FSS, 16);
    setSpi1BaudRate(20000000, 40000000);

    enablePort(PORTA);
    selectPinPushPullOutput(LDAC_PIN);

    setPinValue(LDAC_PIN, 1);


    //Sets up an interrupt timer for the DAC depending on the user defined rate.
    //32-bit timer that counts up and is periodic.
    //It will cause an interrupt of WTIMER1A.
    //Default symbol rate is 0.5 every second.
    //Note: That this timer should run 2x faster than the symbol rate.
    disableNvicInterrupt(INT_WTIMER1A);
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER1_CFG_R = 0;
    WTIMER1_TAMR_R = TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_PERIOD;
    WTIMER1_TAILR_R = 400;

    WTIMER1_ICR_R = TIMER_ICR_TATOCINT;
    enableNvicInterrupt(INT_WTIMER1A);
    WTIMER1_IMR_R |= TIMER_IMR_TATOIM;

    //Pulse Timer in ms
    disableNvicInterrupt(INT_WTIMER2A);
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER2_CFG_R = 0;
    WTIMER2_TAMR_R = TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_PERIOD;
    WTIMER2_TAILR_R = 400;

    WTIMER2_ICR_R = TIMER_ICR_TATOCINT;
    enableNvicInterrupt(INT_WTIMER2A);
    WTIMER2_IMR_R |= TIMER_IMR_TATOIM;



    wavA.levelFactor = 1;
    wavA.phase = 0;
    wavA.taPhase = 0;
    wavA.cycles = 0;
    wavA.refCycles = 0;
    wavA.frequency = 0;
    wavA.endless = true;
    wavA.lastSymbol = 0xFFFF;
    wavA.maxClip = DAC_AMP_MAX_A;

    wavB.levelFactor = 1;
    wavB.phase = 0;
    wavB.taPhase = 0;
    wavB.cycles = 0;
    wavB.refCycles = 0;
    wavB.frequency = 0;
    wavB.endless = true;
    wavB.lastSymbol = 0xFFFF;
    wavB.maxClip = DAC_AMP_MAX_B;
    //Initial State
    initLUTCos(DAC_OUTA, 1, 0);
    initLUTSine(DAC_OUTB, 1, 0);

}


void initLUTDC(DAC_OUTPUT dac, float dacDC)
{
    uint16_t i;

    if(dac == DAC_OUTA || dac == DAC_OUTBOTH)
    {
        wavA.form = WF_DC;
        wavA.dc = dacDC;
        wavA.amp = 1;
        if(levelingFlag)
            dacDC *= wavA.levelFactor;
    }
    if(dac == DAC_OUTB || dac == DAC_OUTBOTH)
    {
        wavB.form = WF_DC;
        wavB.dc = dacDC;
        wavB.amp = 1;
        if(levelingFlag)
            dacDC *= wavB.levelFactor;
    }

    uint16_t offsetLut = 2048 + convertVoltToBit(dac, dacDC);

    for(i = 0; i < 4096; i++)
    {
        if(dac == DAC_OUTA)
        {
            LUTA[i] = offsetLut;
        }
        else if(dac == DAC_OUTB)
        {
            LUTB[i] = offsetLut;
        }
        else
        {
            LUTA[i] = offsetLut;
            LUTB[i] = offsetLut;
        }
    }
}
//Initializes a 4096 LUT with sine values in it.
//Assumes amp and dc inputs are in volts.
//Vrms is calculated as Vp/sqrt(2)
void initLUTSine(DAC_OUTPUT dac, float dacAmp, float dacDC)
{
   uint16_t i;
   double calc;
   int16_t offsetLut;
   uint16_t maxClip;
   if(dac == DAC_OUTA | DAC_OUTBOTH)
   {
       wavA.form = WF_SINE;
       wavA.amp = dacAmp;
       wavA.dc = dacDC;
       maxClip = wavA.maxClip;
       if(levelingFlag)
       {
           dacDC *= wavA.levelFactor;
           dacAmp *= wavA.levelFactor;
       }
   }
   if(dac == DAC_OUTB | DAC_OUTBOTH)
   {
       wavB.form = WF_SINE;
       wavB.amp = dacAmp;
       wavB.dc = dacDC;
       maxClip = wavB.maxClip;
       if(levelingFlag)
       {
           dacDC *= wavB.levelFactor;
           dacAmp *= wavB.levelFactor;
       }
   }

   //dacAmp = convertVoltToBit(dac, dacAmp);
   //dacDC = convertVoltToBit(dac, dacDC);

   for(i = 0; i < LUT_SIZE; i++)
   {
       //offsetLut = dacAmp*sin( (i*2*M_PI)/(4096) ) + dacDC;
       offsetLut = convertVoltToBit(dac, dacAmp*sin( (i*2*M_PI)/(4096) ) + dacDC);
       if(offsetLut > maxClip)
       {
           offsetLut = maxClip;
       }
       else if(offsetLut < -1*maxClip)
       {
           offsetLut = -1*maxClip;
       }


       calc = 2048 + offsetLut;
       if(dac == DAC_OUTA)
           LUTA[i] = calc;
       else
           LUTB[i] = calc;
   }
}


//Initializes a 4096 LUT with cos values in it.
//Offset 90(specific to output a)
void initLUTCos(DAC_OUTPUT dac, float dacAmp, float dacDC)
{
   uint16_t i;
   double calc;
   int16_t offsetLut;
   uint16_t maxClip;
   if(dac == DAC_OUTA | DAC_OUTBOTH)
   {
       wavA.form = WF_COS;
       wavA.amp = dacAmp;
       wavA.dc = dacDC;
       maxClip = wavA.maxClip;
       if(levelingFlag)
       {
           dacDC *= wavA.levelFactor;
           dacAmp *= wavA.levelFactor;
       }
   }
   if(dac == DAC_OUTB | DAC_OUTBOTH)
   {
       wavB.form = WF_COS;
       wavB.amp = dacAmp;
       wavB.dc = dacDC;
       maxClip = wavB.maxClip;
       if(levelingFlag)
       {
           dacDC *= wavB.levelFactor;
           dacAmp *= wavB.levelFactor;
       }
   }


   for(i = 0; i < LUT_SIZE; i++)
   {
       offsetLut = convertVoltToBit( dac, dacAmp*cos( (i*2*M_PI)/(4096) ) + dacDC);
       if(offsetLut > maxClip)
       {
           offsetLut = maxClip;
       }
       else if(offsetLut < -1*maxClip)
       {
           offsetLut = -1*maxClip;
       }

       calc = 2048 + offsetLut;
       if(dac == DAC_OUTA)
         LUTA[i] = calc;
       else
         LUTB[i] = calc;

   }
}

//Vrms is calculated as Vp/sqrt(3)
void initLutSawTooth(DAC_OUTPUT dac, float dacAmp, float dacDC)
{
    uint16_t i;
    double calc;
    int16_t offsetLut;
    uint16_t maxClip;
    if(dac == DAC_OUTA | DAC_OUTBOTH)
    {
        wavA.form = WF_SAWTOOTH;
        wavA.amp = dacAmp;
        wavA.dc = dacDC;
        maxClip = wavA.maxClip;
        if(levelingFlag)
        {
            dacDC *= wavA.levelFactor;
            dacAmp *= wavA.levelFactor;
        }
    }
    if(dac == DAC_OUTB | DAC_OUTBOTH)
    {
        wavB.form = WF_SAWTOOTH;
        wavB.amp = dacAmp;
        wavB.dc = dacDC;
        maxClip = wavB.maxClip;
        if(levelingFlag)
        {
            dacDC *= wavB.levelFactor;
            dacAmp *= wavB.levelFactor;
        }
    }

    dacAmp = convertVoltToBit(dac, dacAmp);
    dacDC = convertVoltToBit(dac, dacDC);


    for(i = 0; i < 4096; i++)
    {
        offsetLut = ( (2*dacAmp * i)/(4096) ) + dacDC;
        if(offsetLut - dacAmp > maxClip)
        {
            offsetLut = maxClip;
        }
        else if(offsetLut -dacAmp < -1*maxClip)
        {
            offsetLut = -1*maxClip;
        }
       calc = (offsetLut -dacAmp + 2048);
       if(dac == DAC_OUTA)
         LUTA[i] = calc;
       else
         LUTB[i] = calc;
    }

}

void initLutSquare(DAC_OUTPUT dac, float dacAmp, float dacDC)
{
    uint16_t i;
    int16_t offsetLut;
    uint16_t maxClip;
    if(dac == DAC_OUTA | DAC_OUTBOTH)
    {
        wavA.form = WF_SQUARE;
        wavA.amp = dacAmp;
        wavA.dc = dacDC;
        maxClip = wavA.maxClip;
        if(levelingFlag)
        {
            dacDC *= wavA.levelFactor;
            dacAmp *= wavA.levelFactor;
        }
    }
    if(dac == DAC_OUTB | DAC_OUTBOTH)
    {
        wavB.form = WF_SQUARE;
        maxClip = wavB.maxClip;
        wavB.amp = dacAmp;
        wavB.dc = dacDC;
        if(levelingFlag)
        {
            dacDC *= wavB.levelFactor;
            dacAmp *= wavB.levelFactor;
        }
    }

    dacAmp = convertVoltToBit(dac, dacAmp);
    dacDC = convertVoltToBit(dac, dacDC);

    offsetLut = dacAmp;
    if(offsetLut > maxClip)
    {
        offsetLut = maxClip;
    }
    else if(offsetLut < -1*maxClip)
    {
        offsetLut = -1*maxClip;
    }

    for(i = 0; i < 4096; i++)
    {
       if(dac == DAC_OUTA)
         LUTA[i] = (i <= 2048) ? offsetLut + 2048 + dacDC: offsetLut *-1 + 2048 + dacDC;
       else
         LUTB[i] = (i <= 2048) ? offsetLut + 2048 + dacDC: offsetLut *-1 + 2048 + dacDC;
    }

}

void initLutTriangle(DAC_OUTPUT dac, float dacAmp, float dacDC)
{
    uint16_t i;
    int16_t j = 0;
    bool sub = false;
    double calc;
    int16_t offsetLut;
    uint16_t maxClip;
    if(dac == DAC_OUTA | DAC_OUTBOTH)
    {
        wavA.form = WF_TRIANGLE;
        wavA.amp = dacAmp;
        wavA.dc = dacDC;
        maxClip = wavA.maxClip;
        if(levelingFlag)
        {
            dacDC *= wavA.levelFactor;
            dacAmp *= wavA.levelFactor;
        }
    }
    if(dac == DAC_OUTB | DAC_OUTBOTH)
    {
        wavB.form = WF_TRIANGLE;
        wavB.amp = dacAmp;
        wavB.dc = dacDC;
        maxClip = wavB.maxClip;
        if(levelingFlag)
        {
            dacDC *= wavB.levelFactor;
            dacAmp *= wavB.levelFactor;
        }
    }

    dacAmp = convertVoltToBit(dac, dacAmp);
    dacDC = convertVoltToBit(dac, dacDC);


    for(i = 0; i < 4096; i++)
    {
        offsetLut = ( (4*dacAmp * j)/(4096) ) + dacDC;
        if(offsetLut -dacAmp > maxClip)
        {
            offsetLut = maxClip;
        }
        else if(offsetLut-dacAmp < -1*maxClip)
        {
            offsetLut = -1*maxClip;
        }

       if(j == 2048)
          sub = true;

       if(sub)
          j--;
       else
          j++;
       calc = (offsetLut - dacAmp + 2048);
       if(dac == DAC_OUTA)
         LUTA[i] = calc;
       else
         LUTB[i] = calc;
    }

}


//****TONES****
void toneDACM22(char* mode, float amp, float frequency)
{

    sendOption = WAVE;
    if( isStrEq(mode, "cos") )
        oneToneCosM22(amp, frequency);

    else if( isStrEq(mode, "sine") )
        oneToneSineM22(amp, frequency);

    else if( isStrEq(mode, "two") )
        twoToneM22(amp, frequency);


}

//Plays a one tone cos sinusoid.
//Assumes voltage is in volts and frequency is in hertz.
void oneToneCosM22(float dacAmp, float frequency)
{
    initLUTCos(DAC_OUTA, dacAmp, 0);
    initLUTSine(DAC_OUTB, dacAmp, 0);

    setPhaseChangeDACM22(DAC_OUTA, frequency);
    setPhaseChangeDACM22(DAC_OUTB, frequency);
}

//Plays a one tone sine sinusoid.
//Assumes voltage is in volts and frequency is in hertz.
void oneToneSineM22(float dacAmp, float frequency)
{
    initLUTSine(DAC_OUTA, dacAmp, 0);
    initLUTCos(DAC_OUTB, dacAmp, 0);

    setPhaseChangeDACM22(DAC_OUTB, frequency);
    setPhaseChangeDACM22(DAC_OUTA, frequency);

}


//Plays a two tone signal. Cosine is the signal being used.
void twoToneM22(float dacAmp, float frequency)
{

    initLUTCos(DAC_OUTA, dacAmp, 0);
    initLUTSine(DAC_OUTB, 0, 0);

    setPhaseChangeDACM22(DAC_OUTA, frequency);
    setPhaseChangeDACM22(DAC_OUTB, 0);

}


//*****SIGNAL FORMS*****
void dcOutM22(char * channel, float volt)
{
   DAC_OUTPUT dac;
   bool okay = findDacChannelM22( channel, &dac);

   if(okay)
   {
       initLUTDC(dac, volt);
   }
}

void sineM22(char * channel, float freq, float dacAmp, float dacDC, float phase)
{
    DAC_OUTPUT dac;
    bool okay = findDacChannelM22( channel, &dac);

    if(okay)
    {
        initLUTSine(dac, dacAmp, dacDC);
        setPhaseChangeDACM22(dac, freq);
        setPhaseDACM22(dac, phase);
    }
}

void sawM22(char * channel, float freq, float dacAmp, float dacDC, float phase)
{
    DAC_OUTPUT dac;
    bool okay = findDacChannelM22( channel, &dac);

    if(okay)
    {
        initLutSawTooth(dac, dacAmp, dacDC);
        setPhaseChangeDACM22(dac, freq);
        setPhaseDACM22(dac, phase);
    }
}


void square1M22(char * channel, float freq, float dacAmp, float dacDC, float phase)
{
    DAC_OUTPUT dac;
    bool okay = findDacChannelM22( channel, &dac);

    if(okay)
    {
        initLutSquare(dac, dacAmp, dacDC);
        setPhaseChangeDACM22(dac, freq);
        setPhaseDACM22(dac, phase);
    }
}

void triangleM22(char * channel, float freq, float dacAmp, float dacDC, float phase)
{
    DAC_OUTPUT dac;
    bool okay = findDacChannelM22( channel, &dac);

    if(okay)
    {
        initLutTriangle(dac, dacAmp, dacDC);
        setPhaseChangeDACM22(dac, freq);
        setPhaseDACM22(dac, phase);
    }
}

void runM22()
{
    if(!wavA.endless)
        wavA.cycles = wavA.refCycles;
    if(!wavB.endless)
        wavB.cycles = wavB.refCycles;
    runningM22 = true;

    WTIMER1_ICR_R = TIMER_ICR_TATOCINT;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;


}

void stopM22()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER1_ICR_R = TIMER_ICR_TATOCINT;

    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER2_ICR_R = TIMER_ICR_TATOCINT;

    runningM22 = false;
    wavA.phase = 0;
    wavB.phase = 0;
    sendADataDACM22(2048);
    sendBDataDACM22(2048);
    pullLDACM22();
}


//*******SETUP FUNCTIONS*******
//Sets up the timer to operate at cycles/(2x symbol).
//Note that symbol rate is the update rate for the signal.
//Thus, the symbol rate can be seen as updates/second.
//As a result, the cycles per second of the main clock divided by updates a second
//will give the value the clock needs to hit before the next update.
void setSymbolRateDACM22(uint32_t update)
{
    symbolRate = update;
    WTIMER1_CTL_R &= (~TIMER_CTL_TAEN);
    uint32_t timerVal =  40000000 / symbolRate;
    WTIMER1_TAILR_R = timerVal;

}

//Sets up the word value for the phase change of the circuit.
//The delta phase is determined by frequency * time
void setPhaseChangeDACM22(DAC_OUTPUT dac, float frequency)
{
    double calc = ((double)frequency*(1/(double)symbolRate)) * (4294967296);

    if(dac == DAC_OUTA)
    {
        wavA.taPhase = calc;
        wavA.frequency = frequency;
    }
    else if(dac == DAC_OUTB)
    {
        wavB.taPhase = calc;
        wavB.frequency = frequency;
    }
    else
    {
        wavA.taPhase = calc;
        wavB.taPhase = calc;
        wavA.frequency = frequency;
        wavB.frequency = frequency;
    }
}

//Assumes degrees
void setPhaseDACM22(DAC_OUTPUT dac, float phase)
{
    phase = ( phase/360 ) * 4294967296;
    if(dac == DAC_OUTA)
        wavA.phase +=  phase;
    if(dac == DAC_OUTB)
        wavB.phase +=  phase;
    else
    {
        wavA.phase += phase;
        wavB.phase += phase;
    }

}

//change in phase(bits) = frequency * time(1/symbolRate) * 4294967296
//frequency = phase/(time * 4294967296)
//frequency = phase * symbolrate/4294967296
void setCyclesDACM22(char * channel, uint32_t cycles)
{
    DAC_OUTPUT dac;
    bool okay = findDacChannelM22(channel, &dac);
    if(okay)
    {
        if(dac == DAC_OUTA)
        {
           wavA.refCycles = cycles;
           wavA.endless = false;
        }
        else if(dac == DAC_OUTB)
        {
           wavB.refCycles = cycles;
           wavB.endless = false;
        }
        else
        {
           wavA.refCycles = cycles;
           wavB.refCycles = cycles;
           wavA.endless = false;
           wavB.endless = false;
        }
    }
    else
       putsUart0("Invalid channel command");

}


//*******INPUT TLV FUNCTIONS*******


//*******MCP4822 SEND FUNCTIONS*******
//sends data to DAC output A with a specified gain of 2 or 1.
void sendADataDACM22(uint16_t data)
{
    uint16_t command = M22_SENDA | M22_GAIN1 | M22_NO_SHDN | ((data & 0x0FFF) + DAC_DC_A);
    writeSpi1Data(command);
}


//sends data to DAC output B with a specified gain of 2 or 1.
void sendBDataDACM22(uint16_t data)
{
    uint16_t command = M22_SENDB | M22_GAIN1 | M22_NO_SHDN | ((data & 0x0FFF) + DAC_DC_B);
    writeSpi1Data(command);
}

//******HELPER FUNCTIONS******
//Assumes that the remaining bits in the data array is enough to create a word.
//Converts the volts given to the function to 12-bit value.
//*Note: increasing the bits decreases the voltage and decreasing the bits increases the voltage.
float convertVoltToBit(DAC_OUTPUT dac, double volts)
{
   if(dac == DAC_OUTA)
   {
        volts *= DAC_CONV_A*-1;
        if(volts > DAC_AMP_MAX_A)
             volts = DAC_AMP_MAX_A;
        else if (volts <  -1*DAC_AMP_MAX_A)
             volts = -1*DAC_AMP_MAX_A;
   }
   else
   {
       volts *= DAC_CONV_B*-1;
       if(volts > DAC_AMP_MAX_B)
            volts = DAC_AMP_MAX_B;
       else if (volts < -1*DAC_AMP_MAX_B)
            volts = -1*DAC_AMP_MAX_B;
   }
   return volts;

}

void pullLDACM22()
{
    setPinValue(LDAC_PIN, 0);
    __asm("    NOP\n    NOP\n   NOP\n   NOP\n   NOP\n   NOP\n");
    setPinValue(LDAC_PIN, 1);
}

void setEndlessDACM22(char* channel)
{
    DAC_OUTPUT dac;
    bool okay = findDacChannelM22(channel, &dac);

    if(okay)
    {
        if(dac == DAC_OUTA)
            wavA.endless = true;
        else if(dac == DAC_OUTB)
            wavB.endless = true;
        else
        {
            wavA.endless = true;
            wavB.endless = true;
        }
    }
    else
        putsUart0("Invalid Endless channel\n");
}

//Sets the dac output to be one of three variables based on
//the channel the user inputted. If not a valid dac output,
//it will return a false otherwise true.
bool findDacChannelM22(char * channel, DAC_OUTPUT *dac)
{
   bool okay = true;
   if( isStrEq(channel, "dacA") )
       *dac = DAC_OUTA;
   else if( isStrEq(channel, "dacB") )
       *dac = DAC_OUTB;
   else if( isStrEq(channel, "both") )
       *dac = DAC_OUTBOTH;
   else
   {
        okay = false;
        putsUart0("Incorrect channel format\n");
   }
   return okay;
}

void gainSweepM22(uint8_t points, double startFreq, double endFreq)
{
    if(levelingFlag)
    {
        putsUart0("Disabling leveling for gainsweep\n");
        levelingFlag = false;
    }
    double powerOf = log10(startFreq);
    uint8_t i;
    double step = ( (log10(endFreq) - log10(startFreq))/(points-1) );

    double vRef, vAtt, gain;

    sineM22("dacA", startFreq, 2.5, 0, 0);
    runM22();
    putsUart0("\n\n");

    putsUart0("          Frequency         Gain\n");
    putsUart0("--------------------------------\n");
    for(i = 0; i < points; i++)
    {
        double freq = pow(10, powerOf);
        uint8_t waitFactor = points >> 5;
        waitFactor = (waitFactor == 0) ? 1 : waitFactor;

        setPhaseChangeDACM22(DAC_OUTA, freq);
        waitMicrosecond(TAU_3/waitFactor);
        /*if(points < 30)
            waitMicrosecond(TAU_3);
        else
            waitMicrosecond(50000);
        */
        //vRef = ( ( ( readAdc1Ss3() + 0.5 ) / 4096.0 * 3.3) + 0.0059)/1.0069;
        vRef = convertBitToVoltADC(ADC_INA, readAdc1Ss3() );
        //vAtt = ( ( (readAdc0Ss3()+0.5) / 4096.0 * 3.3) + 0.0014)/1.0081;
        vAtt = convertBitToVoltADC(ADC_INB, readAdc0Ss3() );
        gain = 20*log10(vAtt/vRef);

        putsUart0("         ");
        printFloatToUart0(freq);
        putsUart0("         ");
        printFloatToUart0(gain);
        putcUart0('\n');
        powerOf += step;


    }
    putcUart0('\n');
    stopM22();
}

//reads raw and voltage from inA and inB
//used for debugging and vin command
void readSensorDACM22(char *channel)
{
    uint16_t raw0, raw1;
    float v0, v1;

    DAC_OUTPUT dac;
    bool ok = findDacChannelM22(channel, &dac);

    // Read sensor
    if(ok)
    {
        if(dac == DAC_OUTA || dac == DAC_OUTBOTH)
        {
            raw1 = readAdc1Ss3();
            v1 = convertBitToVoltADC(ADC_INA, raw1);
            putsUart0("Raw ADC(A): ");
            printUInt32ToUart0(raw1);
            putcUart0('\n');

            putsUart0("\nVolts(A): ");
            printFloatToUart0(v1);
            putsUart0("\n\n\n----------------------------\n");
        }

        if(dac == DAC_OUTB || dac == DAC_OUTBOTH)
        {
            raw0 = readAdc0Ss3();
            v0 = convertBitToVoltADC(ADC_INB, raw0);

            putsUart0("Raw ADC(B): ");
            printUInt32ToUart0(raw0);
            putcUart0('\n');

            putsUart0("\nVolts(B): ");
            printFloatToUart0(v0);

            putsUart0("\n\n\n----------------------------\n");
        }
    }
    else
    {
        putsUart0("Channel Error\n");
    }

}

#if 1
bool detectM22(DAC_OUTPUT dacOut, ADC_INPUT adcIn)
{
    bool detect = false;
    if(runningM22)
    {
        WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        WTIMER1_ICR_R = TIMER_ICR_TATOCINT;
    }

    switch(dacOut)
    {
        case DAC_OUTA:
            sendADataDACM22( convertVoltToBit(DAC_OUTA, 4.0) + 2048 );
            break;
        case DAC_OUTB:
            sendBDataDACM22( convertVoltToBit(DAC_OUTB, 4.0) + 2048 );
            break;
        case DAC_OUTBOTH:
            sendADataDACM22( convertVoltToBit(DAC_OUTA, 4.0) + 2048 );
            sendBDataDACM22( convertVoltToBit(DAC_OUTB, 4.0) + 2048 );
            break;
    }

    pullLDACM22();
    waitMicrosecond(TAU_3);
    float v1, v0;
    if(adcIn == ADC_INA)
    {
       v1 = convertBitToVoltADC(ADC_INA, readAdc1Ss3());
       detect = v1 > 0.1;
    }
    else if(adcIn == ADC_INB)
    {
       v0 = convertBitToVoltADC(ADC_INB, readAdc0Ss3());
       detect = v0 > 0.1;
    }
    else
    {
       v1 = convertBitToVoltADC(ADC_INA, readAdc1Ss3());
       v0 = convertBitToVoltADC(ADC_INB, readAdc0Ss3());
       detect = v1 > 0.5 && v0 > 0.5;
    }

    return detect;
}
#endif

//Tries to level the circuit with various loads.
//will stop the circuit, and user needs to regenerate
//circuit by reissuing
void levelOnM22()
{
    bool okA, okB;
    okA = detectM22(DAC_OUTA, ADC_INA);
    okB = detectM22(DAC_OUTB, ADC_INB);


    float vAtt;
    if(okA)
    {
        levelingFlag = true;
        sendADataDACM22( convertVoltToBit(DAC_OUTA, 2.0) + 2048 );
        pullLDACM22();

        waitMicrosecond(TAU_3);
        vAtt = convertBitToVoltADC( ADC_INA, readAdc1Ss3() );
        wavA.levelFactor = 2.0/vAtt;
        regenerateWaveM22(DAC_OUTA);

        sendADataDACM22( 2048 );
        pullLDACM22();
    }
    else
    {
        wavA.levelFactor = 1;
        putsUart0("Could not detect adcA on dacA\n");
    }

    if(okB)
    {
        levelingFlag = true;
        sendBDataDACM22( convertVoltToBit(DAC_OUTA, 2.0) + 2048 );
        pullLDACM22();

        waitMicrosecond(TAU_3);
        vAtt = convertBitToVoltADC( ADC_INB, readAdc0Ss3() );
        wavB.levelFactor = 2.0/vAtt;
        regenerateWaveM22(DAC_OUTB);

        sendBDataDACM22( 2048 );
        pullLDACM22();
    }
    else
    {
        wavB.levelFactor = 1;
        putsUart0("Could not detect adcB on dacB\n");
    }

    if(!okA && !okB)
    {
        putsUart0("Could not level either outputs, turning off leveling...\n");
        levelingFlag = false;
    }

    if(runningM22)
    {
        WTIMER1_CTL_R |= TIMER_CTL_TAEN;
        WTIMER1_ICR_R = TIMER_ICR_TATOCINT;
    }



}


void levelOffM22()
{
    levelingFlag = false;
}

void regenerateWaveM22(DAC_OUTPUT dac)
{
    struct Wave *wptr = (dac == DAC_OUTA) ? &wavA:&wavB;
    if(wptr->form == WF_DC)
    {
        initLUTDC(dac, wptr->dc);
    }
    else if(wptr->form == WF_SINE)
    {
        initLUTSine(dac, wptr->amp, wptr->dc);
    }
    else if(wptr->form == WF_COS)
    {
        initLUTCos(dac, wptr->amp, wptr->dc);
    }
    else if(wptr->form == WF_SQUARE)
    {
        initLutSquare(dac, wptr->amp, wptr->dc);
    }
    else if(wptr->form == WF_SAWTOOTH)
    {
        initLutSawTooth(dac, wptr->amp, wptr->dc);
    }
    else
    {
        initLutTriangle(dac, wptr->amp, wptr->dc);
    }

}

//Converts the voltage to bit value based on which port chose
//as well as the bit value given by the adc.
//Defaults dacB
float convertBitToVoltADC(ADC_INPUT adcIn, uint16_t voltBit)
{
    float volt;
    if(adcIn == ADC_INA)
    {
        volt = ( ( (voltBit+0.5) / 4096.0 * 3.3) + 0.0059)/1.0069;
    }
    else if(adcIn == ADC_INB)
    {
        volt = ( ( (voltBit+0.5) / 4096.0 * 3.3) - 0.0014)/1.0081;
    }
    return volt;
}

void reset()
{
    APINT_REG = 0x05FA0004; //Unlock and reset
}
