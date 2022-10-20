//MCP_4821_DAC Interface


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
    PA4: ~SHDN
DEV CONTROL
****************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "MCP_4822_DAC_Config.h"


#ifndef __MCP_4822_DAC_H
#define __MCP_4822_DAC_H

#define M22_GAIN1 0x2000
#define M22_GAIN2 0

#define M22_SENDA 0
#define M22_SENDB 0x8000

#define M22_NO_SHDN 0x1000

#define M22_90deg 0x400


//Command to DACM22 to send to DAC A and B respectively with gain of 1, and no shutdown.
#define SEND_DACA (0x3000 + DAC_DC_A)
#define SEND_DACB (0xB000 + DAC_DC_B)

#define OFFSET_A 0
#define OFFSET_B 0



typedef enum __SYMBOL_FORMAT
{
    BPSK = 1, QPSK = 2, PSK8 = 3, QAM16 = 4, QAM64 = 6
} SYMBOL_FORMAT;


typedef enum __SEND_OPTION
{
    WAVE, STREAM
} SEND_OPTION;

typedef enum __DAC_OUTPUT
{
    DAC_OUTBOTH, DAC_OUTA, DAC_OUTB
} DAC_OUTPUT;

typedef enum __ADC_INPUT
{
   ADC_INA, ADC_INB, ADC_INBOTH
} ADC_INPUT;

typedef enum __DAC_WAVEFORM
{
    WF_DC, WF_SINE, WF_COS, WF_SQUARE, WF_SAWTOOTH, WF_TRIANGLE
} DAC_WAVEFORM;

struct Wave
{
    DAC_WAVEFORM form;
    float amp;
    float dc;
    float levelFactor;
    uint32_t phase;
    uint32_t taPhase;
    uint32_t cycles;
    uint32_t refCycles;
    float frequency;
    bool endless;
    float maxClip;
};

void cyclesAIRS();
void cyclesBIRS();

void sendDACM22();
void sendWaveDACM22();
void sendStreamDACM22();

void initDACM22();
void initLUTSine(DAC_OUTPUT dac, float dacAmp, float dacDC);
void initLUTCos(DAC_OUTPUT dac, float dacAmp, float dacDC);
void initLUTDC(DAC_OUTPUT dac, float dacDC);
void initLutSawTooth(DAC_OUTPUT dac, float dacAmp, float dacDC);
void initLutSquare(DAC_OUTPUT dac, float dacAmp, float dacDC);
void initLutTriangle(DAC_OUTPUT dac, float dacAmp, float dacDC);

void setSymbolRateDACM22(uint32_t update);
void setPhaseChangeDACM22(DAC_OUTPUT dac, float frequency);
void setPhaseDACM22(DAC_OUTPUT dac, float phase);
void setCyclesDACM22(char * channel, uint32_t cycles);
void setEndlessDACM22(char * channel);

void startSendDACM22(char* mode, uint32_t dataCount);
void runM22();
void stopM22();
void sendADataDACM22(uint16_t data);
void sendBDataDACM22(uint16_t data);


void pulseM22(char *format, float volts, uint16_t ms);
void sendPulseM22(void);


void toneDACM22(char* mode, float amp, float frequency);
void oneToneCosM22(float volt, float frequency);
void oneToneSineM22(float volt, float frequency);
void twoToneM22(float volt, float frequency);

void dcOutM22(char * dac, float volt);
void sineM22(char * channel, float freq, float dacAmp, float dacDc, float phase);
void sawM22(char * channel, float freq, float dacAmp, float dacDC, float phase);
void square1M22(char * channel, float freq, float dacAmp, float dacDC, float phase);
void triangleM22(char * channel, float freq, float dacAmp, float dacDC, float phase);

void streamM22(char* format, uint8_t dataCount);
void setUpEqualToneM22();
void setSymbolFormat(SYMBOL_FORMAT sy);

void RRCOSDACM22(uint16_t *symbolI, uint16_t *symbolQ);
void turnOffRRCDACM22();
void turnOnRRCDACM22();

float convertVoltToBit(DAC_OUTPUT dac, double volts);
float convertBitToVoltADC(ADC_INPUT adc, uint16_t voltBit);
void pullLDACM22();
void setClipVoltValueDACM22(DAC_OUTPUT dac, float volt);
bool findDacChannelM22(char * channel, DAC_OUTPUT *dac);


void readSensorDACM22();
void reset();
void preambleDACM22();
bool detectM22(DAC_OUTPUT dacOut, ADC_INPUT dacIn);
void gainSweepM22(uint8_t points, double startFreq, double endFreq);
#endif
