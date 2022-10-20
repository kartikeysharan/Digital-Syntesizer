// ADC1 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// ADC1 SS3

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "adc0.h"

#define ADC_CTL_DITHER          0x00000040

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initAdc1Ss3()
{
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;
    _delay_cycles(16);

    // Configure ADC
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

// Set SS3 input sample average count
void setAdc1Ss3Log2AverageCount(uint8_t log2AverageCount)
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_SAC_R = log2AverageCount;                   // sample HW averaging
    if (log2AverageCount == 0)
        ADC1_CTL_R &= ~ADC_CTL_DITHER;               // turn-off dithering if no averaging
    else
        ADC1_CTL_R |= ADC_CTL_DITHER;                // turn-on dithering if averaging
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

// Set SS3 analog input
void setAdc1Ss3Mux(uint8_t input)
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_SSMUX3_R = input;                           // Set analog input for single sample
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

// Request and read one sample from SS3
int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    while (ADC1_SSFSTAT3_R & ADC_SSFSTAT3_EMPTY);
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}
