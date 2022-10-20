// ADC1  Library
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

#ifndef ADC1_H_
#define ADC1_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initAdc1Ss3();
void setAdc1Ss3Log2AverageCount(uint8_t log2AverageCount);
void setAdc1Ss3Mux(uint8_t input);
int16_t readAdc1Ss3();

#endif
