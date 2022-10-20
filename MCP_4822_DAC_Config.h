#ifndef __MCP_4822_DAC_Config

#define __MCP_4822_DAC_Config

//#define __WIRELESSCOM


//Config
#define DAC_MAX 4095

//Describes the numerical value sent to the DAC to the voltage given(V).
//#/Volts
#ifndef __WIRELESSCOM

//PIN DEFINES
#define LDAC_PIN PORTD,2

#define DAC_AMP_MAX_A 2008
#define DAC_AMP_MAX_B 2010


#define DAC_CONV_A 376.97
#define DAC_CONV_B 378.97
//-48
#define DAC_DC_A -48
//-37
#define DAC_DC_B -45


#endif






#ifdef __WIRELESSCOM
//PIN DEFINES
#define LDAC_PIN PORTA,3

#define DAC_AMP_MAX_A 1930
#define DAC_AMP_MAX_B 1930

#define DAC_CONV_A 4013.4
#define DAC_CONV_B 4033.4
//79
#define DAC_DC_A 79
//94
#define DAC_DC_B 94

#endif



#define SAMPLING_INIT 0
//in microseconds. Specified for 47kohm and 1μf series circuit
#define TAU_3 141000

#endif
