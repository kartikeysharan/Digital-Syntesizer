#include <stdio.h>

#include "MCP_Menu.h"
#include "MCP_4822_DAC.h"
#include "uart0.h"
#include "uart_UI.h"
#include "wait.h"
#include "adc0.h"
#include "adc1.h"
#include "wait.h"


void initHW(void)
{
    initSystemClockTo40Mhz();

    _delay_cycles(3);
    initUart0();
    setUart0BaudRate(115200, 40e6);
    initDACM22();

    initAdc0Ss3();
    initAdc1Ss3();

    setAdc0Ss3Mux(0);
    setAdc0Ss3Log2AverageCount(2);
    setAdc1Ss3Mux(1);
    setAdc1Ss3Log2AverageCount(2);

}




int main(void)
{
    initHW();

    MCP_Menu2();



}
