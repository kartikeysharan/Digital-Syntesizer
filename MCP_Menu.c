   #include "MCP_4822_DAC.h"
#include "MCP_Menu.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "uart0.h"
#include "uart_UI.h"
#include "clock.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

void MCP_Menu1()
{
    //USER_DATA instruction;

    while(1)
    {
    }
}


void MCP_Menu2()
{
    USER_DATA instruction;
    setSymbolRateDACM22(100000);
    stopM22();
    float dacDC, phase;
    while(1)
    {
        putsUart0("Enter a command: ");
        getsUart0(&instruction);
        parseFields(&instruction);

        if( isCommand(&instruction, "help", 0) )
        {
            putsUart0("Help received\n");
        }
        else if( isCommand(&instruction, "Vin", 1) )
        {
            if( instruction.fieldType[1] == 'a')
            {
                readSensorDACM22( getFieldString(&instruction, 1) );
            }
        }
        else if( isCommand(&instruction, "reset", 0) )
        {
            putsUart0("Resetting Hardware...\n");
            reset();
        }
        else if( isCommand(&instruction, "dcOut", 2) )
        {
            if( checkFields("an", 2, instruction) )
            {
                dcOutM22( getFieldString(&instruction, 1), getFieldFloat(&instruction, 2) );
            }
        }
        else if( isCommand(&instruction, "run", 0) )
        {
            runM22();
        }
        else if( isCommand(&instruction, "sine", 3) )
        {
            if( checkFields( "ann", 3, instruction) )
            {
                dacDC = (instruction.fieldCount >= 4) ? getFieldFloat(&instruction, 4) : 0;
                phase = (instruction.fieldCount >= 5) ? getFieldFloat(&instruction, 5) : 0;

                sineM22( getFieldString(&instruction, 1), getFieldFloat(&instruction, 2), getFieldFloat(&instruction, 3), dacDC, phase );
            }
            else
            {
                putsUart0("Invalid sine format\n");
            }
        }
        else if( isCommand(&instruction, "sawtooth", 3) )
        {
            if( checkFields( "ann", 3, instruction) )
            {
                dacDC = (instruction.fieldCount >= 4) ? getFieldFloat(&instruction, 4) : 0;
                phase = (instruction.fieldCount >= 5) ? getFieldFloat(&instruction, 5) : 0;

                sawM22( getFieldString(&instruction, 1), getFieldFloat(&instruction, 2), getFieldFloat(&instruction, 3), dacDC, phase );
            }
            else
            {
                putsUart0("Invalid saw format\n");
            }
        }
        else if( isCommand(&instruction, "square1", 3) )
        {
            if( checkFields( "ann", 3, instruction) )
            {
                dacDC = (instruction.fieldCount >= 4) ? getFieldFloat(&instruction, 4) : 0;
                phase = (instruction.fieldCount >= 5) ? getFieldFloat(&instruction, 5) : 0;

                square1M22( getFieldString(&instruction, 1), getFieldFloat(&instruction, 2), getFieldFloat(&instruction, 3), dacDC, phase );
            }
            else
            {
                putsUart0("Invalid square 1 format\n");
            }
        }
        else if( isCommand(&instruction, "triangle", 3) )
        {
            if( checkFields( "ann", 3, instruction) )
            {
                dacDC = (instruction.fieldCount >= 4) ? getFieldFloat(&instruction, 4) : 0;
                phase = (instruction.fieldCount >= 5) ? getFieldFloat(&instruction, 5) : 0;

                triangleM22( getFieldString(&instruction, 1), getFieldFloat(&instruction, 2), getFieldFloat(&instruction, 3), dacDC, phase );
            }
            else
            {
                putsUart0("Invalid square 1 format\n");
            }
        }
        else if(isCommand( &instruction, "stop", 0) )
        {
            stopM22();
        }
        else if( isCommand(&instruction, "cycles", 2) )
        {
            if( checkFields( "an", 2, instruction) )
            {
                setCyclesDACM22( getFieldString(&instruction, 1), getFieldInteger(&instruction, 2) );
            }
            else if( isStrEq( getFieldString(&instruction, 2), "continuous") )
            {
                setEndlessDACM22( getFieldString(&instruction, 1) );
            }
            else
            {
                putsUart0("Invalid cycles format\n");
            }
        }
        else if( isCommand(&instruction, "endless", 1) )
        {
            if( instruction.fieldType[1] == 'a' )
            {
                setEndlessDACM22( getFieldString(&instruction, 1) );
            }
        }
        else if( isCommand(&instruction, "gainSweep", 3) )
        {
            if(checkFields("nnn", 3, instruction))
            {
                gainSweepM22( getFieldInteger(&instruction, 1), getFieldFloat(&instruction, 2), getFieldFloat(&instruction, 3) );
            }
        }
        else if( isCommand(&instruction, "level", 1) )
        {
            if(instruction.fieldType[1] == 'a')
            {
                if( isStrEq(getFieldString(&instruction, 1), "ON" ) )
                {
                    levelOnM22();
                }
                else if( isStrEq(getFieldString(&instruction, 1), "OFF" ) )
                {
                    levelOffM22();
                }
                else
                {
                    putsUart0("Non-valid option for leveling\n");
                }
            }
            else
            {
                putsUart0("Invalid level format\n");
            }
        }
        else
        {
            putsUart0("Invalid Command.\n");
        }


    }
}
