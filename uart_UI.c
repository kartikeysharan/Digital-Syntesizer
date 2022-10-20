//UART Interface



/****************************************************************************************************
TARGET DEVICE : TM4c123gh6pm
INTERFACE : TM4c123gh6pm Uart0 debugging interface.
PURPOSE : Utilizes Uart0 to allow the user to communicate with the redboard through commands in a terminal.
****************************************************************************************************/
//Directives

#include "uart_UI.h"
#include "uart0.h"

void getsUart0(USER_DATA *input)
{
    uint8_t count = 0;

    bool cFlag = 1;
    char c;

    while(cFlag)
    {
        c = getcUart0();

        //Backspace function. Apparently in windows it thinks that backspace is delete and delete as escape. ?????
        if(c == 8 || c == 127)
        {
            if(count > 0)
            {
                count--;
            }
        }
        else
        {
            //Enter Function
            if(c == 10 || c == 13)
            {
                input->buffer[count] = '\0';
                cFlag = 0;
            }
            else
            {
                //Printable Character Function
                if(c >= 32)
                {
                    input->buffer[count++] = c;
                    if(count >= MAX_CHARS)
                    {
                        input->buffer[MAX_CHARS] = '\0';
                        cFlag = 0;
                    }
                }
            }

        }
    }
}



//Parses fields in the buffer of the USER_DATA structure to be part of the alphabet, number, or delimiter... not really with delimiter.
void parseFields(USER_DATA *input)
{
    bool pDelim = true;
    input->fieldCount = 0;
    uint8_t count = 0;

    while(input->buffer[count] != '\0' && input->fieldCount < MAX_FIELDS)
    {
        if(pDelim == false)
        {
            if(checkDelim(&(input->buffer[count])))
            {
                pDelim = true;
                input->buffer[count] = '\0';
            }
            count++;
        }
        else
        {
            if(checkDelim(&(input->buffer[count])))
            {
                input->buffer[count] = '\0';
                count++;
            }
            else
            {
                pDelim = false;
                input->fieldPosition[input->fieldCount] = count;

                if(checkAlpha(&(input->buffer[count])))
                    input->fieldType[input->fieldCount] = 'a';
                else
                    input->fieldType[input->fieldCount] = 'n';

                (input->fieldCount)++;
                count++;

            }

        }
    }

}

//First, we check if we have an argument in the first place. Then, we check if USER DATA has at least
//the minimum number of arguments passed is equal to less than or equal to field count -1(this is because field count includes the initial
//command which should be removed).
//If both of these conditions are true, we then check at the first position of the first field which should be the string command, and if it
//matches the passed in string strCommand then this should the command we are looking for. Thus, we return a true value.
bool isCommand(USER_DATA *input, const char strCommand[], uint8_t minArguments)
{
    bool flag = false;
    if(input->fieldCount != 0 && (input->fieldCount - 1) >= minArguments)
    {
        uint8_t pos = input->fieldPosition[0];
        uint8_t x = 0;
        while(input->buffer[pos] == strCommand[x] && flag == false)
        {
            if(input->buffer[pos] != '\0')
            {
                pos++;
                x++;
            }
            else
                flag = true;
        }
    }
    return flag;
}

//Finds the char pointer at field number, if it exists, and returns it.
//If it doesn't exist, return NULL(0).
char *getFieldString(USER_DATA *input, uint8_t fieldNumber)
{
    char *c = 0;

    if(fieldNumber < input->fieldCount)
    {
        if(input->fieldType[fieldNumber]  == 'a')
        {
            uint8_t pos = input->fieldPosition[fieldNumber];
            c = &(input->buffer[pos]);
        }
    }
    return c;

}


//Finds the char string at field number, and, if it is an integer, convert it to a int32_t.
//If it does not exist, return 0.
int32_t getFieldInteger(USER_DATA *input, uint8_t fieldNumber)
{
    int32_t x = 0;

    if(fieldNumber < input->fieldCount)
    {
        if(input->fieldType[fieldNumber] == 'n')
        {
            uint8_t pos = input->fieldPosition[fieldNumber];
            bool negative = false;

            if(input->buffer[pos] == '-')
            {
                negative = true;
                pos++;
            }

            while(input->buffer[pos] != '\0')
            {
                x *= 10;
                x += input->buffer[pos] - '0';
                pos++;
            }

            if(negative)
                x *= -1;
        }

    }
    return x;
}

//Finds the char string at field number, and, if it is a number, convert it to a float.
//If it does not exist, return 0.
float getFieldFloat(USER_DATA *input, uint8_t fieldNumber)
{
    float x = 0;
    uint8_t decimalPlace = 1;
    float decimal = 0;

    if(fieldNumber < input->fieldCount)
    {
        if(input->fieldType[fieldNumber] == 'n')
        {
            uint8_t pos = input->fieldPosition[fieldNumber];
            bool negative = false;

            if(input->buffer[pos] == '-')
            {
                negative = true;
                pos++;
            }

            while( input->buffer[pos] != '.' && input->buffer[pos] != '\0')
            {
                x *= 10;
                x += input->buffer[pos] - '0';
                pos++;
            }

            if(input->buffer[pos] == '.')
                pos++;

            while(input->buffer[pos] != '\0')
            {
                decimal = input->buffer[pos] - '0';

                uint8_t i;
                for(i = 0; i < decimalPlace; i++)
                {
                    decimal /= 10;
                }
                x += decimal;
                pos++;
                decimalPlace++;
            }

            if(negative)
                x *= -1;
        }

    }
    return x;
}



//****************************************************************************************************
//EXTRANEOUS FUNCTIONS

//Checks if char is part of the english alphabet.
bool checkAlpha(const char *c)
{
    return ((*c >= 65 && *c <= 90) || (*c >= 97 && *c <= 122));
}

//Checks if the char is number.
bool checkNum(const char *c)
{
    return ((*c >= 48 && *c <= 57) || *c == 45 || *c == 46);
}

//Checks if the char is a delimiter.
bool checkDelim(const char *c)
{
    return !(checkAlpha(c) || checkNum(c));
}

//This functions prints up to an int32_t num.
void printInt32ToUart0(int32_t num)
{
    char c[11];
    int8_t x = -1;
    int32_t helper = 0;

    if(num == 0)
    {
        putcUart0('0');
    }
    else
    {
        if(num < 0)
        {
            putcUart0('-');
            num *= -1;
        }

        helper = num;
        while(helper != 0)
        {
            helper = helper/10;
            x++;
        }

        int i;
        for(i = x; i >=0 ; i--)
        {
            c[i] = (num % 10) + '0';
            num = num/10;
        }
        c[x+1] = '\0';
        putsUart0(c);
    }
}


//Prints an uint32_t variable to the terminal.
void printUInt32ToUart0(uint32_t num)
{
    char c[11];
    int8_t x = -1;
    int32_t helper = 0;

    if(num == 0)
    {
        putcUart0('0');
    }
    else
    {
        helper = num;
        while(helper != 0)
        {
            helper = helper/10;
            x++;
        }

        int i;
        for(i = x; i >=0 ; i--)
        {
            c[i] = (num % 10) + '0';
            num = num/10;
        }
        c[x+1] = '\0';
        putsUart0(c);
    }
}

//Prints a float number
#define FLOAT_PRECISION 3
void printFloatToUart0(float num)
{
    char c[10 + 1 + FLOAT_PRECISION];
    int8_t i = 0;

    if(num < 0)
    {
        putcUart0('-');
        num *= -1;
    }

    uint32_t numUInt = num;
    float numFlt = num - numUInt;



    c[10] = '.';
    for(i = 11; i < (10 + 1 + FLOAT_PRECISION); i++)
    {
        numFlt *= 10;
        c[i] = (int)(numFlt) + '0';
        numFlt = numFlt - (int)numFlt;
    }


    for(i = 9; i >= 0 && numUInt != 0; i--)
    {
        c[i] = (numUInt % 10) + '0';
        numUInt /= 10;
    }

    for(i = i+1; i < (10 + 1 + FLOAT_PRECISION); i++)
    {
        putcUart0(c[i]);
    }

}

//Strcmp but boolean
bool isStrEq(char *str1, char *str2)
{
    bool eq = false;

    uint32_t i = 0;
    while(str1[i] == str2[i] && str1[i] != '\0')
    {
       if(str1[i+1]  == '\0')
          eq = true;
       i++;
    }
    return eq;
}

//Checks 1 field.
bool check1Field(char fieldType, uint8_t fieldNum, USER_DATA input)
{
    return input.fieldType[fieldNum] == fieldType;
}

//Checks multiple fields based on the string passed through.
bool checkFields(char *fields, uint8_t numFields, USER_DATA input)
{
    uint8_t i = 1;
    bool okay = true;
    while( i <= numFields && okay)
    {
        if(input.fieldType[i] != fields[i-1])
            okay = false;

        i++;
    }
    return okay;
}




