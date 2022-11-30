// Author - Sanket S Hiremath
// UTA ID- 1001830284

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "uart0.h"
#include "gpio.h"
#include "shell.h"



//char c;
char str[maxChar] = {};
int32_t testNum;




/**
  * @brief  Callback function to reboot the MCU.
  */
void rebootMCU()
{
    __asm(" SVC  #15");

}

/**
  * @brief  Callback function to display the process status.
  */
void ps()
{
    putsUart0("PS called\n\r");
    putsUart0("\r\n>");

}

/**
  * @brief  Displays memory usage by the process (thread) with the matching PID.
  * @param  pid: PID id number.
  */
void pmap(uint32_t pid)
{
    putsUart0("Memory usage by PID:\n\r");
    putsUart0("\r\n>");
}



/**
  * @brief  Displays the PID of the process (thread).
  * @param  name: Process name.
  */
void pidof(uint32_t* pidNum, char name[])
{
    putsUart0("\r\n>");
    __asm(" SVC  #20");
}



//----------------------------------------------------------------//

uint32_t strLen(const char* str)
{
    uint32_t len = 0;
    while(str[len]) len++;
    return len;
}

void convertDec_Hex(uint32_t decNum)
{

    uint8_t i, j = 0;
    uint32_t quot, result;
    static char hexNum[16];

    quot = decNum;
     while (quot != 0)
     {
         result = quot % 16;
         if (result < 10)
             hexNum[j++] = 48 + result;
         else
             hexNum[j++] = 55 + result;
         quot = quot / 16;
     }

     // display integer into character
     for (i = 16; i > 0; i--)
     {
         putcUart0(hexNum[i]);
     }
     putcUart0(hexNum[0]);

}

void convertNumToString(uint32_t num)
{
    char dataBuffer[10] = {0};
    int8_t i = 9;

    if(num == 0)
    {
        putcUart0('0');
    }
    while(num)
    {
        dataBuffer[i] = (num % 10) + '0';
        num /= 10;
        i--;
    }
    if(i < 0)
    {
        i = 0;
    }
    for(i = 0; i < 10; i++)
    {
        putcUart0(dataBuffer[i]);
    }
}


//change
void printfString(uint8_t spaceToReserve, char* s)
{
    putsUart0(s);
    spaceToReserve -= strLen(s);
    while(spaceToReserve--)
        putcUart0(' ');
}

/**
  * @brief This function compares each character of  "fromStrCompare" with "toStrCompare".

  */
bool compareString(const char fromStrCompare[], const char toStrCompare[], uint8_t strSize)
{
    uint8_t index = 0;


    while((fromStrCompare[index] != '\0') && (toStrCompare[index] != '\0') && (index < strSize))
    {
        if(fromStrCompare[index] != toStrCompare[index])
        {
            return false;
        }
        index++;
    }

    return true;
}

/**
  * @brief This function gets the character string entered by the user on the terminal and stores it
  *        in a struct variable for further processing.
  * @param Struct variable to stores all the data related to the input char string.
  */
void getString(inputData* data)
{

    uint8_t index = 0;


    while(true)
    {
    char c = getcUart0();

    // when pressed enter or /r
    if (c == 13)
    {
        for(index = 0; index < 20; index++ )
        {

//          putcUart0(data->inputBuffer[index]);
          data->prevCommand[index] = data->inputBuffer[index];
          data->inputBuffer[index] = 0;
        }

        putsUart0("\r\n>");
        index = 0;
        return;
    }

    // when entered special characters, numbers & alphabets
    else if (c > 31)
    {
        putcUart0(c);

        if(c >= 65 && c<=90)
        {
            c = c+32;
        }

        data->inputBuffer[index++] = c;

      // if the number of characters exceeds the input buffer length of 20 characters
        if (index == (maxChar-1))
            {
                putsUart0("Command length exceed, please type the correct command! \n\r");
                data->inputBuffer[index++] = 13;
            }
    }
    else if(c < 31)
        continue;

    }
}

/**
  * @brief  This function matches the user entered command (not the argument) with a pre-defined string.
  * @param  data: Struct variable which stores all the info related to the user input string.
  * @param  userCommand: A string which will be compared to the command string entered by the user.
  * @param  mode: 0- If there is only a command and no argument needed to be entered in the terminal.
  *               1- If there is an argument along with the command that is needed to be entered in the terminal.
  * @retval  True if the user entered command matches the string in the  @param "userCommand".
  */
bool matchCommand(inputData* data, char userCommand[], uint8_t mode)
{
    uint8_t index = 0;

    if (mode == 0)
    {
        while((data->prevCommand[index] != '\0') && (userCommand[index] != '\0') && index < maxChar)

            {
                if(data->prevCommand[index] != userCommand[index])
                    return false;
                index++;
            }

        return true;
    }
    else
    {

        while((data->prevCommand[index] != 32) && (userCommand[index] != '\0') && index < maxChar)
        {
            if(data->prevCommand[index] != userCommand[index])
                return false;
            index++;
            data->bufferIndex = index;
        }

        return true;

    }
}

/**
  * @brief   This function gets the command argument entered by the user on the terminal and matches
  *          it with a pre-defined argument string.
  * @param   data: Struct variable which stores all the info related to the user input string.
  * @param   userArgument: A string which will be compared to the argument entered by the user.
  * @retval  True if the user entered argument matches the string in the @param "userArgument".
  */

bool matchCommandArg(inputData* data, char userArgument[])
{
    uint8_t index = 0;
    uint8_t count;
    uint8_t i;

        index = 0;
        count = data->bufferIndex + 1;
        for(i = count; i < 20; i++)
        {
            // extracting the argument from the user entered command on putty
            data->argument[index] = data->prevCommand[i];
            index++;
        }

        index = 0;
        while(data->argument[index] != '\0' && index < maxChar)
        {
            if(data->argument[index] != userArgument[index])
                return false;
            index++;
        }

        return true;

}


/**
  * @brief   This function extracts the 32-bit integer entered by the user as an argument.
  * @param   data: Struct variable which stores all the info related to the user input string.
  * @retval  32-bit integer value retrieved from the user entered command string.
  */
int32_t getNum(inputData* data)
{
    int32_t signedInteger32bits = 0;
    uint8_t index = 0;
    uint8_t count;
    uint8_t i;

    count = data->bufferIndex + 1;
    for(i = count; i < 20; i++)
    {
        // extracting the argument from the user entered command on putty
        data->argument[index] = data->prevCommand[i];
        index++;
    }


    for(i = 0; data->argument[i] != '\0'; i++)
        signedInteger32bits = (signedInteger32bits * 10) + (data->argument[i] - '0');

    return signedInteger32bits;
}

/**
  * @brief This function will run the CLI process.
  */
/*
void runShell()
{
    bool testdata;

    static inputData data;

    initSystemClockTo40Mhz();                // Set system clock to 40Mhz
    initUart0();                             // Initialize the UART0
    setUart0BaudRate(115200, 40e6);          // Setup UART0 baud rate

    // Enable clocks
    enablePort(PORTF);
    // Configure LED and pushbutton pins
    selectPinPushPullOutput(GREEN_LED_ONBOARD);
    selectPinPushPullOutput(RED_LED_ONBOARD);

    putsUart0("-----------------------------------------\n\r");
    putsUart0("--------------WELCOME HUMAN--------------\n\r");
    putsUart0("-----------------------------------------\n\r");
    putsUart0("\r\n>");


    while(1)
    {
        getString(&data);
        if (matchCommand(&data, "on", 0))
        {
            testdata = 1;
            setPinValue(RED_LED_ONBOARD, 1);
        }
        else if (matchCommand(&data, "off", 0))
        {
            testdata = 0;
            setPinValue(RED_LED_ONBOARD, 0);
        }
        else if (matchCommand(&data, "led", 1))
        {
            if (matchCommandArg(&data, "on"))
                setPinValue(GREEN_LED_ONBOARD, 1);
            else if (matchCommandArg(&data, "off"))
                setPinValue(GREEN_LED_ONBOARD, 0);
        }
        else if (matchCommand(&data, "kill", 1))
        {
            testNum = getNum(&data);
            kill(testNum);
        }
        else if (matchCommand(&data, "reboot", 0))
        {
            rebootMCU();
        }
        else if (matchCommand(&data, "ps", 0))
        {
            ps();
        }
        else if (matchCommand(&data, "ipcs", 0))
        {
            ipcs();
        }
        else if (matchCommand(&data, "kill", 1))
        {

            if (matchCommandArg(&data, "pid"))
            {
                kill(1);

            }
        }
        else if (matchCommand(&data, "pmap", 1))
        {
            if (matchCommandArg(&data, "pid"))
            {
                pmap(1);

            }
        }
        else if (matchCommand(&data, "preempt", 1))
        {
            if (matchCommandArg(&data, "on"))
            {
                preempt(1);

            }
            else if (matchCommandArg(&data, "off"))
            {
                preempt(0);

            }
        }
        else if (matchCommand(&data, "sched", 1))
        {
            if (matchCommandArg(&data, "prio"))
            {
                sched(1);
            }
            else if (matchCommandArg(&data, "rr"))
            {
                sched(0);

            }
        }
        else if (matchCommand(&data, "pidof", 1))
        {
            if (matchCommandArg(&data, "proc_name"))
            {
                pidof("test");
            }
        }
        else if (matchCommand(&data, "run", 1))
        {
            if (matchCommandArg(&data, "proc_name"))
            {
                setPinValue(RED_LED_ONBOARD, 1);
            }
        }

    }
}
*/
