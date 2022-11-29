// Author - Sanket S Hiremath
// UTA ID- 1001830284

#ifndef SHELL_H_
#define SHELL_H_

#define maxChar 20
#define GREEN_LED_ONBOARD PORTF,3
#define RED_LED_ONBOARD PORTF,1

typedef struct input_data
{
    char inputBuffer[maxChar];
    char prevCommand[maxChar];
    char argument[maxChar];
    uint8_t bufferIndex;

}inputData;



void getString(inputData* data);
bool matchCommand(inputData* data, char userCommand[], uint8_t mode);
bool matchCommandArg(inputData* data, char userArgument[]);
int32_t getNum(inputData* data);
void runShell();
void rebootMCU();
void ps();
//void ipcs(semaphoreInfo* DATA);
void kill(uint32_t pid);
void pmap(uint32_t pid);
void preempt(bool on);
void sched(bool prio_on);
void pidof(const char name[]);
void convertNumToString(uint32_t num);
void convertDec_Hex(uint32_t decNum);
void copyString(const char* fromStr, char* toStr);
void printfString(uint8_t spaceToReserve, char* s);


#endif
