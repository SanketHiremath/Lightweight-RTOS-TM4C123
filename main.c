// RTOS Framework - Fall 2022
// J Losh

// Student Name:Sanket Shashikant Hiremath
// TO DO: Add your name(s) on this line.
//        Do not include your ID number(s) in the file.

// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PE0 (lengthy and general)
// Orange: PA2 (idle)
// Yellow: PA3 (oneshot and general)
// Green:  PA4 (flash4hz)
// PBs on these pins
// PB0:    PD6 (set red, toggle yellow)
// PB1:    PD7 (clear red, post flash_request semaphore)
// PB2:    PC4 (restart flash4hz)
// PB3:    PC5 (stop flash4hz, uncoop)
// PB4:    PC6 (lengthy priority increase)
// PB5:    PC7 (errant)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Memory Protection Unit (MPU):
//   Region to allow peripheral access (RW) or a general all memory access (RW)
//   Region to allow flash access (XRW)
//   Regions to allow 32 1KiB SRAM access (RW or none)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "wait.h"
// TODO: Add header files here for your strings functions, ...
#include "shell.h"

#define BLUE_LED   PORTF,2 // on-board blue LED
#define RED_LED    PORTE,0 // off-board red LED
#define ORANGE_LED PORTA,2 // off-board orange LED
#define YELLOW_LED PORTA,3 // off-board yellow LED
#define GREEN_LED  PORTA,4 // off-board green LED

#define PUSH_BUTTON1 PORTD,6
#define PUSH_BUTTON2 PORTD,7
#define PUSH_BUTTON3 PORTC,4
#define PUSH_BUTTON4 PORTC,5
#define PUSH_BUTTON5 PORTC,6
#define PUSH_BUTTON6 PORTC,7

// Region numbers
#define REGION0         0x00
#define REGION1         0x01
#define REGION2         0x02
#define REGION3         0x03
#define REGION4         0x04
#define REGION5         0x05
#define REGION6         0x06

// SRAM regions
#define SRAM_REGION_BASE0       0x20000000
#define SRAM_REGION_BASE1       0x20002000
#define SRAM_REGION_BASE2       0x20004000
#define SRAM_REGION_BASE3       0x20006000

// SRAM sub-regions mask
#define DISABLE_ALL_SUBREGIONS  0xFF00

// Region size masks
#define REGION_SIZE(x)      (x<<1)
#define SIZE_8KB            0x0C
#define SIZE_4GB            0x1F

// Access control masks
#define AP_NO_ACCESS           (0 << 24)
#define AP_RW_PRIVILEGED_ONLY  (1 << 24)
#define AP_RW_RO_UNPRIVILEGED  (2 << 24)
#define AP_FULL_ACCESS         (3 << 24)
#define AP_RO_PRIVILEGED       (5 << 24)

extern void enablePSPMode();
extern void  setPSP(uint32_t address);
extern uint32_t* getPsp();
extern uint32_t* getMsp();
extern void disablePrivileged();
extern void pushStack();
extern void popStack();
extern void pushPsp(uint32_t r0);

typedef enum svcNum
{ YIELD = 10,
  SLEEP = 11,
  WAIT  = 12,
  POST  = 13
}svcNumber;

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];
#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
static uint32_t task_point;


// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 7=lowest
    uint32_t ticks;                // ticks until sleep complete
    uint32_t srd;                  // MPU subregion disable bits (one per 1 KiB)
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];


//-----------------------------------------------------------------------------
// Memory Manager and MPU Funcitons
//-----------------------------------------------------------------------------

// TODO: add your malloc code here and update the SRD bits for the current thread
void * mallocFromHeap(uint32_t size_in_bytes)
{
//    static void *heap = 0x20001800;
    uint32_t num;
    static uint32_t prevAddr =  0x20001800;;

//    prevAddr =(uint32_t) heap;
    num = (((size_in_bytes - 1) / 1024) + 1);
//    heap += num;
    prevAddr = (prevAddr + (num * 1024));

    return (void*)prevAddr;
}

// REQUIRED: add your MPU functions here

void sramSubregionToggle(uint8_t regionNum, uint8_t mask)
{

    NVIC_MPU_NUMBER_R = regionNum;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_ENABLE | REGION_SIZE(SIZE_8KB) | NVIC_MPU_ATTR_CACHEABLE | NVIC_MPU_ATTR_SHAREABLE | AP_RW_PRIVILEGED_ONLY | ((uint32_t)mask << 8);
}

void setSRAMBits(uint32_t bit_mask)
{
    uint8_t mask0, mask1, mask2, mask3;

    mask0 = (bit_mask &  0x000000FF);          // Mask for the sram block 0
    mask1 = (bit_mask &  0x0000FF00) >> 8;     // Mask for the sram block 1
    mask2 = (bit_mask &  0x00FF0000) >> 16;    // Mask for the sram block 2
    mask3 = (bit_mask &  0xFF000000) >> 24;    // Mask for the sram block 3

    // writing the mask for the different sub-regions to the MPU ATRR register
    sramSubregionToggle(REGION3, mask0);            // Writing the SRD bits mask to the region 3 of SRAM.
    sramSubregionToggle(REGION4, mask1);            // Writing the SRD bits mask to the region 4 of SRAM.
    sramSubregionToggle(REGION5, mask2);            // Writing the SRD bits mask to the region 5 of SRAM.
    sramSubregionToggle(REGION6, mask3);            // Writing the SRD bits mask to the region 6 of SRAM.


}

void enableExceptionHandler(void)
{
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGE;
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_BUS;
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEM;
}

// Turns ON the MPU
void enableMPU(void)
{
    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE;
//    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_HFNMIENA;
}

void setBackgroundRule(void)
{

    NVIC_MPU_BASE_R = REGION0 | NVIC_MPU_BASE_VALID | 0x00000000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_ENABLE | REGION_SIZE(SIZE_4GB) | NVIC_MPU_ATTR_BUFFRABLE | NVIC_MPU_ATTR_CACHEABLE | NVIC_MPU_ATTR_SHAREABLE | AP_FULL_ACCESS;
}

void allowFlashAccess(void)
{
    NVIC_MPU_BASE_R = REGION1 | NVIC_MPU_BASE_VALID | 0x00000000;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_ENABLE | REGION_SIZE(0x11) | NVIC_MPU_ATTR_CACHEABLE | AP_FULL_ACCESS;
}

//Not used as peripherals are covered by the background rule.
void allowPeripheralAccess(void)
{
    NVIC_MPU_BASE_R = REGION2 | NVIC_MPU_BASE_VALID | 0x4000000;

}

void setupSramAccess(void)
{
    // SRAM 8KiB block 0
    NVIC_MPU_BASE_R = REGION3 | NVIC_MPU_BASE_VALID | SRAM_REGION_BASE0;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_ENABLE | REGION_SIZE(SIZE_8KB) | NVIC_MPU_ATTR_CACHEABLE | NVIC_MPU_ATTR_SHAREABLE | AP_RW_PRIVILEGED_ONLY;

    // SRAM 8KiB block 1
    NVIC_MPU_BASE_R = REGION4 | NVIC_MPU_BASE_VALID | SRAM_REGION_BASE1;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_ENABLE | REGION_SIZE(SIZE_8KB) | NVIC_MPU_ATTR_CACHEABLE | NVIC_MPU_ATTR_SHAREABLE | AP_RW_PRIVILEGED_ONLY;

    // SRAM 8KiB block 2
    NVIC_MPU_BASE_R = REGION5 | NVIC_MPU_BASE_VALID | SRAM_REGION_BASE2;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_ENABLE | REGION_SIZE(SIZE_8KB) | NVIC_MPU_ATTR_CACHEABLE | NVIC_MPU_ATTR_SHAREABLE | AP_RW_PRIVILEGED_ONLY;

    // SRAM 8KiB block 3
    NVIC_MPU_BASE_R = REGION6 | NVIC_MPU_BASE_VALID | SRAM_REGION_BASE3;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_ENABLE | REGION_SIZE(SIZE_8KB) | NVIC_MPU_ATTR_CACHEABLE | NVIC_MPU_ATTR_SHAREABLE | AP_RW_PRIVILEGED_ONLY;

}

uint8_t getSvcNumber()
{
    uint32_t* psp = getPsp();
    uint8_t N = *(uint16_t*)(*(psp + 6) - 2) & 0xFF;

    return N;
}

// REQUIRED: initialize MPU here
void initMpu(void)
{
    // REQUIRED: call your MPU functions here
    enableExceptionHandler();                               // Enable all exception handling
    setBackgroundRule();                                    // Setting background rule for entire memory map.
    allowFlashAccess();                                     // Setting background rule for 256KiB flash.
    setupSramAccess();                                      // Dividing SRAM into 4 regions & setting access rules.
    enableMPU();


}

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    uint8_t num;
    uint32_t subregionNum, bitMask;
    uint32_t* heap = (uint32_t*)0x20001400;
    uint32_t addr;
    uint32_t* tempAddr;
    static uint32_t prevSrdBits = 0;
//    uint32_t* brk = (uint32_t*)0x20007FFF;

    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED:
    // store the thread name
    copyString(name, tcb[i].name);
    // allocate stack space and store top of stack in sp and spInit

    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            num = (((stackBytes - 1) / 1024) + 1);

            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;


             if (i == 0)
             {
                addr = (uint32_t)heap;
             }
             else
             {
                addr = (uint32_t)tcb[i-1].spInit;
                tempAddr = mallocFromHeap(1023);
                putsUart0("malloc= ");
                convertDec_Hex((uint32_t) tempAddr);
                putsUart0("\n\r");
             }

            // Calculating the SRD bits
            subregionNum = (addr - (0x20000000)) / 1024;  // Calculate no. of times to shift
            stackBytes = stackBytes / 1024;               // Calculate no. of bits to flip
            bitMask = (1 << stackBytes) - 1;              // Calculate the number of SRD bits to be high
            bitMask = (bitMask << subregionNum);          // Moving the bits to the correct subregion.
            prevSrdBits = prevSrdBits | bitMask;          // New bitmask is ORed with the previous bitmask.

            tcb[i].sp = (void*)(addr + (num * 1024));
            tcb[i].spInit = (void*)(addr + (num * 1024));

            putsUart0("spinit= ");
            convertDec_Hex((uint32_t) tcb[i].spInit);
            putsUart0("\n\r");

            tcb[i].priority = priority;
            tcb[i].srd = (prevSrdBits);

//            convertDec_Hex((uint32_t) tcb[i].srd);
//            putsUart0("\n\r");
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
}

// REQUIRED: modify this function to stop a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void stopThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, TMPL bit, and PC
void startRtos()
{
    // Calling the RTOS scheduler to get the current task running.
    taskCurrent = (uint8_t)rtosScheduler();

    // Setting the stack pointer to the stack space of the process
    // and setting the ASP bit.
    setPSP((uint32_t)tcb[taskCurrent].sp);
    enablePSPMode();

    // Setting the process id of the current task to the function pointer.
    _fn fn = (_fn)tcb[taskCurrent].pid;
    tcb[taskCurrent].state = STATE_READY;

//    setSRAMBits(tcb[taskCurrent].srd);
    setSRAMBits(0xFFFFFFFF);
    convertDec_Hex((uint32_t) tcb[taskCurrent].srd);
    putsUart0("\n\r");

    // Turning on the TMPL bit and calling the function
    disablePrivileged();
    fn();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
//    putcUart0('y');
    __asm(" SVC  #10");

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm(" SVC  #11");
}

// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t semaphore)
{
    __asm(" SVC  #12");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm(" SVC  #13");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;

    for(i = 0; i <= taskCount; i++)
    {
        if (tcb[i].state == STATE_DELAYED)
        {
            if (tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
            tcb[i].ticks--;
        }
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
/*
    putcUart0('p');

    pushStack();
    tcb[taskCurrent].sp = (void*)getPsp();
    // calling the scheduler
    taskCurrent = (uint8_t)rtosScheduler();
    tcb[taskCurrent].state = STATE_READY;
    setSRAMBits(tcb[taskCurrent].srd);
    setPSP((uint32_t)tcb[taskCurrent].sp);
    popStack();
*/

    pushStack();
    tcb[taskCurrent].sp = (void*)getPsp();
    taskCurrent = (uint8_t)rtosScheduler();

//    setSRAMBits(tcb[taskCurrent].srd);
    setSRAMBits(0xFFFFFFFF);  // currently giving access to the entire SRAM

    if(tcb[taskCurrent].state == STATE_READY)
    {
        setPSP((uint32_t)tcb[taskCurrent].sp);  // Put into PSP the task current to switch task
        popStack();                             // Pop R4 - R11
    }

    else
    {
        tcb[taskCurrent].state = STATE_READY;
        setPSP((uint32_t)tcb[taskCurrent].spInit);  // Point the sp to the sp of the task
        pushPsp(0x61000000);                        // xPSR
        pushPsp((uint32_t)tcb[taskCurrent].pid);    // PC
        pushPsp(0xFFFFFFFD);                        // LR
        pushPsp(0x00);                              // R12
        pushPsp(0x00);                              // R3
        pushPsp(0x00);                              // R2
        pushPsp(0x00);                              // R1
        pushPsp(0x00);                              // R0
    }


}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint8_t num;
    uint32_t* psp;
    uint8_t currentTaskInQueue;
    uint8_t i = 0;

    psp = getPsp();
//    num = getSvcNumber();
    num = *(uint16_t*)(*(psp + 6) - 2) & 0xFF;

    switch(num)
    {
    case YIELD:
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;       // Call the pendsv ISR
        break;

    case SLEEP:
        tcb[taskCurrent].ticks = *psp;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;       // Call the pendsv ISR
        break;

    case WAIT:
        if(semaphores[*psp].count > 0)
        {
            semaphores[*psp].count--;
        }
        else
        {
            if(semaphores[*psp].queueSize < MAX_QUEUE_SIZE)
            {
                semaphores[*psp].processQueue[semaphores[*psp].queueSize++] = taskCurrent;
                tcb[taskCurrent].semaphore = (void*)(semaphores + *psp);
                tcb[taskCurrent].state = STATE_BLOCKED;
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            }
        }
        break;

    case POST:
        // Increment the count for the selected semaphore
        semaphores[*psp].count++;

        if(semaphores[*psp].count >= 1 && semaphores[*psp].queueSize > 0)
        {
            currentTaskInQueue = semaphores[*psp].processQueue[0];
            semaphores[*psp].count--;
            tcb[currentTaskInQueue].state = STATE_READY;
            for(i = 0; i < semaphores[*psp].queueSize - 1; i++)
            {
                semaphores[*psp].processQueue[i] = semaphores[*psp].processQueue[i + 1];
            }
            semaphores[*psp].queueSize--;
        }
        break;
    }
}

// REQUIRED: code this function
void mpuFaultIsr()
{
    uint32_t* address1;
    uint32_t* address2;

    putsUart0("MPU fault occured\n\r");


    // Print MSP and PSP
    putsUart0("MSP = 0x");
    address1 = getMsp();
    convertDec_Hex((uint32_t)address1);
    putsUart0("\r\n");

    putsUart0("PSP = 0x");
    address2 = getPsp();
    convertDec_Hex((uint32_t)address2);
    putsUart0("\r\n");

    putsUart0("mFault Flags = 0x");
    convertDec_Hex(NVIC_FAULT_STAT_R & 0xFF);
    putsUart0("\r\n");


    // Printing the reg r0-r3, r12...
     uint32_t *stackdump;
     stackdump = getPsp();

    putsUart0("R0 = 0x");
    convertDec_Hex(*(stackdump));
    putsUart0("\r\n");

    putsUart0("R1 = 0x");
    convertDec_Hex(*(stackdump + 1));
    putsUart0("\r\n");

    putsUart0("R2 = 0x");
    convertDec_Hex(*(stackdump + 2));
    putsUart0("\r\n");

    putsUart0("R3 = 0x");
    convertDec_Hex(*(stackdump + 3));
    putsUart0("\r\n");

    putsUart0("R12 = 0x");
    convertDec_Hex(*(stackdump + 4));
    putsUart0("\r\n");

    putsUart0("LR = 0x");
    convertDec_Hex(*(stackdump + 5));
    putsUart0("\r\n");

    putsUart0("PC = 0x");
    convertDec_Hex(*(stackdump + 6));
    putsUart0("\r\n");

    putsUart0("xPSR = 0x");
    convertDec_Hex(*(stackdump + 7));
    putsUart0("\r\n");
    putsUart0("-------------------------------------------------------\n\r");

    // Clearing the MPU fault
    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_MEMP;

    // Triggering a PendSV ISR
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: code this function
void hardFaultIsr()
{
    uint32_t* address1;
    uint32_t* address2;
    uint32_t test2;

    putsUart0("hard fault occured\n\r");

    // Print MSP and PSP
    putsUart0("MSP = 0x");
    address1 = getMsp();
    convertDec_Hex((uint32_t)address1);
    putsUart0("\r\n");

    putsUart0("PSP = 0x");
    address2 = getPsp();
    convertDec_Hex((uint32_t)address2);
    putsUart0("\r\n");

    putsUart0("Hard Fault Flags = 0x");
    test2 = (NVIC_HFAULT_STAT_R & 0xFF);
    convertDec_Hex(test2);

    putsUart0("\r\n");
    putsUart0("-------------------------------------------------------\n\r");
}

// REQUIRED: code this function
void busFaultIsr()
{
    putsUart0("bus fault occured\n\r");
    putsUart0("-------------------------------------------------------\n\r");
}

// REQUIRED: code this function
void usageFaultIsr()
{
    putsUart0("usage fault occured\n\r");
    putsUart0("-------------------------------------------------------\n\r");
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    enablePort(PORTA);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);

    // Configure LED pins
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(ORANGE_LED);
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(YELLOW_LED);

    // Configure push button pins
    selectPinDigitalInput(PUSH_BUTTON1);
    enablePinPullup(PUSH_BUTTON1);
    selectPinDigitalInput(PUSH_BUTTON2);
    enablePinPullup(PUSH_BUTTON2);
    selectPinDigitalInput(PUSH_BUTTON3);
    enablePinPullup(PUSH_BUTTON3);
    selectPinDigitalInput(PUSH_BUTTON4);
    enablePinPullup(PUSH_BUTTON4);
    selectPinDigitalInput(PUSH_BUTTON5);
    enablePinPullup(PUSH_BUTTON5);
    selectPinDigitalInput(PUSH_BUTTON6);
    enablePinPullup(PUSH_BUTTON6);

    //Initialise the systick timer
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;
    NVIC_ST_RELOAD_R = (0x9C3F) & (0x00FFFFFF);


}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t value;

    if(getPinValue(PUSH_BUTTON1)==0)
    {
        value = 1;
    }

    if(getPinValue(PUSH_BUTTON2)==0)
    {
        value = 2;
    }

    if(getPinValue(PUSH_BUTTON3)==0)
    {
        value = 4;
    }

    if(getPinValue(PUSH_BUTTON4)==0)
    {
        value = 8;
    }

    if(getPinValue(PUSH_BUTTON5)==0)
    {
        value = 16;
    }

    if(getPinValue(PUSH_BUTTON6)==0)
    {
        value = 32;
    }

    return value;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
void idle2()
{
    while(true)
    {
        setPinValue(RED_LED, 1);
        waitMicrosecond(1000);
        setPinValue(RED_LED, 0);
        yield();
    }
}


// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        setPinValue(ORANGE_LED, 1);
        waitMicrosecond(1000);
        setPinValue(ORANGE_LED, 0);
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        setPinValue(GREEN_LED, !getPinValue(GREEN_LED));
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        setPinValue(YELLOW_LED, 1);
        sleep(1000);
        setPinValue(YELLOW_LED, 0);
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    uint8_t *p;

    // Example of allocating memory from stack
    // This will show up in the pmap command for this thread
    p = mallocFromHeap(1024);
    *p = 0;

    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        setPinValue(RED_LED, !getPinValue(RED_LED));
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            setPinValue(YELLOW_LED, !getPinValue(YELLOW_LED));
            setPinValue(RED_LED, 1);
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            setPinValue(RED_LED, 0);
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            stopThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        setPinValue(BLUE_LED, 1);
        sleep(1000);
        setPinValue(BLUE_LED, 0);
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    while (true)
    {
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();

    initUart0();
    initMpu();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(250000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);
//    ok &=  createThread(idle2, "Idle2", 7, 1024);

    // Add other processes
//    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
    ok &= createThread(oneshot, "OneShot", 2, 1024);
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
    ok &= createThread(debounce, "Debounce", 6, 1024);
//    ok &= createThread(important, "Important", 0, 1024);
//    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
//    ok &= createThread(errant, "Errant", 6, 1024);
//    ok &= createThread(shell, "Shell", 6, 2048);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        setPinValue(RED_LED, 1);

    return 0;
}
