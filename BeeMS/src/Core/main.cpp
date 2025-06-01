/*
 * main.cpp
 *
 *  Created on: May 9, 2025
 *      Author: turtl
 *
 * do not touch the CSS project settings, this was a abomination to setup
 */

/*
 * project files
 *
 * ProjectRoot/
 * |
 * |-- FreeRTOS/
 * |   |-- Config/
 * |   |   |-- FreeRTOSConfig.h
 * |   |
 * |   |-- Kernel/
 * |   |   |-- source/              : freeRTOS core source files
 * |   |
 * |   |-- TCP/
 * |       |-- source/              : freeRTOS+TCP source files
 * |
 * |-- src/
 *     |-- Core/                    : Application entry point (main.c, system init)
 *     |-- Drivers/                 : custom peripheral drivers. (HAL drivers come from DriverLib so arn't included here)
 *     |-- Middleware/              : stacks like FatFS, etc.
 *     |-- Tasks/                   : application tasks (e.g., sensor, comms)
 *
 */

#include <stdint.h>

#include <inc/hw_sysctl.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_ints.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/emac.h>
#include <FreeRTOS.h>
#include <NetworkInterface.h>
#include <FreeRTOS_TCP_WIN.h>
#include <task.h>

#include "Core/system.hpp"
#include "Core/system_init.hpp"
#include "Tasks/blink_task.hpp"

void fiddleTask(void * args){
    for(;;){
        FreeRTOS_SendPingRequest(IPV4_TO_INT(192,123,123,123), 0, pdMS_TO_TICKS(100));
        System::nputsUIUART(STRANDN("fiddle task" NEWLINE));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void firstTask(void * args);

Task::Blink::Args blink_indicator_args = {
        .pin = System::GPIO::GPIO_REG {
                .GPIO_PORTn_BASE    = GPIO_PORTN_BASE,
                .GPIO_PIN_n         = GPIO_PIN_0,
            },
        .period_ms = Task::Blink::PERIOD_NORMAL,
    };

void customprintf(const char * format, ...) {
    char buffer[MAX_ERROR_MSG_LEN]; // Adjust the size as needed

    va_list args;
    va_start(args, format);

    vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    System::SYSTEM_UART_PRIM_UI.nputs_for_freertos(buffer, sizeof(buffer));
}

int main(){

    /* --- Initialize on-chip ------------------------------------- */

    system_init_onchip();


    /* --- POST on-chip ------------------------------------------- */
    // fail here suggest error with chip, maybe you toasted it; go replace it
    // TODO make actual tests, e.g: loop back on the UARTS, confirm tx rx. that sort of thing
    // patented trust me bro testing technology

    ASSERT_FATAL(System::CPU_FREQ == configCPU_CLOCK_HZ, "failed to set MOSC");


    /* --- display software information --------------------------- */
    {
        System::nputsUIUART(STRANDN("\033[2J\033[H"));

//        constexpr char logo[] =
//            "  ____             __   __ ______  " NEWLINE
//            " |  _ \\           |  \\ /  |\\  ___) " NEWLINE
//            " | |_) ) ___  ___ |   v   | \\ \\    " NEWLINE
//            " |  _ ( / __)/ __)| |\\_/| |  > >   " NEWLINE
//            " | |_) )> _) > _) | |   | | / /__  " NEWLINE
//            " |____/ \\___)\\___)|_|   |_|/_____) " NEWLINE;
//        System::nputsUIUART(logo, sizeof(logo));
    }
    System::nputsUIUART(STRANDN(" " PROJECT_NAME "   " PROJECT_VERSION NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE));


    /* --- Initialize off-chip ------------------------------------ */
    System::nputsUIUART(STRANDN("--- Initializing off chip" NEWLINE));




    /* --- POST off-chip ------------------------------------------ */
//    System::nputsUIUART(STRANDN("POST-ing off chips ..." NEWLINE));
    // TODO


    /* --- Start -------------------------------------------------- */
    System::nputsUIUART(STRANDN("--- Starting tasks " NEWLINE));


    // indicator led
    xTaskCreate(Task::Blink::task,
        "blink indicator 1",
        configMINIMAL_STACK_SIZE,
        (void *)&blink_indicator_args,
        tskIDLE_PRIORITY,
        NULL);

    xTaskCreate(firstTask,
        "initializing task",
        configMINIMAL_STACK_SIZE * 2,
        NULL,
        tskIDLE_PRIORITY,
        NULL);


    vTaskStartScheduler();

    for(;;)
        // go crazy
        System::FailHard("FreeRTOS scheduler crashed");
}

/*-----------------------------------------------------------*/

void firstTask(void * args) {
    // ethernet test
    {
        System::nputsUIUART(STRANDN("ethernet test" NEWLINE));

        GPIOPinConfigure(GPIO_PF0_EN0LED0);
        GPIOPinConfigure(GPIO_PF4_EN0LED1);
        GPIOPinConfigure(GPIO_PF1_EN0LED2);
        GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_1);

        GPIOPinConfigure(GPIO_PG0_EN0PPS);
        GPIOPinTypeEthernetMII(GPIO_PORTB_BASE, GPIO_PIN_0);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);

        if(FreeRTOS_IPInit_Multi() == pdFAIL) {
            blink_indicator_args.period_ms = Task::Blink::PERIOD_FAULT;
            System::FailHard("failed FreeRTOS IP init" NEWLINE);
        }

        if(xNetworkInterfaceInitialise() == pdFAIL){
            blink_indicator_args.period_ms = Task::Blink::PERIOD_FAULT;
            System::FailHard("failed to init network interface");
        }
    }

    vTaskDelete(NULL);
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    IntMasterDisable();
    for( ;; );
}


/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}


/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for( ;; );
}


/*-----------------------------------------------------------*/

void *malloc( size_t xSize )
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for( ;; );
}


/*-----------------------------------------------------------*/

const char * pcApplicationHostnameHook( void ) {
    return PROJECT_NAME " " PROJECT_VERSION;
}

/*-----------------------------------------------------------*/

/*
* Callback that provides the inputs necessary to generate a randomized TCP
* Initial Sequence Number per RFC 6528.  In this case just a psuedo random
* number is used so THIS IS NOT RECOMMENDED FOR PRODUCTION SYSTEMS.
*/
extern uint32_t ulApplicationGetNextSequenceNumber(
    uint32_t ulSourceAddress,
    uint16_t usSourcePort,
    uint32_t ulDestinationAddress,
    uint16_t usDestinationPort )
{
     ( void ) ulSourceAddress;
     ( void ) usSourcePort;
     ( void ) ulDestinationAddress;
     ( void ) usDestinationPort;

     static uint32_t randomnumber; // very random number
     return randomnumber++;
}

/*-----------------------------------------------------------*/

BaseType_t xApplicationGetRandomNumber( uint32_t * pulNumber ){
    return *pulNumber ^ 0xBEE;
}

/*-----------------------------------------------------------*/

/* called when the network connects or disconnects
 * https://freertos.org/Documentation/03-Libraries/02-FreeRTOS-plus/02-FreeRTOS-plus-TCP/09-API-reference/58-vApplicationIPNetworkEventHook_Multi
 */
void vApplicationIPNetworkEventHook_Multi(
                                           eIPCallbackEvent_t eNetworkEvent,
                                           struct xNetworkEndPoint * pxEndPoint
                                         ){
    // explode
    System::nputsUIUART(STRANDN("network connect/disconnect vApplicationIPNetworkEventHook_Multi" NEWLINE));
}

/*-----------------------------------------------------------*/

/* decides how to resolve DHCP communication decision points
 * https://freertos.org/Documentation/03-Libraries/02-FreeRTOS-plus/02-FreeRTOS-plus-TCP/09-API-reference/64-xApplicationDHCPHook_Multi
 */
eDHCPCallbackAnswer_t xApplicationDHCPHook_Multi( eDHCPCallbackPhase_t eDHCPPhase,
                                                  struct xNetworkEndPoint * pxEndPoint,
                                                  IP_Address_t * pxIPAddress
                                                ){
    // vaporise
    return eDHCPContinue;
}

/*-----------------------------------------------------------*/

/* stop reading this, what do you think it does. use ur brain. we didnt leave agartha for you to be retarded. (((we))) left so we can be different colors
 * https://www.freertos.org/Documentation/03-Libraries/02-FreeRTOS-plus/02-FreeRTOS-plus-TCP/09-API-reference/59-vApplicationPingReplyHook
 */
void vApplicationPingReplyHook( ePingReplyStatus_t eStatus, uint16_t usIdentifier ){
    System::nputsUIUART(STRANDN("pong " NEWLINE));
}

/*-----------------------------------------------------------*/


