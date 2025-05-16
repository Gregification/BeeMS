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
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <FreeRTOS.h>
#include <task.h>

#include "Core/system.hpp"
#include "Core/system_init.hpp"
#include "Tasks/blink_task.hpp"

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
        constexpr char logo[] =
            "  ____             __   __ ______  " NEWLINE
            " |  _ \\           |  \\ /  |\\  ___) " NEWLINE
            " | |_) ) ___  ___ |   v   | \\ \\    " NEWLINE
            " |  _ ( / __)/ __)| |\\_/| |  > >   " NEWLINE
            " | |_) )> _) > _) | |   | | / /__  " NEWLINE
            " |____/ \\___)\\___)|_|   |_|/_____) " NEWLINE;

        System::nputsUIUART(logo, sizeof(logo));
    }
    System::nputsUIUART(STRANDN(" " PROJECT_NAME "   " PROJECT_VERSION NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE));


    /* --- Initialize off-chip ------------------------------------ */
    System::nputsUIUART(STRANDN("Initializing off chip ..." NEWLINE));

    system_init_offchip();


    /* --- POST off-chip ------------------------------------------ */
//    System::nputsUIUART(STRANDN("POST-ing off chips ..." NEWLINE));
    // TODO


    /* --- Start -------------------------------------------------- */
    System::nputsUIUART(STRANDN("Starting FreeRTOS ..." NEWLINE));

    xTaskCreate(Task::Blink::main,
                "blink",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL);
//    xTaskCreate(pxTaskCode, pcName, uxStackDepth, pvParameters, uxPriority, pxCreatedTask)

    vTaskStartScheduler();


    for(;;)
        // go crazy
        System::FailHard("FreeRTOS scheduler crashed");
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
