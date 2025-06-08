/*
 * main.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *      https://cataas.com/cat/says/accumulating
 */

#include <FreeRTOS.h>
#include <task.h>

#include "system.hpp"

#include "fiddle.hpp"

int main(){
    System::init();

    System::UART::setBaudTarget(UARTUI, 115200);
    DL_UART_enable(UARTUI);

    xTaskCreate(fiddle_task,
            "fiddle task",
            configMINIMAL_STACK_SIZE * 10,
            NULL,
            tskIDLE_PRIORITY,
            NULL);

    vTaskStartScheduler();

    while(true)
    {}
}


// HOOKS
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /*
     * vApplicationMallocFailedHook() will only be called if
     * configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a
     * hook function that will get called if a call to pvPortMalloc() fails.
     * pvPortMalloc() is called internally by the kernel whenever a task,
     * queue, timer or semaphore is created. It is also called by various
     * parts of the demo application. If heap_1.c or heap_2.c are used,
     * then the size of the heap available to pvPortMalloc() is defined by
     * configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the
     * xPortGetFreeHeapSize() API function can be used to query the size of
     * free heap space that remains (although it does not provide information
     * on how the remaining heap might be fragmented).
     */
    taskDISABLE_INTERRUPTS();
    for (;;)
        ;
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    /*
     * vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
     * to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the
     * idle task. It is essential that code added to this hook function never
     * attempts to block in any way (for example, call xQueueReceive() with a
     * block time specified, or call vTaskDelay()). If the application makes
     * use of the vTaskDelete() API function (as this demo application does)
     * then it is also important that vApplicationIdleHook() is permitted to
     * return to its calling function, because it is the responsibility of the
     * idle task to clean up memory allocated by the kernel to any task that
     * has since been deleted.
     */
}

/*-----------------------------------------------------------*/

#if (configCHECK_FOR_STACK_OVERFLOW)
/*
     *  ======== vApplicationStackOverflowHook ========
     *  When stack overflow checking is enabled the application must provide a
     *  stack overflow hook function. This default hook function is declared as
     *  weak, and will be used by default, unless the application specifically
     *  provides its own hook function.
     */
#if defined(__IAR_SYSTEMS_ICC__)
__weak void vApplicationStackOverflowHook(
    TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__TI_COMPILER_VERSION__))
#pragma WEAK(vApplicationStackOverflowHook)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__GNUC__) || defined(__ti_version__))
void __attribute__((weak))
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#endif
{
    (void) pcTaskName;
    (void) pxTask;

    /*
     * Run time stack overflow checking is performed if
     * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook
     * function is called if a stack overflow is detected.
     */
    taskDISABLE_INTERRUPTS();
    for (;;)
        ;
}
#endif

