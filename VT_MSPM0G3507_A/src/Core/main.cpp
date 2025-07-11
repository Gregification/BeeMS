/*
 * main.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *      https://cataas.com/cat/says/accumulating
 */

#include <stdio.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#include "system.hpp"
#include "Tasks/blink_task.hpp"
#include "Tasks/Test_UART_Task.hpp"

#include "Tasks/fiddle.hpp"

int main(){
    System::init();

    System::uart_ui.setBaudTarget(115200);

    System::uart_ui.nputs(ARRANDN("\033[2J\033[H"));
    System::uart_ui.nputs(ARRANDN(" " PROJECT_NAME "   " PROJECT_VERSION NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE));

    xTaskCreate(Task::blink_task,
            "blink status",
            configMINIMAL_STACK_SIZE,
            NULL,
            configMAX_PRIORITIES,
            NULL);

//    xTaskCreate(Task::fiddle_task,
//            "fiddle task",
//            configMINIMAL_STACK_SIZE * 50,
//            NULL,
//            tskIDLE_PRIORITY,
//            NULL);

    xTaskCreate(Task::UART_Task,
                "UART_Task",
                configMINIMAL_STACK_SIZE * 10,
                NULL,
                tskIDLE_PRIORITY,
                NULL);

    vTaskStartScheduler();

    while(true) {
        System::FailHard("reached end of main");
    }
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
        System::uart_ui.nputs(ARRANDN("vApplicationMallocFailedHook" NEWLINE));
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
    System::uart_ui.nputs(ARRANDN("vApplicationIdleHook" NEWLINE));
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
    taskDISABLE_INTERRUPTS();

    char str[MAX_STR_ERROR_LEN];
    snprintf(str,sizeof(str), "vApplicationStackOverflowHook: %s", pcTaskName);

    for (;;)
        System::uart_ui.nputs(ARRANDN(str));
        ;
}
#endif

