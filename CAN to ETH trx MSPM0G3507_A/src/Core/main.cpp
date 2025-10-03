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


#include <Tasks/BQ769x2_PROTOCOL_Test_V.hpp>
//#include <Tasks/BQ769x2_PROTOCOL_Test_T.hpp>
//#include <Tasks/SPI_example_task.hpp>

#include "system.hpp"

#include "Tasks/blink_task.hpp"
#include "Tasks/Test_UART_Task.hpp"
#include "Tasks/fiddle.hpp"

void task_UI_and_watchdog(void*); // also has blink
void task_CAN_ETH_TRX(void*);

int main(){
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET));
    System::uart_ui.nputs(ARRANDN(CLIGOOD " " PROJECT_NAME "   " PROJECT_VERSION NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    TaskHandle_t taskHandleToWatch;

    xTaskCreate(task_CAN_ETH_TRX,
            "task_CAN_ETH_TRX",
            configMINIMAL_STACK_SIZE * 10, // eye-balled
            NULL,
            tskIDLE_PRIORITY,
            &taskHandleToWatch);

    xTaskCreate(task_UI_and_watchdog,
            "task_UI_and_watchdog",
            configMINIMAL_STACK_SIZE,
            &taskHandleToWatch,
            tskIDLE_PRIORITY, //configMAX_PRIORITIES,
            NULL);

    vTaskStartScheduler();

    while(true) {
        System::FailHard("reached end of main" NEWLINE);
        delay_cycles(20e6);
    }
}

void task_UI_and_watchdog(void * arg) {
    TaskHandle_t * taskHandleToWatch = (TaskHandle_t *)arg;

    while(1){
        if(taskHandleToWatch == NULL)
            System::FailHard("task_UI_and_watchdog determined taskHandleToWatch has failed. faulting system" NEWLINE);
    }

    vTaskDelete(NULL);
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
    for (;;) {
        System::uart_ui.nputs(ARRANDN("vApplicationMallocFailedHook" NEWLINE));
        delay_cycles(20e6);
    }
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

    for (;;){
        System::uart_ui.nputs(ARRANDN(str));
        System::uart_ui.nputs(ARRANDN(NEWLINE));
        delay_cycles(20e6);
    }
}
#endif

