/*
 * main.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *      https://cataas.com/cat/says/accumulating
 */
/*
 * University of Texas at Arlington, FSAE Racing team , BMS code
 * - intended for E27 vehicle
 * - created as part of Senior Design (CSE 4316) starting Fall25
 */

/*
 * eventually someone needs to go though and rewrite this program using
 *      something cool like ETL for safety & good practice. most worried about
 *      dynamic allocations. end goal is to have absolutely no dynamic allocation,
 *      all static. it is possible and is done as such on industry projects.
 *
 * good luck getting the CCS RTOS debugger working. I couldn't.
 */

#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"
#include "Tasks/task_BMS.hpp"
#include "Tasks/task_ethModbus.hpp"
#include "Tasks/examples/example_blink_task.hpp"


int main(){
    System::init();

    System::UART::uart_ui.setBaudTarget(115200);
    System::UART::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET));
    System::UART::uart_ui.nputs(ARRANDN(CLIGOOD " " PROJECT_NAME "   " PROJECT_VERSION NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));
    System::UART::uart_ui.nputs(ARRANDN("\t Device: "));
    System::UART::uart_ui.putu32h(System::mcuID);
    System::UART::uart_ui.nputs(ARRANDN(NEWLINE));

    MstrB::init();

    MstrB::IL::control.clear();

    MstrB::Indi::LED::i1.set();
    MstrB::Indi::LED::i2.set();
    MstrB::Indi::LED::fault.set();
    MstrB::Indi::LED::scheduler.set();
    delay_cycles(2 * System::CLK::CPUCLK);
    MstrB::Indi::LED::i1.clear();
    MstrB::Indi::LED::i2.clear();
    MstrB::Indi::LED::fault.clear();
    MstrB::Indi::LED::scheduler.clear();

    xTaskCreate(Task::blink_task,
            "blink_task",
            configMINIMAL_STACK_SIZE,
            NULL,
            tskIDLE_PRIORITY, //configMAX_PRIORITIES,
            NULL);

//    xTaskCreate(Task::BMS_task,
//            "BMS_task",
//            configMINIMAL_STACK_SIZE*20,
//            NULL,
//            tskIDLE_PRIORITY, //configMAX_PRIORITIES,
//            NULL);

//    xTaskCreate(Task::BQ769x2_PROTOCOL_Test_V_Task,
//            "BQ769x2_PROTOCOL_Test_V_Task",
//            configMINIMAL_STACK_SIZE*207
//            NULL,
//            tskIDLE_PRIORITY, //configMAX_PRIORITIES,
//            NULL);

    xTaskCreate(Task::ethModbus_task,
            "non_BMS_functions_task",
            configMINIMAL_STACK_SIZE * 2,
            NULL,
            tskIDLE_PRIORITY, //configMAX_PRIORITIES,
            NULL);

    vTaskStartScheduler();

    taskDISABLE_INTERRUPTS();
    while(true) {
        System::FailHard("reached end of main" NEWLINE);
        delay_cycles(20e6);
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
    for (;;) {
        System::UART::uart_ui.nputs(ARRANDN("vApplicationMallocFailedHook" NEWLINE));
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
    System::UART::uart_ui.nputs(ARRANDN("vApplicationIdleHook" NEWLINE));
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

    for (;;){
        System::UART::uart_ui.nputs(ARRANDN("vApplicationStackOverflowHook: "));
        System::UART::uart_ui.nputs(ARRANDN(pcTaskName));
        System::UART::uart_ui.nputs(ARRANDN(CLIRESET CLIERROR NEWLINE));
        delay_cycles(20e6);
    }
}
#endif

