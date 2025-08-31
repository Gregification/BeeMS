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

void thing( void * ){
    auto &bled = System::GPIO::PA27;
    DL_GPIO_initDigitalOutputFeatures(
            bled.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(bled));
    DL_GPIO_enableOutput(GPIOPINPUX(bled));

    for(;;){
        DL_GPIO_togglePins(GPIOPINPUX(bled));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}


int main(){
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN("\033[2J\033[H"));
    System::uart_ui.nputs(ARRANDN(" " PROJECT_NAME "   " PROJECT_VERSION NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE));


    xTaskCreate(thing,
                "blink",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY, //configMAX_PRIORITIES,
                NULL);
    xTaskCreate(Task::blink_task,
               "asd",
               configMINIMAL_STACK_SIZE,
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
        System::uart_ui.nputs(ARRANDN("vApplicationMallocFailedHook" NEWLINE));
        delay_cycles(20e6);
    }
}

/*-----------------------------------------------------------*/

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

