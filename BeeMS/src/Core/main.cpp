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
#include <FreeRTOS_IP.h>
#include <NetworkBufferManagement.h>
#include <NetworkInterface.h>
#include <FreeRTOS_Sockets.h>
#include <task.h>

#include "Core/system.hpp"
#include "Core/system_init.hpp"
#include "Core/hooks.h"
#include "Tasks/blink_task.hpp"

void fiddleTask(void *){

    for(;;){
        static System::ETHC::IPv4 local;

        vTaskDelay(pdMS_TO_TICKS(2000));

        uint32_t tmp = FreeRTOS_GetIPAddress();

        if(tmp == local.value)
            continue;

        local.value = tmp;

        char str[32];
        snprintf(str, sizeof(str), "local is: %01d.%01d.%01d.%01d" NEWLINE, local.raw[0],local.raw[1],local.raw[2],local.raw[3]);

        System::nputsUIUART(str, sizeof(str));
    }

//    vTaskDelete(NULL);
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
    System::nputsUIUART(STRANDN("Initializing off chip" NEWLINE));




    /* --- POST off-chip ------------------------------------------ */
//    System::nputsUIUART(STRANDN("POST-ing off chips ..." NEWLINE));
    // TODO


    /* --- Start -------------------------------------------------- */
    System::nputsUIUART(STRANDN("starting tasks " NEWLINE));


    // indicator led
    xTaskCreate(Task::Blink::task,
        "blink indicator task",
        configMINIMAL_STACK_SIZE,
        (void *)&blink_indicator_args,
        tskIDLE_PRIORITY,
        NULL);

    xTaskCreate(firstTask,
        "initializing task",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY,
        NULL);

    xTaskCreate(fiddleTask,
        "fiddle task",
        configMINIMAL_STACK_SIZE,
        NULL,
        100e3,
        NULL);

    System::nputsUIUART(STRANDN("starting scheduler " NEWLINE));

    vTaskStartScheduler();

    for(;;)
        // go crazy
        System::FailHard("FreeRTOS scheduler crashed");
}

/*-----------------------------------------------------------*/

void firstTask(void * args) {
    vTaskDelete(NULL);
}
