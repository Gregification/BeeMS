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

void fiddleTask(void * args){

    for(;;){
        FreeRTOS_SendPingRequest(IPV4_TO_INT(192,123,123,123), 0, pdMS_TO_TICKS(100));
        System::nputsUIUART(STRANDN("fiddle task" NEWLINE));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void firstTask(void * args);

BaseType_t interfaceInitialise_wrapper ( struct xNetworkInterface *) {
    return xNetworkInterfaceInitialise();
}

BaseType_t xNetworkInterfaceOutput_wrapper( struct xNetworkInterface *,
                                    NetworkBufferDescriptor_t * const pxNetworkBuffer,
                                    BaseType_t xReleaseAfterSend ){
    return xNetworkInterfaceOutput(pxNetworkBuffer, xReleaseAfterSend);
}
NetworkInterface_t * pxFillInterfaceDescriptor( BaseType_t, NetworkInterface_t * pxInterface ) {
    pxInterface->pcName = "da interface";
    pxInterface->pvArgument = NULL;
    pxInterface->pfInitialise = interfaceInitialise_wrapper;
    pxInterface->pfOutput = xNetworkInterfaceOutput_wrapper;
    FreeRTOS_AddNetworkInterface(pxInterface);
    return pxInterface;
}

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
    System::nputsUIUART(STRANDN("--- starting tasks " NEWLINE));


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

    {
        System::nputsUIUART(STRANDN("ethernet test" NEWLINE));

        GPIOPinConfigure(GPIO_PF0_EN0LED0);
        GPIOPinConfigure(GPIO_PF4_EN0LED1);
        GPIOPinConfigure(GPIO_PF1_EN0LED2);
        GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_1);

        GPIOPinConfigure(GPIO_PG0_EN0PPS);
        GPIOPinTypeEthernetMII(GPIO_PORTB_BASE, GPIO_PIN_0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);

        static System::ETHC::IPv4 ip        = {.value = IPV4_TO_INT(1,1,1,1)};
        static System::ETHC::IPv4 mask      = {.value = IPV4_TO_INT(255,255,255,0)};
        static System::ETHC::IPv4 gateway   = {.value = IPV4_TO_INT(8,8,8,8)};
        static System::ETHC::IPv4 dns       = {.value = IPV4_TO_INT(7,7,7,7)};
        static System::ETHC::MAC  mac       = {1,2,3,4,5,6};

        static NetworkInterface_t interface;
        pxFillInterfaceDescriptor(0, &interface);

        if(FreeRTOS_IPInit(
                    ip.raw,
                    mask.raw,
                    gateway.raw,
                    dns.raw,
                    mac.raw
                )) {
            blink_indicator_args.period_ms = Task::Blink::PERIOD_FAULT;
            System::FailHard("failed FreeRTOS IP init");
        }

    }

    System::nputsUIUART(STRANDN("--- starting scheduler " NEWLINE));

    vTaskStartScheduler();

    for(;;)
        // go crazy
        System::FailHard("FreeRTOS scheduler crashed");
}

/*-----------------------------------------------------------*/

void firstTask(void * args) {
    vTaskDelete(NULL);
}
