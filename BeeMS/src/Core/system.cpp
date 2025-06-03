/*
 * system.cpp
 *
 *  Created on: May 6, 2025
 *      Author: turtl
 */

#include <stdint.h>

#include "Core/system.hpp"

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <task.h>

/*--- definitions ----------------------------------------------------------------------------*/

namespace System {
    #ifdef PROJECT_ENABLE_UART0
        UART::UART<UART::UART_REG_MFC_MS> uart0(uart0_regs);
    #endif

    namespace ETHC {
        System::ETHC::IPv4 ip        = {.raw = {169,254,212,183}};
        System::ETHC::IPv4 mask      = {.raw = {255,255,0,0}};
        System::ETHC::IPv4 gateway   = {.raw = {169,254,212,181}};
        System::ETHC::IPv4 dns       = {.raw = {169,254,212,182}};
        System::ETHC::MAC  mac       = {0x00,0x0F,0x8B,0x00,0x00,0x00}; // Orion MultiSystems Inc
    }
}

/*--- miscellaneous asserts ------------------------------------------------------------------*/

#if (!defined PROJECT_ENABLE_UART0) || (!defined SYSTEM_UART_PRIM_UI) \
    || (SYSTEM_UART_PRIM_UI != uart0)
    #error "UART0 is current not the main UART UI. UART0 should always be the main UI"
    /* UART0 can be enabled in the configuration section of "system.hpp".
     * UART0 should always be the primary UART UI because thats what
     * the LP uses. (its for convenience)
     */
#endif


/*--- function definitions -------------------------------------------------------------------*/

void System::FailHard(char const * error_description) {
    // its over, the BeeMS has fallen

    // stop all interrupts
    IntMasterDisable();

    // trigger fault line

    // stop the scheduler if its running
    if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        vTaskEndScheduler();

    // yap yap, idk, we'll put something better here one day
    for(;;){
        static char str[] = "System::FailHard -> ";
        System::SYSTEM_UART_PRIM_UI.nputs(str, sizeof(str));
        System::SYSTEM_UART_PRIM_UI.nputs(error_description, MAX_ERROR_MSG_LEN);
        System::SYSTEM_UART_PRIM_UI.nputs(NEWLINE, sizeof(NEWLINE));

        SysCtlDelay(System::CPU_FREQ / 2);
    }
}

void System::GPIO::GPIO_REG::defaultInitAsOutput() const
{
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTn_BASE, GPIO_PIN_n);
    MAP_GPIOPadConfigSet(GPIO_PORTn_BASE, GPIO_PIN_n, GPIO_STRENGTH_10MA, GPIO_PIN_TYPE_STD);
}

void System::GPIO::GPIO_REG::defaultInitAsInput() const
{
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTn_BASE, GPIO_PIN_n);
    MAP_GPIOPadConfigSet(GPIO_PORTn_BASE, GPIO_PIN_n, GPIO_STRENGTH_10MA, GPIO_PIN_TYPE_STD);
}

void System::GPIO::GPIO_REG::setValue(bool v) const
{
    MAP_GPIOPinWrite(GPIO_PORTn_BASE, GPIO_PIN_n, v);
}

uint32_t System::GPIO::GPIO_REG::getValue() const
{
    return MAP_GPIOPinRead(GPIO_PORTn_BASE, GPIO_PIN_n);
}

void System::nputsUIUART(const char *str, uint32_t n)
{
    System::SYSTEM_UART_PRIM_UI.nputs(str, n);
}
