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
#include <driverlib/can.h>
#include <task.h>

/*--- definitions ----------------------------------------------------------------------------*/

namespace System {
    #ifdef PROJECT_ENABLE_UART0
        UART::UART<UART::UART_REG_MFC_MS> uart0(uart0_regs);
    #endif
    #ifdef PROJECT_ENABLE_CAN0
        CAN::CAN can0(can0_regs);
    #endif
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

        SysCtlDelay(SysCtlClockGet() / 2);
    }
}

void System::GPIO::GPIO_REG::defaultInitAsOutput() const
{
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTn_BASE, GPIO_PIN_n);
    MAP_GPIODirModeSet(GPIO_PORTn_BASE, GPIO_PIN_n, GPIO_DIR_MODE_OUT);
    MAP_GPIOPadConfigSet(GPIO_PORTn_BASE, GPIO_PIN_n, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

void System::GPIO::GPIO_REG::defaultInitAsInput() const
{
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTn_BASE, GPIO_PIN_n);
    MAP_GPIODirModeSet(GPIO_PORTn_BASE, GPIO_PIN_n, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTn_BASE, GPIO_PIN_n, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

void System::GPIO::GPIO_REG::setValue(bool v) const
{
    MAP_GPIOPinWrite(GPIO_PORTn_BASE, GPIO_PIN_n, v ? GPIO_PIN_n : 0);
}

uint32_t System::GPIO::GPIO_REG::getValue() const
{
    return MAP_GPIOPinRead(GPIO_PORTn_BASE, GPIO_PIN_n);
}

void System::nputsUIUART(const char *str, uint32_t n)
{
    System::SYSTEM_UART_PRIM_UI.nputs(str, n);
}

void System::CAN::CAN::preinit()
{
    semph = xSemaphoreCreateMutex();

    GPIOPinConfigure(regs.GPIO_PIN_CONFIG_CANnRX);
    GPIOPinConfigure(regs.GPIO_PIN_CONFIG_CANnTX);
    GPIOPinTypeCAN(regs.GPIO_PORTn_BASE, regs.GPIO_PIN_nrx | regs.GPIO_PIN_ntx);

    SysCtlPeripheralEnable(regs.SYSCTL_PERIPH_CANn);

    while(!SysCtlPeripheralReady(regs.SYSCTL_PERIPH_CANn))
        {}

    CANInit(regs.CANn_BASE);

    // for full init
    /*
    CANBitRateSet(regs.CANn_BASE, SysCtlClockGet(), 500e3);
    CANEnable(regs.CANn_BASE);
    */
}

