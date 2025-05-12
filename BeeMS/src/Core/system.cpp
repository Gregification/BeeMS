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

    // kill all processes
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
