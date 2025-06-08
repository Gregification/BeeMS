/*
 * system.hpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

// i give up about making this all fancy and modular. its just going to be a slap job

#ifndef SRC_CORE_SYSTEM_HPP_
#define SRC_CORE_SYSTEM_HPP_

#include <stdint.h>
#include <stdbool.h>
#include <ti/driverlib/driverlib.h>

/*--- meta ---------------------------------------------*/

#define PROJECT_NAME            "Voltage Tap"
#define PROJECT_DESCRIPTION     "github.com/Gregification/BeeMS"
#define PROJECT_VERSION         "0.0.0"


/*--- shorthand ----------------------------------------*/

#define BV(X) (1 << (X))
#define STRINGIFY(X) #X
#define TOSTRING(X) STRINGIFY(X)

/* if fails BMS will immediately trigger a shutdown */
#define ASSERT_FATAL(X, STR)        if(!(X)) System::FailHard(STR " @assert:line" TOSTRING(__LINE__) "," __FILE__);

/* for the uart nputs(char*,num len) command */
#define STRANDN(STR)                STR,sizeof(STR)

/* OCCUPY macro defines a static const variable with a unique name per ID
    If the same ID is used again in the same translation unit, it will cause redefinition error
    IMPORTANT: this macro will only work with things in the same scope!
    * physical pins will be in the "System" name space */
#define OCCUPY(ID)                  constexpr int const __PROJECT_OCCUPY_##ID = 0;

/*--- configuration maybe ------------------------------*/

#define NEWLINE                     "\n\r"
#define MAX_STR_LEN_COMMON          255   // assumed max length of a string if not specified. to minimize the damage of overruns.
#define MAX_STR_ERROR_LEN           (MAX_STR_LEN_COMMON * 2)
#define POWER_STARTUP_DELAY         16
#define UARTUI                      UART0

/*------------------------------------------------------*/

namespace System {
    OCCUPY(UART0)   // UI
}

namespace System {
    namespace CLK {
        /* womp womp. no constexpr's plz */

        extern uint32_t busclkUART; // has different values depending on the UART's PD. solution is to not give a toot about power usage
    }

    void init();

    /* put string to the UART responsible for UI */

    /* bring system to immediate stop . requires chip reset to escape this */
    void FailHard(char const * str = nullptr);

    namespace UART {
        /** a generic initialization*/
        void partialInit(UART_Regs *);

        void setBaudTarget(UART_Regs *, uint32_t target_baud, uint32_t clk = System::CLK::busclkUART);

        void nputs(UART_Regs *, char const * str, uint32_t n);
    }

    namespace SPI
    {
        void partialInit(SPI_Regs *);

    }

}


#endif /* SRC_CORE_SYSTEM_HPP_ */
