/*
 * system.hpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

/*
 * general wrappers and control macros for the project. A small layer on top of DriverLib, this is by no means
 *  a replacement for DriverLib.
 *
 * - defaults to MFCLK
 */

#ifndef SRC_CORE_SYSTEM_HPP_
#define SRC_CORE_SYSTEM_HPP_

#include <stdint.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <semphr.h>

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

/*------------------------------------------------------*/

#define PROJECT_ENABLE_UART0

namespace System {
    OCCUPY(UART0)   // UI
    OCCUPY(PINCM21) //PA10
    OCCUPY(PINCM22) //PA11
}

namespace System {
    struct Lockable { // Java gang
        SemaphoreHandle_t semph = NULL;

        Lockable() {
            semph = xSemaphoreCreateRecursiveMutex();
            while(semph == NULL){
                // bad stuff
            }
        }

    };

    /* see clock tree diagram ... and SysConfig's */
    namespace CLK {
        /* no constexpr's plz */

        extern uint32_t LFCLK;
        extern uint32_t ULPCLK;
        extern uint32_t &MCLK;
        extern uint32_t CPUCLK;
        extern uint32_t CANCLK;
        extern uint32_t MFPCLK;
        constexpr uint32_t MFCLK = 4e6;
    }

    namespace UART {
        struct UART : Lockable {
            UART_Regs * reg;

            void partialInit();
            void setBaudTarget(uint32_t target_baud, uint32_t clk = System::CLK::MFCLK);

            /** transmits - blocking - a string of at most size n */
            void nputs(char const * str, uint32_t n);
        };

    }

    namespace SPI {

        /* TODO: missing a "transfer" function because I dont feel like making it, you can make one */

        void partialInit(SPI_Regs *);
        void setSCLKTarget(uint32_t target, uint32_t clk = System::CLK::MFCLK);

        void tx_blocking(void const * data, uint16_t size);
        void rx_blocking(void * data, uint16_t size);
    }


    void init();

    /* bring system to immediate stop . requires chip reset to escape this */
    void FailHard(char const * str = nullptr);

    #ifdef PROJECT_ENABLE_UART0
        extern UART::UART uart0;
    #endif
    extern UART::UART &uart_ui;
}


#endif /* SRC_CORE_SYSTEM_HPP_ */
