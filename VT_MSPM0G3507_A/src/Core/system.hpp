/*
 * system.hpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *
 *  target MCU : MSPM0G3507         https://www.ti.com/product/MSPM0G3507
 *
 */

/*** OVERVIEW **************************************************************************************
 * general wrappers and control macros for the project. A small layer on top of DriverLib, this is by no means
 *  a replacement for DriverLib. Purpose is to make development easier without having to actively research
 *  the chip.
 *
 *** NOTES *****************************************************************************************
 * - all peripherals have wrappers. no matter how pointless it is. for semaphore control stuff
 *      - the semaphore is runtime so the compiler probably wont optimize it.
 * - "FDS" : family specific data sheet                 https://www.ti.com/lit/ug/slau846b/slau846b.pdf?ts=1749245238762&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSPM0G3507
 * - "TDS" : chip specific technical data sheet         https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf?ts=1749166832439&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSPM0G3507
 * - "LPDS" : launch pad user-guide/data-sheet          https://www.ti.com/lit/ug/slau873d/slau873d.pdf?ts=1749180414460&ref_url=https%253A%252F%252Fwww.ti.com%252Ftool%252FLP-MSPM0G3507
 * - "LP" : launch-pad/evaluation-board                 https://www.ti.com/tool/LP-MSPM0G3507
 * - citations
 *      - always have the section
 *      - page number if possible
 *      - eg: family data sheet , section 19.2.1 - which is on page 1428
 *          - "FDS.19.2.1/1428" or omit the page "FDS.19.2.1"
 * - default clock is MFCLK : this is factory set to 4Mhz on the MSPM0G3507
 * - the comment style is what ever I feel like. no Doxygen, no JavaDoc, we ball
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
#define ARRANDN(ARR)                ARR,sizeof(ARR)

/* OCCUPY macro defines a static const variable with a unique name per ID
    If the same ID is used again in the same translation unit, it will cause redefinition error
    IMPORTANT: this macro will only work with things in the same scope!
    * physical pins will be in the "System" name space */
#define OCCUPY(ID)                  constexpr int const __PROJECT_OCCUPY_##ID = 0;

/*--- configuration maybe ------------------------------*/

#define NEWLINE                     "\n\r"
#define MAX_STR_LEN_COMMON          125   // assumed max length of a string if not specified. to minimize the damage of overruns.
#define MAX_STR_ERROR_LEN           (MAX_STR_LEN_COMMON * 2)
#define POWER_STARTUP_DELAY         16

/*------------------------------------------------------*/
/* so many pin conflicts. TDS.6.2/10 */

#define PROJECT_ENABLE_UART0        // LP

#define PROJECT_ENABLE_SPI0         // up to 2Mhz
//#define PROJECT_ENABLE_SPI1         // up to 32Mhz, restrictions based on CPU clock. FDS.19.2.1/1428 , TDS.7.20.1/46

//#define PROJECT_ENABLE_I2C0
#define PROJECT_ENABLE_I2C1


/*--- common peripheral pins ---------------------------*/

namespace System {
    OCCUPY(UART0)   // UI
    OCCUPY(PINCM21) //PA10
    OCCUPY(PINCM22) //PA11
}


/*------------------------------------------------------*/

namespace System {
    struct Lockable { // Java gang
        SemaphoreHandle_t semph = NULL;

        Lockable() {
            semph = xSemaphoreCreateRecursiveMutex();
            while(semph == NULL){
                // bad stuff
                //TODO: hard restart
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
            UART_Regs * const reg;

            void partialInit();
            void setBaudTarget(uint32_t target_baud, uint32_t clk = System::CLK::MFCLK);

            /** transmits - blocking - a string of at most size n */
            void nputs(char const * str, uint32_t n);
        };

    }

    namespace GPIO {
        #define GPIOPINPUX(X) X.port,X.pin

        struct GPIO {
            GPIO_Regs * port;   // eg: GPIOA
            uint32_t    pin;    // eg: DL_GPIO_PIN_0
            uint32_t    iomux;  // eg: IOMUX_PINCM0

            inline void set() { DL_GPIO_setPins(port, pin); }
            inline void clear() { DL_GPIO_clearPins(port, pin); }
        };

        // Port A (PA) pins
        extern const GPIO PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
                     PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
                     PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23,
                     PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31;

        // Port B (PB) pins
        extern const GPIO PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7,
                     PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
                     PB16, PB17, PB18, PB19, PB20, PB21, PB22, PB23,
                     PB24, PB25, PB26, PB27;

    }

    namespace SPI {
        /* - you must manually control CS
         * - for any transmission speeds worth a crap you will have to use DL
         * - functions here are general and are nowhere near peak performance
         */

        /* TODO: missing a SPI "transfer" function because I don't feel like making it, you can make one */
        struct SPI : Lockable {
            SPI_Regs * const reg;

            /* see system.cpp top comments for example */
            void partialInit();
            void setSCLKTarget(uint32_t target, uint32_t clk = System::CLK::ULPCLK);

            void tx_blocking(const void * data, uint16_t size, GPIO::GPIO * cs = NULL);
            void rx_blocking(void * data, uint16_t size, GPIO::GPIO * cs = NULL);
        };
    }

    namespace I2C {
        struct I2C : Lockable {
            I2C_Regs * const reg;

            void partialInitController();
            void setSCLTarget(uint32_t target, uint32_t clk = System::CLK::ULPCLK);

            /** return 0 on success */
            uint8_t tx_ctrl_blocking(uint8_t addr, void const *, uint8_t size);
            /** return 0 on success */
            uint8_t rx_ctrl_blocking(uint8_t addr, void *, uint8_t size);
        };
    }

    void init();

    /* bring system to immediate stop . requires chip reset to escape this */
    void FailHard(char const * str = nullptr);


    /*--- system globals -----------------------------------*/

    /* a reference to the UART acting as the main text UI */
    extern UART::UART &uart_ui;

    #ifdef PROJECT_ENABLE_UART0
        extern UART::UART uart0;
    #endif
    #ifdef PROJECT_ENABLE_SPI0
        extern SPI::SPI spi0;
    #endif
    #ifdef PROJECT_ENABLE_SPI1
        extern SPI::SPI spi1;
    #endif

    #ifdef PROJECT_ENABLE_I2C0
        #error "I2C0 not implimented"
    #endif
    #ifdef PROJECT_ENABLE_I2C1
        extern I2C::I2C i2c1;
    #endif

}


#endif /* SRC_CORE_SYSTEM_HPP_ */
