/*
 * system.hpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *
 *  target MCU : MSPM0G3507         https://www.ti.com/product/MSPM0G3507
 *
 *  Intended for use with FreeRTOS
 */

/*** OVERVIEW **************************************************************************************
 * general wrappers and control macros for the project. A small layer on top of DriverLib, this is by no means
 *  a replacement for DriverLib. Purpose is to make development easier without having to actively research
 *  the chip.
 *
 *** NOTES *****************************************************************************************
 * - THE CALLING FUNCTION MUST LOCK THE PERIPHERIAL RESOURCE BEFORE USINGIT
 * - all peripherals have wrappers. no matter how pointless it is. for semaphore control stuff
 *      - the semaphore is runtime so the compiler probably want to optimize it.
 *
 * - citation formatting
 *      - always have the section
 *      - page number if possible
 *      - eg: family data sheet , section 19.2.1 - which is on page 1428
 *          - "FDS.19.2.1/1428" or omit the page "FDS.19.2.1"
 *      - "LP" : launch-pad                                  https://www.ti.com/tool/LP-MSPM0G3507
 *      - "LPDS" : launch pad user-guide/data-sheet          https://www.ti.com/lit/ug/slau873d/slau873d.pdf?ts=1749180414460&ref_url=https%253A%252F%252Fwww.ti.com%252Ftool%252FLP-MSPM0G3507
 *      - "FDS" : family specific data sheet                 https://www.ti.com/lit/ug/slau846b/slau846b.pdf?ts=1749245238762&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSPM0G3507
 *      - "TDS" : chip specific technical data sheet         https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf?ts=1749166832439&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSPM0G3507
 *
 *      - "BQ" : evaluation-board                            https://www.ti.com/tool/BQ76952EVM
 *      - "BQDS" : BQ data sheet                   https://www.ti.com/lit/ds/symlink/bq76952.pdf?ts=1751601724825&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FBQ76952
 *      - "BQTRM" : BQ technical reference manual  https://www.ti.com/lit/ug/sluuby2b/sluuby2b.pdf?ts=1751657887923&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FBQ76952%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dapp-null-null-GPN_EN-cpc-pf-google-ww_en_cons%2526utm_content%253DBQ76952%2526ds_k%253DBQ76952+Datasheet%2526DCM%253Dyes%2526gad_source%253D1%2526gad_campaignid%253D1767856010%2526gbraid%253D0AAAAAC068F3kMVn5JB15cZNcLXZ2ysu0t%2526gclid%253DCj0KCQjw953DBhCyARIsANhIZoa8LrrvSAnWBtKYvyJsSyVJWRKfkSw7Zxzr4w8DOEBf7oJBMp3RtwcaAklgEALw_wcB%2526gclsrc%253Daw.ds
 *
 * - default clock is MFCLK : this is factory set to 4Mhz on the MSPM0G3507
 * - the comment style is what ever I feel like. no Doxygen, no JavaDoc, we ball
 * - hardware resources are coordinated across 'tasks' using mutex's, even if we decide that a
 *      specific peripheral will only ever be used by a specific 'task' still use resource locking.
 */

#ifndef SRC_CORE_SYSTEM_HPP_
#define SRC_CORE_SYSTEM_HPP_

#include <stdint.h>
#include <stdbool.h>

#include <ti/driverlib/driverlib.h>


/*--- meta ---------------------------------------------*/

#define PROJECT_NAME            "Voltage Tap MSPM0L1305"
#define PROJECT_DESCRIPTION     "github.com/Gregification/BeeMS"
#define PROJECT_VERSION         "B.2.0" // [project version].[hardware version].[software version]

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


/*--- general configuration ----------------------------*/

#define NEWLINE                     "\n\r"
#define MAX_STR_LEN_COMMON          125   // assumed max length of a string if not specified. to minimize the damage of overruns.
#define MAX_STR_ERROR_LEN           (MAX_STR_LEN_COMMON * 2)
#define POWER_STARTUP_DELAY         16

typedef uint16_t buffsize_t;

/*--- peripheral configuration -------------------------*/
/* so many pin conflicts. TDS.6.2/10 */

#define PROJECT_ENABLE_UART0        // LP

#define PROJECT_ENABLE_SPI0         // up to 2Mhz
//#define PROJECT_ENABLE_SPI1         // up to 32Mhz, restrictions based on CPU clock. FDS.19.2.1/1428 , TDS.7.20.1/46

//#define PROJECT_ENABLE_I2C0
#define PROJECT_ENABLE_I2C1


/*--- common peripheral pins ---------------------------*/

/*
 * UART0 : ui
 * SPI0 : Ethernet and CAN controller
 * I2C1 : BQ76952
 */

namespace System {
    #ifdef PROJECT_ENABLE_UART0
        OCCUPY(PA8)
        OCCUPY(PA9)
    #endif

    #ifdef PROJECT_ENABLE_I2C1
        OCCUPY(PA15)
        OCCUPY(PA16)
    #endif

    #ifdef PROJECT_ENABLE_SPI0
        OCCUPY(PA10)// MISO
        OCCUPY(PA11)// SCLK
        OCCUPY(PA5) // MOSI
    #endif
}


/*------------------------------------------------------*/

namespace System {
    /* see clock tree diagram ... and SysConfig's */
    namespace CLK {
        constexpr uint32_t FBUS  = 16e6;

        // values determined during init
        constexpr uint32_t MCLK  = 32e6;
        constexpr uint32_t ULPCLK= 32e6;
        constexpr uint32_t CPUCLK= 32e6;
        constexpr uint32_t ADCCLK= 32e6;
        constexpr uint32_t LFCLK = 32768;
        constexpr uint32_t MFCLK = 4e6;
    }

    namespace UART {
        struct UART {
            UART_Regs * const reg;

            void setBaudTarget(uint32_t target_baud, uint32_t clk = System::CLK::MCLK);

            /** transmits - blocking - a string of at most size n */
            void nputs(char const * str, buffsize_t n);
            void ngets(char * str, buffsize_t n);

            // make your own irq's
        };
    }

    namespace GPIO {
        #define GPIOPINPUX(X) (X).port,(X).pin

        struct GPIO {
            GPIO_Regs * port;   // eg: GPIOA
            uint32_t    pin;    // eg: DL_GPIO_PIN_0
            uint32_t    iomux;  // eg: IOMUX_PINCM0

            inline void set()   { DL_GPIO_setPins(port, pin); }
            inline void clear() { DL_GPIO_clearPins(port, pin); }
        };

        // Port A (PA) pins
        extern GPIO PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
                     PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
                     PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23,
                     PA24, PA25, PA26, PA27;

    }

    namespace SPI {
        /* transmitted when RX is needed but no TX is provided */
        constexpr uint8_t TRANSFER_FILLER_BYTE = 0x0;

        /* - you must manually control CS
         * - for any transmission speeds worth a crap you will have to use DL
         * - functions here are general and are nowhere near peak performance
         * - master only device
         */
        struct SPI {
            SPI_Regs * const reg;

            void setSCLKTarget(uint32_t target, uint32_t clk = System::CLK::FBUS);

            void transfer(void * tx, void * rx, buffsize_t len);
            inline void transfer_blocking(void * tx, void * rx, buffsize_t len){
                transfer(tx, rx, len);
                while(isBusy());
            }

            bool isBusy() { return ((_trxBuffer.tx_i < _trxBuffer.len) || (_trxBuffer.rx_i < _trxBuffer.len)) && DL_SPI_isBusy(reg); }

            // should be private but eh
            struct {
                uint8_t *tx, *rx;
                buffsize_t len;
                buffsize_t tx_i, rx_i;
            } _trxBuffer;
            void _irq();
        };
    }

    namespace I2C {


        /** I2C peripheral controller interface
         * - master only device */
        struct I2C {

            I2C_Regs * const reg;

            void setSCLTarget(uint32_t target, uint32_t clk = System::CLK::ULPCLK);
            void _irq();

            inline bool isBusy() { return _trxBuffer.error == ERROR::IN_USE; }

            /** blocks the task calling this function until TX is complete or timeout.
             * uses IRQ+Notifications. other tasks can run while this is blocking
             * @return true if TX success. returns false if timed out, lost arbitration, or received NACK
             */
            void tx(uint8_t addr, void * data, buffsize_t size);
            inline bool tx_blocking(uint8_t addr, void * data, buffsize_t size) {
                tx(addr, data, size);
                while(isBusy());
                return _trxBuffer.error == ERROR::NONE;
            }

            /** blocks the task calling this function until RX is complete or timeout.
             *  uses IRQ+Notifications. other tasks can run while this is blocking
             *  @return true if RX success. returns false if timed out, lost arbitration, received NACK,
             *      or received less than expected amount of bytes.
             */
            void rx(uint8_t addr, void * data, buffsize_t size);
            inline bool rx_blocking(uint8_t addr, void * data, buffsize_t size) {
                rx(addr, data, size);
                while(isBusy());
                return _trxBuffer.error == ERROR::NONE;
            }


            enum ERROR : uint8_t {
                IN_USE,
                NONE,
                NACK,
                TIMEOUT
            };

            struct {
                uint8_t * data;
                buffsize_t data_length;    // total bytes of data
                buffsize_t nxt_index;      // index next byte is read/written by
                ERROR error;
            } _trxBuffer;
        };
    }

    void init();

    /* bring system to immediate stop . requires chip reset to escape this */
    void FailHard(char const * str = nullptr);

    void waitUS(uint32_t us);

    /*--- system globals -----------------------------------*/

    /** a reference to the UART acting as the main text UI */
    extern UART::UART &uart_ui;

    #ifdef PROJECT_ENABLE_UART0
        extern UART::UART uart0;
    #endif
    #ifdef PROJECT_ENABLE_SPI0
        extern SPI::SPI spi0;
    #endif

    #ifdef PROJECT_ENABLE_I2C0
        #error "I2C0 not implimented"
    #endif
    #ifdef PROJECT_ENABLE_I2C1
        extern I2C::I2C i2c1;
    #endif

}

/*--- idiot detection ------------------------------------------------------------------*/

#if !defined(PROJECT_ENABLE_UART0)
    #error "uart0 should always be enabled and used for the UI. better be a good reason otherwise."
    /* uart0 is used by the LP */
#endif

#endif /* SRC_CORE_SYSTEM_HPP_ */

