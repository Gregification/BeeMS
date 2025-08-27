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

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <ti/driverlib/driverlib.h>


/*--- meta ---------------------------------------------*/

#define PROJECT_NAME            "Voltage Tap"
#define PROJECT_DESCRIPTION     "github.com/Gregification/BeeMS"
#define PROJECT_VERSION         "2.1" // [project version].[hardware version].[software version]


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

/* the System IRQ functions rely on notifications to sync with tasks, this is the specific index used as the notification
 * - the max index value is determined by the arbitrary value of "configTASK_NOTIFICATION_ARRAY_ENTRIES"
 * - index 0 is the default so avoid using that since it may get triggered by something else
 */
#define TASK_NOTIFICATION_ARRAY_INDEX_FOR_SYSTEM_I2C_IRQ 1
#define TASK_NOTIFICATION_ARRAY_INDEX_FOR_SYSTEM_SPI_IRQ 2
#if configTASK_NOTIFICATION_ARRAY_ENTRIES < 2
    #error "increase size of configTASK_NOTIFICATION_ARRAY_ENTRIES"
#endif

/*--- peripheral configuration -------------------------*/
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

    /** wrapper for controlling access to limited hardware resource.
     * purpose is to standardize resource access.
     */
    class Lockable { // Java gang
        /* theres a design consideration between sharing a resource across multiple 'tasks' though
         * locks or having a trx buffer they all reference and a single 'task' handle the resource.
         * theres benefits to both but Im going with a locking design since its allows 'tasks' more
         * detailed hardware control.
         */

        SemaphoreHandle_t semph = NULL;

    public:

        Lockable() {
            semph = xSemaphoreCreateRecursiveMutex();
            while(semph == NULL){
                // ran out of memory

                // TODO: handle this problem somehow, probably just restart the device. just
                //      make sure the system is in a state where nothing dangerous is enabled
                //      as this is happening.
            }
        }

        /** takes the recursive lock.
         * returns true if resource was acquired
         */
        bool takeResource(TickType_t timeout);

        /** releases the recursive lock.
         * returns true of the resource was successfully released
         *      can fail in cases such as releasing a resource without taking it first.
         */
        void releaseResource();

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
            void ngets(char * str, uint32_t n);
        };

    }

    namespace GPIO {
        #define GPIOPINPUX(X) (X).port,(X).pin

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
        /* transmitted when RX is needed but no TX is provided */
        constexpr uint8_t TRANSFER_FILLER_BYTE = 0x0;

        /* - you must manually control CS
         * - for any transmission speeds worth a crap you will have to use DL
         * - functions here are general and are nowhere near peak performance
         * - master only device
         */
        struct SPI : Lockable {
            SPI_Regs * const reg;

            /* see system.cpp top comments for example */
            void partialInit();
            void setSCLKTarget(uint32_t target, uint32_t clk = System::CLK::ULPCLK);
            void _irq();

            bool transfer(void * tx, void * rx, uint16_t len, TickType_t timeout);

            // should be private but eh
            struct {
                TaskHandle_t host_task;
                uint8_t *tx, *rx;
                uint8_t len;
                uint8_t tx_i, rx_i;
            } _trxBuffer;
        };
    }

    namespace I2C {


        /** I2C peripheral controller interface
         * - master only device */
        struct I2C : Lockable {

            I2C_Regs * const reg;

            void partialInitController();
            void setSCLTarget(uint32_t target, uint32_t clk = System::CLK::ULPCLK);
            void _irq();

            /** blocks the task calling this function until TX is complete or timeout.
             * uses IRQ+Notifications. other tasks can run while this is blocking
             * @return true if TX success. returns false if timed out, lost arbitration, or received NACK
             */
            bool tx_blocking(uint8_t addr, void * data, uint8_t size, TickType_t timeout);

            /** blocks the task calling this function until RX is complete or timeout.
             *  uses IRQ+Notifications. other tasks can run while this is blocking
             *  @return true if RX success. returns false if timed out, lost arbitration, received NACK,
             *      or received less than expected amount of bytes.
             */
            bool rx_blocking(uint8_t addr, void * data, uint8_t size, TickType_t timeout);

            // should be private but then you'll need to make a constructor and all that boiler plate.
            struct {
                TaskHandle_t host_task;
                uint8_t * data;
                uint8_t data_length;    // total bytes of data
                uint8_t nxt_index;      // index next byte is read/written by
            } _trxBuffer;
        };
    }

    void init();

    /* bring system to immediate stop . requires chip reset to escape this */
    void FailHard(char const * str = nullptr);


    /*--- system globals -----------------------------------*/

    /** a reference to the UART acting as the main text UI */
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

/*--- idiot detection ------------------------------------------------------------------*/

#if !defined(PROJECT_ENABLE_UART0)
    #error "uart0 should always be enabled and used for the UI. better be a good reason otherwise."
    /* uart0 is used by the LP */
#endif

// i fear for the day this happens
static_assert(pdTRUE == true,
        "pdTRUE != true . the FreeRTOS definition of \"true\" is not the same value as c/c++s \
        definition. code probably wont work. maybe FreeRTOS files were edited. consider reinstall."
    );
static_assert(pdFALSE == false,
        "pdFALSE != false . the FreeRTOS definition of \"false\" is not the same value as c/c++s \
        definition. code probably wont work. maybe FreeRTOS files were edited. consider reinstall."
    );


#endif /* SRC_CORE_SYSTEM_HPP_ */

