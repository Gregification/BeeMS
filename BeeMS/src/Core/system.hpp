/*
 * system.hpp
 *
 *  Created on: May 6, 2025
 *      Author: turtl
 */

/*
 * corner stones of this project
 *  - this hardware is more replaceable than anything else
 *  - error resolution first, reporting second
 *  - explicit initialization of unused variables
 *  - don't care about on chip power consumption
 *
 * side notes
 *  - when adding a new peripheral that uses a pin or limited resource define a macro to indicate that
 *      resource is inuse, so other parts of the code wont try to use it. e.g: for things that need a
 *      physical pin on the chip, define "PROJECT_OCCUPY_PIN_n __COUNTER__" where n is the pins package
 *       number. using the counter macro makes it throw a error if its redefined anywhere else.
 *  - driverLib is linked to a pre-compiled CSS project, consider min-maxxing the optimizations on that.
 *  - some of the variables can be macros but they're const-expression variables instead so its
 *      easier for people to discover them. no one reads the documentation.
 *  - explicit constexpr compilers because the compiler dosen't support designators on aggregate types.
 *      using the TI Clang compiler would fix but DriverLib isn't compatible and I haven't been able to
 *      port it, TI has a massive document going over migration but I couldn't get it to work; compiled
 *      fine but got a warning about a assembly section being empty. Couldn't track it down. DL didn't
 *      work in that instance.
 *  - i love cpp 14 (not). i love making stupidly bloated code to do the smallest things! i love ti for
 *      not updating their own library of their flagship chip for the last decade! i love burning time
 *      and time and time trying to patch what they pay their critters big bucks for! i just want a bms
 */

#ifndef SRC_CORE_SYSTEM_HPP_
#define SRC_CORE_SYSTEM_HPP_

#include <stdint.h>
#include <stdbool.h>

// DL
#include <inc/hw_sysctl.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>

// FreeRTOS
#include <FreeRTOS.h>
#include <semphr.h>

#include "utils custom/is_base_of_custom.hpp"


/*--- meta ---------------------------------------------*/

#define PROJECT_NAME            "BeeMS"
#define PROJECT_DESCRIPTION     "github.com/Gregification/BeeMS"
#define PROJECT_VERSION         "0.0.0"


/*--- shorthand ----------------------------------------*/

#define BV(X) (1 << (X))
#define STRINGIFY(X) #X
#define TOSTRING(X) STRINGIFY(X)

/* if fails BMS will immediately trigger a shutdown */
#define ASSERT_FATAL(X, STR) if(!(X)) System::FailHard(STR " @assert:line" TOSTRING(__LINE__) "," __FILE__);

/* for the uart nputs(char*,num len) command */
#define STRANDN(STR) STR,sizeof(STR)

/* OCCUPY macro defines a static const variable with a unique name per ID
    If the same ID is used again in the same translation unit, it will cause redefinition error
    IMPORTANT: this macro will only work with things in the same scope!
    * physical pins will be in the "System" name space */
#define OCCUPY(ID) constexpr int const __PROJECT_OCCUPY_##ID = 0;

#define IPV4_TO_INT(D,C,B,A) ( (uint32_t)( (A << (4 *3)) | (B << (4 *2)) | (C << (4 *1)) | (D) ) )

/*--- constants ----------------------------------------*/

#define NEWLINE "\n\r"
#define MAX_COMMON_STRING_LEN 255   // assumed max length of a string if not specified. to minimize the damage of overruns.
#define MAX_ERROR_MSG_LEN (MAX_COMMON_STRING_LEN * 2)
#define MAX_RESOURCE_LOCK_TIMEOUT_UART 5

/*--- hardware configuration ---------------------------*/

#define PROJECT_ENABLE_UART0
//#define PROJECT_ENABLE_UART1
//#define PROJECT_ENABLE_UART2
//#define PROJECT_ENABLE_UART3
//#define PROJECT_ENABLE_UART4
//#define PROJECT_ENABLE_UART5
//#define PROJECT_ENABLE_UART6
//#define PROJECT_ENABLE_UART7

#define SYSTEM_UART_PRIM_UI uart0       // uart responsible for the primary UI
namespace System { OCCUPY(UART0) }

// ethernet
namespace System {
    OCCUPY(PF0) // link led
    OCCUPY(PF4) // status led
    OCCUPY(PF1) // connection led
    OCCUPY(PG0) // PHY PPS signal
    OCCUPY(EMAC0)
    OCCUPY(EPHY0)
    OCCUPY(EN0)
}

/*------------------------------------------------------*/

namespace System {

    /*--- structs --------------------------------------------------------------------------------------------*/

    /* a single lock resource */
    struct LOCKABLE_SIMPLE {
        SemaphoreHandle_t semph = NULL;

        virtual inline bool _aquire(TickType_t blockTime);
        virtual inline void _release();
    };

    namespace GPIO {
        struct GPIO_REG {
            /* e.g: GPIO_PORTN_BASE */
            uint32_t GPIO_PORTn_BASE;

            /* e.g: GPIO_PIN_0 */
            uint32_t GPIO_PIN_n;

            // these functions should be inline but it gets cluttered here
            /* generic act as output */
            void defaultInitAsOutput() const;
            /* generic act as input */
            void defaultInitAsInput() const;
            void setValue(bool) const;
            uint32_t getValue() const;
        };
        /*
         *  custom init example
         *  "
         *      MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
         *      MAP_GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1,
         *                   GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
         *  "
         *  GPIO_PIN_TYPE_... meanings , ripped from DL gpio.h
         *      STD     : push pull
         *      STD_WPU : push pull with weak pull up
         *      STD_WPD : push pull with weak pull down
         *      OD      : open drain // fent reactors critical
         *      ANALOG  : analog comparator
         *      WAKE_HIGH   : hibernate wake, high
         *      WAKE_LOW    : hibernate wake, low
         */
    }

    namespace UART {
        constexpr uint32_t BAUD_UI = 115200;

        /* core control registers
         * for UART0,1,2,3,4,5,6,7 */
        struct UART_REG {
            /* e.g: GPIO_PA0_U0RX */
            uint32_t GPIO_PIN_CONFIG_UnRX;
            /* e.g: GPIO_PIN_0 */
            uint32_t GPIO_PIN_nrx;

            /* e.g: GPIO_PA0_U0TX */
            uint32_t GPIO_PIN_CONFIG_UnTX;
            /* e.g: GPIO_PIN_1 */
            uint32_t GPIO_PIN_ntx;

            /* e.g: SYSCTL_PERIPH_UART0 */
            uint32_t SYSCTL_PERIPH_UARTn;

            /* e.g: UART0_BASE */
            uint32_t UARTn_BASE;

            /* e.g: UART_CLOCK_PIOSC */
            uint32_t UART_CLOCK_src;

            /* e.g: GPIO_PORTA_BASE */
            uint32_t GPIO_PORTn_BASE;

            constexpr UART_REG(
                    uint32_t SYSCTL_PERIPH_UARTn,
                    uint32_t UARTn_BASE,
                    uint32_t GPIO_PORTn_BASE,
                    uint32_t UART_CLOCK_src,
                    uint32_t GPIO_PIN_CONFIG_UnRX,
                    uint32_t GPIO_PIN_nrx,
                    uint32_t GPIO_PIN_CONFIG_UnTX,
                    uint32_t GPIO_PIN_ntx)
                    : GPIO_PIN_CONFIG_UnRX(GPIO_PIN_CONFIG_UnRX),
                      GPIO_PIN_nrx(GPIO_PIN_nrx),
                      GPIO_PIN_ntx(GPIO_PIN_ntx),
                      GPIO_PIN_CONFIG_UnTX(GPIO_PIN_CONFIG_UnTX),
                      SYSCTL_PERIPH_UARTn(SYSCTL_PERIPH_UARTn),
                      UARTn_BASE(UARTn_BASE),
                      UART_CLOCK_src(UART_CLOCK_src),
                      GPIO_PORTn_BASE(GPIO_PORTn_BASE)
                {}
        };

        /* modem flow control registers
         * for UART0,1,2,3,4 */
        struct UART_REG_MFC : UART_REG {
            // TODO

            constexpr UART_REG_MFC(UART_REG const reg) : UART_REG(reg) {}
        };

        /* modem flow control and modem status registers
         * for UART0,1 */
        struct UART_REG_MFC_MS : UART_REG_MFC {
            // TODO

            constexpr UART_REG_MFC_MS(UART_REG_MFC const reg) : UART_REG_MFC(reg) {}
        };

        template <typename UART_TYPE>
        struct UART : LOCKABLE_SIMPLE {
            static_assert(is_base_of_custom<UART_TYPE, UART_REG>::value, "must inherit from UART_REG");

            UART_TYPE const regs;

            constexpr UART(UART_TYPE const r) : regs(r) {}

            /* a partial init */
            void preinit() {
                // configure pin muxing
                MAP_GPIOPinConfigure(regs.GPIO_PIN_CONFIG_UnRX);
                MAP_GPIOPinConfigure(regs.GPIO_PIN_CONFIG_UnTX);
                // enable UARTn
                MAP_SysCtlPeripheralEnable(regs.SYSCTL_PERIPH_UARTn);
                while(!MAP_SysCtlPeripheralReady(regs.SYSCTL_PERIPH_UARTn))
                    {}

                // set clock
                MAP_UARTClockSourceSet(regs.UARTn_BASE, regs.UART_CLOCK_src);
                // set alternative pin function
                MAP_GPIOPinTypeUART(regs.GPIO_PORTn_BASE, regs.GPIO_PIN_nrx | regs.GPIO_PIN_ntx);

                semph = xSemaphoreCreateMutex();
            }
            inline bool _aquire(TickType_t blockTime){ return xSemaphoreTake(semph, blockTime) == pdTRUE; }
            inline void _release(){ xSemaphoreGive(semph); }

            /* transmits the string of max size n */
            void nputs(char const * str, uint32_t n) const {
                for(uint32_t i = 0; (i < n) && (str[i] != '\0'); i++){
                    MAP_UARTCharPut(regs.UARTn_BASE, str[i]);
                }
            }

            void nputs_for_freertos(char const * str, uint32_t n) const {
                for(uint32_t i = 0; (i < n) && (str[i] != '\0'); i++){
                    MAP_UARTCharPut(regs.UARTn_BASE, str[i]);
                    if(str[i] == '\n')
                        MAP_UARTCharPut(regs.UARTn_BASE, '\r');
                }
            }
        };
    }

    /* Ethernet Controller */
    namespace ETHC {

        /* IPv4 address */
        union IPv4 {
            uint8_t raw[4];
            unsigned long value : 32;
        };

        /* IPv6 address */
        union IPv6 {
            uint8_t raw[4];
            struct {
                IPv4 lower32;
                IPv4 upper32;
            };
        };

        union MAC {
            uint8_t raw[6];
            unsigned long long value : 48;
        };

        extern System::ETHC::IPv4 ip;
        extern System::ETHC::IPv4 mask;
        extern System::ETHC::IPv4 gateway;
        extern System::ETHC::IPv4 dns;
        extern System::ETHC::MAC  mac;
    }

    /*--- variables ------------------------------------------------------------------------------------------*/

    /** CPU clock speed (Hz) */
    uint32_t CPU_FREQ;
    constexpr uint32_t PIOSC_FREQ = 16e6;

    #ifdef PROJECT_ENABLE_UART0
        OCCUPY(PA0);
        OCCUPY(PA1);
        constexpr const UART::UART_REG_MFC_MS uart0_regs(
            UART::UART_REG_MFC(
                UART::UART_REG(
                    SYSCTL_PERIPH_UART0,    // SYSCTL_PERIPH_UARTn
                    UART0_BASE,             // UARTn_BASE
                    GPIO_PORTA_BASE,        // GPIO_PORTn_BASE
                    UART_CLOCK_PIOSC,       // UART_CLOCK_src
                    GPIO_PA0_U0RX,          // GPIO_PIN_CONFIG_UnRX
                    GPIO_PIN_0,             // GPIO_PIN_nrx
                    GPIO_PA1_U0TX,          // GPIO_PIN_CONFIG_UnTX
                    GPIO_PIN_1              // GPIO_PIN_ntx
                )
            )
        );
        extern UART::UART<UART::UART_REG_MFC_MS> uart0;
    #endif
    #if defined PROJECT_ENABLE_UART1 || defined PROJECT_ENABLE_UART2 \
        || defined PROJECT_ENABLE_UART3 || defined PROJECT_ENABLE_UART4 \
        || defined PROJECT_ENABLE_UART5 || defined PROJECT_ENABLE_UART6 \
        || defined PROJECT_ENABLE_UART7
        #error "that specific UART isn't implemented. you found this, your responsible for implementing it."
        // TODO add the UARTs'
        // just copy and paste from the one before, its not yet done because its tedious
        //  to track down all the pins from the data sheet
        // make sure to use the correct version of the UART register struct, see datasheet
    #endif

    /*--- functions ------------------------------------------------------------------------------------------*/

    /* put string to the UART responsible for UI */
    void nputsUIUART(char const * str, uint32_t n);

    /* bring system to immediate stop . requires chip reset to escape this */
    void FailHard(char const * str = nullptr);
}

#endif
