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
 */

#ifndef SRC_CORE_SYSTEM_HPP_
#define SRC_CORE_SYSTEM_HPP_

#include <stdint.h>
#include <stdbool.h>

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
#include <FreeRTOS.h>

#include "utils custom/is_base_of_custom.hpp"


/*--- meta ---------------------------------------------*/

#define NEWLINE "\n\r"

#define PROJECT_NAME            "BeeMS"
#define PROJECT_DESCRIPTION     "github.com/Gregification/BeeMS"
#define PROJECT_VERSION         "0.0.0"


/*--- shorthand ----------------------------------------*/

#define BV(X) (1 << (X))
#define STRINGIFY(X) #X
#define TOSTRING(X) STRINGIFY(X)

/* if fails BMS will immediately trigger a shutdown */
#define FATAL_ASSERT(X) if(!(X)) System::FailHard("line " TOSTRING(__LINE__) " in " __FILE__);

/*--- configuration ------------------------------------*/

// OCCUPY macro defines a static const variable with a unique name per PIN
// If the same PIN is used again in the same translation unit, it will cause redefinition error
// IMPORTANT: this macro will only work with things in the same scope!
#define OCCUPY(ID) constexpr int const __PROJECT_USE_##ID = 0;

#define PROJECT_ENABLE_UART0
//#define PROJECT_ENABLE_UART1
//#define PROJECT_ENABLE_UART2
//#define PROJECT_ENABLE_UART3
//#define PROJECT_ENABLE_UART4
//#define PROJECT_ENABLE_UART5
//#define PROJECT_ENABLE_UART6
//#define PROJECT_ENABLE_UART7

/*------------------------------------------------------*/

namespace System {
    struct LOCKABLE {
        // TODO, some freeRTOS thread logic or something
    };

    /** CPU clock speed (Hz) */
    extern uint32_t CPU_FREQ;

    /* bring system to immediate stop . requires chip reset to escape this */
    void FailHard(char const * str = nullptr);

    namespace UART {
        constexpr uint32_t BAUD_UI = 115200;

        struct UART_REG {
            /* e.g: GPIO_PA0_U0RX */
            uint32_t GPIO_PIN_CONFIG_UnRX;

            /* e.g: GPIO_PIN_0 */
            uint32_t GPIO_PIN_nrx;

            /* e.g: GPIO_PIN_1 */
            uint32_t GPIO_PIN_ntx;

            /* e.g: GPIO_PA0_U0TX */
            uint32_t GPIO_PIN_CONFIG_UnTX;

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

        /* modem flow control */
        struct UART_REG_MFC : UART_REG {
            // TODO

            constexpr UART_REG_MFC(UART_REG const reg) : UART_REG(reg) {}
        };

        /* modem flow control and modem status */
        struct UART_REG_MFC_MS : UART_REG_MFC {
            // TODO

            constexpr UART_REG_MFC_MS(UART_REG_MFC const reg) : UART_REG_MFC(reg) {}
        };

        template <typename UART_TYPE>
        struct UART : LOCKABLE {
            static_assert(is_base_of_custom<UART_TYPE, UART_REG>::value, "must inherit from UART_REG");

            UART_TYPE regs;

            constexpr UART(UART_TYPE const r) : regs(r) {}

            /* a partial init */
            void preinit();
        };
    }

    #ifdef PROJECT_ENABLE_UART0
        OCCUPY(PA0);
        OCCUPY(PA1);
        constexpr UART::UART<UART::UART_REG_MFC_MS> uart0(
                UART::UART_REG_MFC_MS(
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
                )
            );
    #endif
    // TODO add the other 7 uarts
}

#undef OCCUPY

#endif
