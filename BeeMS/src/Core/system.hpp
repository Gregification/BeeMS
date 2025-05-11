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

/*--- meta ---------------------------------------------*/

#define NEWLINE "\n\r"

#define PROJECT_NAME            "BeeMS"
#define PROJECT_DESCRIPTION     "github.com/Gregification/BeeMS"
#define PROJECT_VERSION         "0.0.0"


/*--- handy shortcuts ----------------------------------*/

#define BV(X) (1 << (X))
#define STRINGIFY(X) #X
#define TOSTRING(X) STRINGIFY(X)

/* if fails BMS will immediately trigger a shutdown */
#define FATAL_ASSERT(X) if(!(X)) System::FailHard("line " TOSTRING(__LINE__) " in " __FILE__);

/*--- configuration ------------------------------------*/

/*------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#include "utils custom/is_base_of_custom.hpp"

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
                    uint32_t GPIO_PIN_CONFIG_UnRX,
                    uint32_t GPIO_PIN_nrx,
                    uint32_t GPIO_PIN_ntx,
                    uint32_t GPIO_PIN_CONFIG_UnTX,
                    uint32_t SYSCTL_PERIPH_UARTn,
                    uint32_t UARTn_BASE,
                    uint32_t UART_CLOCK_src,
                    uint32_t GPIO_PORTn_BASE)
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
        };

        constexpr UART<UART_REG_MFC_MS> uart0(
                UART_REG_MFC_MS(
                    UART_REG_MFC(
                        UART_REG(
                            0,  // GPIO_PIN_CONFIG_UnRX,
                            0,  // GPIO_PIN_nrx,
                            0,  // GPIO_PIN_ntx,
                            0,  // GPIO_PIN_CONFIG_UnTX,
                            0,  // SYSCTL_PERIPH_UARTn,
                            0,  // UARTn_BASE,
                            0,  // UART_CLOCK_src,
                            0   // GPIO_PORTn_BASE
                        )
                    )
                )
            );

    }
}

#endif
