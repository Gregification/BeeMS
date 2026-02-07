/*
 * MasterBoard.hpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#ifndef SRC_MASTERBOARD_HPP_
#define SRC_MASTERBOARD_HPP_

#include "system.hpp"
#include "Middleware/W5500/wizchip_conf.h"

namespace System {

}

namespace CEB {
    using namespace System;

    /** sets up remaining peripherals & pins not handled by "System::init()"; */
    void init();

    /** Indicator LEDs */
    namespace Indi {
        namespace LED {
            // System::GPIO::PB26; // LP RED
            // System::GPIO::PB27; // LP GREEN
            // System::GPIO::PB22; // LP BLUE

            const GPIO::GPIO
                canRX       = GPIO::PB27,
                ethRX       = GPIO::PB26,
                scheduler   = GPIO::PB22;
        }
    }

    /** CAN<->Ethernet bridge*/
    namespace Bridge {
        extern uint8_t ethBroadcastIP[4];
        extern wiz_NetInfo netConfig;
        const uint8_t wiz_sn = 0;   // [0,8], arbitrary, socket must not be used elsewhere
        static_assert(wiz_sn >= 0 && wiz_sn <= 8);
        extern uint16_t wiz_IP_port;

        extern SPI::SPI & wiz_spi;  // w5500 spi
        const GPIO::GPIO
            wiz_cs      = GPIO::PA15,
            wiz_reset   = GPIO::PA24, // MCU <- W5500
            wiz_irq     = GPIO::PB3; // W5500 -> MCU

        extern CANFD::CANFD & can;
    }

    /** gets a 8b number that represents the board.
     * - unit ID is physically configurable on the board
     * - used by the network to identity instances
     */
    uint8_t getUnitBoardID();
}


#endif /* SRC_MASTERBOARD_HPP_ */
