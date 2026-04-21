/*
 * Board.hpp
 *
 *  Created on: April 20, 2025
 *      Author: turtl
 */

#ifndef SRC_MASTERBOARD_HPP_
#define SRC_MASTERBOARD_HPP_

#include "system.hpp"
#include "Middleware/MCP33151/MCP33151.hpp"

namespace Board {
    using namespace System;

    /**
     * inits everything that is not a basic MCU peripheral or board dependent
     */
    void init();

    /** Indicator LEDs */
    namespace LED {
        const GPIO::GPIO
            indicators[]= {GPIO::PA0};
    }

    /** Ethernet interface */
//    namespace Eth {
//        extern SPI::SPI & spi;
//        const GPIO::GPIO
//            cs          = GPIO::PA15,
//            reset       = GPIO::PB14, // MCU <- W5500
//            irq         = GPIO::PB15; // W5500 -> MCU
//    }

    /** is not a exhaustive test.
     * Returns 0 on successful post.
     */
    uint32_t POST(char * error_msg, uint16_t max_msg_len);

}


#endif /* SRC_BOARD_HPP_ */
