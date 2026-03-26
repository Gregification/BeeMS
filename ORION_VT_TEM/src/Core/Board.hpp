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

namespace BOARD {
    using namespace System;

    extern CANFD::CANFD & can;

    /** sets up remaining peripherals & pins not handled by "System::init()"; */
    void init();

    namespace UI {
        namespace LED {
            const GPIO::GPIO
                fault       = GPIO::PA9,
                scheduler   = GPIO::PA6;
        }

        namespace SWITCHES {
            const GPIO::GPIO
                cb_1        = GPIO::PA21,
                cb_2        = GPIO::PA22,
                uid_1       = GPIO::PA23;
            extern ADC::ChannelMap const   cm;

            uint8_t getSelectionMask();
        }
    }

    namespace Therm {
        struct ThermBank_t {
            GPIO::GPIO const        &a,&b,&c;
            ADC::ChannelMap const   cm;
            uint16_t degcC[8]; // units of 0.1 degC
        };

        constexpr unsigned int THERM_N = 3;
        extern ThermBank_t TB[THERM_N];
    }
}


#endif /* SRC_MASTERBOARD_HPP_ */
