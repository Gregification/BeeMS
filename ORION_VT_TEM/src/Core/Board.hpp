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

    enum CAN_MODE {
            A500k_D500k,
            A500k_D2M,
            A1M_D1M,
            A1M_D4M,
        };
    extern CAN_MODE can_mode;
    extern uint8_t id;

    extern CANFD::CANFD & can;

    /** sets up remaining peripherals & pins not handled by "System::init()"; */
    void init();


    namespace UI {
        namespace LED {
            const GPIO::GPIO
                fault       = GPIO::PA6,
                scheduler   = GPIO::PA9;
        }

        namespace SWITCHES {
            const GPIO::GPIO
                cb_1        = GPIO::PA21,
                cb_2        = GPIO::PA22,
                uid_1       = GPIO::PA23;
            extern ADC::ChannelMap const   cm;

            uint8_t getUID();

            CAN_MODE getCANmode();
        }
    }

    namespace Therm {
        struct ThermBank_t {
            constexpr static TickType_t STAB_TIME = pdMS_TO_TICKS(100);

            GPIO::GPIO const        &a,&b,&c;
            ADC::ChannelMap const   cm;
            int32_t degcC[8]; // units of 0.1 degC
            bool error[8];

            void update();
        };

        constexpr unsigned int THERMB_N = 3;
        extern ThermBank_t TB[THERMB_N];

        uint8_t getThermID(uint8_t tb_i, uint8_t t_i);
    }

    void setCanMode(CAN_MODE newmode);
}


#endif /* SRC_MASTERBOARD_HPP_ */
