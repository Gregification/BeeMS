/*
 * MasterBoard.hpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#ifndef SRC_MASTERBOARD_HPP_
#define SRC_MASTERBOARD_HPP_

#include "system.hpp"

namespace System {
    OCCUPY(ADC0); // used by TempSense
}

namespace MstrB {
    using namespace System;

    /** sets up remaining peripherals & pins not handled by "System::init()";
     */
    void init();

    /** Main Current Hall Sensor */
    namespace MHCS {
        extern SPI::SPI & spi;
        const GPIO::GPIO
            cs_precise      = GPIO::PB18,
            cs_imprecise    = GPIO::PB19;
    }

    /** Indicator LEDs */
    namespace Indi {
        namespace LED {
            const GPIO::GPIO
                i1          = GPIO::PA24,
                i2          = GPIO::PA6,
                fault       = GPIO::PA5,
                scheduler   = GPIO::PA23;
        }
    }

    /** Inter-lock */
    namespace IL {
        const GPIO::GPIO
            sense       = GPIO::PB7,
            control     = GPIO::PB8;
    }

    /** on board temperature sensors */
    // TODO: test if adc code actually does anything. do it as a modbus register
    namespace TempSense {
//        extern ADC12_Regs * const adc;

//        const ADC::ChannelMap ts1 = {
//            .mem = DL_ADC12_MEM_IDX::DL_ADC12_MEM_IDX_0,
//            .chan = DL_ADC12_INPUT_CHAN_3,
//        };
    }

    /** High Risk Low Voltage system , aka TSBP LV */
    namespace HRLV {
        const GPIO::GPIO
            presence_HRLV   = GPIO::PB20,
            presence_IL     = GPIO::PB24;
    }

    /** Ethernet interface */
    namespace Eth {
        extern SPI::SPI & spi;
        const GPIO::GPIO
            cs          = GPIO::PA15,
            reset       = GPIO::PB14, // MCU <- W5500
            irq         = GPIO::PB15; // W5500 -> MCU
    }

    namespace FS {
        extern SPI::SPI & spi;
        const GPIO::GPIO
            cs          = GPIO::PB16;
    }

    /** gets a 8b number that represents the board.
     * - unit ID is physically configurable on the board
     * - used by the network to identity instances
     */
    uint8_t getUnitBoardID();
}


#endif /* SRC_MASTERBOARD_HPP_ */
