/*
 * MasterBoard.hpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#ifndef SRC_MASTERBOARD_HPP_
#define SRC_MASTERBOARD_HPP_

#include "system.hpp"
#include "Middleware/MCP33151/MCP33151.hpp"
#include "Core/BMS/BMSCommon.hpp"

namespace System {
    //OCCUPY(ADC0); // used by TempSense
}

namespace MstrB {
    using namespace System;

    /**
     * operator configuration
     * - use explicit variable names
     */
    struct __attribute__((__packed__)) OpProfile_t {
        bool GLV_IL_RELAY_allow_usr_ovrd            : 1;
        bool GLV_IL_RELAY_usr_requested             : 1;
        TickType_t maxModuleUpdatePeriod_mS         = 200;
    };
    extern OpProfile_t opProfile;

    /**
     * operation variables. for internal software use.
     */
    struct __attribute__((__packed__)) OpVars_t {
        bool GLV_IL_RELAY_module_dsrd              : 1; // IL desired by module IL
        bool GLV_IL_RELAY_engage                   : 1; // IL desired by software

        BMSCommon::Module modules[BMSCommon::Module::MAX_MODULES];
    };
    extern OpVars_t opVars;

    /** sets up remaining peripherals & pins not handled by "System::init()";
     */
    void init();

    /** Main Current Hall Sensor */
    namespace MCHS {
        extern MCP33151 ADCpercise;
        extern MCP33151 ADCimpercise;

        const uint16_t  ADCReference_100uV = 40960;
        const uint8_t   ADCResolution_b = 14;
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
            _sense       = GPIO::PB7,
            _control     = GPIO::PB8;

        /** returns fails if fails to set to desired value */
        bool setEnable(bool);
        /** returns what the software wants it set to */
        bool getEnable();
        /** returns true if IL is in OK state*/
        bool getStatus();
        /** returns true if IL input is present*/
        bool getInput();
        bool isUserOverriding();
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

    /** is not a exhaustive test.
     * Returns 0 on successful post.
     */
    uint32_t POST(char * error_msg, uint16_t max_msg_len);
}


#endif /* SRC_MASTERBOARD_HPP_ */
