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

    union __attribute__((__packed__)) MstrSafteyStatus_t {
        BMSCommon::SafteyStatus_t Raw;
        struct __attribute__((__packed__)) {
            bool pack_OC                : 1;
            bool pack_OV                : 1;
            bool pack_module_timeout    : 1;
            bool pack_module_error      : 1;
            uint8_t module              = BMSCommon::Module::BAD_MODULE_ID;
            static_assert(BMSCommon::Module::BAD_MODULE_ID == 0 && sizeof(BMSCommon::Module::BAD_MODULE_ID) == 1);
        };
    };
    static_assert(sizeof(MstrSafteyStatus_t) == sizeof(BMSCommon::SafteyStatus_t));

    /**
     * operator configuration
     * - use explicit variable names
     */
    struct __attribute__((__packed__)) OpProfile_t {
        bool GLV_IL_RELAY_allow_usr_ovrd            : 1;
        bool GLV_IL_RELAY_usr_requested             : 1;
        TickType_t maxModuleUpdatePeriod_mS         = 200;

        int16_t MCHS_percise_zero_mV                : 14;
        int16_t MCHS_impercise_zero_mV              : 14;
        int16_t MCHS_maxA                           ;
        int16_t MCHS_maxA_SURGE                     ;
        uint16_t MCHS_surge_maxTime_mS              ;
        uint8_t MCHS_samplingPeriod_mS              = 10;
    };
    extern OpProfile_t opProfile;

    /**
     * operation variables. for internal software use.
     */
    struct __attribute__((__packed__)) OpVars_t {
        bool GLV_IL_RELAY_module_dsrd              : 1; // IL desired by module IL
        bool GLV_IL_RELAY_engage                   : 1; // IL desired by software

        BMSCommon::Module modules[BMSCommon::Module::MAX_MODULES];
        TickType_t lastStatus[BMSCommon::Module::MAX_MODULES];
        static const TickType_t STATUS_TIMEOUT = pdMS_TO_TICKS(100);

        uint32_t packcurrentmA;
        MstrSafteyStatus_t masterSafteyStatus;
    };
    extern OpVars_t opVars;

    /** sets up remaining peripherals & pins not handled by "System::init()";
     */
    void init();

    /** Main Current Hall Sensor */
    namespace MCHS {
        extern MCP33151 ADCpercise;
        extern MCP33151 ADCimpercise;

        void recalADC();
        void zeroV();
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

    void logSnapshot(bool forceLog);

    /** Retrieves the last reported voltage of cell, 0 based indexing, starting from negative most cell.
     * indexing does not include disabled cells
     *  eg: 2 modules, each with 6 total cells but only 3 enabled . n=5 would be M2#2 not M1#5
     * returns false if cell out of range
     */
    bool getCellmV(uint16_t num, BMSCommon::cellmV_t & out);

    /** Retrieves the last reported temperature of cell, 0 based indexing, starting from negative most cell.
     * temperature may not actually be of cell, is possible to be a interpolated value depending on module implementation
     * indexing does not include disabled cells.
     *  eg: 2 modules, each with 6 total cells but only 3 enabled . n=5 would be M2#2 not M1#5
     * returns false if cell out of range
     */
    bool getCelldC(uint16_t num, BMSCommon::dDegC_t & out);
}


#endif /* SRC_MASTERBOARD_HPP_ */
