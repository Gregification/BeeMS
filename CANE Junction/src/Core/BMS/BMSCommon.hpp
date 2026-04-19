/*
 * BMSCommon.hpp
 *
 *  Created on: Nov 20, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_BMS_BMSCOMMON_HPP_
#define SRC_CORE_BMS_BMSCOMMON_HPP_

#include "Core/common.h"
#include "FreeRTOS.h"

namespace BMSCommon {
    typedef uint32_t SafteyStatus_t;
    /** units of 0.001 volts ,
     *  capable of representing 1 cell
     */
    typedef uint16_t cellmV_t;

    /** units of 0.01 volts ,
     *  capable of representing the pack
     */
    typedef uint32_t packcV_t;

    /** units of 0.1 mS,
     *  capable of holding time over total run time. without overflow
     */
    typedef uint32_t time_mS_t;
    static_assert(sizeof(time_mS_t) >= sizeof(uint32_t), "calculate max run time. make sure its acceptable");

    /** units of 0.1 degree C
     * capable of representing any temperature involving the BMS */
    typedef int16_t dDegC_t;

    /** abstract representation of a module. a bundle of series cells, not like a 'Enepaq module'.
     *  if u want to mix and match IC's then rewrite for inheritance, this is intended for all physically identical modules
     */
    struct __attribute__((__packed__)) Module {
        /** max modules total in the pack */
        static constexpr uint8_t MAX_MODULES = 8 * 2;

        /** max cells in series */
        static constexpr uint8_t MAX_CELLS = 16;

        /** maximum number IC's per "module". some module settings are IC specific */
        static constexpr uint8_t MAX_ICs = 1;

        static constexpr uint8_t BAD_MODULE_ID = 0;
        /** unique module ID*/
        uint8_t unitID = BAD_MODULE_ID;

        bool enabled : 1 = false;

        /** cell mV */
        cellmV_t cells_mV[MAX_CELLS];
        packcV_t stack_cV;
        dDegC_t cells_dDegC[MAX_CELLS];
        dDegC_t ambient_dDegC;
        dDegC_t max_IC;


        /** bit flags to indicate error of IC.
         * if any of the bits are high a error is triggered and this value is explicitly logged */
        BMSCommon::SafteyStatus_t safetyStatus[MAX_ICs];
        TickType_t lastSafteyStatusUpdate;

        enum STATE_e : uint8_t {
            INITING,    // is starting up
            BALANCING,  // balancing and ready are distinct states to help comply with FSAE rule: no balancing during running or what ever it says.
            READY,      // car go vroom
            RECOVERING, // is doing something, is not faulted but is also not ready
            FAULT,      // needs human intervention
        } state;

    };

    constexpr uint32_t PACK_MAX_CELLS = Module::MAX_CELLS * Module::MAX_MODULES;
};

#endif /* SRC_CORE_BMS_BMSCOMMON_HPP_ */
