/*
 * VTDevice.hpp
 *
 *  Created on: Dec 9, 2025
 *      Author: turtl
 */

#ifndef SRC_VTDEVICE_HPP_
#define SRC_VTDEVICE_HPP_

#include "Core/common.h"
#include "system.hpp"
#include "Middleware/BQ769x2/BQ76952.hpp"

/** variables, functions, stuff... focused on the voltage tap. exists for organizational reasons */
namespace VT {
    using namespace System;

    /**
     * operator configuration
     * - use explicit variable names
     */
    struct __attribute__((__packed__)) OpProfile_t {

    };
    extern OpProfile_t opProfile;

    /**
     * operation variables. for internal software use.
     */
    struct __attribute__((__packed__)) OpVars_t {
        uint16_t cell_mV[14];
        uint16_t stack_10mV;
        uint16_t cell_mK[7];
        uint16_t die_mK;

        bool HRLV_IL_sw_dsrd            : 1;        // software desired state of IL enable
    };
    extern OpVars_t opVars;

    namespace Indicator {
        const GPIO::GPIO
            scheduler   = GPIO::PA3;
    };

    namespace BBQ {

        struct BBQ_t {
            BQ76952 bq;
            GPIO::GPIO const & resetPin;
        };

        extern BBQ_t bbqs[1];
        extern const uint8_t bbqs_n;

        /** write to non volatile storage */
        bool storeSetting(buffersize_t idx, BQ76952::BQ76952SSetting const *);// TODO
        /** read from non volatile storage */
        bool recalSetting(buffersize_t idx, BQ76952::BQ76952SSetting *);// TODO
        /** write to BBQ */
        bool applySetting(BQ76952 &, BQ76952::BQ76952SSetting const *);// TODO
        /** read from BBQ */
        bool retreiveSetting(BQ76952 const &, BQ76952::BQ76952SSetting *);// TODO
    }

    void preScheduler_init();
    void postScheduler_init();

    // slave id . must be uint8_t to meet J1939 CAN standards
    uint8_t getID();
};



#endif /* SRC_VTDEVICE_HPP_ */
