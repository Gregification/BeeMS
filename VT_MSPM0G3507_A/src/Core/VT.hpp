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

    constexpr uint8_t NUM_BBQs = 1;

    namespace Indicator {
        const GPIO::GPIO
            scheduler   = GPIO::PA3;
    };

    namespace BBQ {

        const extern BQ76952::BQ76952SSetting DEFAULT_BBQ_SETTING;

        /** write to non volatile storage */
        bool storeSetting(buffersize_t idx, BQ76952::BQ76952SSetting const *);// TODO
        /** read from non volatile storage */
        bool recalSetting(buffersize_t idx, BQ76952::BQ76952SSetting *);// TODO
        /** write to BBQ */
        bool applySetting(BQ76952 &, BQ76952::BQ76952SSetting const *);// TODO
        /** read from BBQ */
        bool retreiveSetting(BQ76952 const &, BQ76952::BQ76952SSetting *);// TODO
    }

    /**
     * operator configuration
     * - use explicit variable names
     */
    struct __attribute__((__packed__)) OpProfile_t {
        bool HRLV_IL_usr_dsrd           : 1;

        bool balancing_enable           : 1;
    };
    extern OpProfile_t opProfile;

    /**
     * operation variables. for internal software use.
     */
    struct __attribute__((__packed__)) OpVars_t {
        struct __attribute__((__packed__)) BBQ_t {
            BQ76952 bq;
            GPIO::GPIO const & resetPin;

            enum State_t : uint8_t {
                INIT,
                INIT_VERI,              // verify init completed successfully
                ON_NORMAL,
                SHUTDOWN,
                SHUTDOWN_VERI,          // verify shutdown completed successfully
                OFF,
            } state = State_t::INIT;

            uint16_t cell_mV[14];
            uint16_t stack_10mV;
            uint16_t cell_mK[7];
            uint16_t die_mK;
            uint16_t cell_balancing_status;     // bit mask of what cells are currently balancing
        } bbqs[NUM_BBQs];
        uint8_t user_selected_BQ;                   // the BQ chip thats selected by the user for edits

        bool HRLV_IL_sw_dsrd            : 1;        // software desired state of IL enable

    };
    extern OpVars_t opVars;

    OpVars_t::BBQ_t & getSelectedBBQ();

    void preScheduler_init();
    void postScheduler_init();

    // slave id . must be uint8_t to meet J1939 CAN standards
    uint8_t getID();
};



#endif /* SRC_VTDEVICE_HPP_ */
