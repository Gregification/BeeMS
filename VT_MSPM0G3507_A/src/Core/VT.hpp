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
    typedef uint16_t UNIT_uint; // a unsigned integer big enough to act as a bit mask for all cells per BBQ

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

        uint16_t cell_mV_min;
        uint16_t cell_mV_max;

        uint16_t cellB_lower_limit_mV;     // MCU enforced lower limit for CB
        uint8_t cellsBalancingAtOnce_MAX : 5;
    };
    extern OpProfile_t opProfile;

    /**
     * operation variables. for internal software use.
     */
    struct __attribute__((__packed__)) OpVars_t {
        struct __attribute__((__packed__)) BBQ_t {

            static constexpr uint8_t MAX_CELLS_N = 14;

            enum class THERM_IDX : uint8_t {
                TS1,TS2,TS3,
                ALERT,
                DCHG,
                DDSG,
                DFETOFF,

                _end
            };
            static constexpr uint8_t MAX_THERMS_N = (uint8_t)THERM_IDX::_end;
            UNIT_uint therms_100mCl[MAX_THERMS_N];

            BQ76952 bq;
            GPIO::GPIO const & resetPin;

            enum class State_t : uint8_t {
                INIT,
                INIT_VERI,              // verify init completed successfully
                ON_NORMAL,
                ON_ERROR_LATCH,         // latch in this state when error, until some sort of explicit user reset
                SHUTDOWN,
                SHUTDOWN_VERI,          // verify shutdown completed successfully
                OFF,
            } state = State_t::INIT;

            uint8_t const cell_n;
            uint16_t cell_mV[MAX_CELLS_N];
            uint16_t stack_cV;
            uint16_t die_dDegC;                 // degrees celsius (10mCl)

            enum class CB_OP_t : uint8_t {
                DISABLED    = 0,
                MANUAL      = 1,    // manually select cells to balance
                THRESH      = 2,    // balance all to predetermined voltage
                AUTO        = 3,    // let the BQ do its thing
                _end,               // software reference
            };
            CB_OP_t cellB_enabled;             // is cell balancing allowed now?
            UNIT_uint cellB_curr_active;             // bit mask of what cells are currently balancing
            UNIT_uint cellB_man_mask;
            uint16_t cellB_man_thresh_mV;

            struct __attribute__((__packed__)) {
                BQ76952::SafetyStatusA A;
                BQ76952::SafetyStatusB B;
                BQ76952::SafetyStatusC C;
            } safetyStatus;

            uint8_t _strikes;                   // internal counter of how many errors have accumulated
        } bbqs[NUM_BBQs];
        uint8_t user_selected_BQ;               // the BQ chip thats selected by the user for edits

        bool HRLV_IL_sw_dsrd            : 1;    // software desired state of IL enable

    };
    extern OpVars_t opVars;

    OpVars_t::BBQ_t & getSelectedBBQ();

    void preScheduler_init();
    void postScheduler_init();

    // slave id . must be uint8_t to meet J1939 CAN standards
    uint8_t getID();
};



#endif /* SRC_VTDEVICE_HPP_ */
