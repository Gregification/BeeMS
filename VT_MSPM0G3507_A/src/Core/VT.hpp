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

    extern uint8_t id; // slave id . must be uint8_t to meet J1939 CAN standards

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

    };
    extern OpVars_t opVars;

    extern BQ76952 bq;
    extern System::GPIO::GPIO const & bqReset;

    namespace Indicator {
        const GPIO::GPIO
            scheduler   = GPIO::PA3;
    };

    void preScheduler_init();
    void postScheduler_init();
};



#endif /* SRC_VTDEVICE_HPP_ */
