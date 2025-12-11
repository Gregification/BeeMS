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
    extern uint8_t id; // slave id . must be uint8_t to meet J1939 CAN standards
    extern uint16_t mcuID; // unique ID of the MCU. different for every chip

    extern BQ76952 bq;
    extern System::GPIO::GPIO const & bqReset;

    void preScheduler_init();
    void postScheduler_init();
};



#endif /* SRC_VTDEVICE_HPP_ */
