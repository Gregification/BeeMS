/*
 * fiddle.hpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

#ifndef SRC_CORE_FIDDLE_HPP_
#define SRC_CORE_FIDDLE_HPP_

#include "Core/system.hpp"

namespace Task {

    /** provides a Modbus-TCP <-> CAN interface */
    void ethcan_task(void *);
}

#endif /* SRC_CORE_FIDDLE_HPP_ */
