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

    /**
     * performs everything that isn't directly part of the BMS
     * - fan control
     * - uses CAN FIFO 1
     * - UI UART
     */
    void bqCanInterface_task(void *);
}

#endif /* SRC_CORE_FIDDLE_HPP_ */
