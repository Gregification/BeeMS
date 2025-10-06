/*
 * CANEthTRXUI.hpp
 *
 *  Created on: Oct 3, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_CANETHTRX_TASK_HPP_
#define SRC_TASKS_CANETHTRX_TASK_HPP_

#include "Core/system.hpp"

namespace System{
    OCCUPY(PA8);    // wiz chip CS
    OCCUPY(PA15);   // wiz chip reset
}

namespace Task {
    void CAN_Eth_TRX_UI_task(void *);
}


#endif /* SRC_TASKS_CANETHTRX_TASK_HPP_ */
