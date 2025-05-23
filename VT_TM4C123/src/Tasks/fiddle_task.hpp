/*
 * fiddle_task.hpp
 *
 *  Created on: May 22, 2025
 *      Author: FSAE
 */

#ifndef SRC_TASKS_FIDDLE_TASK_HPP_
#define SRC_TASKS_FIDDLE_TASK_HPP_

#include "Core/system.hpp"

#include <FreeRTOS.h>
#include <task.h>


namespace System {
    OCCUPY(PB4);
    OCCUPY(PB5);
}

namespace Task {
    namespace Fiddle {
        void main(void * args);
    };
}


#endif /* SRC_TASKS_FIDDLE_TASK_HPP_ */
