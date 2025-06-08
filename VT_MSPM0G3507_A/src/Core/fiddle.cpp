/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

#include "fiddle.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <stdio.h>

#include "system.hpp"

namespace System {
    OCCUPY(PINCM21) //PA10
    OCCUPY(PINCM22) //PA11
}

void fiddle_task(void *){


    vTaskDelete(NULL);
}
