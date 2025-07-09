/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf?ts=1749927951492&ref_url=https%253A%252F%252Fwww.google.com%252F
 */

#include "fiddle.hpp"

#include <FreeRTOS.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <stdio.h>
#include <Core/system.hpp>

// template
/*
    void Task::fiddle_task(void *){
        vTaskDelete(NULL);
    }
*/

void Task::fiddle_task(void *){

    vTaskDelete(NULL);
}
