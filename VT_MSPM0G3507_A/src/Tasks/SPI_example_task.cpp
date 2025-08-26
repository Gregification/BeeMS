/*
 * SPI_example_task.cpp
 *
 *  Created on: Aug 26, 2025
 *      Author: turtl
 */

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <ti/driverlib/driverlib.h>
#include <Tasks/SPI_example_task.hpp>


void Task::SPI_example_task(void *){
    System::uart_ui.nputs(ARRANDN("SPI example task start" NEWLINE));



    System::uart_ui.nputs(ARRANDN("SPI example task end" NEWLINE));
    vTaskDelete(NULL);
}

