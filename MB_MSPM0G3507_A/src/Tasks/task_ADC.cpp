#include <FreeRTOS.h>
#include <Middleware/MCP33151/MCP33151.hpp>
#include <task.h>
#include <Tasks/task_ADC.hpp>
#include <cstdint>
#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"



void Task::adc_task(void *){
    System::UART::UART & uartt = System::UART::uart_ui;
    uartt.nputs(ARRANDN("adc_task start" NEWLINE));

    while(1){
        int16_t val = 0xbeef;
        val = MstrB::MCHS::ADCimpercise.readmV();

        uartt.nputs(ARRANDN("adc: "));
        uartt.putu32d(val);
        uartt.nputs(ARRANDN(NEWLINE));
    }

    System::FailHard("adc_task ended" NEWLINE);
    vTaskDelete(NULL);
}



