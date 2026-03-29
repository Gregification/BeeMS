#include <FreeRTOS.h>
#include <Middleware/MCP33151/MCP33151.hpp>
#include <task.h>
#include <Tasks/task_CC_sampler.hpp>
#include <cstdint>
#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"



void Task::adc_CC_sampler(void *){
    System::UART::UART & uartt = System::UART::uart_ui;
    uartt.nputs(ARRANDN("adc_CC_sampler start" NEWLINE));

    while(1){
        int16_t imp, per;
        imp = MstrB::MCHS::ADCimpercise.readmV();
        per = MstrB::MCHS::ADCpercise.readmV();

//        uartt.nputs(ARRANDN("adc: "));
//        uartt.putu32d(val);
//        uartt.nputs(ARRANDN(NEWLINE));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    System::FailHard("adc_CC_sampler ended" NEWLINE);
    vTaskDelete(NULL);
}



