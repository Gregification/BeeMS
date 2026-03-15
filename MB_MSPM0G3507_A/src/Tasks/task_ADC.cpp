#include <FreeRTOS.h>
#include <Middleware/MCP33151/MCP33151.hpp>
#include <task.h>
#include <Tasks/task_ADC.hpp>
#include <cstdint>
#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"

System::UART::UART &            uartt = System::UART::uart_ui;

uint16_t val1;
uint16_t val2;

void Task::adc_task(void *){
    uartt.nputs(ARRANDN("adc_task start" NEWLINE));
    while(1)
    {

        if(MstrB::MHCS::calibrationADCImp())
            uartt.nputs(ARRANDN(CLIYES "PASS:"));
        else
            uartt.nputs(ARRANDN(CLINO "FAIL:"));

        vTaskDelay(pdMS_TO_TICKS(10));

        val1 = MstrB::MHCS::readImp_mV();
        uartt.putu32h(val1);

        uartt.nputs(ARRANDN("\t"));

        if(MstrB::MHCS::calibrationADCPer())
            uartt.nputs(ARRANDN(CLIYES "PASS:"));
        else
            uartt.nputs(ARRANDN(CLINO "FAIL:"));

        val2 = MstrB::MHCS::readPer_mV();
        uartt.putu32h(val2);

        vTaskDelay(pdMS_TO_TICKS(10));

        uartt.nputs(ARRANDN(NEWLINE));

    }
    System::FailHard("adc_task ended" NEWLINE);
    vTaskDelete(NULL);
}



