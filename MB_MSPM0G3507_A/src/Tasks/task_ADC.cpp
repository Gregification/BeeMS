#include <FreeRTOS.h>
#include <task.h>
#include <Tasks/task_ADC.hpp>
#include <cstdint>
#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"
#include "Middleware/MCP33151/adc.hpp"

System::UART::UART &            uartt = System::UART::uart_ui;

uint16_t val1;
uint16_t val2;

void Task::adc_task(void *){
    uartt.nputs(ARRANDN("adc_task start" NEWLINE));
    while(1)
    {
        uartt.nputs(ARRANDN("PRECISE: "));
        val1 = ADC::read_precise_adc();
        uartt.putu32d(val1 * 250e-6);
        uartt.nputs(ARRANDN("V" NEWLINE));

        uartt.nputs(ARRANDN("IMPRECISE: "));
        val2 = ADC::read_imprecise_adc();
        uartt.putu32d(val2 * 250e-6);
        uartt.nputs(ARRANDN("V" NEWLINE));

    }
    System::FailHard("adc_task ended" NEWLINE);
    vTaskDelete(NULL);
}



