#include <FreeRTOS.h>
#include <Middleware/MCP33151/MCP33151.hpp>
#include <task.h>
#include <Tasks/task_CC_sampler.hpp>
#include <cstdint>
#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"



void Task::sampler_packCurrent(void *){
    System::UART::UART & uartt = System::UART::uart_ui;
    uartt.nputs(ARRANDN("adc_CC_sampler start" NEWLINE));

    // https://www.lem.com/sites/default/files/products_datasheets/dhab_s161_public_datasheet_v2.pdf
    vTaskDelay(pdMS_TO_TICKS(210)); // wait for power up and stabalization
    MstrB::MCHS::ADCimpercise.calabrate();
    MstrB::MCHS::ADCpercise.calabrate();

    // adc buffers
    int32_t impf, perf;

    TickType_t time_former, time_now;
    time_former = xTaskGetTickCount();

    while(1){
        {
            time_now = xTaskGetTickCount();
            TickType_t dt = time_now - time_former;
            if(dt < pdMS_TO_TICKS(MstrB::opProfile.MCHS_samplingPeriod_mS))
                vTaskDelay(dt - pdMS_TO_TICKS(MstrB::opProfile.MCHS_samplingPeriod_mS));
        }

        // read ADC & filter
        {
            int32_t imp, per;
            imp = MstrB::MCHS::ADCimpercise.readmV();
            per = MstrB::MCHS::ADCpercise.readmV();

            // you can go bonkers here : https://www.meme.net.au/butterworth.html
            //      and here : https://fiiir.com/
            perf = (perf + per) >> 1; // >>1 ~= 11% , >>2 ~=4.7%
            impf = (impf + imp) >> 1;
        }

        int32_t mA = (impf - MstrB::opProfile.MCHS_impercise_zero_mV) * 400;
        if(mA < 75000 && mA > -75000) { // use precise if within adc range
            mA = (perf - MstrB::opProfile.MCHS_percise_zero_mV) * 58; // datasheet says it should be 50 but this seems more accurate from testing
        }

        MstrB::opVars.packcurrentmA = mA;

        static TickType_t surge_start;

        if(impf > MstrB::opProfile.MCHS_maxA_SURGE) {

        }

//        uartt.nputs(ARRANDN("imp: "));
//        uartt.put32d(impf);
//        uartt.nputs(ARRANDN(" \t\tper: "));
//        uartt.put32d(perf);
//        uartt.nputs(ARRANDN("\t mA: "));
//        uartt.put32d(mA);
//        uartt.nputs(ARRANDN(NEWLINE));
    }

    System::FailHard("adc_CC_sampler ended" NEWLINE);
    vTaskDelete(NULL);
}



