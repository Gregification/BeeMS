#include <FreeRTOS.h>
#include <Middleware/MCP33151/MCP33151.hpp>
#include <task.h>
#include <Tasks/task_sampler_packCurrent.hpp>
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
            TickType_t tt = pdMS_TO_TICKS(MstrB::opProfile.MCHS_samplingPeriod_mS);
            if(tt < pdMS_TO_TICKS(5)) {
                tt = pdMS_TO_TICKS(5);
                MstrB::opProfile.MCHS_samplingPeriod_mS = 5;
            }
            vTaskDelay(tt);
//            time_now = xTaskGetTickCount();
//            TickType_t dt = time_now - time_former;
//            TickType_t tt = pdMS_TO_TICKS(MstrB::opProfile.MCHS_samplingPeriod_mS);
//            if(dt < tt) {
//                vTaskDelay(tt - dt);
//                time_former = xTaskGetTickCount();
//            }
        }

        // read ADC & filter
        {
            int32_t imp, per;
            imp = MstrB::MCHS::ADCimpercise.readmV() - MstrB::opProfile.MCHS_impercise_zero_mV;
            per = MstrB::MCHS::ADCpercise.readmV() - MstrB::opProfile.MCHS_percise_zero_mV;

            // you can go bonkers here : https://www.meme.net.au/butterworth.html
            //      and here : https://fiiir.com/
            perf = (perf + per) >> 1; // >>1 ~= 11% , >>2 ~=4.7%
            impf = (impf + imp) >> 1;
        }

        int32_t mA = impf * 400;
        if(mA < 75000 && mA > -75000) { // use precise if within adc range
            mA = perf * 58; // datasheet says it should be 50 but this seems more accurate from testing
        }

        MstrB::opVars.packcurrentmA = mA;

//        uartt.nputs(ARRANDN("imp: "));
//        uartt.put32d(impf);
//        uartt.nputs(ARRANDN(" \t\tper: "));
//        uartt.put32d(perf);
//        uartt.nputs(ARRANDN("\t mA: "));
//        uartt.put32d(mA);
//        uartt.nputs(ARRANDN(NEWLINE));

        // if over absolute max
        if(mA > ((int32_t)MstrB::opProfile.MCHS_maxA_SURGE * 1000)) {
            MstrB::opVars.masterSafteyStatus.pack_OC = true;
//            uartt.nputs(ARRANDN(CLIERROR "FAULT: pack current ("));
//            uartt.put32d(impf);
//            uartt.nputs(ARRANDN(") over absMax ("));
//            uartt.put32d(((int32_t)MstrB::opProfile.MCHS_maxA_SURGE * 1000));
//            uartt.nputs(ARRANDN(")" CLIRESET NEWLINE));
            MstrB::logSnapshot(false);
        }

        static bool surging = false;
        if(mA > ((int32_t)MstrB::opProfile.MCHS_maxA * 1000) && mA < ((int32_t)MstrB::opProfile.MCHS_maxA_SURGE * 1000)) {
            // in surge condition
            static TickType_t surge_start = xTaskGetTickCount();
            if(!surging) {
                surge_start = xTaskGetTickCount();
                surging = true;
            }

            // if surging longer than allowed
            if((xTaskGetTickCount() - surge_start) >= pdMS_TO_TICKS(MstrB::opProfile.MCHS_surge_maxTime_mS)) {
                MstrB::opVars.masterSafteyStatus.pack_OC = true;
                MstrB::logSnapshot(false);
//                uartt.nputs(ARRANDN(CLIERROR "FAULT: pack current ("));
//                uartt.put32d(impf);
//                uartt.nputs(ARRANDN(") surge >("));
//                uartt.put32d(((int32_t)MstrB::opProfile.MCHS_maxA * 1000));
//                uartt.nputs(ARRANDN(")" CLIRESET NEWLINE));
            }
        } else
            surging = false;
    }

    System::FailHard("adc_CC_sampler ended" NEWLINE);
    vTaskDelete(NULL);
}



