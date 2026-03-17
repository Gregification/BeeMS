/*
 * task_BMS.cpp
 *
 *  Created on: Mar 16, 2026
 *      Author: turtl
 */

#include "task_BMS.hpp"
#include "Middleware/BQ769x2/BQ76952.hpp"
#include "Core/system.hpp"
#include "Core/VT.hpp"

using namespace System;

void loop(VT::OpVars_t::BBQ_t &, uint8_t idx);

void Task::BMS(void *) {
    using namespace VT;

    {
        auto & uart = System::uart_ui;
        uart.nputs(ARRANDN("BMS_task start" NEWLINE));
        uart.nputs(ARRANDN("postScheduler init ..." NEWLINE));
    }

    VT::postScheduler_init();

    while(true) {
        for(uint8_t i = 0; i < sizeof(VT::OpVars_t::bbqs)/sizeof(VT::OpVars_t::bbqs[0]); i++) {
            loop(VT::opVars.bbqs[i], i);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    System::FailHard("BMS_task ended" NEWLINE);
    vTaskDelete(NULL);
}

void loop(VT::OpVars_t::BBQ_t & batch, uint8_t idx) {
    using namespace VT;
    using namespace VT::BBQ;

    switch(batch.state) {

        case OpVars_t::BBQ_t::State_t::INIT: {
                /*
                 * 1. read settings from memory
                 * 2. if crc dosent match use default settings
                 *      - do not write to memory
                 */

                BQ76952::BQ76952SSetting settings;
                if(!VT::BBQ::recalSetting(idx, &settings)) {
                    settings = VT::BBQ::DEFAULT_BBQ_SETTING;
                }



            } break;

        case OpVars_t::BBQ_t::State_t::ON_NORMAL: {
            } break;

        case OpVars_t::BBQ_t::State_t::SHUTDOWN: {
            } break;

        case OpVars_t::BBQ_t::State_t::OFF: {
            } break;

        default: System::FailHard("BMS loop, unknown batch state");
    }
}
