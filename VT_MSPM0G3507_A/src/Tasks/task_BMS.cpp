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

int32_t error = 0;

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

    auto & bq = batch.bq;

    switch(batch.state) {

        case OpVars_t::BBQ_t::State_t::INIT: {
                /*
                 * 1. read settings from memory
                 * 2. if crc dosent match use default settings
                 *      - do not write to memory
                 */

                bq.spi.setSCLKTarget(2e6 + 1); //2MHz max

                DL_GPIO_enableOutput(GPIOPINPUX(bq.cs)); // SPI CS
                DL_GPIO_initDigitalOutputFeatures(
                        bq.cs.iomux,
                        DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
                        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                    );
                DL_GPIO_enableOutput(GPIOPINPUX(batch.resetPin)); // BBQ reset
                DL_GPIO_initDigitalOutputFeatures(
                        batch.resetPin.iomux,
                        DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                    );

                bq.cs.clear();
                batch.resetPin.set();

                vTaskDelay(pdMS_TO_TICKS(50)); // CS needs some time to get recognized by the slave

                batch.resetPin.clear();

                vTaskDelay(pdMS_TO_TICKS(10));

                if(!bq.spi.takeResource(pdMS_TO_TICKS(10)))
                    return;
                do{
                    BQ76952::BQ76952SSetting settings;
                    if(!VT::BBQ::recalSetting(idx, &settings)) {
                        settings = VT::BBQ::DEFAULT_BBQ_SETTING;
                    }

                    if(!bq.setConfig(&settings))
                        break;

                    batch.state = OpVars_t::BBQ_t::State_t::INIT_VERI;

                }while(false);
                bq.spi.giveResource();

            } break;

        case OpVars_t::BBQ_t::State_t::INIT_VERI: {
                // idk man, just see if a spi command gets though
                // BQ76952 requires at least 3 cells so just make sure at least the 3 cells seem ok

                if(bq.spi.takeResource(10))
                    break;
                do {
                    bool bad = false;
                    for(int i = 0; i < 3 && !bad; i++) {

                        if(!bq.sendDirectCommandR(
                                (BQ769X2_PROTOCOL::CmdDrt)(BQ769X2_PROTOCOL::CmdDrt::Cell1Voltage + 2 * i),
                                batch.cell_mV + i,
                                sizeof(batch.cell_mV)
                            ))
                            bad = true;

                        if(batch.cell_mV[i] < 1.2e3 || 4.4e3 < batch.cell_mV[i])
                            bad = true;
                    }

                    if(bad)
                        break;

                    batch.state = OpVars_t::BBQ_t::State_t::ON_NORMAL;
                } while(false);
                bq.spi.giveResource();

            } break;

        case OpVars_t::BBQ_t::State_t::ON_NORMAL: {

                if(bq.spi.takeResource(10))
                    break;
                do {
                    for(int i = 0; i < VT::OpVars_t::BBQ_t::MAX_CELLS_N && !error; i++) {

                        if(!bq.sendDirectCommandR(
                                (BQ769X2_PROTOCOL::CmdDrt)(BQ769X2_PROTOCOL::CmdDrt::Cell1Voltage + 2 * i),
                                batch.cell_mV + i,
                                sizeof(batch.cell_mV)
                            )) {
                            error = __LINE__;
                            break;
                        }

//                        if(batch.cell_mV[i] < VT::opProfile.cell_mV_min || VT::opProfile.cell_mV_max < batch.cell_mV[i]){
//                            error = __LINE__;
//                            break;
//                        }
                    }

                    if(error) {
                        batch.state = OpVars_t::BBQ_t::State_t::ON_ERROR_LATCH;
                        break;
                    }

                } while(false);
                bq.spi.giveResource();

            } break;

        case OpVars_t::BBQ_t::State_t::ON_ERROR_LATCH: {
                System::uart_ui.nputs(ARRANDN("task BMS > ON_ERROR_LATCH: "));
                System::uart_ui.put32d(error);
                System::uart_ui.nputs(ARRANDN(NEWLINE));
            } break;

        case OpVars_t::BBQ_t::State_t::SHUTDOWN: {
            } break;

        case OpVars_t::BBQ_t::State_t::SHUTDOWN_VERI: {
            } break;

        case OpVars_t::BBQ_t::State_t::OFF: {
            } break;

        default: System::FailHard("BMS loop, unknown batch state");
    }
}
