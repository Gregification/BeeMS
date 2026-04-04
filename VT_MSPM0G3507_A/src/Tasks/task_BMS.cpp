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
#include "Core/std alternatives/string.hpp"

using namespace System;

// a proper logging system would be the way to go
int32_t error = 0;
char errorStr[MAX_STR_ERROR_LEN];

void loop(VT::OpVars_t::BBQ_t &, uint8_t idx);

void Task::BMS(void *) {
    using namespace VT;

    {
        auto & uart = System::uart_ui;
        vTaskDelay(pdMS_TO_TICKS(10));
        uart.nputs(ARRANDN("BMS_task start" NEWLINE "postScheduler init ..." NEWLINE));

        VT::postScheduler_init();

        uart.nputs(ARRANDN("bms loop start ... " NEWLINE));
    }




    while(true) {
        for(uint8_t i = 0; i < sizeof(VT::OpVars_t::bbqs)/sizeof(VT::OpVars_t::bbqs[0]); i++) {
            loop(VT::opVars.bbqs[i], i);
            vTaskDelay(pdMS_TO_TICKS(100));
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
                batch.resetPin.clear();

                if(!bq.spi.takeResource(pdMS_TO_TICKS(10)))
                    return;
                do{

                    BQ76952::BatteryStatus_t batstat;
                    if(!bq.sendDirectCommandR(
                            BQ769X2_PROTOCOL::CmdDrt::BatteryStatus,
                            &batstat,
                            2
                        )){
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " failed SPI test transaction");
                        break;
                        static_assert(sizeof(batstat) == 2);
                    }


                    if(batstat.SEC == 0) // is not initialized
                        break;

                    System::uart_ui.nputs(ARRANDN("BBQ security state:" NEWLINE));
                    if(batstat.SEC == 3) { // is sealed
                        System::uart_ui.nputs(ARRANDN("SEALED , attempting security key (0x"));
                        System::uart_ui.putu32h(BQ76952::DEFAULT_UNSEAL_KEY);
                        System::uart_ui.nputs(ARRANDN(") ..." NEWLINE));
                        batch.bq.unseal(BQ76952::DEFAULT_UNSEAL_KEY);
                        break;
                    }
                    if(batstat.SEC == 2) {
                        System::uart_ui.nputs(ARRANDN("UNSEALED" NEWLINE));
                    }
                    else if(batstat.SEC == 1) {
                        System::uart_ui.nputs(ARRANDN("FULL ACCESS" NEWLINE));
                    }

                    if(batstat.POR) {
                        BQ76952::BQ76952SSetting settings;
                        if(!VT::BBQ::recalSetting(idx, &settings))
                            settings = VT::BBQ::DEFAULT_BBQ_SETTING;

                        batch.resetPin.set();
                        vTaskDelay(pdMS_TO_TICKS(50)); // CS needs some time to get recognized by the slave
                        batch.resetPin.clear();
                        vTaskDelay(pdMS_TO_TICKS(10));

                        if(!bq.setConfig(&settings))
                            break;
                    }

                    batch.state = OpVars_t::BBQ_t::State_t::INIT_VERI;
                    batch._strikes = 0;
                    System::uart_ui.nputs(ARRANDN(CLIHIGHLIGHT "INIT VERIFICATION" CLIRESET NEWLINE));
                }while(false);
                batch._strikes++;
                bq.spi.giveResource();

                if(error){
                    if(batch._strikes > 10) {
                        batch.state = OpVars_t::BBQ_t::State_t::SHUTDOWN;
                        batch._strikes = 0;
                        System::uart_ui.nputs(ARRANDN("task BMS > "));
                        System::uart_ui.put32d(batch._strikes);
                        System::uart_ui.nputs(ARRANDN(" > INIT error: "));
                        System::uart_ui.nputs(ARRANDN(errorStr));
                        System::uart_ui.nputs(ARRANDN("\t --> "));
                        System::uart_ui.put32d(error);
                        System::uart_ui.nputs(ARRANDN(NEWLINE));
                    }

                    vTaskDelay(pdMS_TO_TICKS(batch._strikes * 10));
                }

            } break;

        case OpVars_t::BBQ_t::State_t::INIT_VERI: {
                // idk man, just see if a spi command gets though
                // BQ76952 requires at least 3 cells so just make sure at least the 3 cells seem ok

                if(bq.spi.takeResource(10))
                    break;

                do {
                    error = 0;

                    //TODO, wait for the INITSTART and INITCOMP bits to do their thing
                    BQ76952::BQ76952SSetting::Alarm_t::DefaultAlarmMask_t alarmstatus;
                    if(!bq.sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt::AlarmRawStatus,
                            &alarmstatus.Raw,
                            2
                        )){
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " failed SPI test transaction");
                        break;
                        static_assert(sizeof(alarmstatus) == 2);
                    }

                    if(!alarmstatus.INITCOMP) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ has not yet completed power up init");
                        break;
                    }

                    if(!alarmstatus.FULLSCAN) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ has not yet completed power up init");
                        break;
                    }

                    batch.state = OpVars_t::BBQ_t::State_t::ON_NORMAL;
                    batch._strikes = 0;
                    System::uart_ui.nputs(ARRANDN(CLIGOOD "NORMAL OPERATION" CLIRESET NEWLINE));
                } while(false);
                bq.spi.giveResource();

                if(error) {
                    System::uart_ui.nputs(ARRANDN("task BMS > "));
                    System::uart_ui.put32d(batch._strikes);
                    System::uart_ui.nputs(ARRANDN(" > INIT_VERI error: "));
                    System::uart_ui.nputs(ARRANDN(errorStr));
                    System::uart_ui.nputs(ARRANDN("\t --> "));
                    System::uart_ui.put32d(error);
                    System::uart_ui.nputs(ARRANDN(NEWLINE));

                    if(batch._strikes > 10) {
                        batch.state = OpVars_t::BBQ_t::State_t::INIT;
                        batch._strikes = 0;
                        System::uart_ui.nputs(ARRANDN(CLIBAD "INIT VERIFICATION FAILED. RESTARTING INIT ... " CLIRESET NEWLINE));
                    } else {
                        vTaskDelay(10 * batch._strikes);
                    }
                }

            } break;

        case OpVars_t::BBQ_t::State_t::ON_NORMAL: {
                uint16_t temp16;
                uint8_t temp8;

                if(bq.spi.takeResource(10))
                    break;
                do {
                    error = 0;

                    BQ76952::BatteryStatus_t batstat;
                    if(!bq.sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt::BatteryStatus,
                            &batstat,
                            2
                        )){
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(batstat) == 2);
                    }

                    switch(batstat.SEC) {
                        case 1: // FULLACCESS
                        case 2: // UNSEALED
                            break;
                        default:
                            batch.state = OpVars_t::BBQ_t::State_t::INIT;
                            batch._strikes = 0;
                            break;
                    }

                    for(int i = 0; i < VT::OpVars_t::BBQ_t::MAX_CELLS_N; i++) {

                        if(!bq.sendDirectCommandR(
                                (BQ769X2_PROTOCOL::CmdDrt)(BQ769X2_PROTOCOL::CmdDrt::Cell1Voltage + 2 * i),
                                batch.cell_mV + i,
                                2
                            )) {
                            error = __LINE__;
                            ALT::srtCpy(ARRANDN(errorStr),  STRM(__LINE__) " BBQ SPI transaction failed");
                            break;
                            static_assert(sizeof(batch.cell_mV[0]) >= 2);
                        }

                        if(!(BV(i) & VT::getSelectedBBQprof().cellPositionMask))
                            continue;

                        if(batch.cell_mV[i] < VT::opProfile.cell_mV_min){
                            error = batch.cell_mV[i];
                            if(!error) error++;
                            ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " cell UV, mV");
                        }

                        if(batch.cell_mV[i] > VT::opProfile.cell_mV_max){
                            error = batch.cell_mV[i];
                            if(!error) error++;
                            ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " cell OV, mV");
                        }
                    }

                    if(!bq.getRegister(BQ769X2_PROTOCOL::RegAddr::VCellMode,
                                &temp16,
                                2
                            )) {
                            error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    if(temp16 != VT::getSelectedBBQprof().cellPositionMask)
                        VT::getSelectedBBQvar().bq.setCellEnableMask(VT::getSelectedBBQprof().cellPositionMask);


                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::StackVoltage,
                            &batch.stack_cV,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr),  STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(batch.stack_cV) >= 2);
                    }

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::IntTemperature,  // 0x68 (100mDegC)
                            &temp16,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.die_dDegC = temp16;

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::TS1Temperature,  // (100mDegC)
                            &temp16,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.therms_100mCl[(UNIT_uint)VT::OpVars_t::BBQ_t::THERM_IDX::TS1] = temp16;

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::TS2Temperature,  // (100mDegC)
                            &temp16,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.therms_100mCl[(UNIT_uint)VT::OpVars_t::BBQ_t::THERM_IDX::TS2] = temp16;

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::TS3Temperature,  // (100mDegC)
                            &temp16,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.therms_100mCl[(UNIT_uint)VT::OpVars_t::BBQ_t::THERM_IDX::TS3] = temp16;

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::ALERTTemperature,  // (100mDegC)
                            &temp16,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.therms_100mCl[(UNIT_uint)VT::OpVars_t::BBQ_t::THERM_IDX::ALERT] = temp16;

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::DCHGTemperature,  // (100mDegC)
                            &temp16,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.therms_100mCl[(UNIT_uint)VT::OpVars_t::BBQ_t::THERM_IDX::DCHG] = temp16;

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::DDSGTemperature,  // (100mDegC)
                            &temp16,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.therms_100mCl[(UNIT_uint)VT::OpVars_t::BBQ_t::THERM_IDX::DDSG] = temp16;

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::DFETOFFTemperature,  // (100mDegC)
                            &temp16,
                            2
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.therms_100mCl[(UNIT_uint)VT::OpVars_t::BBQ_t::THERM_IDX::DFETOFF] = temp16;


                    if(!bq.sendCommandR( BQ769X2_PROTOCOL::Cmd::CB_ACTIVE_CELLS,
                                &temp16,
                                2
                            )) {
                            error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(temp16) >= 2);
                    }
                    batch.cellB_curr_active = temp16;

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::SafetyStatusA,
                            &batch.safetyStatus.A,
                            sizeof(batch.safetyStatus.A)
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(batch.safetyStatus.A) == 1);
                    }

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::SafetyStatusB,
                            &batch.safetyStatus.B,
                            sizeof(batch.safetyStatus.B)
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(batch.safetyStatus.B) == 1);
                    }

                    if(!bq.sendDirectCommandR( BQ769X2_PROTOCOL::CmdDrt::SafetyStatusC,
                            &batch.safetyStatus.C,
                            sizeof(batch.safetyStatus.C)
                        )) {
                        error = __LINE__;
                        ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                        break;
                        static_assert(sizeof(batch.safetyStatus.C) == 1);
                    }

                    do{
                        { // every 500mS
                            static TickType_t former = xTaskGetTickCount();
                            TickType_t t = xTaskGetTickCount();
                            if((t - former) * portTICK_PERIOD_MS < 500)
                                break;
                            former = t;
                        }

                        uint8_t cb_n;
                        if(!bq.getRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMaxCells,
                                    &cb_n,
                                    sizeof(cb_n)
                                )) {
                                error = __LINE__;
                            ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                            break;
                            static_assert(sizeof(cb_n) == 1);
                        }

                        // set proper cell count for balancing
                        switch(batch.cellB_enabled) {
                            default:
                            case VT::OpVars_t::BBQ_t::CB_OP_t::DISABLED:
                                if(cb_n)
                                    batch.bq.setMaxBalCells(0);
                                break;

                            case VT::OpVars_t::BBQ_t::CB_OP_t::THRESH:
                            case VT::OpVars_t::BBQ_t::CB_OP_t::AUTO:
                            case VT::OpVars_t::BBQ_t::CB_OP_t::MANUAL:
                                if(cb_n != VT::getSelectedBBQprof().cellsBalancingAtOnce_MAX)
                                    batch.bq.setMaxBalCells(VT::getSelectedBBQprof().cellsBalancingAtOnce_MAX);
                        }

                        switch(batch.cellB_enabled) {
                            default:
                            case VT::OpVars_t::BBQ_t::CB_OP_t::DISABLED:
                                break;

                            case VT::OpVars_t::BBQ_t::CB_OP_t::MANUAL:
                                if(!bq.sendSubcommandW(BQ769X2_PROTOCOL::Cmd::CB_ACTIVE_CELLS,
                                            batch.cellB_man_mask,
                                            sizeof(batch.cellB_man_mask)
                                        )) {
                                        error = __LINE__;
                                    ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                                    static_assert(sizeof(batch.cellB_man_mask) == 2);
                                }
                                break;

                            case VT::OpVars_t::BBQ_t::CB_OP_t::THRESH:
                                if(!bq.sendSubcommandW(BQ769X2_PROTOCOL::Cmd::CB_SET_LVL,
                                            batch.cellB_man_thresh_mV,
                                            sizeof(batch.cellB_man_thresh_mV)
                                        )) {
                                        error = __LINE__;
                                    ALT::srtCpy(ARRANDN(errorStr), STRM(__LINE__) " BBQ SPI transaction failed");
                                    static_assert(sizeof(batch.cellB_man_thresh_mV) == 2);
                                }
                                break;

                            case VT::OpVars_t::BBQ_t::CB_OP_t::AUTO:
                                // it just does it automatically
                                break;
                        }
                        if(error) break;

                        // other bal stuff

                    } while(false);
                    if(error) break;


                } while(false);
                bq.spi.giveResource();

                if(error) {
//                    batch.state = OpVars_t::BBQ_t::State_t::ON_ERROR_LATCH;
                    System::uart_ui.nputs(ARRANDN("task BMS > "));
                    System::uart_ui.put32d(batch._strikes);
                    System::uart_ui.nputs(ARRANDN(" > NORMAL_ON error: "));
                    System::uart_ui.nputs(ARRANDN(errorStr));
                    System::uart_ui.nputs(ARRANDN("\t --> "));
                    System::uart_ui.put32d(error);
                    System::uart_ui.nputs(ARRANDN(NEWLINE));
                    break;
                }

            } break;

        case OpVars_t::BBQ_t::State_t::ON_ERROR_LATCH: {
                System::uart_ui.nputs(ARRANDN("task BMS > "));
                System::uart_ui.put32d(batch._strikes);
                System::uart_ui.nputs(ARRANDN(" > ON_ERROR_LATCH error: "));
                System::uart_ui.nputs(ARRANDN(errorStr));
                System::uart_ui.nputs(ARRANDN("\t --> "));
                System::uart_ui.put32d(error);
                System::uart_ui.nputs(ARRANDN(NEWLINE));
            } break;

        case OpVars_t::BBQ_t::State_t::SHUTDOWN: {
                System::uart_ui.nputs(ARRANDN("task BMS > "));
                System::uart_ui.put32d(batch._strikes);
                System::uart_ui.nputs(ARRANDN(" > SHUTDOWN error: "));
                System::uart_ui.nputs(ARRANDN(errorStr));
                System::uart_ui.nputs(ARRANDN("\t --> "));
                System::uart_ui.put32d(error);
                System::uart_ui.nputs(ARRANDN(NEWLINE));
                batch.state = OpVars_t::BBQ_t::State_t::SHUTDOWN_VERI;
            } break;

        case OpVars_t::BBQ_t::State_t::SHUTDOWN_VERI: {
                System::uart_ui.nputs(ARRANDN("task BMS > "));
                System::uart_ui.put32d(batch._strikes);
                System::uart_ui.nputs(ARRANDN(" > SHUTDOWN_VERI error: "));
                System::uart_ui.nputs(ARRANDN(errorStr));
                System::uart_ui.nputs(ARRANDN("\t --> "));
                System::uart_ui.put32d(error);
                System::uart_ui.nputs(ARRANDN(NEWLINE));
                batch.state = OpVars_t::BBQ_t::State_t::OFF;
            } break;

        case OpVars_t::BBQ_t::State_t::OFF: {
            System::uart_ui.nputs(ARRANDN("task BMS > "));
            System::uart_ui.put32d(batch._strikes);
            System::uart_ui.nputs(ARRANDN(" > OFF error: "));
            System::uart_ui.nputs(ARRANDN(errorStr));
            System::uart_ui.nputs(ARRANDN("\t --> "));
            System::uart_ui.put32d(error);
            System::uart_ui.nputs(ARRANDN(NEWLINE));
            batch.state = OpVars_t::BBQ_t::State_t::INIT;
            } break;

        default: System::FailHard("BMS loop, unknown batch state");
    }
}
