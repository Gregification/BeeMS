/*
 * task_CanPeriodicPackets.cpp
 *
 *  Created on: Mar 27, 2026
 *      Author: turtl
 */

#include "Tasks/task_CanPeriodicPackets.hpp"
#include "Core/system.hpp"
#include "Core/VT.hpp"
#include "BMSComms.hpp"

void Task::task_CanPeriodicPackets(void *) {
    using namespace BMSComms;

    while(true) {
        vTaskDelay(pdMS_TO_TICKS(VT::opProfile.canPacketSpacing_mS));
        { // send status update
            SM_STATUS1_t d;

            for(uint8_t i = 0; i < VT::NUM_BBQs; i++) {
                switch(VT::opVars.bbqs[i].state) {
                    case VT::OpVars_t::BBQ_t::State_t::OFF:
                    case VT::OpVars_t::BBQ_t::State_t::INIT_VERI:
                    case VT::OpVars_t::BBQ_t::State_t::INIT:
                        d.state = BMSCommon::Module::STATE_e::INITING;
                        break;

                    case VT::OpVars_t::BBQ_t::State_t::ON_NORMAL:
                        d.state = BMSCommon::Module::STATE_e::READY;
                        break;

                    case VT::OpVars_t::BBQ_t::State_t::SHUTDOWN_VERI:
                    case VT::OpVars_t::BBQ_t::State_t::SHUTDOWN:
                        d.state = BMSCommon::Module::STATE_e::RECOVERING;
                        break;

                    case VT::OpVars_t::BBQ_t::State_t::ON_ERROR_LATCH:
                    default:
                        d.state = BMSCommon::Module::STATE_e::FAULT;
                        break;
                };
            }

            for(int i = 0; i < VT::NUM_BBQs; i++) {
                // translate device specific safety status to generic one.
                // the compiler gets angry when i do a reinterpert cast. idk
                static_assert(3 <= sizeof(d.ICSafetyStatus[0]));
                static_assert(1 == sizeof(VT::opVars.bbqs[i].safetyStatus.A));
                static_assert(1 == sizeof(VT::opVars.bbqs[i].safetyStatus.B));
                static_assert(1 == sizeof(VT::opVars.bbqs[i].safetyStatus.C));

                d.ICSafetyStatus[i] = 0;
                d.ICSafetyStatus[i] |= VT::opVars.bbqs[i].safetyStatus.A.Raw;
                d.ICSafetyStatus[i] |= VT::opVars.bbqs[i].safetyStatus.B.Raw << 8;
                d.ICSafetyStatus[i] |= VT::opVars.bbqs[i].safetyStatus.C.Raw << 16;
            }
            static_assert(sizeof(d.ICSafetyStatus[0]) >= sizeof(VT::opVars.bbqs[0].safetyStatus));

            sendPacket(J1939_PF_e::SM, PktSM_JS_e::STATUS1, 0, &d, sizeof(d));
        }

        vTaskDelay(pdMS_TO_TICKS(VT::opProfile.canPacketSpacing_mS));
        { // send status update
            SM_STATUS2_t d;

            d.ambient = 0;
            for(int i = 0; i < VT::NUM_BBQs; i++) {
                static_assert(VT::NUM_BBQs == 1); // do quick division here
                d.ambient += VT::opVars.bbqs[i].die_dDegC;
            }

            d.board.avg_dDegC = d.ambient;
            d.board.min_dDegC = d.ambient;
            d.board.max_dDegC = d.ambient;


            sendPacket(J1939_PF_e::SM, PktSM_JS_e::STATUS2, 0, &d, sizeof(d));
        }

        vTaskDelay(pdMS_TO_TICKS(VT::opProfile.canPacketSpacing_mS));
        { // send cell voltages
            SM_CELLV_t d;
            constexpr unsigned int CELL_N = VT::OpVars_t::BBQ_t::MAX_CELLS_N * VT::NUM_BBQs;
            for(uint16_t i = 0; i < CELL_N;) {
                d.base_cell = i;

                if(i + d.MAX_CELL_N < CELL_N)   d.cellCount = d.MAX_CELL_N;
                else                            d.cellCount = CELL_N - i;
                for(uint16_t j = 0; j < d.cellCount; j++) {
                    uint16_t bbq_i = (i+j) / VT::OpVars_t::BBQ_t::MAX_CELLS_N; // bbq index
                    uint16_t bbq_c = (i+j) % VT::OpVars_t::BBQ_t::MAX_CELLS_N; // cell index
                    d.mV[j] = VT::opVars.bbqs[bbq_i].cell_mV[bbq_c];
                }

                i += d.cellCount;
                sendPacket(J1939_PF_e::SM, PktSM_JS_e::CELLV, 0, &d, sizeof(d));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(VT::opProfile.canPacketSpacing_mS));
        { // send cell temperatures
            SM_CELLT_t d;
            constexpr unsigned int CELL_N = VT::OpVars_t::BBQ_t::MAX_CELLS_N * VT::NUM_BBQs;
            for(uint16_t i = 0; i < CELL_N;) {
                d.base_cell = i;

                if(i + d.MAX_CELL_N < CELL_N)   d.cellCount = d.MAX_CELL_N;
                else                            d.cellCount = CELL_N - i;
                for(uint16_t j = 0; j < d.cellCount; j++) {
                    uint16_t bbq_i = (i+j) / VT::OpVars_t::BBQ_t::MAX_CELLS_N; // bbq index
                    d.dDegC[j] = VT::opVars.bbqs[bbq_i].die_dDegC;
                }

                i += d.cellCount;
                sendPacket(J1939_PF_e::SM, PktSM_JS_e::CELLT, 0, &d, sizeof(d));
            }
        }
    }

    vTaskDelete(NULL);
}
