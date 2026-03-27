/*
 * task_CanPeriodicPackets.cpp
 *
 *  Created on: Mar 27, 2026
 *      Author: turtl
 */


#include "Tasks/task_CanPeriodicPackets.hpp"
#include "Core/system.hpp"
#include "Core/VT.hpp"
#include "Core/BMS/BMSComms.hpp"

void Task::task_CanPeriodicPackets(void *) {
    using namespace BMSComms;

    while(true) {
        vTaskDelay(pdMS_TO_TICKS(VT::opProfile.canPacketSpacing_mS));
        { // send status update
            SM_STATUS1_t d;
            d.IL_passing    = VT::opVars.HRLV_IL_sw_dsrd;
            for(int i = 0; i < VT::NUM_BBQs; i++)
                d.safetyStatus[i] = VT::opVars.bbqs[i].safetyStatus;

            sendPacket(J1939_PF_e::SM, PktSM_JS_e::STATUS1, 0, &d, sizeof(d));
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
