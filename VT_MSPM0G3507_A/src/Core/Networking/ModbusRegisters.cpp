/*
 * ModbusRegisters.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: turtl
 */

#include "Core/system.hpp"
#include "ModbusRegisters.hpp"
#include "Core/VT.hpp"

bool Networking::Modbus::VTRegisters::getReg(uint16_t addr, uint16_t * out) {
    if(!out)
        return false;

    /*
     * DO NOT assume enum numbering. be explicit. make it idiot proof.
     */
    switch(addr) {
        default: return false;

        case RegAddr::HW_ID:        *out = System::mcuID; break;
        case RegAddr::SW_VER:       *out = 3; break;

        case RegAddr::CELL_POSITIONS_mask:      *out = VT::getSelectedBBQprof().cellPositionMask; break;
        case RegAddr::STACK_cV:     *out = VT::getSelectedBBQvar().stack_cV; break;
        case RegAddr::CELL1_mV:     *out = VT::getSelectedBBQvar().cell_mV[0]; break;
        case RegAddr::CELL2_mV:     *out = VT::getSelectedBBQvar().cell_mV[1]; break;
        case RegAddr::CELL3_mV:     *out = VT::getSelectedBBQvar().cell_mV[2]; break;
        case RegAddr::CELL4_mV:     *out = VT::getSelectedBBQvar().cell_mV[3]; break;
        case RegAddr::CELL5_mV:     *out = VT::getSelectedBBQvar().cell_mV[4]; break;
        case RegAddr::CELL6_mV:     *out = VT::getSelectedBBQvar().cell_mV[5]; break;
        case RegAddr::CELL7_mV:     *out = VT::getSelectedBBQvar().cell_mV[6]; break;
        case RegAddr::CELL8_mV:     *out = VT::getSelectedBBQvar().cell_mV[7]; break;
        case RegAddr::CELL9_mV:     *out = VT::getSelectedBBQvar().cell_mV[8]; break;
        case RegAddr::CELL10_mV:    *out = VT::getSelectedBBQvar().cell_mV[9]; break;
        case RegAddr::CELL11_mV:    *out = VT::getSelectedBBQvar().cell_mV[10]; break;
        case RegAddr::CELL12_mV:    *out = VT::getSelectedBBQvar().cell_mV[11]; break;
        case RegAddr::CELL13_mV:    *out = VT::getSelectedBBQvar().cell_mV[12]; break;
        case RegAddr::CELL14_mV:    *out = VT::getSelectedBBQvar().cell_mV[13]; break;

        case RegAddr::DIE_dDegC:       *out = VT::getSelectedBBQvar().die_dDegC; break;
        case RegAddr::CELL1_dDegC:     *out = VT::getSelectedBBQvar().therms_100mCl[0]; break;
        case RegAddr::CELL2_dDegC:     *out = VT::getSelectedBBQvar().therms_100mCl[1]; break;
        case RegAddr::CELL3_dDegC:     *out = VT::getSelectedBBQvar().therms_100mCl[2]; break;
        case RegAddr::CELL4_dDegC:     *out = VT::getSelectedBBQvar().therms_100mCl[3]; break;
        case RegAddr::CELL5_dDegC:     *out = VT::getSelectedBBQvar().therms_100mCl[4]; break;
        case RegAddr::CELL6_dDegC:     *out = VT::getSelectedBBQvar().therms_100mCl[5]; break;
        case RegAddr::CELL7_dDegC:     *out = VT::getSelectedBBQvar().therms_100mCl[6]; break;

        case RegAddr::CB_MODE_SELECT:               *out = (uint16_t)VT::getSelectedBBQvar().cellB_enabled; break;
        case RegAddr::CB_MAX_ACTIVE_CELLS:          *out = VT::getSelectedBBQprof().cellsBalancingAtOnce_MAX; break;
        case RegAddr::CB_MAN_BY_MASK_mask:          *out = VT::getSelectedBBQvar().cellB_man_mask; break;
        case RegAddr::CB_MAN_BY_THERSH_thresh_mV:   *out = VT::getSelectedBBQvar().cellB_man_thresh_mV; break;

        case RegAddr::CELL1_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(0); break;
        case RegAddr::CELL2_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(1); break;
        case RegAddr::CELL3_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(2); break;
        case RegAddr::CELL4_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(3); break;
        case RegAddr::CELL5_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(4); break;
        case RegAddr::CELL6_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(5); break;
        case RegAddr::CELL7_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(6); break;
        case RegAddr::CELL8_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(7); break;
        case RegAddr::CELL9_CB_active:      *out = VT::getSelectedBBQvar().cellB_curr_active & BV(8); break;
        case RegAddr::CELL10_CB_active:     *out = VT::getSelectedBBQvar().cellB_curr_active & BV(9); break;
        case RegAddr::CELL11_CB_active:     *out = VT::getSelectedBBQvar().cellB_curr_active & BV(10); break;
        case RegAddr::CELL12_CB_active:     *out = VT::getSelectedBBQvar().cellB_curr_active & BV(11); break;
        case RegAddr::CELL13_CB_active:     *out = VT::getSelectedBBQvar().cellB_curr_active & BV(12); break;
        case RegAddr::CELL14_CB_active:     *out = VT::getSelectedBBQvar().cellB_curr_active & BV(13); break;

        case RegAddr::BQ_SAFETY_STATUS_A:   *out = VT::getSelectedBBQvar().safetyStatus.A.Raw; break;
        case RegAddr::BQ_SAFETY_STATUS_B:   *out = VT::getSelectedBBQvar().safetyStatus.B.Raw; break;
        case RegAddr::BQ_SAFETY_STATUS_C:   *out = VT::getSelectedBBQvar().safetyStatus.C.Raw; break;

    }

    return true;
}

bool Networking::Modbus::VTRegisters::setReg(uint16_t addr, uint16_t val) {
    switch(addr) {
        default:
            System::UART::uart_ui.nputs(ARRANDN("UNKNOWN WRITE REG: "));
            System::UART::uart_ui.putu32d(addr);
            System::UART::uart_ui.nputs(ARRANDN(NEWLINE));
            return false;

        case RegAddr::CELL_POSITIONS_mask: VT::getSelectedBBQprof().cellPositionMask = val; break;

        case RegAddr::CB_MODE_SELECT: {
                static_assert((uint16_t)VT::OpVars_t::BBQ_t::CB_OP_t::_end > 0);
                if(val >= (uint16_t)VT::OpVars_t::BBQ_t::CB_OP_t::_end)
                    val = (uint16_t)VT::OpVars_t::BBQ_t::CB_OP_t::_end - 1;

                VT::getSelectedBBQvar().cellB_enabled = (VT::OpVars_t::BBQ_t::CB_OP_t)val;
            } break;
        case RegAddr::CB_MAX_ACTIVE_CELLS: {
                if(val > 16) val = 16;
                VT::getSelectedBBQprof().cellsBalancingAtOnce_MAX = val;
            } break;
        case RegAddr::CB_MAN_BY_MASK_mask:          VT::getSelectedBBQvar().cellB_man_mask = val; break;
        case RegAddr::CB_MAN_BY_THERSH_thresh_mV:   VT::getSelectedBBQvar().cellB_man_thresh_mV = val; break;
    }

    return true;
}

bool Networking::Modbus::VTCommands::command(uint16_t command, uint16_t data) {
    return false;
}
