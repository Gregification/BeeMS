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

        case RegAddr::STACK_10mV:   *out = VT::getSelectedBBQ().stack_10mV; break;
        case RegAddr::CELL1_mV:     *out = VT::getSelectedBBQ().cell_mV[0]; break;
        case RegAddr::CELL2_mV:     *out = VT::getSelectedBBQ().cell_mV[1]; break;
        case RegAddr::CELL3_mV:     *out = VT::getSelectedBBQ().cell_mV[2]; break;
        case RegAddr::CELL4_mV:     *out = VT::getSelectedBBQ().cell_mV[3]; break;
        case RegAddr::CELL5_mV:     *out = VT::getSelectedBBQ().cell_mV[4]; break;
        case RegAddr::CELL6_mV:     *out = VT::getSelectedBBQ().cell_mV[5]; break;
        case RegAddr::CELL7_mV:     *out = VT::getSelectedBBQ().cell_mV[6]; break;
        case RegAddr::CELL8_mV:     *out = VT::getSelectedBBQ().cell_mV[7]; break;
        case RegAddr::CELL9_mV:     *out = VT::getSelectedBBQ().cell_mV[8]; break;
        case RegAddr::CELL10_mV:    *out = VT::getSelectedBBQ().cell_mV[9]; break;
        case RegAddr::CELL11_mV:    *out = VT::getSelectedBBQ().cell_mV[10]; break;
        case RegAddr::CELL12_mV:    *out = VT::getSelectedBBQ().cell_mV[11]; break;
        case RegAddr::CELL13_mV:    *out = VT::getSelectedBBQ().cell_mV[12]; break;
        case RegAddr::CELL14_mV:    *out = VT::getSelectedBBQ().cell_mV[13]; break;

        case RegAddr::CELL1_mK:     *out = VT::getSelectedBBQ().cell_10mCl[0] / 100; break;
        case RegAddr::CELL2_mK:     *out = VT::getSelectedBBQ().cell_10mCl[1] / 100; break;
        case RegAddr::CELL3_mK:     *out = VT::getSelectedBBQ().cell_10mCl[2] / 100; break;
        case RegAddr::CELL4_mK:     *out = VT::getSelectedBBQ().cell_10mCl[3] / 100; break;
        case RegAddr::CELL5_mK:     *out = VT::getSelectedBBQ().cell_10mCl[4] / 100; break;
        case RegAddr::CELL6_mK:     *out = VT::getSelectedBBQ().cell_10mCl[5] / 100; break;
        case RegAddr::CELL7_mK:     *out = VT::getSelectedBBQ().cell_10mCl[6] / 100; break;

        case RegAddr::DIE_mK:       *out = VT::getSelectedBBQ().die_10mCl  / 100; break;
    }

    return true;
}

bool Networking::Modbus::VTRegisters::setReg(uint16_t addr, uint16_t val) {
    return false;
}
