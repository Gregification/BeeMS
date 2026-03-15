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

        case RegAddr::CELL1_mV:     *out = VT::opVars.cell_mV[0]; break;
        case RegAddr::CELL2_mV:     *out = VT::opVars.cell_mV[1]; break;
        case RegAddr::CELL3_mV:     *out = VT::opVars.cell_mV[2]; break;
        case RegAddr::CELL4_mV:     *out = VT::opVars.cell_mV[3]; break;
        case RegAddr::CELL5_mV:     *out = VT::opVars.cell_mV[4]; break;
        case RegAddr::CELL6_mV:     *out = VT::opVars.cell_mV[5]; break;
        case RegAddr::CELL7_mV:     *out = VT::opVars.cell_mV[6]; break;
        case RegAddr::CELL8_mV:     *out = VT::opVars.cell_mV[7]; break;
        case RegAddr::CELL9_mV:     *out = VT::opVars.cell_mV[8]; break;
        case RegAddr::CELL10_mV:    *out = VT::opVars.cell_mV[9]; break;
        case RegAddr::CELL11_mV:    *out = VT::opVars.cell_mV[10]; break;
        case RegAddr::CELL12_mV:    *out = VT::opVars.cell_mV[11]; break;
        case RegAddr::CELL13_mV:    *out = VT::opVars.cell_mV[12]; break;
        case RegAddr::CELL14_mV:    *out = VT::opVars.cell_mV[13]; break;
        static_assert(sizeof(VT::opVars.cell_mV)/sizeof(VT::opVars.cell_mV[0]) == 14);

        case RegAddr::CELL1_mK:     *out = VT::opVars.cell_mK[0]; break;
        case RegAddr::CELL2_mK:     *out = VT::opVars.cell_mK[1]; break;
        case RegAddr::CELL3_mK:     *out = VT::opVars.cell_mK[2]; break;
        case RegAddr::CELL4_mK:     *out = VT::opVars.cell_mK[3]; break;
        case RegAddr::CELL5_mK:     *out = VT::opVars.cell_mK[4]; break;
        case RegAddr::CELL6_mK:     *out = VT::opVars.cell_mK[5]; break;
        case RegAddr::CELL7_mK:     *out = VT::opVars.cell_mK[6]; break;
        static_assert(sizeof(VT::opVars.cell_mK)/sizeof(VT::opVars.cell_mK[0]) == 7);

        case RegAddr::DIE_mK:       *out = VT::opVars.die_mK; break;
    }

    return true;
}

bool Networking::Modbus::VTRegisters::setReg(uint16_t addr, uint16_t val) {
    return false;
}
