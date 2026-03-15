/*
 * ModbusRegisters.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: turtl
 */

#include "Core/system.hpp"
#include "ModbusRegisters.hpp"

bool Networking::Modbus::VTRegisters::getReg(uint16_t addr, uint16_t * out) {
    if(!out)
        return false;

    switch(addr) {
        default: return false;

        case RegAddr::HW_ID: // <input>
            *out = System::mcuID;
            break;

        case RegAddr::SW_VER: // <input>
            *out = 67;
            break;
    }

    return true;
}

bool Networking::Modbus::VTRegisters::setReg(uint16_t addr, uint16_t val) {
    return false;
}
