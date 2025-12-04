/*
 * MasterModbusRegisters.cpp
 *
 *  Created on: Dec 2, 2025
 *      Author: turtl
 */

#include "MasterModbusRegisters.hpp"

#include "Core/system.hpp"

using namespace Networking::Modbus::MasterRegisters;

bool Networking::Modbus::MasterRegisters::getReg(uint16_t addr, uint16_t * out) {
    if(!out)
        return false;

    switch(addr){
        default: return false;

        case RegAddr::SOFTWARE_VERSION: // <input>
            *out = PROJECT_VERSION_N;
            break;
    }

    return true;
}

bool Networking::Modbus::MasterRegisters::setReg(uint16_t addr, uint16_t val) {
    return false;
}
