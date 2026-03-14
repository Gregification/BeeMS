/*
 * MasterModbusRegisters.cpp
 *
 *  Created on: Dec 2, 2025
 *      Author: turtl
 */

#include "Core/Networking/ModbusRegisters.hpp"

#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"


using namespace Networking::Modbus::MasterRegisters;

bool Networking::Modbus::MasterRegisters::getReg(uint16_t addr, volatile uint16_t * out) {
    if(!out)
        return false;

    switch(addr){
        default: return false;

        case RegAddr::MCU_HARDWARE_ID: // <input>
            *out = System::mcuID;
            break;

        case RegAddr::SOFTWARE_VERSION: // <input>
            *out = 1100;
            break;

        case RegAddr::LED_INDICATOR1:   // <discrete>
            *out = MstrB::Indi::LED::i1.getOutput();
            break;

        case RegAddr::LED_INDICATOR2:   // <discrete>
            *out = MstrB::Indi::LED::i2.getOutput();
            break;

        case RegAddr::LED_BMS_FAULT:    // <discrete>
            *out = MstrB::Indi::LED::fault.getOutput();
            break;

        case RegAddr::GLV_IL_PRESENCE:  // <discrete>
            *out = MstrB::IL::getInput();
            break;

        case RegAddr::GLV_IL_FORWARD:   // <coil>
            *out = MstrB::IL::getStatus();
            break;

        case RegAddr::HRLV_IL_PRESENCE: // <discrete>
            *out = MstrB::HRLV::presence_IL.get();
            break;

        case RegAddr::HRLV_PRESENCE:    // <discrete>
            *out = MstrB::HRLV::presence_HRLV.get();
            break;
    }

    return true;
}

bool Networking::Modbus::MasterRegisters::setReg(uint16_t addr, uint16_t val) {

    switch(addr){
        default: return false;

        case RegAddr::GLV_IL_FORWARD:   // <coil>
            MstrB::opProfile.GLV_IL_RELAY_usr_requested = val;
            break;
    }

    return true;
}

bool Networking::Modbus::MasterCommands::command(uint16_t command, void const * data) {
    switch(command) {
        default: return false;

        case CmdAddr::SET_GLV_IL_FORWARD_OVERRIDE:
            MstrB::opProfile.GLV_IL_RELAY_allow_usr_ovrd = !data ? false : *(bool const *)data;
            break;

        case CmdAddr::SET_GLV_IL_FORWARD_user_req:
            MstrB::opProfile.GLV_IL_RELAY_usr_requested = !data ? false : *(bool const *)data;
            break;
    }

    return true;
}
