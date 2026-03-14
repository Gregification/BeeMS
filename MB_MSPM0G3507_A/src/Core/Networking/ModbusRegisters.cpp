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

        case RegAddr::GLV_IL_CTRL_STATUS:   // <coil>
            *out = MstrB::IL::getStatus();
            break;

        case RegAddr::GLV_IL_CTRL_sw_dsrd:   // <coil>
            *out = MstrB::opVars.GLV_IL_RELAY_engage;
            break;

        case RegAddr::GLV_IL_CTRL_usr_dsrd:   // <coil>
            *out = MstrB::opProfile.GLV_IL_RELAY_usr_requested;
            break;

        case RegAddr::GLV_IL_CTRL_usr_ovrd:   // <coil>
            *out = MstrB::opProfile.GLV_IL_RELAY_allow_usr_ovrd;
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

        case RegAddr::GLV_IL_CTRL_usr_dsrd:   // <coil>
            MstrB::opProfile.GLV_IL_RELAY_usr_requested = val;
            break;

        case RegAddr::GLV_IL_CTRL_usr_ovrd:   // <coil>
            MstrB::opProfile.GLV_IL_RELAY_allow_usr_ovrd = val;
            break;

        case RegAddr::GLV_IL_CTRL_STATUS:   // <coil>
            MstrB::opProfile.GLV_IL_RELAY_usr_requested = val;
            break;
    }

    return true;
}

bool Networking::Modbus::MasterCommands::command(uint16_t command, uint16_t data) {
    switch(command) {
        default: return false;

        case CmdAddr::GLV_IL_RELAY_burp: {
                static TaskHandle_t task;
                static void (*func)(void *) = [](void * p) -> void {
                    uint8_t i = GB(1, (uint32_t)p);
                    uint8_t d = GB(0, (uint32_t)p);

                    for(; i > 0; i--) {
                        DL_GPIO_togglePins(GPIOPINPUX(MstrB::IL::_control));
                        vTaskDelay(pdMS_TO_TICKS(d * 10));
                    }

                    MstrB::IL::getStatus(); // reset to proper value

                    task = NULL;
                    vTaskDelete(NULL);
                };

                if(task == NULL || eTaskGetState(task) == eDeleted) {
                    xTaskCreate(func,
                        "GLV_IL_RELAY_burp",
                        configMINIMAL_STACK_SIZE,
                        (void *)data,
                        tskIDLE_PRIORITY+1, //configMAX_PRIORITIES,
                        &task);
                }
            } break;
    }

    return true;
}
