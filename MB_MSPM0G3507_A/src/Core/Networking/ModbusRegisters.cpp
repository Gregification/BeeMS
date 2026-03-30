/*
 * MasterModbusRegisters.cpp
 *
 *  Created on: Dec 2, 2025
 *      Author: turtl
 */

#include "Core/Networking/ModbusRegisters.hpp"

#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"
#include "Core/BMS/BMSCommon.hpp"


using namespace Networking::Modbus::MasterRegisters;

bool Networking::Modbus::MasterRegisters::getReg(uint16_t addr, volatile uint16_t * out) {
    if(!out)
        return false;

    switch(addr){

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

        case RegAddr::MCHS_maxA:
            *out = MstrB::opProfile.MCHS_maxA;
            break;

        case RegAddr::MCHS_maxA_surge:
            *out = MstrB::opProfile.MCHS_maxA_SURGE;
            break;

        case RegAddr::MCHS_surge_time_mS:
            *out = MstrB::opProfile.MCHS_surge_maxTime_mS;
            break;

        case RegAddr::MCHS_sampling_period_mS:
            *out = MstrB::opProfile.MCHS_samplingPeriod_mS;
            break;

        case RegAddr::MCHS_pack_current_mA_0_15:
            *out = MstrB::opVars.packcurrentmA & 0xFFFF;
            break;

        case RegAddr::MCHS_pack_current_mA_16_31:
            *out = (MstrB::opVars.packcurrentmA >> 16) & 0xFFFF;
            break;

        case RegAddr::MSTR_SafteyStatus_0_15:
            static_assert(sizeof(BMSCommon::SafteyStatus_t) > 2);
            *out = MstrB::opVars.masterSafteyStatus & 0xFFFF;
            break;

        case RegAddr::MSTR_SafteyStatus_16_31:
            static_assert(sizeof(BMSCommon::SafteyStatus_t) > 2);
            *out = (MstrB::opVars.masterSafteyStatus >> 16) & 0xFFFF;
            break;

        default: {
          if(addr < RegAddr::_modules_uid_end && addr > RegAddr::_modules_uid_start) {
              uint16_t module_i = addr - RegAddr::_modules_uid_start - 1;
              static_assert(0 == RegAddr::M1_unitID - RegAddr::_modules_uid_start - 1);

              BMSCommon::Module & module = MstrB::opVars.modules[module_i];
              *out = module.unitID;
              return true;
          }

          if(addr < RegAddr::_modules_enable_end && addr > RegAddr::_modules_enable_start) {
              uint16_t module_i = addr - RegAddr::_modules_enable_start - 1;
              static_assert(0 == RegAddr::M1_enable - RegAddr::_modules_enable_start - 1);

              BMSCommon::Module & module = MstrB::opVars.modules[module_i];
              *out = module.enabled;
              return true;
          }

          if(addr < RegAddr::_modules_saftey_status_end && addr > RegAddr::_modules_saftey_status_start) {
              uint16_t module_i = addr - RegAddr::_modules_saftey_status_start - 1;
              static_assert(0 == RegAddr::M1_safety_status - RegAddr::_modules_saftey_status_start - 1);

              BMSCommon::Module & module = MstrB::opVars.modules[module_i];
              for(int i = 0; i < BMSCommon::Module::MAX_ICs; i++)
                  if(module.safetyStatus[i]) {
                      // need to squeeze into 16b, just to it non zero if the original is non zero
                      //    user diagnosing should be done on the module level anyways
                      *out = module.safetyStatus[i] | (module.safetyStatus[i] >> 16);
                      return true;
                      static_assert(sizeof(BMSCommon::SafteyStatus_t) == 4);
                  }
              *out = 0;
              return true;
          }

          return 67;
        } break;
    }

    return true;
}

bool Networking::Modbus::MasterRegisters::setReg(uint16_t addr, uint16_t val) {

    switch(addr){
        case RegAddr::GLV_IL_CTRL_usr_dsrd:   // <coil>
            MstrB::opProfile.GLV_IL_RELAY_usr_requested = val;
            break;

        case RegAddr::GLV_IL_CTRL_usr_ovrd:   // <coil>
            MstrB::opProfile.GLV_IL_RELAY_allow_usr_ovrd = val;
            break;

        case RegAddr::GLV_IL_CTRL_STATUS:   // <coil>
            MstrB::opProfile.GLV_IL_RELAY_usr_requested = val;
            break;

        case RegAddr::MCHS_maxA:
            MstrB::opProfile.MCHS_maxA = val;
            break;

        case RegAddr::MCHS_maxA_surge:
            MstrB::opProfile.MCHS_maxA_SURGE = val;
            break;

        case RegAddr::MCHS_surge_time_mS:
            MstrB::opProfile.MCHS_surge_maxTime_mS = val;
            break;

        case RegAddr::MCHS_sampling_period_mS:
            MstrB::opProfile.MCHS_samplingPeriod_mS = val;
            break;

        case RegAddr::MSTR_SafteyStatus_0_15:
            MstrB::opVars.masterSafteyStatus = 0; // reset safety
        case RegAddr::MSTR_SafteyStatus_16_31:
            break;

        default: {

                if(addr < RegAddr::_modules_uid_end && addr > RegAddr::_modules_uid_start) {
                    uint16_t module_i = addr - RegAddr::_modules_uid_start - 1;
                    static_assert(0 == RegAddr::M1_unitID - RegAddr::_modules_uid_start - 1);

                    BMSCommon::Module & module = MstrB::opVars.modules[module_i];
                    module.unitID = val;
                    return true;
                }

                if(addr < RegAddr::_modules_enable_end && addr > RegAddr::_modules_enable_start) {
                    uint16_t module_i = addr - RegAddr::_modules_enable_start - 1;
                    static_assert(0 == RegAddr::M1_enable - RegAddr::_modules_enable_start - 1);

                    BMSCommon::Module & module = MstrB::opVars.modules[module_i];
                    module.enabled = val != 0;
                    return true;
                }

                return false;
            } break;
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

        case CmdAddr::MCHS_zero_adc :
            MstrB::MCHS::zeroV();
            break;
    }

    return true;
}
