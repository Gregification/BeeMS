/*
 * MasterModbusRegisters.hpp
 *
 *  Created on: Dec 2, 2025
 *      Author: turtl
 *
 * - mildly long file name but ill fight u on it
 * - must be run from within a FreeRTOS task
 */

#ifndef SRC_TASKS_MASTERMODBUSREGISTERS_HPP_
#define SRC_TASKS_MASTERMODBUSREGISTERS_HPP_

#include <stdint.h>
#include "Core/BMS/BMSCommon.hpp"

/**
 * Master board "registers" available on the Modbus interface
 * Modbus register naming convention
 *      - Holding   : 16b read/write
 *      - Input     : 16b read only . non intuitive name? go pester Schneider Electric
 *      - Coil      : 1b read/write
 *      - Discrete  : 1b read only
 */
namespace Networking {
    namespace Modbus {
        /**
         * for the love of god use explicit numbering
         */

        namespace MasterRegisters {

            enum RegAddr : uint16_t {
                MCU_HARDWARE_ID             = 0, // <input>         MCU silicon level serial number.
                SOFTWARE_VERSION            = 1, // <input>         software revision.

                LED_INDICATOR1              = 5, // <discrete>      status of indicator 1 led.
                LED_INDICATOR2              = 6, // <discrete>      status of indicator 2 led.
                LED_BMS_FAULT               = 7, // <discrete>      status of fault led.

                GLV_IL_PRESENCE             = 20, // <discrete>     active if a voltage >19.4V is input side of the devices IL.
                GLV_IL_CTRL_STATUS          = 25, // <discrete>     actual state of GLV IL relay control signal.
                GLV_IL_CTRL_usr_dsrd        = 26, // <coil>         user defined state of GLV IL relay. software will prefer this state unless it conflicts with safety.
                GLV_IL_CTRL_usr_ovrd        = 27, // <coil>         forces the GLV IL to match the user defined state regardless of safety conflicts.
                GLV_IL_CTRL_sw_dsrd         = 28, // <discrete>     forces the GLV IL to match the user defined state regardless of safety conflicts.

                HRLV_IL_PRESENCE            = 30, // <discrete>     active if HRLV IL is OK; a voltage >11V is present on the HRLV IL. This IL operates as a common fault bus. Single PU, all nodes are open drain.
                HRLV_PRESENCE               = 31, // <discrete>     active if HRLV has sufficient power; a voltage >6V is present on the HRLV LV network.

                _modules_uid_start          = 50,
                M1_unitID                   ,       // <holding>
                M2_unitID                   ,
                M3_unitID                   ,
                M4_unitID                   ,
                M5_unitID                   ,
                M6_unitID                   ,
                M7_unitID                   ,
                M8_unitID                   ,
                M9_unitID                   ,
                M10_unitID                  ,
                M11_unitID                  ,
                M12_unitID                  ,
                M13_unitID                  ,
                M14_unitID                  ,
                M15_unitID                  ,
                M16_unitID                  ,
                _modules_uid_end            ,
                _modules_enable_start       = 80,
                M1_enable                   ,       // <coil>
                M2_enable                   ,
                M3_enable                   ,
                M4_enable                   ,
                M5_enable                   ,
                M6_enable                   ,
                M7_enable                   ,
                M8_enable                   ,
                M9_enable                   ,
                M10_enable                  ,
                M11_enable                  ,
                M12_enable                  ,
                M13_enable                  ,
                M14_enable                  ,
                M15_enable                  ,
                M16_enable                  ,
                _modules_enable_end         ,
                _modules_saftey_status_start= 100,
                M1_safety_status            ,       // <input>
                M2_safety_status            ,
                M3_safety_status            ,
                M4_safety_status            ,
                M5_safety_status            ,
                M6_safety_status            ,
                M7_safety_status            ,
                M8_safety_status            ,
                M9_safety_status            ,
                M10_safety_status           ,
                M11_safety_status           ,
                M12_safety_status           ,
                M13_safety_status           ,
                M14_safety_status           ,
                M15_safety_status           ,
                M16_safety_status           ,
                _modules_saftey_status_end  ,

                _end
            };
            static_assert(BMSCommon::Module::MAX_MODULES == RegAddr::_modules_uid_end - RegAddr::_modules_uid_start - 1); // -1 for enum offset
            static_assert(BMSCommon::Module::MAX_MODULES == RegAddr::_modules_enable_end - RegAddr::_modules_enable_start - 1);
            static_assert(BMSCommon::Module::MAX_MODULES == RegAddr::_modules_saftey_status_end - RegAddr::_modules_saftey_status_start - 1);

            /** returns true if is valid operation */
            bool getReg(uint16_t addr, volatile uint16_t * out);

            /** returns true if is valid operation */
            bool setReg(uint16_t addr, uint16_t val);
        }

        /**
         * commands are a rapidSCADA definition, they are essentially write only registers.
         * i keep them separate from the other register enum for organization reasons.
         * - commands are write operations that trigger complex behavior
         * - all commands should map to a read only register
         * - COMMAND NUMBERS CANNOT CONFLIT WITH REGISTER NUMBERS
         */
        namespace MasterCommands {
            enum CmdAddr : uint16_t {
                /** IL burping
                 * - data: 16b
                 *      - [0B] : delay between transitions. "x * 10mS".
                 *      - [1B] : number of transitions.
                 */
                GLV_IL_RELAY_burp          = MasterRegisters::RegAddr::GLV_IL_CTRL_sw_dsrd,

            };

            bool command(uint16_t command, uint16_t data);
        }
    }
}


#endif /* SRC_TASKS_MASTERMODBUSREGISTERS_HPP_ */
