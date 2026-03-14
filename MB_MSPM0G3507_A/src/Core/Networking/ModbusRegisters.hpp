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
                MCU_HARDWARE_ID             = 0, // <input>R        MCU silicon level serial number.
                SOFTWARE_VERSION            = 1, // <input>R        software revision.

                LED_INDICATOR1              = 5, // <discrete>R     status of indicator 1 led.
                LED_INDICATOR2              = 6, // <discrete>R     status of indicator 2 led.
                LED_BMS_FAULT               = 7, // <discrete>R     status of fault led.

                GLV_IL_PRESENCE             = 20, // <discrete>R    active if a voltage >19.4V is input side of the devices IL.
                GLV_IL_CTRL_STATUS          = 25, // <coil>R        actual state of GLV IL relay control signal.
                GLV_IL_CTRL_usr_dsrd        = 26, // <coil>RW       user defined state of GLV IL relay. software will prefer this state unless it conflicts with safety.
                GLV_IL_CTRL_usr_ovrd        = 27, // <coil>RW       forces the GLV IL to match the user defined state regardless of safety conflicts.
                GLV_IL_CTRL_sw_dsrd         = 28, // <coil>R        forces the GLV IL to match the user defined state regardless of safety conflicts.

                HRLV_IL_PRESENCE            = 30, // <discrete>R    active if HRLV IL is OK; a voltage >11V is present on the HRLV IL. This IL operates as a common fault bus. Single PU, all nodes are open drain.
                HRLV_PRESENCE               = 31, // <discrete>R    active if HRLV has sufficient power; a voltage >6V is present on the HRLV LV network.

                _end
            };

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
                ABUSE_GLV_IL_RELAY          = MasterRegisters::RegAddr::GLV_IL_CTRL_sw_dsrd, // <coil>          makes click noises
            };

            bool command(uint16_t command, uint16_t data);
        }
    }
}


#endif /* SRC_TASKS_MASTERMODBUSREGISTERS_HPP_ */
