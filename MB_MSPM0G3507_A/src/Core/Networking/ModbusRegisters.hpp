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
        namespace MasterRegisters {

            enum RegAddr : uint16_t {
                MCU_HARDWARE_ID             = 0, // <input>         MCU silicon level serial number.
                SOFTWARE_VERSION            = 1, // <input>         software revision.

                LED_INDICATOR1              = 5, // <discrete>      status of indicator 1 led.
                LED_INDICATOR2              = 6, // <discrete>      status of indicator 2 led.
                LED_BMS_FAULT               = 7, // <discrete>      status of fault led.

                GLV_IL_PRESENCE             = 20, // <discrete>     active if a voltage >19.4V is input side of the devices IL.
                GLV_IL_FORWARD              = 25, // <coil>         actual state of GLV IL relay control signal.
                GLV_IL_FORWARD_usr_req      = 26, // <coil>         user defined state of GLV IL relay. software will prefer this state unless it conflicts with safety.
                GLV_IL_FORWARD_override     = 27, // <coil>         forces the GLV IL to match the user defined state regardless of safety conflicts.

                HRLV_IL_PRESENCE            = 30, // <discrete>     active if HRLV IL is OK; a voltage >11V is present on the HRLV IL. This IL operates as a common fault bus. Single PU, all nodes are open drain.
                HRLV_PRESENCE               = 31, // <discrete>     active if HRLV has sufficient power; a voltage >6V is present on the HRLV LV network.
            };

            /** returns true if is valid operation */
            bool getReg(uint16_t addr, volatile uint16_t * out);

            /** returns true if is valid operation */
            bool setReg(uint16_t addr, uint16_t val);
        }

        namespace MasterCommands {
            enum CmdAddr : uint16_t {
                SET_GLV_IL_FORWARD_OVERRIDE      = 20, // <coil>         enables forcing GLV IL status to match user requested value
                SET_GLV_IL_FORWARD_user_req      = 21, // <coil>         sets user defined GLV IL relay state. optionally override safety (see above).
            };

            bool command(uint16_t command, void const * data);
        }
    }
}


#endif /* SRC_TASKS_MASTERMODBUSREGISTERS_HPP_ */
