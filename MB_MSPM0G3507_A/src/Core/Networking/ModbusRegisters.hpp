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
                MCU_HARDWARE_ID             = 0, // <input>
                SOFTWARE_VERSION            = 1, // <input>

                LED_INDICATOR1              = 5, // <discrete>
                LED_INDICATOR2              = 6, // <discrete>
                LED_BMS_FAULT               = 7, // <discrete>

                GLV_IL_PRESENCE             = 20, // <discrete>
                GLV_IL_FORWARD              = 25, // <coil>

                HRLV_IL_PRESENCE            = 30, // <discrete>
                HRLV_PRESENCE               = 31, // <discrete>

            };

            /** returns true if is valid operation */
            bool getReg(uint16_t addr, volatile uint16_t * out);

            /** returns true if is valid operation */
            bool setReg(uint16_t addr, uint16_t val);
        }
    }
}


#endif /* SRC_TASKS_MASTERMODBUSREGISTERS_HPP_ */
