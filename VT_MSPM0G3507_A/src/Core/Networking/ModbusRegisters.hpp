/*
 * ModbusRegisters.hpp
 *
 *  Created on: Dec 11, 2025
 *      Author: turtl
 *
 * - must be run from within a FreeRTOS task
 */

#ifndef SRC_CORE_NETWORKING_MODBUSREGISTERS_HPP_
#define SRC_CORE_NETWORKING_MODBUSREGISTERS_HPP_

#include <stdint.h>

namespace Networking {
    namespace Modbus {
        namespace VTRegisters {
            /**
             * Voltage-Tap/Slave-board "registers" available on the CAN interface.
             * uses Modbus TCP packets, limited to 64B.
             * Modbus register naming convention
             *      - Holding   : 16b read/write
             *      - Input     : 16b read only . non intuitive name? go pester Schneider Electric
             *      - Coil      : 1b read/write
             *      - Discrete  : 1b read only
             */
            enum RegAddr : uint16_t {
                SOFTWARE_VERSION            = 0, // <input>
            };

            /** returns true if is valid operation */
            bool getReg(uint16_t addr, uint16_t * out);

            /** returns true if is valid operation */
            bool setReg(uint16_t addr, uint16_t val);
        }
    }
}



#endif /* SRC_CORE_NETWORKING_MODBUSREGISTERS_HPP_ */
