/*
 * MasterModbusRegisters.hpp
 *
 *  Created on: Dec 2, 2025
 *      Author: turtl
 *
 * mildly long file name but ill fight u on it
 */

#ifndef SRC_TASKS_MASTERMODBUSREGISTERS_HPP_
#define SRC_TASKS_MASTERMODBUSREGISTERS_HPP_

#include <stdint.h>

/**
 * Master board "registers" available on the Modbus interface
 * Modbus register convention
 *      - Holding   : 16b read/write
 *      - Input     : 16b read only . non intuitive name? go pester Schneider Electric
 *      - Coil      : 1b read/write
 *      - Discrete  : 1b read only
 */
namespace Networking {
    namespace Modbus {
        namespace MasterRegisters {

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


#endif /* SRC_TASKS_MASTERMODBUSREGISTERS_HPP_ */
