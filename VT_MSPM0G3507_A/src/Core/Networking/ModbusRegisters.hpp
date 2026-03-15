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
                HW_ID                               = 0,    // <input>
                SW_VER                              = 1,    // <input>

                CELL1_mV                            = 20,   // <input>          cell voltage in mV
                CELL2_mV                            = 21,   // <input>
                CELL3_mV                            = 22,   // <input>
                CELL4_mV                            = 23,   // <input>
                CELL5_mV                            = 24,   // <input>
                CELL6_mV                            = 25,   // <input>
                CELL7_mV                            = 26,   // <input>
                CELL8_mV                            = 27,   // <input>
                CELL9_mV                            = 28,   // <input>
                CELL10_mV                           = 29,   // <input>
                CELL11_mV                           = 30,   // <input>
                CELL12_mV                           = 31,   // <input>
                CELL13_mV                           = 32,   // <input>
                CELL14_mV                           = 33,   // <input>

                CELL1_mK                            = 70,   // <input>          cell temperature in m-Kelvin
                CELL2_mK                            = 71,   // <input>
                CELL3_mK                            = 72,   // <input>
                CELL4_mK                            = 73,   // <input>
                CELL5_mK                            = 74,   // <input>
                CELL6_mK                            = 75,   // <input>
                CELL7_mK                            = 76,   // <input>

                _end                                ,       // software reference
            };

            /** returns true if is valid operation */
            bool getReg(uint16_t addr, uint16_t * out);

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
        namespace VTCommands {
            enum CmdAddr : uint16_t {
                /** IL burping
                 * - data: 16b
                 *      - [0B] : delay between transitions. "x * 10mS".
                 *      - [1B] : number of transitions.
                 */
                //GLV_IL_RELAY_burp          = MasterRegisters::RegAddr::GLV_IL_CTRL_sw_dsrd,

            };

            bool command(uint16_t command, uint16_t data);
        }
    }
}



#endif /* SRC_CORE_NETWORKING_MODBUSREGISTERS_HPP_ */
