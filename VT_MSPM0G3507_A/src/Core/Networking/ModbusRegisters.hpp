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

                STACK_cV                            = 20,   // <input>          stack voltage (10mV)
                CELL1_mV                            = 21,   // <input>          cell voltage (mV)
                CELL2_mV                            ,       // <input>
                CELL3_mV                            ,       // <input>
                CELL4_mV                            ,       // <input>
                CELL5_mV                            ,       // <input>
                CELL6_mV                            ,       // <input>
                CELL7_mV                            ,       // <input>
                CELL8_mV                            ,       // <input>
                CELL9_mV                            ,       // <input>
                CELL10_mV                           ,       // <input>
                CELL11_mV                           ,       // <input>
                CELL12_mV                           ,       // <input>
                CELL13_mV                           ,       // <input>
                CELL14_mV                           ,       // <input>

                DIE_dDegC                           = 70,   // <input>          on chip bbq temperature
                CELL1_dDegC                         ,       // <input>          cell temperature (m-Kelvin)
                CELL2_dDegC                         ,       // <input>
                CELL3_dDegC                         ,       // <input>
                CELL4_dDegC                         ,       // <input>
                CELL5_dDegC                         ,       // <input>
                CELL6_dDegC                         ,       // <input>
                CELL7_dDegC                         ,       // <input>

                CB_MODE_SELECT                      = 90,   // <holding>
                CB_MAX_ACTIVE_CELLS                 ,       // <holding>
                CB_MAN_BY_THERSH_thresh_mV          ,       // <holding>
                CB_MAN_BY_MASK_mask                 ,       // <holding>
                CELL1_CB_active                     = 100,  // <discrete>
                CELL2_CB_active                     ,       // <discrete>
                CELL3_CB_active                     ,       // <discrete>
                CELL4_CB_active                     ,       // <discrete>
                CELL5_CB_active                     ,       // <discrete>
                CELL6_CB_active                     ,       // <discrete>
                CELL7_CB_active                     ,       // <discrete>
                CELL8_CB_active                     ,       // <discrete>
                CELL9_CB_active                     ,       // <discrete>
                CELL10_CB_active                    ,       // <discrete>
                CELL11_CB_active                    ,       // <discrete>
                CELL12_CB_active                    ,       // <discrete>
                CELL13_CB_active                    ,       // <discrete>
                CELL14_CB_active                    ,       // <discrete>

                BQ_SAFETY_STATUS_A                  = 130,  // <input>
                BQ_SAFETY_STATUS_B                  ,       // <input>
                BQ_SAFETY_STATUS_C                  ,       // <input>

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
                /**- data: 16b
                 *      - [15:0] : bit mask of cells that will have balancing activated
                 * - write 0 to disable
                 * - WARNING: will NOT automatically stop balancing
                 */
//                CB_MANUAL_MASK              = VTRegisters::RegAddr::CB_CURRENTLY_ACTIVE_MASK,

            };

            bool command(uint16_t command, uint16_t data);
        }
    }
}



#endif /* SRC_CORE_NETWORKING_MODBUSREGISTERS_HPP_ */
