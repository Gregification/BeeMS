/*
 * BQ76952.hpp
 *
 *  Created on: Jul 20, 2025
 *      Author: FSAE
 */

#ifndef SRC_MIDDLEWARE_BQ76952_HPP_
#define SRC_MIDDLEWARE_BQ76952_HPP_

#include <stdint.h>

#include "Core/system.hpp"

#include "BQ769x2_PROTOCOL.hpp"


/*
 * a wrapper Class over the C style protocol library
 */
class BQ76952 {
public:
    System::SPI::SPI * const spi;
    System::GPIO::GPIO const * const cs;

    bool sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd cmd);

    bool sendSubcommandR(BQ769X2_PROTOCOL::Cmd cmd, uint8_t readOut[32]);
    bool sendSubcommandW(BQ769X2_PROTOCOL::Cmd cmd, uint8_t data);
    bool sendSubcommandW2(BQ769X2_PROTOCOL::Cmd cmd,uint16_t data);

    bool sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt command, void * readValue, uint8_t datalen);
    bool sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt command, uint16_t data_out, uint8_t datalen);

    /* writes a 16b register of data memory
     * - see bqTM.13.9/198 for register lengths. at most is 32b, so 4
     */
    bool setRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint16_t reg_data, uint8_t datalen);

    /* reads a 16b register of data memory
     * - bqTM.13.9/198
     */
    bool getRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint16_t * reg_data_out);

};


#endif /* SRC_MIDDLEWARE_BQ76952_HPP_ */
