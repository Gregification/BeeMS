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

    bool sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt command, uint16_t * readValue);
    bool sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt command, uint16_t data);

    bool setRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint32_t reg_data, uint8_t datalen);

};


#endif /* SRC_MIDDLEWARE_BQ76952_HPP_ */
