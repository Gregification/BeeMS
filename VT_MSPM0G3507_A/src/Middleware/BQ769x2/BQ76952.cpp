/*
 * BQ76952.cpp
 *
 *  Created on: Aug 13, 2025
 *      Author: FSAE
 */

#include "BQ76952.hpp"
#include "Middleware/BQ769x2/BQ769x2_PROTOCOL.hpp"

bool BQ76952::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd command) {
//    return false;
    return BQ769X2_PROTOCOL::sendCommandSubcommand(spi, cs, command);
}

bool BQ76952::sendSubcommandR(BQ769X2_PROTOCOL::Cmd cmd, uint8_t readOut[32]) {
    return false;
//    return BQ769X2_PROTOCOL::sendSubcommandR(*i2c_controller, i2c_addr, cmd, readOut);
}

bool BQ76952::sendSubcommandW(BQ769X2_PROTOCOL::Cmd cmd, uint8_t data) {
    return false;
//    return BQ769X2_PROTOCOL::sendSubcommandW(*i2c_controller, i2c_addr, cmd, data);
}

bool BQ76952::sendSubcommandW2(BQ769X2_PROTOCOL::Cmd cmd,uint16_t data) {
    return false;
//    return BQ769X2_PROTOCOL::sendSubcommandW2(*i2c_controller, i2c_addr, cmd, data);
}

bool BQ76952::sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt command, uint16_t * readOut) {
    return false;
//    return BQ769X2_PROTOCOL::sendDirectCommandR(*i2c_controller, i2c_addr, command, readOut);
}

bool BQ76952::sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt command, uint16_t data) {
    return false;
//    return BQ769X2_PROTOCOL::sendDirectCommandW(*i2c_controller, i2c_addr, command, data);
}

bool BQ76952::setRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint16_t reg_data, uint8_t datalen) {
    return BQ769X2_PROTOCOL::setRegister(spi, cs, reg_addr, reg_data, datalen);
}

bool BQ76952::getRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint16_t * reg_out){
    return BQ769X2_PROTOCOL::readRegister(spi, cs, reg_addr, reg_out);
}
