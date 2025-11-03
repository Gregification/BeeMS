/*
 * BQ76952.cpp
 *
 *  Created on: Aug 13, 2025
 *      Author: FSAE
 */

#include "BQ76952.hpp"
#include "Middleware/BQ769x2/BQ769x2_PROTOCOL.hpp"

bool BQ76952::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd command) {
    return BQ769X2_PROTOCOL::sendCommandSubcommand(*i2c_controller, i2c_addr, command);
}

bool BQ76952::sendSubcommandR(BQ769X2_PROTOCOL::Cmd cmd, uint8_t readOut[32]) {
    return BQ769X2_PROTOCOL::sendSubcommandR(*i2c_controller, i2c_addr, cmd, readOut);
}

bool BQ76952::sendSubcommandW(BQ769X2_PROTOCOL::Cmd cmd, uint8_t data) {
    return BQ769X2_PROTOCOL::sendSubcommandW(*i2c_controller, i2c_addr, cmd, data);
}

bool BQ76952::sendSubcommandW2(BQ769X2_PROTOCOL::Cmd cmd,uint16_t data) {
    return BQ769X2_PROTOCOL::sendSubcommandW2(*i2c_controller, i2c_addr, cmd, data);
}

bool BQ76952::sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt command, uint16_t * readOut) {
    return BQ769X2_PROTOCOL::sendDirectCommandR(*i2c_controller, i2c_addr, command, readOut);
}

bool BQ76952::sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt command, uint16_t data) {
    return BQ769X2_PROTOCOL::sendDirectCommandW(*i2c_controller, i2c_addr, command, data);
}

bool BQ76952::setRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint32_t reg_data, uint8_t datalen) {
    return BQ769X2_PROTOCOL::setRegister(*i2c_controller, i2c_addr, reg_addr, reg_data, datalen);
}

bool BQ76952::I2C_ReadReg(uint16_t reg_addr, void *reg_data, uint8_t count) {
    return BQ769X2_PROTOCOL::I2C_ReadReg(*i2c_controller, i2c_addr, reg_addr, (uint8_t*)reg_data, count);
}

bool BQ76952::I2C_WriteReg(uint16_t reg_addr, void *reg_data, uint8_t count){
    return BQ769X2_PROTOCOL::I2C_WriteReg(*i2c_controller, i2c_addr, reg_addr, (uint8_t*)reg_data, count);
}

bool BQ76952::I2C_WriteReg(uint16_t reg_addr, uint8_t data){
    return BQ769X2_PROTOCOL::I2C_WriteReg(*i2c_controller, i2c_addr, reg_addr, &data, 1);
}

// --- Coulomb-counter helpers ----------------------------------------------

bool BQ76952::read_DAStatus5(uint8_t out32[32]) {
    return BQ769X2_PROTOCOL::sendSubcommandR(*i2c_controller, i2c_addr,
                                             BQ769X2_PROTOCOL::DASTATUS5, out32);
}

bool BQ76952::read_DAStatus6(uint8_t out32[32]) {
    return BQ769X2_PROTOCOL::sendSubcommandR(*i2c_controller, i2c_addr,
                                             BQ769X2_PROTOCOL::DASTATUS6, out32);
}

bool BQ76952::reset_passq() {
    return BQ769X2_PROTOCOL::sendCommandSubcommand(*i2c_controller, i2c_addr,
                                                   BQ769X2_PROTOCOL::RESET_PASSQ);
}


