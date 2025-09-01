/*
 * BQ76952.cpp
 *
 *  Created on: Aug 13, 2025
 *      Author: FSAE
 */

#include "BQ76952.hpp"
#include "Middleware/BQ769x2/BQ769x2_PROTOCOL.hpp"

using namespace BQ769X2_PROTOCOL;

inline bool BQ76952::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd command,  TickType_t timeout) {
    return BQ769X2_PROTOCOL::sendCommandSubcommand(*i2c_controller, i2c_addr, command, timeout);
}

inline bool BQ76952::sendSubcommandR(Cmd cmd, uint8_t readOut[32], TickType_t timeout) {
    return BQ769X2_PROTOCOL::sendSubcommandR(*i2c_controller, i2c_addr, cmd, readOut, timeout);
}

inline bool BQ76952::sendSubcommandW(Cmd cmd, uint8_t data, TickType_t timeout) {
    return BQ769X2_PROTOCOL::sendSubcommandW(*i2c_controller, i2c_addr, cmd, data, timeout);
}

inline bool BQ76952::sendSubcommandW2(Cmd cmd,uint16_t data, TickType_t timeout) {
    return BQ769X2_PROTOCOL::sendSubcommandW2(*i2c_controller, i2c_addr, cmd, data, timeout);
}

inline bool BQ76952::sendDirectCommandR(CmdDrt command, uint16_t * readOut, TickType_t timeout) {
    return BQ769X2_PROTOCOL::sendDirectCommandR(*i2c_controller, i2c_addr, command, readOut, timeout);
}

inline bool BQ76952::sendDirectCommandW(CmdDrt command, uint16_t data, TickType_t timeout) {
    return BQ769X2_PROTOCOL::sendDirectCommandW(*i2c_controller, i2c_addr, command, data, timeout);
}

inline bool BQ76952::setRegister(RegAddr reg_addr, uint32_t reg_data, uint8_t datalen, TickType_t timeout) {
    return BQ769X2_PROTOCOL::setRegister(*i2c_controller, i2c_addr, reg_addr, reg_data, datalen, timeout);
}

inline bool BQ76952::I2C_ReadReg(uint8_t reg_addr, void *reg_data, uint8_t count, TickType_t timeout) {
    return BQ769X2_PROTOCOL::I2C_ReadReg(*i2c_controller, i2c_addr, reg_addr, (uint8_t*)reg_data, count, timeout);
}

inline bool BQ76952::I2C_WriteReg(uint8_t reg_addr, void *reg_data, uint8_t count, TickType_t timeout){
    return BQ769X2_PROTOCOL::I2C_WriteReg(*i2c_controller, i2c_addr, reg_addr, (uint8_t*)reg_data, count, timeout);
}
