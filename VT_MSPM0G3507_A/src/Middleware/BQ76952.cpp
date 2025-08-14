/*
 * BQ76952.cpp
 *
 *  Created on: Aug 13, 2025
 *      Author: FSAE
 */

#include "BQ76952.hpp"
#include "Middleware/BQ769x2_PROTOCOL.hpp"

using namespace BQ769X2_PROTOCOL;


bool I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout) {
    return BQ769X2_PROTOCOL::I2C_ReadReg(i2c_controller, i2c_addr, reg_addr, reg_data, count, timeout);
}
bool I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout){
    return BQ769X2_PROTOCOL::I2C_WriteReg(i2c_controller, i2c_addr, reg_addr, reg_data, count, timeout);
}
