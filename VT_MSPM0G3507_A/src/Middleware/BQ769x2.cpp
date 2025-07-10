/*
 * BQ769x2.cpp
 *
 *  Created on: Jul 6, 2025
 *      Author: FSAE
 */

#include "Middleware/BQ769x2.hpp"


inline void BQ769x2::I2C_WriteReg(uint8_t const * data, uint8_t count) {
    BQ769X2_PROTOCOL::I2C_WriteReg(i2c.reg, i2c_write_address, data, count);
}

inline void BQ769x2::I2C_ReadReg(uint8_t *data, uint8_t count) {
    BQ769X2_PROTOCOL::I2C_ReadReg(i2c.reg, i2c_read_address, data, count);
}
