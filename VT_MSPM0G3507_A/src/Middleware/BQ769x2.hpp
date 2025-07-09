/*
 * BQ769x2.hpp
 *
 *  Created on: Jul 5, 2025
 *      Author: FSAE
 */

#ifndef SRC_MIDDLEWARE_BQ769X2_HPP_
#define SRC_MIDDLEWARE_BQ769X2_HPP_

#include "Core/system.hpp"
#include "Middleware/BQ769x2_PROTOCOL.hpp"

/**
 * I2C based BQ769x2 device
 */
class BQ769x2 {
public:
    System::I2C::I2C i2c;
    uint8_t i2c_write_address;
    uint8_t i2c_read_address;

    void I2C_WriteReg(uint8_t const * data, uint8_t count);
    void I2C_ReadReg(uint8_t * data, uint8_t count);
};

#endif /* SRC_MIDDLEWARE_BQ769X2_HPP_ */
