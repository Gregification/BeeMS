/*
 * BQ76952.hpp
 *
 *  Created on: Jul 20, 2025
 *      Author: FSAE
 */

#ifndef SRC_MIDDLEWARE_BQ76952_HPP_
#define SRC_MIDDLEWARE_BQ76952_HPP_

#include "Core/system.hpp"
#include "Middleware/BQ769x2_PROTOCOL.hpp"

class BQ76952 {
using namespace BQ769X2_PROTOCOL; // CCS indexer dosen't like this for some reason
public:

    System::I2C::I2C i2cBus;
    BQ769X2_PROTOCOL::I2C_addr addr;


};


#endif /* SRC_MIDDLEWARE_BQ76952_HPP_ */
