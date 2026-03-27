/*
 * adc.hpp
 *
 *  Created on: Feb 17, 2026
 *      Author: Roman
 */

#ifndef SRC_CORE_MCP33151_HPP_
#define SRC_CORE_MCP33151_HPP_

#include <stdint.h>
#include "Core/system.hpp"

struct MCP33151 {
    System::SPI::SPI & spi;
    System::GPIO::GPIO const & cs;

    /* 14b ADC raw*/
    uint16_t read();

    /* takes ~400mS to complete */
    bool calabrate();
};

#endif /* SRC_CORE_MCP33151_HPP_ */
