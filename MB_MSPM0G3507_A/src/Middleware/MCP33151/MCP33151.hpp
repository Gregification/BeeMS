/*
 * adc.hpp
 *
 *  Created on: Feb 17, 2026
 *      Author: Roman
 */

#ifndef SRC_CORE_ADC_HPP_
#define SRC_CORE_ADC_HPP_

#include <stdint.h>
#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"

namespace ADC 
{
    uint16_t MCP33151_R(System::SPI::SPI &spi, System::GPIO::GPIO const &cs);

    uint16_t read_precise_adc();
    uint16_t read_imprecise_adc();
}

#endif /* SRC_CORE_ADC_HPP_ */
