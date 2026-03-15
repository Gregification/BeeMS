// read function for MCP33151 ADC
// need to add coloumb counting 

#include <Middleware/MCP33151/MCP33151.hpp>

namespace ADC
{
    uint16_t MCP33151_R(System::SPI::SPI &spi, System::GPIO::GPIO const &cs)
    {
        uint8_t tx[2] = { 0x00, 0x00 };   // get 16 clocks for 16 bits (2 leading zeros- data is 14 bits)
        uint8_t rx[2];

        spi.transfer_blocking(tx, rx, 2, &cs);

        return ((uint16_t) rx[0] << 8) | rx[1];
    }

    uint16_t read_precise_adc()
    {
        return MCP33151_R(MstrB::MHCS::spi, MstrB::MHCS::cs_precise);
    }

    uint16_t read_imprecise_adc()
    {
        return MCP33151_R(MstrB::MHCS::spi, MstrB::MHCS::cs_imprecise);
    }

}
