// read function for MCP33151 ADC
// need to add coloumb counting 

#include <Middleware/MCP33151/MCP33151.hpp>

uint16_t MCP33151::read() {
    uint16_t rx;

    spi.takeResource(0);
    spi.transfer_blocking(NULL, &rx, 2, &cs);
    spi.giveResource();
    rx >>= 2;
    return ((rx & 0x3F) << 8) | ((rx & 0xFF00) >> 8);
}

bool MCP33151::calabrate() {
    spi.takeResource(0);

    cs.set();
    uint8_t rx[8];
    for(int i = 0; i < 1024/sizeof(rx); i++){
        static_assert(sizeof(rx) * (1024/sizeof(rx)) == 1024);

        spi.transfer_blocking(NULL, rx, sizeof(rx), NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    cs.clear();

    spi.giveResource();

    return 0;
}
