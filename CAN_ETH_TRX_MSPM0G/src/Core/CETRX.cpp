/*
 * MasterBoard.cpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#include <Core/CETRX.hpp>
#include <stdint.h>

// absurd naming!!! yippie!

namespace System::UART {
    UART & uart_ui = uart0;
}


namespace CEB {
    System::SPI::SPI & Bridge::wiz_spi   = System::SPI::spi1;

    namespace Bridge {
        CANFD::CANFD & can = System::CANFD::canFD0;

        uint8_t ethBroadcastIP[4] = {192,168,1,255};
        wiz_NetInfo netConfig = {
                   .mac = {0xBE,0xEE,0xEE,0x00,0x00,0x00}, // arbitrary. last byte is overwritten with MCU serial number
                   .ip  = {192,168,1,220},
                   .sn  = {255,255,255,0},
                   .gw  = {192,168,1,1},
                   .dns = {8,8,8,8},
                   .dhcp= NETINFO_STATIC
            };
        uint16_t wiz_IP_port = 42067;
    }
}

/*
 * Everything would probably be fine without the explicit GPIO setup, mostly for consistency
 */
void CEB::init() {

    {
        using namespace Indi;

        DL_GPIO_initDigitalOutputFeatures(
                LED::i1.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(LED::i1));
        DL_GPIO_enableOutput(GPIOPINPUX(LED::i1));

        DL_GPIO_initDigitalOutputFeatures(
                LED::scheduler.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(LED::scheduler));
        DL_GPIO_enableOutput(GPIOPINPUX(LED::scheduler));
    }

}

uint8_t CEB::getUnitBoardID() {
    // TODO: should be physically configurable on the board, just read back those settings.
    return System::mcuID;
}
