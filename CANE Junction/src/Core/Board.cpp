/*
 * MasterBoard.cpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#include "Board.hpp"
#include "task.h"
#include "Core/std alternatives/string.hpp"

// absurd naming!!! yippie!

namespace Board {

}

namespace System::UART {
    UART & uart_ui = uart0;
}

void Board::init() {
    {
        using namespace LED;

        for(auto & pin : indicators) {
            DL_GPIO_initDigitalOutputFeatures(
                    pin.iomux,
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                    DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                );
            DL_GPIO_clearPins(GPIOPINPUX(pin));
            DL_GPIO_enableOutput(GPIOPINPUX(pin));
        }
    }
}

uint32_t Board::POST(char * error_msg, uint16_t max_msg_len) {
    // TODO: actually test stuff
    return 0;
}
