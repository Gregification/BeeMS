/*
 * system.cpp
 *
 *  Created on: May 6, 2025
 *      Author: turtl
 */

#include <Core/system.hpp>

uint32_t System::CPU_FREQ = 0;

void System::FailHard(char const * error_description) {
    // its over, the BeeMS has fallen

    (void) error_description;


    for(;;);
}
