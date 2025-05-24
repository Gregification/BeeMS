/*
 * OrionBMS.cpp
 *
 *  Created on: May 23, 2025
 *      Author: FSAE
 */

#include "OrionBMS.hpp"


void OrionBMS::TEM::CAN::TMtoBMS::Packet::updateChecksum() {
    checksum = sizeof(Packet) + 0x39;
    for(uint8_t i = 0; i < sizeof(Packet) - sizeof(checksum); i++)
        checksum += ((uint8_t *)this)[i];
}
