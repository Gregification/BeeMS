/*
 * VT.cpp
 *
 *  Created on: Dec 9, 2025
 *      Author: turtl
 */

#include "VT.hpp"
#include "Core/system.hpp"

// variables
namespace VT {
    uint8_t id = 0;

    OpProfile_t opProfile = {

    };

    OpVars_t opVars = {

    };
}

namespace VT::BBQ {
    const uint8_t bbqs_n = sizeof(bbqs)/sizeof(bbqs[0]);
    static_assert((uint32_t)bbqs_n == (uint32_t)sizeof(bbqs)/sizeof(bbqs[0]), "undersized. also why are there so many");

    BBQ_t bbqs[1] = {
            {
                .bq = {
                    .spi    = System::spi1,
                    .cs     = System::GPIO::PA14,
                },
                .resetPin   = System::GPIO::PA15,
            }
        };

}

void VT::preScheduler_init(){

}

void VT::postScheduler_init(){

}

/** write to non volatile storage */
bool VT::BBQ::storeSetting(buffersize_t i, BQ76952::BQ76952SSetting const *) {
    return false;
}

/** read from non volatile storage */
bool VT::BBQ::recalSetting(buffersize_t i, BQ76952::BQ76952SSetting *) {
    return false;
}

/** write to BBQ */
bool VT::BBQ::applySetting(BQ76952 & bq, BQ76952::BQ76952SSetting const *) {
    return false;
}

/** read from BBQ */
bool VT::BBQ::retreiveSetting(BQ76952 const & bq, BQ76952::BQ76952SSetting *) {
    return false;
}

uint8_t VT::getID() {
    return System::mcuID;
}
