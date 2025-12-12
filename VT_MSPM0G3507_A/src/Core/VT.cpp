/*
 * VT.cpp
 *
 *  Created on: Dec 9, 2025
 *      Author: turtl
 */

#include "VT.hpp"

// variables
namespace VT {
    BQ76952 bq = {
            .spi  = &System::spi1,
            .cs   = &System::GPIO::PB19, // v3
    //        .cs   = &System::GPIO::PA15, // v2.2
        };
    System::GPIO::GPIO const & bqReset = System::GPIO::PA15; // v3
    //    System::GPIO::GPIO &bqReset = System::GPIO::PA21; // v2.2
}

void VT::preScheduler_init(){

}

void VT::postScheduler_init(){

}
