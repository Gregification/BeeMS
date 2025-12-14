/*
 * MasterBoard.hpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#ifndef SRC_MASTERBOARD_HPP_
#define SRC_MASTERBOARD_HPP_

#include <stdint.h>

namespace MasterBoard {

    /** gets a 8b number that represents the board.
     * - unit ID is physically configurable on the board
     * - used by the network to identity instances
     */
    uint8_t getUnitBoardID();
}


#endif /* SRC_MASTERBOARD_HPP_ */
