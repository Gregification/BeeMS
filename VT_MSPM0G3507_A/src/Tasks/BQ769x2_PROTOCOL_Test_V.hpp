/*
 * BQ769x2_PROTOCOL_test.hpp
 *
 *  Created on: Jul 13, 2025
 *      Author: FSAE
 */

#ifndef SRC_TASKS_BQ769X2_PROTOCOL_TEST_V_HPP_
#define SRC_TASKS_BQ769X2_PROTOCOL_TEST_V_HPP_

#include "Core/system.hpp"

namespace System {
    OCCUPY(i2c1);
    OCCUPY(PA15);
    OCCUPY(PA16);
}

namespace Task {
    void BQ769x2_PROTOCOL_Test_V(void *);
}

#endif /* SRC_TASKS_BQ769X2_PROTOCOL_TEST_V_HPP_ */
