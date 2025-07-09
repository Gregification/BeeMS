/*
 * BQ769x2_protocol_test.hpp
 *
 *  Created on: Jul 6, 2025
 *      Author: FSAE
 */

#ifndef SRC_TASKS_BQ769X2_PROTOCOL_TEST_HPP_
#define SRC_TASKS_BQ769X2_PROTOCOL_TEST_HPP_

#include <Core/system.hpp>

namespace System {
    OCCUPY(PA15);
    OCCUPY(PA16);
    OCCUPY(I2C1);
}

namespace Task {
    void BQ769x2_protocol_test_task(void *);
}



#endif /* SRC_TASKS_BQ769X2_PROTOCOL_TEST_HPP_ */
