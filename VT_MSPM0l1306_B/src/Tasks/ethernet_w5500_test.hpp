/*
 * ethernet_w5500_test.hpp
 *
 *  Created on: Aug 31, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_ETHERNET_W5500_TEST_HPP_
#define SRC_TASKS_ETHERNET_W5500_TEST_HPP_

#include <Core/system.hpp>

namespace System {
    OCCUPY(PA4);    // CS
}

namespace Task {
    void ethernetw5500_test(void *);
}


#endif /* SRC_TASKS_ETHERNET_W5500_TEST_HPP_ */
