/*
 * test_flash_storage.hpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_EXAMPLE_FLASH_STORAGE_HPP_
#define SRC_TASKS_EXAMPLE_FLASH_STORAGE_HPP_


/**
 * example code for RW of flash.
 * TRM.6.2/492
 *
 * WARNING !!! flash may contain factory programmed parameters !!! see TRM.6-2/492
 *
 * note:
 *  - see errata about flash power sequence.
 *  - FLASHCTL registers may not always be configured to default values after a reset.
 *      caused by BCR or BSL. at time of writing these are not used.
 */
namespace Task {
    void example_flash_storage(void *);
}


#endif /* SRC_TASKS_EXAMPLE_FLASH_STORAGE_HPP_ */
