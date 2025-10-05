/*
 * example_flash_storage.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#include "example_flash_storage.hpp"

#include "Core/system.hpp"

void Task::example_flash_storage(void *) {
    System::uart_ui.nputs(ARRANDN("communication handler start" NEWLINE));

    // honestly just look at the flashcrl_multiple_size_write example code in TI's SDK.

    System::uart_ui.nputs(ARRANDN("communication handler end" NEWLINE));
    vTaskDelete(NULL);
}
