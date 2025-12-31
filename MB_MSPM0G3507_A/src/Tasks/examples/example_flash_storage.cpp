/*
 * example_flash_storage.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#include "example_flash_storage.hpp"

#include "Core/system.hpp"

void Task::example_flash_storage(void *) {
    System::UART::uart_ui.nputs(ARRANDN("example_flash_storage start" NEWLINE));

    // honestly just look at the flashcrl_multiple_size_write example code in TI's SDK.
    //  just be sure where ever your writing to isnt system critical
    // C:\ti\mspm0_sdk_2_05_00_05\examples\nortos\LP_MSPM0G3507\driverlib\flashctl_multiple_size_write
    // C:\ti\mspm0_sdk_2_05_00_05\examples\nortos\LP_MSPM0G3507\driverlib\flashctl_multiple_size_read_verify

    System::UART::uart_ui.nputs(ARRANDN("example_flash_storage end" NEWLINE));
    vTaskDelete(NULL);
}
