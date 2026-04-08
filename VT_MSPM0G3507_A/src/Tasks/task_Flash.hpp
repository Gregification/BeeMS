/*
 * task_Flash.hpp
 *
 *  Created on: Apr 1, 2026
 *      Author: Zakir
 */

#ifndef SRC_TASKS_TASK_FLASH_HPP_
#define SRC_TASKS_TASK_FLASH_HPP_

#include <stdint.h>
#include <stdbool.h>

#define FLASH_USER_START_ADDR   0x0003F000
#define FLASH_MAGIC_KEY         0xDEADBEEF

typedef struct {
    uint32_t magic;
    uint8_t  num_taps;
    uint8_t  _pad[3];          // explicit padding to keep 4-byte alignment
    uint32_t tap_voltage_min_mV;
    uint32_t tap_voltage_max_mV;
    uint32_t checksum;
} FlashTapConfig_t;

namespace Task {
    /**
     * Reads voltage tap board config from flash.
     * Writes defaults if config is missing or corrupted.
     * Call before vTaskStartScheduler().
     */
    void flash_init(void);
    bool flash_write_tap_config(const FlashTapConfig_t *cfg);
    bool flash_read_tap_config(FlashTapConfig_t *cfg);
    bool flash_validate_tap_config(const FlashTapConfig_t *cfg);
}

#endif /* SRC_TASKS_TASK_FLASH_HPP_ */
