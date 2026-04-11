/*
 * task_Flash.cpp
 *
 *  Created on: Apr 1, 2026
 *      Author: Zakir
 */

#include "task_flash.hpp"
#include "Core/system.hpp"

extern "C" {
#include <ti/driverlib/dl_flashctl.h>
#include <ti/devices/DeviceFamily.h>
}

static uint32_t compute_checksum(const FlashTapConfig_t *cfg)
{
    uint32_t cs = cfg->magic;
    cs ^= (uint32_t)cfg->num_taps;
    cs ^= cfg->tap_voltage_min_mV;
    cs ^= cfg->tap_voltage_max_mV;
    return cs;
}

bool Task::flash_validate_tap_config(const FlashTapConfig_t *cfg)
{
    if (cfg->magic != FLASH_MAGIC_KEY)                          return false;
    if (cfg->num_taps == 0 || cfg->num_taps > 16)               return false;
    if (cfg->tap_voltage_min_mV == 0)                           return false;
    if (cfg->tap_voltage_min_mV >= cfg->tap_voltage_max_mV)     return false;
    if (cfg->checksum != compute_checksum(cfg))                 return false;
    return true;
}

bool Task::flash_write_tap_config(const FlashTapConfig_t *cfg)
{
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t addr = FLASH_USER_START_ADDR;

    DL_FlashCTL_unprotectMainMemory(FLASHCTL);

    status = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, addr, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED)
        return false;

    const uint32_t *words = (const uint32_t *)cfg;
    uint32_t num_words    = sizeof(FlashTapConfig_t) / sizeof(uint32_t);

    for (uint32_t i = 0; i < num_words; i++) {
        status = DL_FlashCTL_programMemoryFromRAM32(
            FLASHCTL, addr + (i * 4), &words[i]);
        if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED)
            return false;
    }
    return true;
}

bool Task::flash_read_tap_config(FlashTapConfig_t *cfg)
{
    uint32_t addr      = FLASH_USER_START_ADDR;
    uint32_t *words    = (uint32_t *)cfg;
    uint32_t num_words = sizeof(FlashTapConfig_t) / sizeof(uint32_t);

    for (uint32_t i = 0; i < num_words; i++)
        words[i] = *((volatile uint32_t *)(addr + (i * 4)));

    return Task::flash_validate_tap_config(cfg);
}

void Task::flash_init(void)
{
    FlashTapConfig_t cfg = {};

    if (!Task::flash_read_tap_config(&cfg)) {
        System::UART::uart_ui.nputs(ARRANDN("flash: no valid config, writing defaults" NEWLINE));

        cfg.magic               = FLASH_MAGIC_KEY;
        cfg.num_taps            = 12;
        cfg.tap_voltage_min_mV  = 2500;
        cfg.tap_voltage_max_mV  = 4200;
        cfg.checksum            = compute_checksum(&cfg);

        if (!Task::flash_write_tap_config(&cfg))
            System::FailHard("flash: write failed" NEWLINE);

        // 1. readback
        FlashTapConfig_t verify = {};
        if (!Task::flash_read_tap_config(&verify) || verify.num_taps != cfg.num_taps)
            System::FailHard("flash: readback mismatch" NEWLINE);

        // 3. hardware verify on magic word
        DL_FLASHCTL_COMMAND_STATUS status = DL_FlashCTL_readVerifyFromRAM32(
            FLASHCTL, FLASH_USER_START_ADDR, &cfg.magic);
        if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED)
            System::FailHard("flash: hardware verify failed" NEWLINE);
    }

    // 2. always print so you can see it on every boot
    System::UART::uart_ui.nputs(ARRANDN("flash: magic=0x"));
    System::UART::uart_ui.putu32h(cfg.magic);
    System::UART::uart_ui.nputs(ARRANDN(NEWLINE "flash: num_taps="));
    System::UART::uart_ui.putu32d(cfg.num_taps);
    System::UART::uart_ui.nputs(ARRANDN(NEWLINE "flash: tap config OK" NEWLINE));

    // 4. power cycle — nothing to add in code, just observe:
    // first boot:  "writing defaults" then "tap config OK"
    // second boot: straight to "tap config OK" — if you see
    //              "writing defaults" again, flash isn't persisting
}

