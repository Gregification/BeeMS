/*
 * task_RX.cpp
 *
 *  Created on: Nov 7, 2025
 *      Author: kn042
 */

#include <stdint.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#include "ti_msp_dl_config.h"
#include "Core/system.hpp"
#include <Tasks/task_RX.hpp>
#include "Tasks/examples/dl_mcan.h"  // where CANFD0 symbols live in your tree

// Handle for notifying the RX task from the ISR
static TaskHandle_t s_rxTaskHandle = nullptr;

// ---- Helper: print a CAN frame nicely over UART ----
static void printRx(const DL_MCAN_RxBufElement &rx)
{
    char line[128];
    uint32_t id = (rx.xtd ? rx.id
                          // std ID lives in [28:18] per TI examples
                          : ((rx.id & 0x1FFC0000u) >> 18));

    int n = snprintf(line, sizeof(line),
                     "RX %s ID=0x%08lX DLC=%u  DATA=",
                     rx.xtd ? "EXT" : "STD",
                     (unsigned long)id,
                     (unsigned)rx.dlc);
    System::uart_ui.nputs(ARRANDN(line));

    for (uint8_t i = 0; i < rx.dlc && i < 64; ++i)
    {
        n = snprintf(line, sizeof(line), "%02X ", rx.data[i]);
        System::uart_ui.nputs(ARRANDN(line));
    }
    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

// ---- FreeRTOS Task: wait for IRQ, drain RX FIFO0 ----
void Task::task_RX(void *)
{
    // Make this task discoverable by the ISR for notifications
    s_rxTaskHandle = xTaskGetCurrentTaskHandle();

    // Enable the MCAN interrupt line that SysConfig generated for CANFD0
    // (Check the exact IRQ name in ti_msp_dl_config.h if it differs)
    NVIC_EnableIRQ(CANFD0_INT_IRQn);

    // Wait until controller is in NORMAL op mode (pins+bit-timing ready)
    while (DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(CANFD0)) { __NOP(); }

    for (;;)
    {
        // Sleep until ISR notifies that something arrived
        ulTaskNotifyTake(pdTRUE /*clear on exit*/, portMAX_DELAY);

        // Drain RX FIFO0 while there are elements
        DL_MCAN_RxFIFOStatus rxFS = {};
        DL_MCAN_getRxFIFOStatus(CANFD0, &rxFS);   // fillLvl, getIdx, num

        // Make sure we’re looking at FIFO0 (most common setup in SysConfig)
        rxFS.num = DL_MCAN_RX_FIFO_NUM_0;

        while (rxFS.fillLvl > 0)
        {
            DL_MCAN_RxBufElement rxMsg = {};

            // Read one element from FIFO0 at the hardware-provided index
            // Note: this mirrors TI’s example API ordering (bufIdx=0, fifoNum)
            DL_MCAN_readMsgRam(CANFD0,
                               DL_MCAN_MEM_TYPE_FIFO,
                               0U,
                               rxFS.num,
                               &rxMsg);

            // Ack that we consumed the element at getIdx so HW can advance
            DL_MCAN_writeRxFIFOAck(CANFD0, rxFS.num, rxFS.getIdx);

            // Do something useful
            printRx(rxMsg);

            // Re-query FIFO status in case more frames arrived
            DL_MCAN_getRxFIFOStatus(CANFD0, &rxFS);
            rxFS.num = DL_MCAN_RX_FIFO_NUM_0;
        }
    }
}

// ---- MCAN IRQ Handler: mark line-1 events and wake the task ----
// Check ti_msp_dl_config.h for the exact symbol; this matches TI naming.
extern "C" void CANFD0_INST_IRQHandler(void)
{
    switch (DL_MCAN_getPendingInterrupt(CANFD0))
    {
        case DL_MCAN_IIDX_LINE1:
        {
            // Snapshot and clear line-1 interrupts (covers RF0N: new RX on FIFO0)
            uint32_t st = DL_MCAN_getIntrStatus(CANFD0);
            DL_MCAN_clearIntrStatus(CANFD0, st, DL_MCAN_INTR_SRC_MCAN_LINE_1);

            // Wake the RX task
            BaseType_t hpw = pdFALSE;
            if (s_rxTaskHandle)
                vTaskNotifyGiveFromISR(s_rxTaskHandle, &hpw);
            portYIELD_FROM_ISR(hpw);
            break;
        }
        default:
            break;
    }
}



