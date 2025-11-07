#include "ti_msp_dl_config.h"
// If you have a central CAN header, include it instead of the example:
#include "Tasks/examples/dl_mcan.h"  // keep if that's where CANFD0 symbols live
#include <Tasks/task_TX.hpp>
#include "Core/system.hpp"



//#include "ti_msp_dl_config.h"
//#include "Tasks/examples/dl_mcan.h"
//
//void Task::task_TX(void *)
//{
//    SYSCFG_DL_init();
//
//    DL_MCAN_TxBufElement txMsg = {0};
//
//    // Configure message parameters
//    txMsg.id  = (0x3U << 18U);  // Standard ID stored in bits [28:18]
//    txMsg.xtd = 0;              // Standard frame
//    txMsg.rtr = 0;              // Data frame (not remote)
//    txMsg.fdf = 0;              // Classic CAN
//    txMsg.brs = 0;              // No bit rate switching
//    txMsg.dlc = 1;              // 1 byte of data
//    txMsg.data[0] = 0x00;       // Payload
//
//    while (DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(MCAN0_INST));
//
//    while (1) {
//        // 1. Write the message into message RAM buffer 0
//        DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0, &txMsg);
//
//        // 2. Trigger transmission request for buffer 0
//        DL_MCAN_TXBufAddReq(MCAN0_INST, (1U << 0));
//
//        for (volatile uint32_t i = 0; i < 1000000; i++);  // Delay between sends
//    }
//}


void Task::task_TX(void *)
{
    // 1) Do NOT re-call SYSCFG_DL_init() here; call it once at boot.
    // SYSCFG_DL_init();

    // 2) Wait until CAN is in NORMAL mode (use the same instance name as BMS)
    while (DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(CANFD0))
    {
        // tiny wait to avoid hammering the bus
        __NOP();
    }

    // 3) Build a frame that matches your BMS task style (extended ID, classic CAN)
    DL_MCAN_TxBufElement tx = {0};
    tx.id  = 0x1;     // using extended ID field like BMS_task
    tx.xtd = 1;       // extended ID (matches BMS_task)
    tx.rtr = 0;       // data frame
    tx.fdf = 0;       // classic CAN
    tx.brs = 0;       // no bit-rate switch
    tx.dlc = 8;       // 8 bytes
    tx.data[0] = 0x11;
    tx.data[1] = 0x22;
    tx.data[2] = 0x33;
    tx.data[3] = 0x44;
    tx.data[4] = 0x55;
    tx.data[5] = 0x66;
    tx.data[6] = 0x77;
    tx.data[7] = 0x88;

    for (;;)
    {
        // Query TX FIFO/Queue status, then write to that slot
        DL_MCAN_TxFIFOStatus tf;
        DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);  // same pattern as BMS_task

        uint32_t putIdx = tf.putIdx;

        // Write into TX FIFO
        DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, putIdx, &tx);

        // Trigger transmission for that FIFO slot
        DL_MCAN_TXBufAddReq(CANFD0, (1U << putIdx)); // (1U << putIdx)tf.getIdx

        // pace transmissions
        vTaskDelay(pdMS_TO_TICKS(200)); // delay ~200ms
    }
}
