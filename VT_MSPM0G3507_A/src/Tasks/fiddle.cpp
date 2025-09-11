/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf?ts=1749927951492&ref_url=https%253A%252F%252Fwww.google.com%252F
 */

#include "fiddle.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <stdio.h>

#include "Core/system.hpp"


void Task::fiddle_task(void *){
    System::uart_ui.nputs(ARRANDN("fiddle task start" NEWLINE));

    DL_MCAN_reset(CANFD0);
    DL_MCAN_enablePower(CANFD0);
    delay_cycles(POWER_STARTUP_DELAY);

    constexpr DL_MCAN_ClockConfig clkConfig = {
       .clockSel= DL_MCAN_FCLK::DL_MCAN_FCLK_SYSPLLCLK1,    // assuming is 20MHz
       .divider = DL_MCAN_FCLK_DIV::DL_MCAN_FCLK_DIV_1,     // need to reach 20MHz
    };

    constexpr DL_MCAN_InitParams initparam = {
      /* Initialize MCAN Init parameters.    */
      .fdMode            = true,    // CAN FD mode?
      .brsEnable         = true,    // enable bit rate switching?
      .txpEnable         = true,    // pause for 2 bit times after successful transmission?
      .efbi              = false,   // edge filtering? 2 consecutive Tq to accept sync
      .pxhddisable       = false,   // do not Tx error frame on protocol error?
      .darEnable         = false,   // auto retransmission of failed frames?
      .wkupReqEnable     = true,    // enable wake up request?
      .autoWkupEnable    = true,    // enable auto-wakeup?
      .emulationEnable   = true,
      .wdcPreload        = 255,     // start value of message ram WDT

      /* Transmitter Delay Compensation parameters. tm.26.4.6/1439 */
      .tdcEnable         = false,   // transmitter delay compensation
      .tdcConfig.tdcf    = 10,      // filter length
      .tdcConfig.tdco    = 6,       // filter offset
    };

    constexpr DL_MCAN_MsgRAMConfigParams  ramConfig = {
        /* Standard ID Filter List Start Address. */
        .flssa                = 0,
        /* List Size: Standard ID. */
        .lss                  = 0,
        /* Extended ID Filter List Start Address. */
        .flesa                = 0,
        /* List Size: Extended ID. */
        .lse                  = 0,
        /* Tx Buffers Start Address. */
        .txStartAddr          = 12,
        /* Number of Dedicated Transmit Buffers. */
        .txBufNum             = 2,
        .txFIFOSize           = 10,
        /* Tx Buffer Element Size. */
        .txBufMode            = 0,
        .txBufElemSize        = DL_MCAN_ELEM_SIZE_64BYTES,
        /* Tx Event FIFO Start Address. */
        .txEventFIFOStartAddr = 640,
        /* Event FIFO Size. */
        .txEventFIFOSize      = 2,
        /* Level for Tx Event FIFO watermark interrupt. */
        .txEventFIFOWaterMark = 0,
        /* Rx FIFO0 Start Address. */
        .rxFIFO0startAddr     = 0,
        /* Number of Rx FIFO elements. */
        .rxFIFO0size          = 0,
        /* Rx FIFO0 Watermark. */
        .rxFIFO0waterMark     = 0,
        .rxFIFO0OpMode        = 0,
        /* Rx FIFO1 Start Address. */
        .rxFIFO1startAddr     = 0,
        /* Number of Rx FIFO elements. */
        .rxFIFO1size          = 0,
        /* Level for Rx FIFO 1 watermark interrupt. */
        .rxFIFO1waterMark     = 0,
        /* FIFO blocking mode. */
        .rxFIFO1OpMode        = 0,
        /* Rx Buffer Start Address. */
        .rxBufStartAddr       = 0,
        /* Rx Buffer Element Size. */
        .rxBufElemSize        = DL_MCAN_ELEM_SIZE_64BYTES,
        /* Rx FIFO0 Element Size. */
        .rxFIFO0ElemSize      = DL_MCAN_ELEM_SIZE_64BYTES,
        /* Rx FIFO1 Element Size. */
        .rxFIFO1ElemSize      = DL_MCAN_ELEM_SIZE_64BYTES,
    };

    static const DL_MCAN_BitTimingParams   bitTimeingParams = {
        /* Arbitration Baud Rate Pre-scaler. */
        .nomRatePrescalar   = 40-1, // 0-> /1 , CAN clk divider to bit time
        /* Arbitration Time segment before sample point. */
        .nomTimeSeg1        = 138,
        /* Arbitration Time segment after sample point. */
        .nomTimeSeg2        = 19,
        /* Arbitration (Re)Synchronization Jump Width Range. */
        .nomSynchJumpWidth  = 19,
        /* Data Baud Rate Pre-scaler. */
        .dataRatePrescalar  = 0,
        /* Data Time segment before sample point. */
        .dataTimeSeg1       = 16,
        /* Data Time segment after sample point. */
        .dataTimeSeg2       = 1,
        /* Data (Re)Synchronization Jump Width.   */
        .dataSynchJumpWidth = 1,
    };

    System::uart_ui.nputs(ARRANDN("fiddle task end" NEWLINE));
    vTaskDelete(NULL);
}
