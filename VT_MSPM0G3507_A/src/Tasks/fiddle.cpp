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

    static_assert(System::CLK::CANCLK == 40e6, "CAN peripheral clock misconfigured"); // adjust the dividers so that the clock is either 20MHz,40MHz, or 80MHz
    constexpr DL_MCAN_ClockConfig clkConfig = {
       .clockSel= DL_MCAN_FCLK::DL_MCAN_FCLK_SYSPLLCLK1,    // assuming is 40MHz
       .divider = DL_MCAN_FCLK_DIV::DL_MCAN_FCLK_DIV_1,     // need to reach 40MHz
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
      .tdcEnable         = true,    // transmitter delay compensation, oscilloscope the time diff between edges of TX->RX signals
      .tdcConfig.tdcf    = 10,      // filter length
      .tdcConfig.tdco    = 6,       // filter offset
    };

    constexpr DL_MCAN_MsgRAMConfigParams  ramConfig = {
        .flssa                = 0,  /* Standard ID Filter List Start Address. */
        .lss                  = 0,  /* List Size: Standard ID. */
        .flesa                = 0,  /* Extended ID Filter List Start Address. */
        .lse                  = 0,  /* List Size: Extended ID. */
        .txStartAddr          = 12, /* Tx Buffers Start Address. */
        .txBufNum             = 2,  /* Number of Dedicated Transmit Buffers. */
        .txFIFOSize           = 10,
        .txBufMode            = 0,  /* Tx Buffer Element Size. */
        .txBufElemSize        = DL_MCAN_ELEM_SIZE_64BYTES,
        .txEventFIFOStartAddr = 640,/* Tx Event FIFO Start Address. */
        .txEventFIFOSize      = 2,  /* Event FIFO Size. */
        .txEventFIFOWaterMark = 0,  /* Level for Tx Event FIFO watermark interrupt. */
        .rxFIFO0startAddr     = 0,  /* Rx FIFO0 Start Address. */
        .rxFIFO0size          = 0,  /* Number of Rx FIFO elements. */
        .rxFIFO0waterMark     = 0,  /* Rx FIFO0 Watermark. */
        .rxFIFO0OpMode        = 0,
        .rxFIFO1startAddr     = 0,  /* Rx FIFO1 Start Address. */
        .rxFIFO1size          = 0,  /* Number of Rx FIFO elements. */
        .rxFIFO1waterMark     = 0,  /* Level for Rx FIFO 1 watermark interrupt. */
        .rxFIFO1OpMode        = 0,  /* FIFO blocking mode. */
        .rxBufStartAddr       = 0,  /* Rx Buffer Start Address. */
        .rxBufElemSize        = DL_MCAN_ELEM_SIZE_64BYTES,  /* Rx Buffer Element Size. */
        .rxFIFO0ElemSize      = DL_MCAN_ELEM_SIZE_64BYTES,  /* Rx FIFO0 Element Size. */
        .rxFIFO1ElemSize      = DL_MCAN_ELEM_SIZE_64BYTES,  /* Rx FIFO1 Element Size. */
    };

    static const DL_MCAN_BitTimingParams   bitTimeingParams = {
        .nomRatePrescalar   = 40-1,     /* Arbitration Baud Rate Pre-scaler. */
        .nomTimeSeg1        = 138,      /* Arbitration Time segment before sample point. */
        .nomTimeSeg2        = 19,       /* Arbitration Time segment after sample point. */
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
