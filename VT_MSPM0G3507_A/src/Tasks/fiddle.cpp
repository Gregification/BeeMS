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

    DL_MCAN_disablePower(CANFD0);
    DL_MCAN_reset(CANFD0);
    DL_MCAN_enablePower(CANFD0);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM60, IOMUX_PINCM60_PF_CANFD0_CANRX); // CANRX, PA27
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM59, IOMUX_PINCM59_PF_CANFD0_CANTX); // CANTX, PA26

    DL_MCAN_enableModuleClock(CANFD0);

    {
        static_assert(System::CLK::CANCLK == 40e6, "CAN peripheral clock misconfigured"); // adjust the dividers so that the clock is either 20MHz,40MHz, or 80MHz
        constexpr DL_MCAN_ClockConfig clkConfig = {
           .clockSel= DL_MCAN_FCLK::DL_MCAN_FCLK_SYSPLLCLK1,
           .divider = DL_MCAN_FCLK_DIV::DL_MCAN_FCLK_DIV_1,
        };
        DL_MCAN_setClockConfig(CANFD0, &clkConfig);
    }

    // have no idea if this is required for the setup
    DL_MCAN_RevisionId revId;
    /* Get MCANSS Revision ID. */
    DL_MCAN_getRevisionId(CANFD0, &revId);

    /* Wait for Memory initialization to be completed. */
    while(false == DL_MCAN_isMemInitDone(CANFD0))
    System::uart_ui.nputs(ARRANDN("wait" NEWLINE));
        ;
    System::uart_ui.nputs(ARRANDN("lkncaekaelknacelknacelnkacelklknnacenlkacelknaceknlcealk" NEWLINE));

    /* Put MCAN in SW initialization mode. */
    DL_MCAN_setOpMode(CANFD0, DL_MCAN_OPERATION_MODE_SW_INIT);
    while(DL_MCAN_OPERATION_MODE_SW_INIT != DL_MCAN_getOpMode(CANFD0))
        ;

    { /* Initialize MCAN module. */
        constexpr DL_MCAN_InitParams initparam = {
              /* Initialize MCAN Init parameters.    */
              .fdMode            = true,    // CAN FD mode?
              .brsEnable         = true,    // enable bit rate switching?
              .txpEnable         = true,    // pause for 2 bit times after successful transmission?
              .efbi              = false,   // edge filtering? 2 consecutive Tq to accept sync
              .pxhddisable       = false,   // do not Tx error frame on protocol error?
              .darEnable         = false,   // auto retransmission of failed frames?
              .wkupReqEnable     = false,   // enable wake up request?
              .autoWkupEnable    = false,   // enable auto-wakeup?
              .emulationEnable   = false,
              .wdcPreload        = 255,     // start value of message ram WDT

              /* Transmitter Delay Compensation parameters. tm.26.4.6/1439 */
              .tdcEnable         = false,   // transmitter delay compensation, oscilloscope the time diff between edges of TX->RX signals
              .tdcConfig.tdcf    = 10,      // filter length
              .tdcConfig.tdco    = 6,       // filter offset
            };
        DL_MCAN_init(CANFD0, &initparam);
    }

    { /* Configure MCAN module. */
        constexpr DL_MCAN_ConfigParams config = {
                .monEnable         = false,     // bus monitoring mode
                .asmEnable         = false,     // restricted operation mode
                .tsPrescalar       = 15,        // timestamp counter prescalar
                .tsSelect          = 0,         // timestamp source selection
                .timeoutSelect     = DL_MCAN_TIMEOUT_SELECT_CONT, // timeout source select
                .timeoutPreload    = 65535,     // load value of timeout counter
                .timeoutCntEnable  = false,     // timeout counter enable
                .filterConfig.rrfe = true,      // reject remote frames extended
                .filterConfig.rrfs = true,      // reject remote frames standard
                .filterConfig.anfe = 1,         // accept non-matching frames extended
                .filterConfig.anfs = 1,         // accept non-matching frames standard
            };
        DL_MCAN_config(CANFD0, &config);
    }

    { /* Configure Bit timings. */
        static const DL_MCAN_BitTimingParams   bitTimeingParams = {
                .nomRatePrescalar   = 3,    /* Arbitration Baud Rate Pre-scaler. */
                .nomTimeSeg1        = 16,   /* Arbitration Time segment before sample point. */
                .nomTimeSeg2        = 1,    /* Arbitration Time segment after sample point. */
                .nomSynchJumpWidth  = 1,    /* Arbitration (Re)Synchronization Jump Width Range. */
                .dataRatePrescalar  = 0,    /* Data Baud Rate Pre-scaler. */
                .dataTimeSeg1       = 0,    /* Data Time segment before sample point. */
                .dataTimeSeg2       = 0,    /* Data Time segment after sample point. */
                .dataSynchJumpWidth = 0,    /* Data (Re)Synchronization Jump Width.   */
            };
        DL_MCAN_setBitTime(CANFD0, &bitTimeingParams);
    }

    { /* Configure Message RAM Sections */
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
        DL_MCAN_msgRAMConfig(CANFD0, &ramConfig);
    }

    /* Set Extended ID Mask. */
    // is ANDed with 29b message id of frame
    DL_MCAN_setExtIDAndMask(CANFD0, 0x1FFFFFFF);

    /* Take MCAN out of the SW initialization mode */
    DL_MCAN_setOpMode(CANFD0, DL_MCAN_OPERATION_MODE_NORMAL);
    while(DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(CANFD0))
        vTaskDelay(pdMS_TO_TICKS(1));


    //--- TX --------------------------------------------------
    {
        DL_MCAN_TxBufElement txmsg = {
            .id     = 0xBEE,    // CAN id
            .rtr    = 1,        // 0: data frame, 1: remote frame
            .xtd    = 0,        // 0: 11b id, 1: 29b id
            .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
            .dlc    = 3,        // data byte count, see DL comments
            .brs    = 0,        // 0: no bit rate switching, 1: yes brs
            .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
            .efc    = 0,        // 0: dont store Tx events, 1: store
            .mm     = 0,        // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
            .data   = {1,2,3}
        };

        uint32_t bufferIndex = 0;
        DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_BUF, bufferIndex, &txmsg);
        DL_MCAN_TXBufAddReq(CANFD0, bufferIndex);
    }

    //--- RX --------------------------------------------------



    System::uart_ui.nputs(ARRANDN("fiddle task end" NEWLINE));
    vTaskDelete(NULL);
}
