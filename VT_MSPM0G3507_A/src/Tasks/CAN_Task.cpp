/*
 * CAN_Task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CAN_Task.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include "Core/system.hpp"

#include <ti/driverlib/driverlib.h>

#include <stdio.h>

//ti_msp_dl_confg.h
#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     32000000




/* Port definition for Pin Group GPIO_SWITCHES */
#define GPIO_SWITCHES_PORT                                               (GPIOB)

/* Defines for USER_SWITCH_1: GPIOB.21 with pinCMx 49 on package pin 20 */
// pins affected by this interrupt request:["USER_SWITCH_1"]
#define GPIO_SWITCHES_INT_IRQN                                  (GPIOB_INT_IRQn)
#define GPIO_SWITCHES_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_SWITCHES_USER_SWITCH_1_IIDX                    (DL_GPIO_IIDX_DIO21)
#define GPIO_SWITCHES_USER_SWITCH_1_PIN                         (DL_GPIO_PIN_21)
#define GPIO_SWITCHES_USER_SWITCH_1_IOMUX                        (IOMUX_PINCM49)


/* Defines for MCAN0 */
#define MCAN0_INST                                                        CANFD0
#define GPIO_MCAN0_CAN_TX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_TX_PIN                                     DL_GPIO_PIN_12
#define GPIO_MCAN0_IOMUX_CAN_TX                                  (IOMUX_PINCM34)
#define GPIO_MCAN0_IOMUX_CAN_TX_FUNC               IOMUX_PINCM34_PF_CANFD0_CANTX
#define GPIO_MCAN0_CAN_RX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_RX_PIN                                     DL_GPIO_PIN_13
#define GPIO_MCAN0_IOMUX_CAN_RX                                  (IOMUX_PINCM35)
#define GPIO_MCAN0_IOMUX_CAN_RX_FUNC               IOMUX_PINCM35_PF_CANFD0_CANRX
#define MCAN0_INST_IRQHandler                                 CANFD0_IRQHandler
#define MCAN0_INST_INT_IRQN                                     CANFD0_INT_IRQn


/* Defines for MCAN0 MCAN RAM configuration */
#define MCAN0_INST_MCAN_STD_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_STD_ID_FILTER_NUM          (0)
#define MCAN0_INST_MCAN_EXT_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_EXT_ID_FILTER_NUM          (0)
#define MCAN0_INST_MCAN_TX_BUFF_START_ADDR         (12)
#define MCAN0_INST_MCAN_TX_BUFF_SIZE               (2)
#define MCAN0_INST_MCAN_FIFO_1_START_ADDR          (0)
#define MCAN0_INST_MCAN_FIFO_1_NUM                 (0)
#define MCAN0_INST_MCAN_TX_EVENT_START_ADDR        (640)
#define MCAN0_INST_MCAN_TX_EVENT_SIZE              (2)
#define MCAN0_INST_MCAN_EXT_ID_AND_MASK            (0x1FFFFFFFU)
#define MCAN0_INST_MCAN_RX_BUFF_START_ADDR         (0)
#define MCAN0_INST_MCAN_FIFO_0_START_ADDR          (0)
#define MCAN0_INST_MCAN_FIFO_0_NUM                 (0)

#define MCAN0_INST_MCAN_INTERRUPTS (DL_MCAN_INTERRUPT_ARA | \
                        DL_MCAN_INTERRUPT_BEU | \
                        DL_MCAN_INTERRUPT_BO | \
                        DL_MCAN_INTERRUPT_DRX | \
                        DL_MCAN_INTERRUPT_ELO | \
                        DL_MCAN_INTERRUPT_EP | \
                        DL_MCAN_INTERRUPT_EW | \
                        DL_MCAN_INTERRUPT_MRAF | \
                        DL_MCAN_INTERRUPT_PEA | \
                        DL_MCAN_INTERRUPT_PED | \
                        DL_MCAN_INTERRUPT_TC | \
                        DL_MCAN_INTERRUPT_TEFN | \
                        DL_MCAN_INTERRUPT_TOO | \
                        DL_MCAN_INTERRUPT_TSW | \
                        DL_MCAN_INTERRUPT_WDI)

//ti_msp_dl_confg.c
static const DL_MCAN_ClockConfig gMCAN0ClockConf = {
    .clockSel = DL_MCAN_FCLK_SYSPLLCLK1,
    .divider  = DL_MCAN_FCLK_DIV_4,
};

static const DL_MCAN_InitParams gMCAN0InitParams= {

/* Initialize MCAN Init parameters.    */
    .fdMode            = true,
    .brsEnable         = true,
    .txpEnable         = false,
    .efbi              = false,
    .pxhddisable       = false,
    .darEnable         = false,
    .wkupReqEnable     = true,
    .autoWkupEnable    = true,
    .emulationEnable   = true,
    .tdcEnable         = true,
    .wdcPreload        = 255,

/* Transmitter Delay Compensation parameters. */
    .tdcConfig.tdcf    = 10,
    .tdcConfig.tdco    = 6,
};

static const DL_MCAN_ConfigParams gMCAN0ConfigParams={
    /* Initialize MCAN Config parameters. */
    .monEnable         = false,
    .asmEnable         = false,
    .tsPrescalar       = 15,
    .tsSelect          = 0,
    .timeoutSelect     = DL_MCAN_TIMEOUT_SELECT_CONT,
    .timeoutPreload    = 65535,
    .timeoutCntEnable  = false,
    .filterConfig.rrfs = true,
    .filterConfig.rrfe = true,
    .filterConfig.anfe = 1,
    .filterConfig.anfs = 1,
};

static const DL_MCAN_MsgRAMConfigParams gMCAN0MsgRAMConfigParams ={

    /* Standard ID Filter List Start Address. */
    .flssa                = MCAN0_INST_MCAN_STD_ID_FILT_START_ADDR,
    /* List Size: Standard ID. */
    .lss                  = MCAN0_INST_MCAN_STD_ID_FILTER_NUM,
    /* Extended ID Filter List Start Address. */
    .flesa                = MCAN0_INST_MCAN_EXT_ID_FILT_START_ADDR,
    /* List Size: Extended ID. */
    .lse                  = MCAN0_INST_MCAN_EXT_ID_FILTER_NUM,
    /* Tx Buffers Start Address. */
    .txStartAddr          = MCAN0_INST_MCAN_TX_BUFF_START_ADDR,
    /* Number of Dedicated Transmit Buffers. */
    .txBufNum             = MCAN0_INST_MCAN_TX_BUFF_SIZE,
    .txFIFOSize           = 10,
    /* Tx Buffer Element Size. */
    .txBufMode            = 0,
    .txBufElemSize        = DL_MCAN_ELEM_SIZE_64BYTES,
    /* Tx Event FIFO Start Address. */
    .txEventFIFOStartAddr = MCAN0_INST_MCAN_TX_EVENT_START_ADDR,
    /* Event FIFO Size. */
    .txEventFIFOSize      = MCAN0_INST_MCAN_TX_EVENT_SIZE,
    /* Level for Tx Event FIFO watermark interrupt. */
    .txEventFIFOWaterMark = 0,
    /* Rx FIFO0 Start Address. */
    .rxFIFO0startAddr     = MCAN0_INST_MCAN_FIFO_0_START_ADDR,
    /* Number of Rx FIFO elements. */
    .rxFIFO0size          = MCAN0_INST_MCAN_FIFO_0_NUM,
    /* Rx FIFO0 Watermark. */
    .rxFIFO0waterMark     = 0,
    .rxFIFO0OpMode        = 0,
    /* Rx FIFO1 Start Address. */
    .rxFIFO1startAddr     = MCAN0_INST_MCAN_FIFO_1_START_ADDR,
    /* Number of Rx FIFO elements. */
    .rxFIFO1size          = MCAN0_INST_MCAN_FIFO_1_NUM,
    /* Level for Rx FIFO 1 watermark interrupt. */
    .rxFIFO1waterMark     = 0,
    /* FIFO blocking mode. */
    .rxFIFO1OpMode        = 0,
    /* Rx Buffer Start Address. */
    .rxBufStartAddr       = MCAN0_INST_MCAN_RX_BUFF_START_ADDR,
    /* Rx Buffer Element Size. */
    .rxBufElemSize        = DL_MCAN_ELEM_SIZE_64BYTES,
    /* Rx FIFO0 Element Size. */
    .rxFIFO0ElemSize      = DL_MCAN_ELEM_SIZE_64BYTES,
    /* Rx FIFO1 Element Size. */
    .rxFIFO1ElemSize      = DL_MCAN_ELEM_SIZE_64BYTES,
};



static const DL_MCAN_BitTimingParams   gMCAN0BitTimes = {
    /* Arbitration Baud Rate Pre-scaler. */
    .nomRatePrescalar   = 0,
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

//mcan_multi_message_tx.c
#define LED0_STATUS_ON ((uint8_t) 0x01)
#define LED0_STATUS_OFF ((uint8_t) 0x00)
#define LED1_STATUS_ON ((uint8_t) 0x01)
#define LED1_STATUS_OFF ((uint8_t) 0x00)

volatile bool gTXMsg;
volatile bool error;

void Task::CAN_test(void*) {

   //SYSCFG_DL_initPower
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_MCAN_reset(MCAN0_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_MCAN_enablePower(MCAN0_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    //SYSCFG_DL_GPIO_init
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_SWITCHES_USER_SWITCH_1_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setUpperPinsPolarity(GPIO_SWITCHES_PORT, DL_GPIO_PIN_21_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(GPIO_SWITCHES_PORT, GPIO_SWITCHES_USER_SWITCH_1_PIN);
    DL_GPIO_enableInterrupt(GPIO_SWITCHES_PORT, GPIO_SWITCHES_USER_SWITCH_1_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_MCAN0_IOMUX_CAN_TX, GPIO_MCAN0_IOMUX_CAN_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_MCAN0_IOMUX_CAN_RX, GPIO_MCAN0_IOMUX_CAN_RX_FUNC);


    //SYSCFG_DL_SYSCTL_init
    //Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);


    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();

    //SYSCFG_DL_MCAN0_init
    DL_MCAN_RevisionId revid_MCAN0;
    System::uart_ui.nputs(ARRANDN("CAN: Breaks Here vvv (If True)" NEWLINE));

    DL_MCAN_enableModuleClock(MCAN0_INST);

    DL_MCAN_setClockConfig(MCAN0_INST, (DL_MCAN_ClockConfig *) &gMCAN0ClockConf);

    /* Get MCANSS Revision ID. */
    DL_MCAN_getRevisionId(MCAN0_INST, &revid_MCAN0);

    /* Wait for Memory initialization to be completed. */
    while(false == DL_MCAN_isMemInitDone(MCAN0_INST));

    /* Put MCAN in SW initialization mode. */

    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_SW_INIT);

    /* Wait till MCAN is not initialized. */
    while (DL_MCAN_OPERATION_MODE_SW_INIT != DL_MCAN_getOpMode(MCAN0_INST));

    /* Initialize MCAN module. */
    DL_MCAN_init(MCAN0_INST, (DL_MCAN_InitParams *) &gMCAN0InitParams);

    /* Configure MCAN module. */
    DL_MCAN_config(MCAN0_INST, (DL_MCAN_ConfigParams*) &gMCAN0ConfigParams);

    /* Configure Bit timings. */
    DL_MCAN_setBitTime(MCAN0_INST, (DL_MCAN_BitTimingParams*) &gMCAN0BitTimes);

    /* Configure Message RAM Sections */
    DL_MCAN_msgRAMConfig(MCAN0_INST, (DL_MCAN_MsgRAMConfigParams*) &gMCAN0MsgRAMConfigParams);



    /* Set Extended ID Mask. */
    DL_MCAN_setExtIDAndMask(MCAN0_INST, MCAN0_INST_MCAN_EXT_ID_AND_MASK );



    /* Loopback mode */

    /* Take MCAN out of the SW initialization mode */
    uint32_t cccr = MCAN0_INST->MCANSS.MCAN.MCAN_CCCR;
    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL);

    System::uart_ui.nputs(ARRANDN("GetOpMode: "));

    char _str[100];
    snprintf(ARRANDN(_str), "%d,%d,%d", DL_MCAN_getOpMode(MCAN0_INST), DL_MCAN_OPERATION_MODE_NORMAL, cccr);
    System::uart_ui.nputs(ARRANDN(_str));

    System::uart_ui.nputs(ARRANDN(NEWLINE "CAN: Breaks Here vvv (Since OpMode not in NORMAL state)" NEWLINE));
    while (DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(MCAN0_INST));
    System::uart_ui.nputs(ARRANDN("CAN: Breaks Here ^^^ (Since OpMode not in NORMAL state)"  NEWLINE));



    /* Enable MCAN mopdule Interrupts */
    DL_MCAN_enableIntr(MCAN0_INST, MCAN0_INST_MCAN_INTERRUPTS, 1U);

    DL_MCAN_selectIntrLine(MCAN0_INST, DL_MCAN_INTR_MASK_ALL, DL_MCAN_INTR_LINE_NUM_1);
    DL_MCAN_enableIntrLine(MCAN0_INST, DL_MCAN_INTR_LINE_NUM_1, 1U);

    /* Enable MSPM0 MCAN interrupt */
    DL_MCAN_clearInterruptStatus(MCAN0_INST,(DL_MCAN_MSP_INTERRUPT_LINE1));
    DL_MCAN_enableInterrupt(MCAN0_INST,(DL_MCAN_MSP_INTERRUPT_LINE1));


    System::uart_ui.nputs(ARRANDN("CAN: Breaks Here vvv" NEWLINE));
    //SYSCFG_DL_SYSCTL_CLK_init
    //while ((DL_SYSCTL_getClockStatus() & (DL_SYSCTL_CLK_STATUS_HFCLK_GOOD
    //     | DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
    //       != (DL_SYSCTL_CLK_STATUS_HFCLK_GOOD
    //     | DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
    //{
        /* Ensure that clocks are in default POR configuration before initialization.
        * Additionally once LFXT is enabled, the internal LFOSC is disabled, and cannot
        * be re-enabled other than by executing a BOOTRST. */
    //    ;
    //}
    System::uart_ui.nputs(ARRANDN("CAN: Breaks Here ^^^"  NEWLINE));

    DL_MCAN_TxBufElement txMsg0;
    DL_MCAN_TxBufElement txMsg1;

    /* Initialize message0 to transmit. */

    /* Identifier Value. */
    txMsg0.id = ((uint32_t)(0x4)) << 18U;
    /* Transmit data frame. */
    txMsg0.rtr = 0U;
    /* 11-bit standard identifier. */
    txMsg0.xtd = 0U;
    /* ESI bit in CAN FD format depends only on error passive flag. */
    txMsg0.esi = 0U;
    /* Transmitting 4 bytes. */
    txMsg0.dlc = 1U;
    /* CAN FD frames transmitted with bit rate switching. */
    txMsg0.brs = 1U;
    /* Frame transmitted in CAN FD format. */
    txMsg0.fdf = 1U;
    /* Store Tx events. */
    txMsg0.efc = 1U;
    /* Message Marker. */
    txMsg0.mm = 0xAAU;
    /* Data bytes. */
    txMsg0.data[0] = LED0_STATUS_ON;

    /* Initialize message1 to transmit. */

    /* Identifier Value. */
    txMsg1.id = ((uint32_t)(0x3)) << 18U;
    /* Transmit data frame. */
    txMsg1.rtr = 0U;
    /* 11-bit standard identifier. */
    txMsg1.xtd = 0U;
    /* ESI bit in CAN FD format depends only on error passive flag. */
    txMsg1.esi = 0U;
    /* Transmitting 4 bytes. */
    txMsg1.dlc = 1U;
    /* CAN FD frames transmitted with bit rate switching. */
    txMsg1.brs = 1U;
    /* Frame transmitted in CAN FD format. */
    txMsg1.fdf = 1U;
    /* Store Tx events. */
    txMsg1.efc = 1U;
    /* Message Marker. */
    txMsg1.mm = 0xAAU;
    /* Data bytes. */
    txMsg1.data[0] = LED1_STATUS_ON;

    NVIC_EnableIRQ(GPIO_SWITCHES_INT_IRQN);

    System::uart_ui.nputs(ARRANDN("CAN: Breaks Here vvv" NEWLINE));
    while (DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(MCAN0_INST));
    System::uart_ui.nputs(ARRANDN("CAN: Breaks Here ^^^" NEWLINE));

    while (1) {
        /* Waits until button is pressed to send the message*/
        while (gTXMsg == false) {
            System::uart_ui.nputs(ARRANDN("Waiting for Button Press" NEWLINE));
            __WFE();
        }
        System::uart_ui.nputs(ARRANDN("CAN: Button Pressed" NEWLINE));
        gTXMsg = false;

        if (txMsg0.data[0] == LED0_STATUS_ON) {
            txMsg0.data[0] = LED0_STATUS_OFF;
        } else {
            txMsg0.data[0] = LED0_STATUS_ON;
        }

        if (txMsg1.data[0] == LED1_STATUS_ON) {
            txMsg1.data[0] = LED1_STATUS_OFF;
        } else {
            txMsg1.data[0] = LED1_STATUS_ON;
        }

        /* Write Tx Message to the Message RAM. */
        DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0, &txMsg0);

        /* Write Tx Message to the Message RAM. */
        DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 1, &txMsg1);

        /* Add request for transmission. */
        DL_MCAN_TXBufAddReq(MCAN0_INST, 0);
        System::uart_ui.nputs(ARRANDN("CAN: Transmit 0" NEWLINE));

        /* Add request for transmission. */
        DL_MCAN_TXBufAddReq(MCAN0_INST, 1);
        System::uart_ui.nputs(ARRANDN("CAN: Transmit 1" NEWLINE));

        while ((DL_MCAN_getTxBufTransmissionStatus(MCAN0_INST) & (1 << 0)) == 0);

        System::uart_ui.nputs(ARRANDN("CAN: Finished Transmitting" NEWLINE));
    }
}

void GROUP1_IRQHandler(void) {
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_SWITCHES_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIO_SWITCHES_PORT)) {
                case DL_GPIO_IIDX_DIO21:
                    gTXMsg = true;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}
