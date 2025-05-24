/*
 * fiddle_task.cpp
 *
 *  Created on: May 22, 2025
 *      Author: FSAE
 */

#include "fiddle_task.hpp"

#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/can.h>

void Task::Fiddle::main(void * v)
{

    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0))
        {}

    CANInit(CAN0_BASE);

    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 250e3);

    CANEnable(CAN0_BASE);

    uint8_t arr[] = {0x1,0x1,0x1,0xf,0x1,0x1,0x1,0x1};

    tCANMsgObject tx_handle;
    tx_handle.ui32MsgID     = 0x400;
    tx_handle.ui32Flags     = 0;
//    tx_handle.ui32Flags     = MSG_OBJ_TYPE_TX;
    tx_handle.ui32MsgLen    = sizeof(arr);
    tx_handle.pui8MsgData   = arr;
    System::nputsUIUART(STRANDN("start"NEWLINE));
    for(;;){
        CANMessageSet(CAN0_BASE, 1, &tx_handle, MSG_OBJ_TYPE_TX);
//        while(CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST)){
            vTaskDelay(pdMS_TO_TICKS(500));
//        }
//        System::nputsUIUART(STRANDN("sent"NEWLINE));
    }

}
