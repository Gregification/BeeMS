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
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinConfigure(GPIO_PB4_CAN0RX);

    SysCtlPeripheralClockGating(SYSCTL_PERIPH_CAN0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0))
        {};

    CANInit(CAN0_BASE);

    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500e3);

    CANEnable(CAN0_BASE);

    uint8_t arr[] = {1,2,3};

    tCANMsgObject tx_handle;
    tx_handle.ui32MsgID = 0x400u;
    tx_handle.ui32MsgIDMask = 0x7f8u;
    tx_handle.ui32MsgLen = sizeof(arr);
    tx_handle.ui32Flags = 0;
    tx_handle.pui8MsgData = arr;
        CANMessageSet(CAN0_BASE, 5,  &tx_handle, MSG_OBJ_TYPE_TX);
    for(;;){

//        while(CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST)){
//
//        }
    }

}
