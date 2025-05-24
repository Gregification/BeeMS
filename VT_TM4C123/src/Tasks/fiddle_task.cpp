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

#include "Core/system.hpp"
#include "Middleware/OrionBMS.hpp"

void Task::Fiddle::main(void *)
{
    auto &can = System::can0;
    CANBitRateSet(can.regs.CANn_BASE, SysCtlClockGet(), 500e3);
    CANEnable(can.regs.CANn_BASE);

    uint8_t arr[] = {0x1,0x1,0x1,0xf,0x1,0x1,0x1,0x1};

    tCANMsgObject tx_handle;
    tx_handle.ui32MsgID     = 0x4;
    tx_handle.ui32Flags     = 0;
    tx_handle.ui32MsgLen    = sizeof(arr);
    tx_handle.pui8MsgData   = arr;
    System::nputsUIUART(STRANDN("start"NEWLINE));

    CANMessageSet(can.regs.CANn_BASE, 1, &tx_handle, MSG_OBJ_TYPE_TX);

    for(;;){
        while(CANStatusGet(can.regs.CANn_BASE, CAN_STS_TXREQUEST)){
            System::nputsUIUART(STRANDN("wait"NEWLINE));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

}
