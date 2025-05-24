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

    OrionBMS::TEM::CAN::TMtoBMS pktModuleBc;
    pktModuleBc.data.module_number  = 1;
    pktModuleBc.data.lowest_val     = 10;
    pktModuleBc.data.highest_val    = 69;
    pktModuleBc.data.average_val    = 99;
    pktModuleBc.data.therms_enabled_count   = 80;
    pktModuleBc.data.highest_id     = 1;
    pktModuleBc.data.lowest_id      = 2;
    pktModuleBc.data.updateChecksum();

    tCANMsgObject tx_handle;
    tx_handle.ui32MsgID     = pktModuleBc.can_id;
    tx_handle.ui32Flags     = 0;
    tx_handle.ui32MsgLen    = sizeof(pktModuleBc.data);
    tx_handle.pui8MsgData   = (uint8_t*)&pktModuleBc.data;
    System::nputsUIUART(STRANDN("start"NEWLINE));

    CANMessageSet(can.regs.CANn_BASE, 1, &tx_handle, MSG_OBJ_TYPE_TX);

    for(;;){
        while(CANStatusGet(can.regs.CANn_BASE, CAN_STS_TXREQUEST)){
            System::nputsUIUART(STRANDN("wait"NEWLINE));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

}
