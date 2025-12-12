/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *
 * - responsible for most can packets
 * - uses Modbus-TCP like packets. limited to 64B of data.
 *      - so long the Modbus-TCP packets fits within 64B it can be transmitted
 *          over CAN bus to this task. Response will be teh data of a Modbus-TCP packet.
 *      - IS NOT A 1:1 TRANSLATION OF MODBUS-TCP.
 *      - CAN-FD handles CRC checking in the absence of actual TCP control.
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf
 */

#include <Core/Networking/CAN.hpp>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <Tasks/task_bqCanInterface.hpp>

#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"
#include "Core/Networking/bridge_CAN_Modbus.hpp"
#include "Core/Networking/ModbusRegisters.hpp"

System::UART::UART &uart    = System::uart_ui;
System::CANFD::CANFD &can   = System::canFD0;
constexpr DL_MCAN_RX_FIFO_NUM canFIFONum = DL_MCAN_RX_FIFO_NUM::DL_MCAN_RX_FIFO_NUM_0;

void Task::bqCanInterface_task(void *){
    using namespace Networking;

    /*
     * handles all non essential functions
     */
    uart.nputs(ARRANDN("bqCanInterface_task start" NEWLINE));


    DL_MCAN_RxBufElement rx;
    DL_MCAN_RxFIFOStatus rxfifostatus = {
           .num = canFIFONum,
        };
    CAN::J1939::ID const * rxid = reinterpret_cast<CAN::J1939::ID *>(&rx.id);

    while(true){
        using namespace Networking::Bridge;


        /*** poll for incoming request ********/

        can.takeResource(0);
        {
            DL_MCAN_getRxFIFOStatus(can.reg, &rxfifostatus);
            if(rxfifostatus.fillLvl == 0) { // is fifo empty?
                can.giveResource();
                vTaskDelay(pdMS_TO_TICKS(50)); // eye-balled value
                continue;
            }

            DL_MCAN_readMsgRam(
                    can.reg,
                    DL_MCAN_MEM_TYPE::DL_MCAN_MEM_TYPE_FIFO,
                    0, // arbitrary. value ignored
                    canFIFONum,
                    &rx
                );
            // assume the fifo is only used by this task. so just flush the item immediately since we have a copy
            DL_MCAN_writeRxFIFOAck(can.reg, rxfifostatus.num, rxfifostatus.getIdx);
        }
        can.giveResource();


        /*** parse packet *********************/

        uint8_t parsebuffer[CANModbus::PKTBUFFSIZE];
        auto rxmbap = reinterpret_cast<Modbus::MBAPHeader * >(parsebuffer);

        if(!CANModbus::CAN_to_ModbusTCP(&rx, rxmbap)) {
            uart.nputs(ARRANDN("failed to parse Modbus over CAN." NEWLINE));
            continue;
        }

        uart.nputs(ARRANDN("parsed Modbus over CAN." NEWLINE));


        /*** process packet *******************/


    }

    System::FailHard("bqCanInterface_task ended" NEWLINE);
    vTaskDelete(NULL);
}




// direct CLI stuff
/*************************************************************/
//bool fcli_dumpConfig(char * userInput, uint8_t strlen, char * msg, uint8_t msglen);
//FancyCli::MenuDir cliMenuRoot = {
//        .name = "Voltage Tap",
//        .description = "",
//         .leafCount = 2,
//         .leafs = new FancyCli::MenuLeaf[]{
//                 {
//                     .name       = "dump config",
//                     .description= "",
//                     .accept     = fcli_dumpConfig,
//                 }, {
//                     .name       = "load config",
//                     .description= "",
//                     .accept     = NULL,
//                 }
//         },
//        .dirCount = 2,
//        .dirs = new FancyCli::MenuDir[]{
//                 {
//                     .name       = "dir0 name",
//                     .description= "dir0 des",
//                     .leafCount = 2,
//                     .leafs = new FancyCli::MenuLeaf[]{
//                             {
//                                 .name       = "dir0 leaf0 name",
//                                 .description= "dir0 leaf0 des",
//                                 .accept     = NULL,
//                             }, {
//                                 .name       = "dir0 leaf1 name",
//                                 .description= "dir0 leaf1 des",
//                                 .accept     = NULL,
//                             }
//                     }
//                 },{
//                    .name       = "dir1 name",
//                    .description= "dir1 des",
//                    .leafCount = 1,
//                    .leafs = new FancyCli::MenuLeaf[]{
//                            {
//                                .name       = "dir1 leaf0 name",
//                                .description= "dir1 leaf0 des",
//                                .accept     = NULL,
//                            }
//                    }
//                }
//         },
//    };
//
//FancyCli fcli = { .root = &cliMenuRoot };
//TickType_t tickStart, tickD;
//TickType_t tickFrameMin = pdMS_TO_TICKS(100);
//System::UART::UART &uart = System::uart_ui;
//
///*************************************************************/
//
//
//void UI();
//
//void Task::non_BMS_functions_task(void *){
//    /*
//     * handles all non essential functions
//     * - expects complete control over uart ui
//     */
//    uart.nputs(ARRANDN("non_BMS_functions_task start" NEWLINE));
//
//    fcli.printFrame(uart, true);
//
//    while(1){
////        tickStart = xTaskGetTickCount();
//
//        // cli ui
//        UI();
//
//        // maintain refresh rate
//        tickD = tickStart - xTaskGetTickCount();
//        if(tickD < tickFrameMin)
//            vTaskDelay(tickFrameMin - tickD);
//    }
//
//    System::FailHard("non_BMS_functions_task ended" NEWLINE);
//    vTaskDelete(NULL);
//}
//
//void UI(){
//    static uint8_t      loopCount;
//    constexpr uint8_t   maxLoop = 10;
//
//    loopCount = 0;
//
//    while(!DL_UART_isRXFIFOEmpty(uart.reg) && (loopCount < maxLoop)){
//        loopCount++;
//
//        if(fcli.charInput(&uart, DL_UART_receiveData(uart.reg))){
//            fcli.printFrame(uart, true);
//        }
//    }
//}
//
//bool fcli_dumpConfig(char * userInput, uint8_t strlen, char * msg, uint8_t msglen){
//    while(1){
//        uart.nputs(ARRANDN("meow" NEWLINE));
//    }
//    return false;
//}
