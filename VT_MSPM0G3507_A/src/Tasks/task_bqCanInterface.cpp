/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *
 * responsible for can packets
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf
 */

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <Tasks/task_bqCanInterface.hpp>

#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"
#include "Core/Networking/CANComm.hpp"

System::UART::UART &uart = System::uart_ui;

void Task::bqCanInterface_task(void *){
    /*
     * handles all non essential functions
     */
    uart.nputs(ARRANDN("bqCanInterface_task start" NEWLINE));

    while(1){
        using namespace Networking::CAN;

        do {
            DL_MCAN_TxBufElement txmsg = {
                    .id     = 0x1,      // CAN id, 11b->[28:18], 29b->[28:0]
                    .rtr    = 0,        // 0: data frame, 1: remote frame
                    .xtd    = 1,        // 0: 11b id, 1: 29b id
                    .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
                    .dlc    = 3,        // data byte count, see DL comments
                    .brs    = 0,        // 0: no bit rate switching, 1: yes brs
                    .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
                    .efc    = 0,        // 0: dont store Tx events, 1: store
                    .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
                };
            txmsg.data[0] = 6;
            txmsg.data[1] = 7;
            txmsg.data[2] = 8;

            J1939::ID canID;
            canID.priority = 0b111;

            DL_MCAN_TxFIFOStatus tf;
            DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);

            uint32_t bufferIndex = tf.putIdx;
            uart.nputs(ARRANDN("TX from buffer "));
            uart.putu32d(bufferIndex);
            uart.nputs(ARRANDN("" NEWLINE));

            DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
            DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);

            vTaskDelay(pdMS_TO_TICKS(400));
        } while(1);
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
