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
#include "Core/VT.hpp"
#include "Core/std alternatives/string.hpp"
#include "Core/Networking/bridge_CAN_Modbus.hpp"
#include "Core/Networking/ModbusRegisters.hpp"

auto & uart     = System::uart_ui;
auto & can      = System::canFD0;
constexpr DL_MCAN_RX_FIFO_NUM canfifo = DL_MCAN_RX_FIFO_NUM::DL_MCAN_RX_FIFO_NUM_0;

void Task::bqCanInterface_task(void *){
    using namespace Networking;

    /*
     * handles all non essential functions
     */
    uart.nputs(ARRANDN("bqCanInterface_task start" NEWLINE));


    DL_MCAN_RxBufElement canrx;
    DL_MCAN_RxFIFOStatus canrxf = { .num = canfifo };

    union _TRXBuffer {
        Modbus::MBAPHeader mbap;
        DL_MCAN_TxBufElement cantx;

        uint8_t arr[Bridge::CANModbus::PKTBUFFSIZE];
    } rxbuf = {0};
    union _TXBuffer {
        Modbus::MBAPHeader mbap;

        uint8_t arr[Bridge::CANModbus::PKTBUFFSIZE];
    } txbuf = {0};


    while(true){

        /*** poll for incoming request ********/
        if(can.takeResource(pdMS_TO_TICKS(3e3))) {
            DL_MCAN_getRxFIFOStatus(can.reg, &canrxf);

            if(canrxf.fillLvl == 0) { // is fifo empty?
                can.giveResource();
                vTaskDelay(pdMS_TO_TICKS(10)); // eye-balled value
                continue;
            }

            DL_MCAN_readMsgRam(
                    can.reg,
                    DL_MCAN_MEM_TYPE::DL_MCAN_MEM_TYPE_FIFO,
                    0, // arbitrary. value ignored
                    canrxf.num,
                    &canrx
                );
            DL_MCAN_writeRxFIFOAck(can.reg, canrxf.num, canrxf.getIdx);

            can.giveResource();
        } else {
            vTaskDelay(pdMS_TO_TICKS(10)); // eye-balled value
            System::uart_ui.nputs(ARRANDN("can timeout" NEWLINE));
            continue;
        }


        /*** parse packet *********************/

        uint8_t socketbuffer;

        if(Bridge::CANModbus::CAN_to_ModbusTCP(&canrx, &rxbuf.mbap, &socketbuffer)) {
            uart.nputs(ARRANDN("parsed Modbus over CAN" NEWLINE));

            /*** validation ***********************/

            if(rxbuf.mbap.adu[0].unitID != VT::id) {
                uart.nputs(ARRANDN("not my("));
                uart.put32d(VT::id);
                uart.nputs(ARRANDN(") id: "));
                uart.put32d(rxbuf.mbap.adu[0].unitID);
                uart.nputs(ARRANDN(NEWLINE));

                continue; // ignore packet
            }

            /*** process packet *******************/

            if(Modbus::ProcessRequest(&rxbuf.mbap, sizeof(rxbuf), &txbuf.mbap, sizeof(txbuf))) {
                uart.nputs(ARRANDN(" processed Modbus request" NEWLINE));

                if(Bridge::CANModbus::ModbusTCP_to_CAN(&txbuf.mbap, &rxbuf.cantx, socketbuffer)){
                    // transmit CAN
                    uart.nputs(ARRANDN("  response CAN packet ready to send" NEWLINE));

                    uart.nputs(ARRANDN(NEWLINE " \tdump CAN: "));
                    for(uint8_t j = 0; j < System::CANFD::DLC2Len(&rxbuf.cantx); j++){
                        if(j % 10 == 0)
                            uart.nputs(ARRANDN(NEWLINE " \t"));

                        uart.nputs(ARRANDN(" "));
                        uart.putu32h(rxbuf.cantx.data[j]);
                    }
                    uart.nputs(ARRANDN(NEWLINE));

                    DL_MCAN_TxFIFOStatus tf;

                    for(uint8_t i = 3; i != 0; i--) {
                        DL_MCAN_getTxFIFOQueStatus(can.reg, &tf);

                        if(tf.fifoFull){
                            vTaskDelay(pdMS_TO_TICKS(2));
                            continue;
                        }

                        DL_MCAN_writeMsgRam(can.reg, DL_MCAN_MEM_TYPE_FIFO, tf.putIdx, &rxbuf.cantx);
                        DL_MCAN_TXBufAddReq(can.reg, tf.getIdx);

                        break;
                    }

                } else
                    uart.nputs(ARRANDN("  failed to translate ModbusTCP response to CAN " NEWLINE));
            } else
                uart.nputs(ARRANDN(" failed to process Modbus request" NEWLINE));
        } else
            uart.nputs(ARRANDN("failed to parse Modbus over CAN." NEWLINE));

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
