/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *
 * if you thought java swing programs got bad get ready for worse. YOU are the build tool
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf
 */

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>

#include "Tasks/task_non_BMS.hpp"
#include "Tasks/task_BMS.hpp"
#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"

System::UART::UART &uart = System::uart_ui;

void RSFrame();

void Task::non_BMS_functions_task(void *){
    /*
     * handles all non essential functions
     * - expects complete control over uart ui
     */
    uart.nputs(ARRANDN("non_BMS_functions_task start" NEWLINE));

    while(1){
        // handles OPC/RapidSCADA comms
        RSFrame();

        taskYIELD();
    }

    System::FailHard("non_BMS_functions_task ended" NEWLINE);
    vTaskDelete(NULL);
}

void RSFrame(){

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
