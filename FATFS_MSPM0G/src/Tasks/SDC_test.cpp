
#include <Core/CETRX.hpp>
#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>

#include <Tasks/SDC_test.hpp>
#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"

/*** setup ***************************************************/

#define GO_IDLE_STATE               0   // Software reset.
#define SEND_OP_COND                1   // Initiate initialization process.
#define APP_SEND_OP_COND            ?   // For only SDC. Initiate initialization process.
#define SEND_IF_COND                8   // For only SDC V2. Check voltage range.
#define SEND_CSD                    9   // Read CSD register.
#define SEND_CID                    10  //Read CID register.
#define STOP_TRANSMISSION           12  //Stop to read data.
#define SET_BLOCKLEN                16  // Change R/W block size.
#define READ_SINGLE_BLOCK           17  // Read a block.
#define READ_MULTIPLE_BLOCK         18  // Read multiple blocks.
#define SET_BLOCK_COUNT             23  // For only MMC. Define number of blocks to transfer with next multi-block read/write command.
#define SET_WR_BLOCK_ERASE_COUNT    ?   // T   For only SDC. Define number of blocks to pre-erase with next multi-block write command.
#define WRITE_BLOCK                 24  //  Write a block.
#define WRITE_MULTIPLE_BLOCK        25  // Write multiple blocks.
#define APP_CMD                     55  // Leading command of ACMD<n> command.
#define READ_OCR                    58  // Read OCR.




/*** setup ***************************************************/
System::UART::UART &            uart1 = System::UART::uart_ui;


void Task::SDC_test_task(void *){
    uart1.nputs(ARRANDN("SDC_test_task start" NEWLINE));


    while(true){
        //TODO uhhhhhhhh....
    }


    System::FailHard("SDC_test_task ended" NEWLINE);
    vTaskDelete(NULL);
}

void SDC_send_command(char command, uint32_t args)
{
    char byte1 = 0x40 | (0x3f & command);   //start bit '0' + transmission bit '1' + command index
    //Byte 2-5 are the args
    char byte6 = 0x01;                      //CRC + end bit '1'

    //TODO start transmission over SPI....
}
