#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>

#include <Tasks/SDC_test.hpp>
#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"

#include <string.h>

#include "Middleware/pff3a/diskio.h"
#include "Middleware/pff3a/pff.h"


/*** setup ***************************************************/


void Task::SDC_test_task(void *){
    System::UART::uart_ui.nputs(ARRANDN("SDC_test_task start" NEWLINE));

    FRESULT res;
    FATFS fs; /* File system object */
    UINT bw;
    WORD n = 0;
    char Line[128];


//    System::UART::uart_ui.nputs(ARRANDN("Disc init.... \n\r res: "));
//    System::UART::uart_ui.putu32h(disk_initialize());
//    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    vTaskDelay(100);


    System::UART::uart_ui.nputs(ARRANDN("Mounting.... \n\r res: "));
    System::UART::uart_ui.putu32h(pf_mount(&fs));
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    vTaskDelay(100);

    System::UART::uart_ui.nputs(ARRANDN("opening file.txt.... \n\r res: "));
    System::UART::uart_ui.putu32h(pf_open("file.txt"));
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    vTaskDelay(100);

    strcat(Line,"Do I need to panik? Probably.\r\n");

    System::UART::uart_ui.nputs(ARRANDN("Writing.... \n\r res: "));
    System::UART::uart_ui.putu32h(pf_write(Line, strlen(Line), &bw));
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    vTaskDelay(100);

    System::UART::uart_ui.nputs(ARRANDN("Finalizing.... \n\r res: "));
    System::UART::uart_ui.putu32h(pf_write(0, 0, &bw));
    System::UART::uart_ui.nputs(ARRANDN("\n\n"));



    vTaskDelay(100);


    System::FailHard("SDC_test_task ended" NEWLINE);
    vTaskDelete(NULL);
}


