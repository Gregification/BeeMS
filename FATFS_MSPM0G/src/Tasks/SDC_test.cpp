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


void printFS(FATFS fs);

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
    System::UART::uart_ui.putu32h(pf_open("FILE.TXT"));
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    printFS(fs);

    vTaskDelay(100);

    Line[0] = 'p';
    Line[1] = 'a';
    Line[2] = 'n';
    Line[3] = 'i';
    Line[4] = 'k';
    Line[5] = '\0';

    System::UART::uart_ui.nputs(ARRANDN("Writing.... \""));
    System::UART::uart_ui.nputs(ARRANDN(Line));
    System::UART::uart_ui.nputs(ARRANDN("\"\n\r res: "));

    System::UART::uart_ui.putu32h(pf_write(Line, 6, &bw));
    System::UART::uart_ui.nputs(ARRANDN("\n\r"));

    System::UART::uart_ui.nputs(ARRANDN("bytes written: "));
    System::UART::uart_ui.putu32h(bw);
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    System::UART::uart_ui.nputs(ARRANDN("Finalizing.... \n\r res: "));
    System::UART::uart_ui.putu32h(pf_write(0, 0, &bw));
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    while(1);


    System::FailHard("SDC_test_task ended" NEWLINE);
    vTaskDelete(NULL);
}

void printFS(FATFS fs)
{
    System::UART::uart_ui.nputs(ARRANDN("FAT sub type: "));
        System::UART::uart_ui.putu32h(fs.fs_type);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("File status: "));
        System::UART::uart_ui.putu32h(fs.flag);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("Number of sectors per cluster: "));
        System::UART::uart_ui.putu32h(fs.csize);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("pad1: "));
        System::UART::uart_ui.putu32h(fs.pad1);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("Number of root directory entries (0 on FAT32): "));
        System::UART::uart_ui.putu32h(fs.n_rootdir);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("Number of FAT entries (= number of clusters + 2): "));
        System::UART::uart_ui.putu32h(fs.n_fatent);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("FAT start sector: "));
        System::UART::uart_ui.putu32h(fs.fatbase);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("Root directory start sector (Cluster# on FAT32): "));
        System::UART::uart_ui.putu32h(fs.dirbase);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("Data start sector: "));
        System::UART::uart_ui.putu32h(fs.database);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("File R/W pointer: "));
        System::UART::uart_ui.putu32h(fs.fptr);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("File size: "));
        System::UART::uart_ui.putu32h(fs.fsize);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("File start cluster: "));
        System::UART::uart_ui.putu32h(fs.org_clust);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("File current cluster: "));
        System::UART::uart_ui.putu32h(fs.curr_clust);
        System::UART::uart_ui.nputs(ARRANDN("\n\r"));

        System::UART::uart_ui.nputs(ARRANDN("File current data sector: "));
        System::UART::uart_ui.putu32h(fs.dsect);
        System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));
}


