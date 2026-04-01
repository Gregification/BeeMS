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

#define READ
#define WRITE
#define BUFF_SIZE 128

void printFS(FATFS fs);

void Task::SDC_test_task(void *){
    System::UART::uart_ui.nputs(ARRANDN("SDC_test_task start" NEWLINE));

    FRESULT res;
    FATFS fs; /* File system object */
    UINT byte_count;
    int total_bytes = 0;
    WORD n = 0;
    char Line[BUFF_SIZE];


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

#ifdef WRITE

    Line[0] = 'p';
    Line[1] = 'a';
    Line[2] = 'n';
    Line[3] = 'i';
    Line[4] = 'k';
    Line[5] = '\n';

    System::UART::uart_ui.nputs(ARRANDN("Writing.... \n\r"));
    System::UART::uart_ui.nputs(ARRANDN("res:"));
    System::UART::uart_ui.putu32h(pf_write(Line, 6, &byte_count));
    System::UART::uart_ui.nputs(ARRANDN("\n\r"));
    System::UART::uart_ui.nputs(ARRANDN("bytes written: "));
    System::UART::uart_ui.putu32d(byte_count);
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    System::UART::uart_ui.nputs(ARRANDN("Writing 500 times.... \n\r"));

    for(int i = 0; i < 500; i++)
    {
        pf_write(Line, 6, &byte_count);
        total_bytes += byte_count;
    }
    System::UART::uart_ui.nputs(ARRANDN("bytes written: "));
    System::UART::uart_ui.putu32d(total_bytes);
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));


    System::UART::uart_ui.nputs(ARRANDN("Finalizing.... \n\r res: "));
    System::UART::uart_ui.putu32h(pf_write(0, 0, &byte_count));
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

#endif

#ifdef READ


    System::UART::uart_ui.nputs(ARRANDN("opening file.txt.... \n\r res: "));
    System::UART::uart_ui.putu32h(pf_open("FILE.TXT"));
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    System::UART::uart_ui.nputs(ARRANDN("Reading.... "));
    System::UART::uart_ui.nputs(ARRANDN("\n\r res: "));

    System::UART::uart_ui.putu32h(pf_read(Line, BUFF_SIZE, &byte_count));
    System::UART::uart_ui.nputs(ARRANDN("\n\r"));

    System::UART::uart_ui.nputs(ARRANDN("bytes read: "));
    System::UART::uart_ui.putu32h(byte_count);
    System::UART::uart_ui.nputs(ARRANDN("\n\n\r"));

    System::UART::uart_ui.nputs(ARRANDN("Data: \n\r"));
    System::UART::uart_ui.nputs(ARRANDN(Line));
#endif
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


