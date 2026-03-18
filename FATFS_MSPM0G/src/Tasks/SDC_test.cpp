#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>

#include <Tasks/SDC_test.hpp>
#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"




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


