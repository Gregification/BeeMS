/*
 * hooks.cpp
 *
 *  Created on: May 31, 2025
 *      Author: FSAE
 */

#include "hooks.h"

#include <Core/system.hpp>
#include <driverlib/interrupt.h>

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void ) {
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    IntMasterDisable();
    for( ;; );
}


/*-----------------------------------------------------------*/

void vApplicationIdleHook( void ) {
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}


/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName ) {
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for( ;; );
}


/*-----------------------------------------------------------*/

void *malloc( size_t xSize ) {
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for( ;; );
}


/*-----------------------------------------------------------*/

const char * pcApplicationHostnameHook( void ) {
    return PROJECT_NAME " " PROJECT_VERSION;
}

/*-----------------------------------------------------------*/

/*
* Callback that provides the inputs necessary to generate a randomized TCP
* Initial Sequence Number per RFC 6528.  In this case just a psuedo random
* number is used so THIS IS NOT RECOMMENDED FOR PRODUCTION SYSTEMS.
*/
extern uint32_t ulApplicationGetNextSequenceNumber(
    uint32_t ulSourceAddress,
    uint16_t usSourcePort,
    uint32_t ulDestinationAddress,
    uint16_t usDestinationPort )
{
     ( void ) ulSourceAddress;
     ( void ) usSourcePort;
     ( void ) ulDestinationAddress;
     ( void ) usDestinationPort;

     static uint32_t randomnumber; // very random number
     return randomnumber++;
}

/*-----------------------------------------------------------*/

BaseType_t xApplicationGetRandomNumber( uint32_t * pulNumber ){
    return *pulNumber ^ 0xBEE;
}

/*-----------------------------------------------------------*/

/* called when the network connects or disconnects
 * https://www.freertos.org/Documentation/03-Libraries/02-FreeRTOS-plus/02-FreeRTOS-plus-TCP/09-API-reference/57-vApplicationIPNetworkEventHook
 */
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent ) {
    // explode
    System::nputsUIUART(STRANDN("network connect/disconnect vApplicationIPNetworkEventHook_Multi" NEWLINE));
}

/*-----------------------------------------------------------*/

/* decides how to resolve DHCP communication decision points
 * https://www.freertos.org/Documentation/03-Libraries/02-FreeRTOS-plus/02-FreeRTOS-plus-TCP/09-API-reference/60-xApplicationDHCPHook
 */
eDHCPCallbackAnswer_t xApplicationDHCPHook( eDHCPCallbackPhase_t eDHCPPhase,uint32_t ulIPAddress ){
    // vaporise
    return eDHCPContinue;
}

/*-----------------------------------------------------------*/

/* stop reading this, what do you think it does. use ur brain. we didnt leave agartha for you to be retarded. (((we))) left so we can be different colors
 * https://www.freertos.org/Documentation/03-Libraries/02-FreeRTOS-plus/02-FreeRTOS-plus-TCP/09-API-reference/59-vApplicationPingReplyHook
 */
void vApplicationPingReplyHook( ePingReplyStatus_t eStatus, uint16_t usIdentifier ){
    System::nputsUIUART(STRANDN("pong " NEWLINE));
}

/*-----------------------------------------------------------*/





