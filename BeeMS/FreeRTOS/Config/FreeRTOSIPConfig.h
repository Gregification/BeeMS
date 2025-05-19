/*
 * FreeRTOSIPConfig.h
 *
 *  Created on: May 18, 2025
 *      Author: FSAE
 */

#ifndef FREERTOS_IP_CONFIG_H
#define FREERTOS_IP_CONFIG_H

#include "FreeRTOSConfig.h"

/*--- device specific ---------------------------------------------------------------------------------------*/

#define ipconfigBYTE_ORDER                              pdFREERTOS_LITTLE_ENDIAN

/*--- resource management -----------------------------------------------------------------------------------*/

#define ipconfigNETWORK_MTU                             1526
#define ipconfigTCP_MSS                                 1460
#define ipconfigTCP_TX_BUFFER_LENGTH                    ( 16 * ipconfigTCP_MSS )
#define ipconfigTCP_RX_BUFFER_LENGTH                    ( 16 * ipconfigTCP_MSS )


/*--- options -----------------------------------------------------------------------------------------------*/

#define ipconfigUSE_TCP_WIN                             1
#define ipconfigUSE_IPv4                                1
#define ipconfigUSE_TCP                                 1
#define ipconfigUSE_DHCP                                1
#define ipconfigDHCP_REGISTER_HOSTNAME                  1
#define ipconfigDHCP_FALL_BACK_AUTO_IP                  1
#define ipconfigMAXIMUM_DISCOVER_TX_PERIOD              pdMS_TO_TICKS(500)
#define ipconfigREPLY_TO_INCOMING_PINGS                 1
#define ipconfigSUPPORT_OUTGOING_PINGS                  1 // for debugging


#endif /* FREERTOS_CONFIG_FREERTOSIPCONFIG_H_ */
