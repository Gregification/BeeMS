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
#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS          16
#define ipconfigTCP_TX_BUFFER_LENGTH                    ( ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS * ipconfigTCP_MSS )
#define ipconfigTCP_RX_BUFFER_LENGTH                    ( ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS * ipconfigTCP_MSS )
#define ipconfigIP_TASK_PRIORITY                        ( configMAX_PRIORITIES - 1 )

/*--- options -----------------------------------------------------------------------------------------------*/

#define ipconfigUSE_TCP_WIN                             1
#define ipconfigUSE_IPv4                                1
#define ipconfigUSE_TCP                                 1
#define ipconfigUSE_UDP                                 1
#define ipconfigUSE_ARP                                 1
#define ipconfigUSE_ICMP                                1
#define ipconfigUSE_DHCP                                0
#define ipconfigUSE_DNS                                 1 // dnc
#define ipconfigUSE_ICMPv6                              0
#define ipconfigUSE_DHCPv6                              0
#define ipconfigUSE_SNTP                                1
#define ipconfigUSE_LLMNR                               1
#define ipconfigUSE_NBNS                                0
#define ipconfigUSE_IPv6                                0
#define ipconfigUSE_NETWORK_EVENT_HOOK                  1
#define ipconfigETHERNET_USE_100MB                      1
#define ipconfigDHCP_REGISTER_HOSTNAME                  1
#define ipconfigDHCP_FALL_BACK_AUTO_IP                  1
#define ipconfigMAXIMUM_DISCOVER_TX_PERIOD              pdMS_TO_TICKS(4500)
#define ipconfigREPLY_TO_INCOMING_PINGS                 1
#define ipconfigSUPPORT_OUTGOING_PINGS                  1
#define ipconfigCOMPATIBLE_WITH_SINGLE                  1
#define ipconfigZERO_COPY_TX_DRIVER                     1 // property of the FreeRTOS port
#define ipconfigZERO_COPY_RX_DRIVER                     1 // property of the FreeRTOS port
#define ipconfigHAS_PRINTF                              1
#define ipconfigETHERNET_AN_ENABLE                      1
#define ipconfigFILTER_OUT_NON_ETHERNET_II_FRAMES       0
#define ipconfigTCP_KEEP_ALIVE				            1
#define ipconfigTCP_KEEP_ALIVE_INTERVAL		            5 /* (seconds) */
#define ipconfigUSE_DNS_CACHE				            1
#define ipconfigDNS_CACHE_NAME_LENGTH		            32
#define ipconfigDNS_CACHE_ENTRIES			            4
#define ipconfigDNS_REQUEST_ATTEMPTS		            2
#define ipconfigMAX_ARP_RETRANSMISSIONS                 3
#define ipconfigMAX_ARP_AGE                             15 /* (tens of seconds, 150 -> 1500 seconds) */
#define ipconfigUDP_TIME_TO_LIVE                        20
#define ipconfigTCP_TIME_TO_LIVE                        20
#define ipconfigCAN_FRAGMENT_OUTGOING_PACKETS           0 /* otherwise unsavory */
#define ipconfigHAS_PRINTF                              1
#define ipconfigHAS_DEBUG_PRINTF                        1
#define ipconfigTCP_IP_SANITY                           1
#define ipconfigETHERNET_MINIMUM_PACKET_BYTES           60
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES     0
#define ipconfigETHERNET_DRIVER_FILTERS_PACKETS         0

#endif /* FREERTOS_CONFIG_FREERTOSIPCONFIG_H_ */
