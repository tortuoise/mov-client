#ifndef __EMPTY__H__
#define __EMPTY__H__

#include <ti/display/Display.h>
#include "semaphore.h"
#include "uart_term.h"

#define MAX_BUF_SIZE            (1400)
#define MAX_TEXT_PAD_SIZE       (256)
#define WLAN_SCAN_COUNT         (30)

#define OS_ERROR                ( \
        "OS error, please refer \"NETAPP ERRORS CODES\" section in errno.h")

static Display_Handle display;

typedef union
{
    uint8_t                    nwData[MAX_BUF_SIZE];
    int8_t                     textPad[MAX_TEXT_PAD_SIZE];
    SlWlanNetworkEntry_t       netEntries[WLAN_SCAN_COUNT];
    SlWlanExtNetworkEntry_t    extNetEntries[WLAN_SCAN_COUNT];
}gDataBuffer_t;

typedef struct connectionControlBlock_t
{
    sem_t    connectEventSyncObj;
    sem_t    ip4acquireEventSyncObj;
    sem_t    ip6acquireEventSyncObj;
    sem_t    eventCompletedSyncObj;
    uint32_t GatewayIP;
    uint8_t  ConnectionSSID[SL_WLAN_SSID_MAX_LENGTH +1];
    uint8_t  ConnectionBSSID[SL_WLAN_BSSID_LENGTH];
    uint32_t DestinationIp;
    uint32_t IpAddr;
    uint32_t StaIp;
    uint32_t Ipv6Addr[4];
}connection_CB;

typedef struct appControlBlock_t
{
    /* Status Variables */
    /* This bit-wise status variable shows the state of the NWP */
    uint32_t Status;                    
     /* This field keeps the device's role (STA, P2P or AP) */
    uint32_t Role;                     
    /* This flag lets the application to exit */
    uint32_t Exit;                      
    /* Sets the number of Ping attempts to send */
    uint32_t PingAttempts;                  
    /* Data & Network entry Union */
    gDataBuffer_t gDataBuffer;

    /* STA/AP mode CB */
    connection_CB CON_CB;

    /* WoWLAN semaphore */
    sem_t WowlanSleepSem;

}appControlBlock;

extern appControlBlock app_CB;



#endif //__EMPTY__H__
