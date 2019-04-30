/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>

/* POSIX Header files */
#include <pthread.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/ADC.h>
#include <ti/display/Display.h>
// #include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
//#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
#include <ti/drivers/net/wifi/simplelink.h>

/* Board Header file */
#include "Board.h"
#include "common.h"
#include "empty.h"
#include "time.h"
//#include "uart_term.h"

/* ADC sample count */
#define ADC_SAMPLE_COUNT  (100)

/* Stack size in bytes */
#define TASKSTACKSIZE               (4096)       
#define SPAWN_TASK_PRIORITY         (9)
#define SL_STOP_TIMEOUT             (200)

#define FRAME_LENGTH                (1000)
#define THREADSTACKSIZE   (1024)//(768)
#define DEST_IP_ADDR                SL_IPV4_VAL(192,168,1,100)

/* Control block definition */
typedef struct _PowerMeasure_ControlBlock_t_
{
    uint32_t        slStatus;    //SimpleLink Status
    //mqd_t           queue;
    //sem_t           sem;
    signed char     frameData[FRAME_LENGTH];
    SlSockAddrIn_t  ipV4Addr;
}PowerMeasure_ControlBlock;

int32_t wlanConnect(void);
int32_t initAppVariables();

pthread_t           spawn_thread = (pthread_t)NULL;
uint16_t adcValue1[ADC_SAMPLE_COUNT];
uint32_t adcValue1MicroVolt[ADC_SAMPLE_COUNT];
PowerMeasure_ControlBlock   PowerMeasure_CB;
appControlBlock     app_CB;
pthread_mutex_t voltageMutex;
/*
 *  ======== threadFxn1 ========
 *  Open a ADC handle and get an array of sampling results after
 *  calling several conversions.
 */
void *threadFxn1(void *arg0)
{
    uint16_t     i;
    ADC_Handle   adc;
    ADC_Params   params;
    int_fast16_t res;
    int32_t             status = 0;

    ADC_Params_init(&params);
    adc = ADC_open(Board_ADC1, &params);

    if (adc == NULL) {
        //Display_printf(display, 0, 0, "Error initializing ADC1\n");
        while (1);
    }

    /*status = wlanConnect();
    if (status < 0) {
        UART_PRINT("\r\n wlanConnect error \r\n");
    }*/

    //int32_t ret = 0;


    for (i = 0; i < ADC_SAMPLE_COUNT; i++) {
        res = ADC_convert(adc, &adcValue1[i]);

        if (res == ADC_STATUS_SUCCESS) {

            adcValue1MicroVolt[i] = ADC_convertRawToMicroVolts(adc, adcValue1[i]);

            UART_PRINT("ADC1 raw result (%d): %d\n", i, adcValue1[i]);
            UART_PRINT("\r ADC1 convert result (%d): %d uV\n", i, adcValue1MicroVolt[i]);
            /*Display_printf(display, 0, 0, "ADC1 raw result (%d): %d\n", i,
                           adcValue1[i]);
            Display_printf(display, 0, 0, "ADC1 convert result (%d): %d uV\n", i,
                adcValue1MicroVolt[i]);*/
		GPIO_toggle(Board_GPIO_LED0);
        }
        else {
            UART_PRINT("\r ADC1 convert failed (%d)\n", i);
            //Display_printf(display, 0, 0, "ADC1 convert failed (%d)\n", i);
        }
        //Display_printf(display, 0, 0, "Calling TCPClient w %d \n", DEST_IP_ADDR);
        //Display_printf(display, 0, 0, "Calling TCPClient w %d \n", dest.ipv4);
        //UART_PRINT("[line:%d, error:%d] %d\n\r", __LINE__, nb, port);
        //ret = TCPClient(nb, port, dest, FALSE /*ipv6*/, numPackets, TRUE);
        //ret = TCPClientTest(nb, port, dest, FALSE /*ipv6*/, numPackets, TRUE);
        /*if (ret != 0) {
            UART_PRINT("[line:%d, error:%d] \n\r", __LINE__, ret);
            //Display_printf(display, 0, 0, "TCPClient failed");
        } */
        UART_PRINT("\r\n ***SLEEPING**** %d %d \r\n", ADC_SAMPLE_COUNT, i);
	usleep(10000000); // 10 seconds
        UART_PRINT("\r\n ***AWAKE**** %d %d \r\n", ADC_SAMPLE_COUNT, i);
    }

    ADC_close(adc);

    return (NULL);
}
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* 1 second delay */
    uint32_t time = 1;
    struct timespec     ts = {0};
    int32_t             status = 0;

    pthread_t           thread1;
    pthread_attr_t      pAttrs_spawn;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 retf;
    int                 detachState;
    UART_Handle         uart;

    /* Call board init functions */
    //Board_initGeneral();
    /* Call driver init functions */
    GPIO_init();
    //Display_init();
    uart = InitTerm();
    ADC_init();
    // I2C_init();
    SPI_init();
    // UART_init();
    // Watchdog_init();
    /* Open the display for output */
    //display = Display_open(Display_Type_UART, NULL);
    //if (display == NULL) {
        /* Failed to open display driver */
    //    while (1);
    //}
    /* initilize the realtime clock */
    clock_settime(CLOCK_REALTIME, &ts);
    /* Create a mutex that will protect voltage variable */
    retc = pthread_mutex_init(&voltageMutex, NULL);
    if (retc != 0) {
        /* pthread_mutex_init() failed */
        UART_PRINT("\r locomov - Couldn't create mutex  - %d\n", retc);
        while (1) {}
    }

    if (uart == NULL) {
        //Display_printf(display, 0, 0, "UART NULL \n");
        while(1);
    }
    //Display_printf(display, 0, 0, "Starting locomov \n");
    
    UART_PRINT("\r\n ***App variables**** \r\n");
    /* Init Application variables */
    status = initAppVariables();
    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
    sleep(1);
    GPIO_toggle(Board_GPIO_LED0);
    sleep(1);
    GPIO_toggle(Board_GPIO_LED0);

    /* Start the SimpleLink Host */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    retc = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, TASKSTACKSIZE);

    retc = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if(retc != 0)
    {
        //Display_printf(display,0,0,"could not create simpleLink task\n\r");
        UART_PRINT("\r Could not create simpleLink task\n\r");
        LOOP_FOREVER();
    }

    /* Create application thread */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retf = pthread_attr_setdetachstate(&attrs, detachState);
    if (retf != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retf |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retf != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    //Display_printf(display, 0, 0, "Starting adc thread \n");
    /* Create threadFxn1 thread */
    retf = pthread_create(&thread1, &attrs, threadFxn1, NULL);
    if (retf != 0) {
        /* pthread_create() failed */
        while (1);
    }

    status = sl_WifiConfig();
    if(status < 0)
    {
        /* Handle Error */
        //Display_printf(display,0,0,"locomov - Couldn't configure Network Processor - %d\n",status);
        UART_PRINT("\r locomov - Couldn't configure Network Processor - %d\n",status);
        LOOP_FOREVER();
    }

    /* Turn NWP on */
    status = sl_Start(NULL, NULL, NULL);
    if(status < 0)
    {
        /* Handle Error */
        //Display_printf(display,0,0,"sl_start failed - %d\n",status);
        UART_PRINT("sl_start failed - %d\n",status);
        LOOP_FOREVER();
    }

    /* Unregister mDNS services */
    status = sl_NetAppMDNSUnRegisterService(0, 0, 0);
    if(status < 0)
    {
        /* Handle Error */
        //Display_printf(display,0,0,"sl_NetAppMDNSUnRegisterService failed - %d\n",status);
        UART_PRINT("sl_NetAppMDNSUnRegisterService failed - %d\n",status);
        LOOP_FOREVER();
    }

    status = sl_Start(0,0,0);
    sleep(5);
    status = wlanConnect();
    if (status < 0) {
        UART_PRINT("\r\n wlanConnect error \r\n");
    }
    int32_t ret = 0;
    ip_t dest;
    //memset(&dest, 0x0, sizeof(dest));
    dest.ipv4 = DEST_IP_ADDR;
    uint8_t nb = 1; // doesn't seem to make a difference.
    int16_t port = 38979;
    uint32_t numPackets = 1;


    /*while (1) {
        sleep(time);
        GPIO_toggle(Board_GPIO_LED0);
    }*/
    //Display_printf(display, 0, 0, "Returning \n");
    while(1){
        ret = TCPClient(nb, port, dest, FALSE /*ipv6*/, numPackets, TRUE);
        if (ret != 0) {
            UART_PRINT("[line:%d, error:%d] \n\r", __LINE__, ret);
            //Display_printf(display, 0, 0, "TCPClient failed");
        } 
        sleep(30);
    }
    return (NULL);
}

int32_t wlanConnect(void)
{
    SlWlanSecParams_t secParams = {0};
    int32_t status = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    status = sl_WlanConnect((signed char*)SSID_NAME, strlen(
                                SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(status);
    
    //Display_printf(display, 0, 0, "Trying to connect to AP : %s\n\r", SSID_NAME);

    // Wait for WLAN Event
    while((!IS_CONNECTED(PowerMeasure_CB.slStatus)) ||
          (!IS_IP_ACQUIRED(PowerMeasure_CB.slStatus)))
    { 
        /* Turn on user LED */
        GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
        usleep(50000);
        /* Turn off user LED */
        GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
        usleep(50000);
    }

    return(0);
   
}

/*!
    \brief          initialize Application's Variables

    This routine initialize the application control block.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1.

    \sa             MainThread

*/
int32_t    initAppVariables(void)
{
    int32_t ret = 0;

    app_CB.Status = 0 ;
    app_CB.Role = ROLE_RESERVED;
    app_CB.Exit = FALSE;

    memset(&app_CB.gDataBuffer, 0x0, sizeof(app_CB.gDataBuffer));
    memset(&app_CB.CON_CB, 0x0, sizeof(app_CB.CON_CB));

    ret = sem_init(&app_CB.CON_CB.connectEventSyncObj,    0, 0);
    if(ret != 0)
    {
        SHOW_WARNING(ret, OS_ERROR);
        return(-1);
    }

    ret = sem_init(&app_CB.CON_CB.eventCompletedSyncObj,  0, 0);
    if(ret != 0)
    {
        SHOW_WARNING(ret, OS_ERROR);
        return(-1);
    }

    ret = sem_init(&app_CB.CON_CB.ip4acquireEventSyncObj, 0, 0);
    if(ret != 0)
    {
        SHOW_WARNING(ret, OS_ERROR);
        return(-1);
    }

    ret = sem_init(&app_CB.CON_CB.ip6acquireEventSyncObj, 0, 0);
    if(ret != 0)
    {
        SHOW_WARNING(ret, OS_ERROR);
        return(-1);
    }

    ret = sem_init(&app_CB.WowlanSleepSem, 0, 0);
    if(ret != 0)
    {
        SHOW_WARNING(ret, OS_ERROR);
        return(-1);
    }

    return(ret);
}

//*****************************************************************************
// SimpleLink Callback Functions
//*****************************************************************************

void SimpleLinkNetAppRequestMemFreeEventHandler (uint8_t *buffer)
{
  // do nothing...
}

void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest,
                                         SlNetAppResponse_t *pNetAppResponse)
{
  // do nothing...
}

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    char syncMsg;

    switch(pWlanEvent->Id)
    {
        case SL_WLAN_EVENT_CONNECT:
        {

            UART_PRINT("\r [WLAN EVENT] Connected [0x%x]\n\r", pWlanEvent->Id);
            SET_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_CONNECTION);
        }
        break;

        case SL_WLAN_EVENT_DISCONNECT:
        {
            UART_PRINT("\r [WLAN EVENT] Disonnected [0x%x]\n\r", pWlanEvent->Id);
            syncMsg = (uint8_t) SL_WLAN_EVENT_DISCONNECT;
            CLR_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_IP_ACQUIRED);
            //mq_send(PowerMeasure_CB.queue, &syncMsg, 1, 0);
        }
        break;

        default:
        {
            //Display_printf(display,0,0,"[WLAN EVENT] Unexpected event [0x%x]\n\r",
            //           pWlanEvent->Id);
            UART_PRINT("\r [WLAN EVENT] Unexpected event [0x%x]\n\r", pWlanEvent->Id);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    switch (slFatalErrorEvent->Id)
    {
        case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
        {
        /*Display_printf(display,0,0,
            "[ERROR] - FATAL ERROR: Abort NWP event detected:"
            " AbortType=%d, AbortData=0x%x\n\r",
            slFatalErrorEvent->Data.DeviceAssert.Code,
            slFatalErrorEvent->Data.DeviceAssert.Value);*/
        UART_PRINT("\r [ERROR] - FATAL ERROR: Abort NWP event detected: AbortType=%d, AbortData=0x%x\n\r", slFatalErrorEvent->Data.DeviceAssert.Code, slFatalErrorEvent->Data.DeviceAssert.Value);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
        {
            //Display_printf(display,0,0,"[ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
            UART_PRINT("\r [ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
        {
        UART_PRINT("\r [ERROR] - FATAL ERROR: No Cmd Ack detected [cmd opcode = 0x%x] \n\r", slFatalErrorEvent->Data.NoCmdAck.Code);
        /*Display_printf(display,0,0,
            "[ERROR] - FATAL ERROR: No Cmd Ack detected"
            " [cmd opcode = 0x%x] \n\r",
            slFatalErrorEvent->Data.NoCmdAck.Code);*/
        }
        break;

        case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
        {
            //Display_printf(display,0,0,"[ERROR] - FATAL ERROR: Sync loss detected n\r");
            UART_PRINT("\r [ERROR] - FATAL ERROR: Sync loss detected n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
        {
        /*Display_printf(display,0,0,
            "[ERROR] - FATAL ERROR: Async event timeout detected"
            " [event opcode =0x%x]  \n\r",
            slFatalErrorEvent->Data.CmdTimeout.Code);*/
        UART_PRINT("\r [ERROR] - FATAL ERROR: Async event timeout detected [event opcode =0x%x]  \n\r", slFatalErrorEvent->Data.CmdTimeout.Code);
        }
        break;

        default:
            /*Display_printf(display,0,0,"[ERROR] - FATAL ERROR: "
                       "Unspecified error detected \n\r");*/
            UART_PRINT("\r [ERROR] - FATAL ERROR: Unspecified error detected \n\r");
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info 
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    char syncMsg;
    struct timespec ts;

    switch(pNetAppEvent->Id)
    {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        {
            SET_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_IP_ACQUIRED);
            syncMsg = STATUS_BIT_IP_ACQUIRED;
            clock_gettime(CLOCK_REALTIME, &ts);
            //mq_timedsend(PowerMeasure_CB.queue, &syncMsg, 1, 0,&ts);
            UART_PRINT("\r [NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
            "Gateway=%d.%d.%d.%d\n\r", 
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,3),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,2),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,1),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,0),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,0));
        }
        break;

        default:
        {
            UART_PRINT("\r [NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Id);
            /*Display_printf(display,0,0,"[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Id);*/
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerEventHandler(
    SlNetAppHttpServerEvent_t *pHttpEvent,
    SlNetAppHttpServerResponse_t *
    pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info 
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("\r [GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->Data.Error.Code,
               pDevEvent->Data.Error.Source);
    /*Display_printf(display,0,0,"[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->Data.Error.Code,
               pDevEvent->Data.Error.Source);*/
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->SocketAsyncEvent.SockTxFailData.Status)
            {
                case SL_ERROR_BSD_ECLOSE:
                    /*Display_printf(display,0,0,"[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\r",
                                    pSock->SocketAsyncEvent.SockTxFailData.Sd);*/
                    UART_PRINT("\r [SOCK ERROR] - close socket (%d) operation failed to transmit all queued packets\n\r", pSock->SocketAsyncEvent.SockTxFailData.Sd);
                    break;
                default: 
            UART_PRINT(
                "[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                pSock->SocketAsyncEvent.SockTxFailData.Sd,
                pSock->SocketAsyncEvent.SockTxFailData.Status);
            /*Display_printf(display,0,0,
                "[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                pSock->SocketAsyncEvent.SockTxFailData.Sd,
                pSock->SocketAsyncEvent.SockTxFailData.Status);*/
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - "
                       "Unexpected Event [%x0x]\n\n",pSock->Event);
            /*Display_printf(display,0,0,"[SOCK EVENT] - "
                       "Unexpected Event [%x0x]\n\n",pSock->Event);*/
          break;
    }

}

