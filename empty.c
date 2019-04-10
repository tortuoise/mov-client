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
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
#include <ti/drivers/net/wifi/simplelink.h>

/* Board Header file */
#include "Board.h"
#include "common.h"
#include "uart_term.h"

/* ADC sample count */
#define ADC_SAMPLE_COUNT  (100)

#define FRAME_LENGTH                (1000)
#define THREADSTACKSIZE   (768)

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

uint16_t adcValue1[ADC_SAMPLE_COUNT];
uint32_t adcValue1MicroVolt[ADC_SAMPLE_COUNT];
static Display_Handle display;
PowerMeasure_ControlBlock   PowerMeasure_CB;
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

    ADC_Params_init(&params);
    adc = ADC_open(Board_ADC1, &params);

    if (adc == NULL) {
        Display_printf(display, 0, 0, "Error initializing ADC1\n");
        while (1);
    }

    for (i = 0; i < ADC_SAMPLE_COUNT; i++) {
        res = ADC_convert(adc, &adcValue1[i]);

        if (res == ADC_STATUS_SUCCESS) {

            adcValue1MicroVolt[i] = ADC_convertRawToMicroVolts(adc, adcValue1[i]);

            Display_printf(display, 0, 0, "ADC1 raw result (%d): %d\n", i,
                           adcValue1[i]);
            Display_printf(display, 0, 0, "ADC1 convert result (%d): %d uV\n", i,
                adcValue1MicroVolt[i]);
		GPIO_toggle(Board_GPIO_LED0);
        }
        else {
            Display_printf(display, 0, 0, "ADC1 convert failed (%d)\n", i);
        }
	sleep(6);
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
    int32_t             status = 0;

    pthread_t           thread1;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Call driver init functions */

    /* Call driver init functions */
    GPIO_init();
    Display_init();
    ADC_init();
    // I2C_init();
    SPI_init();
    // UART_init();
    // Watchdog_init();

    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        /* Failed to open display driver */
        while (1);
    }

    Display_printf(display, 0, 0, "Starting locomov \n");
    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
    sleep(1);
    GPIO_toggle(Board_GPIO_LED0);
    sleep(1);
    GPIO_toggle(Board_GPIO_LED0);


    status = sl_WifiConfig();
    if(status < 0)
    {
        /* Handle Error */
        UART_PRINT("Power Measurement - Couldn't configure Network Processor - %d\n",status);
        LOOP_FOREVER();
    }

    /* Create application threads */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    /* Create threadFxn1 thread */
    retc = pthread_create(&thread1, &attrs, threadFxn1, (void* )0);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    /*while (1) {
        sleep(time);
        GPIO_toggle(Board_GPIO_LED0);
    }*/
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
    
    UART_PRINT("Trying to connect to AP : %s\n\r", SSID_NAME);

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

            SET_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_CONNECTION);
        }
        break;

        case SL_WLAN_EVENT_DISCONNECT:
        {
            syncMsg = (uint8_t) SL_WLAN_EVENT_DISCONNECT;
            CLR_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_IP_ACQUIRED);
            //mq_send(PowerMeasure_CB.queue, &syncMsg, 1, 0);
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Id);
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
        UART_PRINT(
            "[ERROR] - FATAL ERROR: Abort NWP event detected:"
            " AbortType=%d, AbortData=0x%x\n\r",
            slFatalErrorEvent->Data.DeviceAssert.Code,
            slFatalErrorEvent->Data.DeviceAssert.Value);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
        {
        UART_PRINT(
            "[ERROR] - FATAL ERROR: No Cmd Ack detected"
            " [cmd opcode = 0x%x] \n\r",
            slFatalErrorEvent->Data.NoCmdAck.Code);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Sync loss detected n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
        {
        UART_PRINT(
            "[ERROR] - FATAL ERROR: Async event timeout detected"
            " [event opcode =0x%x]  \n\r",
            slFatalErrorEvent->Data.CmdTimeout.Code);
        }
        break;

        default:
            UART_PRINT("[ERROR] - FATAL ERROR: "
                       "Unspecified error detected \n\r");
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
            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
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
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Id);
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
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->Data.Error.Code,
               pDevEvent->Data.Error.Source);
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
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->SocketAsyncEvent.SockTxFailData.Status)
            {
                case SL_ERROR_BSD_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\r",
                                    pSock->SocketAsyncEvent.SockTxFailData.Sd);
                    break;
                default: 
            UART_PRINT(
                "[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                pSock->SocketAsyncEvent.SockTxFailData.Sd,
                pSock->SocketAsyncEvent.SockTxFailData.Status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - "
                       "Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }

}

