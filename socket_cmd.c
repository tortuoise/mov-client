/* Standard includes */
#include <stdlib.h>
#include <ti/display/Display.h>

#include "socket_cmd.h"
#include "common.h"
#include "empty.h"
#define SECURE_SOCKET
//#define CLIENT_AUTHENTICATION

#ifdef SECURE_SOCKET
#define TCP_PROTOCOL_FLAGS    SL_SEC_SOCKET
#define ROOT_CA_CERT_FILE     "dummy-root-ca-cert"
#define PRIVATE_KEY_FILE      "dummy-trusted-cert-key"
#define TRUSTED_CERT_FILE     "dummy-trusted-cert"
#define TRUSTED_CERT_CHAIN    "trusted-chain.pem"

#define DEVICE_YEAR                 (2017)
#define DEVICE_MONTH                (4)
#define DEVICE_DATE                 (5)

#define BUF_LEN                (MAX_BUF_SIZE - 20)
#else
#define TCP_PROTOCOL_FLAGS      0
#define BUF_LEN                (MAX_BUF_SIZE)
#endif

int32_t TCPClient(uint8_t nb,
                  uint16_t portNumber,
                  ip_t ipAddress,
                  uint8_t ipv6,
                  uint32_t numberOfPackets,
                  uint8_t tx);

typedef union
{
    SlSockAddrIn6_t in6;       /* Socket info for Ipv6 */
    SlSockAddrIn_t in4;        /* Socket info for Ipv4 */
}sockAddr_t;

/*!
    \brief          TCP Client.

    This routine shows how to set up a simple TCP client.
    It shows sending and receiving packets as well.

    \param          nb              -   Sets the socket type: blocking or
										non-blocking.

    \param          portNumber      -   Decides which port is affiliated
										with the server's socket.

    \param          ipv6            -   Sets the version of the L3 IP
										protocol, IPv4 or IPv6.

    \param          numberOfPackets -   Sets the Number of packets
										to send \ receive.

    \param          tx              -   Decides if the function would
										transmit data. If this flag
                                        is set to false, 
										this function would receive.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             cmdSendCallback, cmdRecvCallback

 */
int32_t TCPClient(uint8_t nb,
                  uint16_t portNumber,
                  ip_t ipAddress,
                  uint8_t ipv6,
                  uint32_t numberOfPackets,
                  uint8_t tx)
{
    Display_printf(display, 0, 0, "Starting send");
    int32_t sock;
    int32_t status;
    uint32_t i = 0;
    int32_t nonBlocking;
    SlSockAddr_t        *sa;
    int32_t addrSize;
    sockAddr_t sAddr;
    
    Display_printf(display, 0, 0, "Starting send");
    appControlBlock app_CB;

    memset(&app_CB.gDataBuffer, 0x0, sizeof(app_CB.gDataBuffer));
    /* clear the global data buffer */
    memset(app_CB.gDataBuffer.nwData, 0x0, MAX_BUF_SIZE);

    /* filling the buffer with data */
    for(i = 0; i < MAX_BUF_SIZE; i++)
    {
        app_CB.gDataBuffer.nwData[i] = (char)(i % 10);
    }

    Display_printf(display, 0, 0, "Filled buffer");
    if(ipv6)
    {
        sAddr.in6.sin6_family = SL_AF_INET6;
        sAddr.in6.sin6_port = sl_Htons(portNumber);
        sAddr.in6.sin6_flowinfo = 0;

        sAddr.in6.sin6_addr._S6_un._S6_u32[0] =
            ((unsigned long*)ipAddress.ipv6)[0];
        sAddr.in6.sin6_addr._S6_un._S6_u32[1] =
            ((unsigned long*)ipAddress.ipv6)[1];
        sAddr.in6.sin6_addr._S6_un._S6_u32[2] =
            ((unsigned long*)ipAddress.ipv6)[2];
        sAddr.in6.sin6_addr._S6_un._S6_u32[3] =
            ((unsigned long*)ipAddress.ipv6)[3];
        sa = (SlSockAddr_t*)&sAddr.in6;
        addrSize = sizeof(SlSockAddrIn6_t);
    }
    else
    {
        /* filling the TCP server socket address */
        sAddr.in4.sin_family = SL_AF_INET;

        /* Since this is the client's side, 
		 * we must know beforehand the IP address
         * and the port of the server wer'e trying to connect.
         */
        sAddr.in4.sin_port = sl_Htons((unsigned short)portNumber);
        sAddr.in4.sin_addr.s_addr = sl_Htonl((unsigned int)ipAddress.ipv4);

        sa = (SlSockAddr_t*)&sAddr.in4;
        addrSize = sizeof(SlSockAddrIn6_t);
    }

    /* Get socket descriptor - this would be the
     * socket descriptor for the TCP session.
     */
    sock = sl_Socket(sa->sa_family, SL_SOCK_STREAM, TCP_PROTOCOL_FLAGS);
    ASSERT_ON_ERROR1(sock, SL_SOCKET_ERROR);

#ifdef SECURE_SOCKET

    SlDateTime_t dateTime;
    dateTime.tm_day = DEVICE_DATE;
    dateTime.tm_mon = DEVICE_MONTH;
    dateTime.tm_year = DEVICE_YEAR;

    sl_DeviceSet(SL_DEVICE_GENERAL, SL_DEVICE_GENERAL_DATE_TIME,
                 sizeof(SlDateTime_t), (uint8_t *)(&dateTime));

    /* Set the following to enable Server Authentication */
    /*sl_SetSockOpt(sock,SL_SOL_SOCKET,SL_SO_SECURE_FILES_CA_FILE_NAME,
                  ROOT_CA_CERT_FILE, strlen(
                      ROOT_CA_CERT_FILE));*/

#ifdef CLIENT_AUTHENTICATION
    /* Set the following to pass Client Authentication */
    sl_SetSockOpt(sock,SL_SOL_SOCKET,SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME,
                  PRIVATE_KEY_FILE, strlen(
                      PRIVATE_KEY_FILE));
    sl_SetSockOpt(sock,SL_SOL_SOCKET,SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME,
                  TRUSTED_CERT_CHAIN, strlen(
                      TRUSTED_CERT_CHAIN));
#endif
#endif

    /* Set socket as non-blocking socket (if needed):
     * Non-blocking sockets allows user to handle 
	 * other tasks rather than block
     * on socket API calls. 
	 * If an API call using the Non-blocking socket descriptor
     * returns 'SL_ERROR_BSD_EAGAIN' - 
	 * this indicate that the user should try the API again later.
     */
    if(TRUE == nb)
    {
        nonBlocking = TRUE;
        status =
            sl_SetSockOpt(sock, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &nonBlocking,
                          sizeof(nonBlocking));

        if(status < 0)
        {
            Display_printf(display, 0, 0,"[line:%d, error:%d] %s\n\r", __LINE__, status,
                       SL_SOCKET_ERROR);
            sl_Close(sock);
            return(-1);
        }
    }

    status = -1;

    while(status < 0)
    {
        /* Calling 'sl_Connect' followed by server's
         * 'sl_Accept' would start session with
         * the TCP server. */
        status = sl_Connect(sock, sa, addrSize);
        if((status == SL_ERROR_BSD_EALREADY)&& (TRUE == nb))
        {
            sleep(1);
            continue;
        }
        else if(status < 0)
        {
            Display_printf(display, 0, 0,"[line:%d, error:%d] %s\n\r", __LINE__, status,
                       SL_SOCKET_ERROR);
            sl_Close(sock);
            return(-1);
        }
        break;
    }

    i = 0;

    if(tx)
    {
        int32_t buflen;
        uint32_t sent_bytes = 0;
        uint32_t bytes_to_send = (numberOfPackets * BUF_LEN);

        while(sent_bytes < bytes_to_send)
        {
            if(bytes_to_send - sent_bytes >= BUF_LEN)
            {
                buflen = BUF_LEN;
            }
            else
            {
                buflen = bytes_to_send - sent_bytes;
            }

            /* Send packets to the server */
            status = sl_Send(sock, &app_CB.gDataBuffer.nwData, buflen, 0);
            if((status == SL_ERROR_BSD_EAGAIN) && (TRUE == nb))
            {
                sleep(1);
                continue;
            }
            else if(status < 0)
            {
                Display_printf(display, 0, 0,"[line:%d, error:%d] %s\n\r", __LINE__, status,
                           SL_SOCKET_ERROR);
                sl_Close(sock);
                return(-1);
            }
            i++;
            sent_bytes += status;
        }

        Display_printf(display, 0, 0,"Sent %u packets (%u bytes) successfully\n\r",
                   i,
                   sent_bytes);
    }
    else
    {
        uint32_t rcvd_bytes = 0;

        while(rcvd_bytes < (numberOfPackets * BUF_LEN))
        {
            status = sl_Recv(sock, &app_CB.gDataBuffer.nwData, MAX_BUF_SIZE, 0);
            if((status == SL_ERROR_BSD_EAGAIN) && (TRUE == nb))
            {
                sleep(1);
                continue;
            }
            else if(status < 0)
            {
                Display_printf(display, 0, 0,"[line:%d, error:%d] %s\n\r", __LINE__, status,
                           BSD_SOCKET_ERROR);
                sl_Close(sock);
                return(-1);
            }
            else if(status == 0)
            {
                Display_printf(display, 0, 0,"TCP Server closed the connection\n\r");
                break;
            }
            rcvd_bytes += status;
        }

        Display_printf(display, 0, 0,"Received %u packets (%u bytes) successfully\n\r",
                   (rcvd_bytes / BUF_LEN), rcvd_bytes);
    }

    /* Calling 'close' with the socket descriptor,
     * once operation is finished. */
    status = sl_Close(sock);
    ASSERT_ON_ERROR1(status, SL_SOCKET_ERROR);

    return(0);
}
