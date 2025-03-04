//
// Created by jarry_goon on 24. 5. 9.
//

#ifndef UDP_H
#define UDP_H

#include "socket.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UDP IPv4 server for IP communication
 * @param port Communication IP port
 * @return Socket information
 */
SOCKET_INFO UDP_IPv4_server(u_short port);

SOCKET_INFO UDP_IPv4_client(u_short port, const char* server_ip, u_char connected);

int UDP_send(SOCKET_INFO socket, const char* data, int data_size);

int UDP_recive(SOCKET_INFO* socket, char* buffer, int buffer_size);

#ifdef __cplusplus
}
#endif

#endif //UDP_H
