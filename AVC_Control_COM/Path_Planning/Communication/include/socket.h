//
// Created by jarry_goon on 24. 5. 9.
//

#ifndef SOCKET_H
#define SOCKET_H

#include <Windows.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum socket_type
{
    SERVER,
    UNCONNETED_UDP,
    CONNETED_UDP,
    TCP
} SOCKET_TYPE;

typedef struct socket_info
{
    SOCKET      socket;
    SOCKADDR_IN server_address;
    SOCKET      TCP_client_socket;  // Socket of connection client in TCP server
    SOCKADDR_IN dest_address; // Address of communication destination
    SOCKET_TYPE type;
} SOCKET_INFO;

void WSA_init();

void WSA_close();

SOCKET socket_init(int af, int socket_type, int protocol);

void socket_close(SOCKET_INFO socket);

#ifdef __cplusplus
}
#endif

#endif //SOCKET_H
