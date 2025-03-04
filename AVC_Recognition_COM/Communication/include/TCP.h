//
// Created by jarry_goon on 24. 5. 9.
//

#ifndef TCP_H
#define TCP_H

#include "socket.h"

#ifdef __cplusplus
extern "C" {
#endif

SOCKET_INFO TCP_IPv4_server(u_short port);

SOCKET_INFO TCP_IPv4_client(u_short port, const char* server_ip);

#ifdef __cplusplus
}
#endif

#endif //TCP_H
