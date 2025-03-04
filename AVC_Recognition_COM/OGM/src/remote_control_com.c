//
// Created by jarry_goon on 24. 7. 1.
//

#include "remote_control_com.h"
#include "communication.h"

#include <stdio.h>
#include <Windows.h>

static volatile RecieveBuffer geo_pos;

static HANDLE      thread;
static SOCKET_INFO socket_info;

static volatile char running;

static DWORD WINAPI UDP_communication(LPVOID param)
{
    while(running)
    {
        UDP_recive(&socket_info, (char*)&geo_pos, sizeof(geo_pos));
    }

    socket_close(socket_info);

    return 0;
}

void init_remote()
{
    DWORD thread_id;

    socket_info = UDP_IPv4_client(SERVER_PORT, SERVER_ADDR, 1);

    if(socket_info.socket == INVALID_SOCKET)
    {
        fprintf(stderr, "Failed to create socket.");

        return;
    }

    thread = CreateThread(NULL, 0, UDP_communication, NULL, 0, &thread_id);

    if(thread == NULL)
    {
        fprintf(stderr, "ERROR %lu: CreateThread Filed", GetLastError());

        return;
    }

    UDP_send(socket_info, "finish", 7);

    running = 1;
}

RecieveBuffer get_geo_data()
{
    return geo_pos;
}

void set_lidar_data(SendBuffer* lidar)
{
    UDP_send(socket_info, (char*)lidar, sizeof(SendBuffer));
}

void clear_remote()
{
    running = 0;

    if(thread)
    {
        CloseHandle(thread);
        WaitForSingleObject(thread, INFINITE);
    }
}
