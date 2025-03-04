//
// Created by jarry_goon on 24. 6. 30.
//

#include "remote_recog_com.h"
#include "communication.h"

#include <stdio.h>
#include <Windows.h>

#define SERVER_PORT 5873

static LidarData lidar_data;

static HANDLE      thread;
static HANDLE      mutex;
static SOCKET_INFO socket_info;

static volatile char running;
static char          is_connected;

static DWORD WINAPI UDP_communication(LPVOID param)
{
    char test[7];
    UDP_recive(&socket_info, test, 7);
    printf("Connect to Com2\n");
    is_connected = 1;

    while(running) {
        UDP_recive(&socket_info, (char *) &lidar_data, sizeof(LidarData));
    }

    return 0;
}

void init_remote()
{
    DWORD thread_id;

    socket_info = UDP_IPv4_server(SERVER_PORT);

    if(socket_info.socket == INVALID_SOCKET)
    {
        perror("Failed to create socket.");

        return;
    }

    mutex = CreateMutex(NULL, FALSE, NULL);

    if(mutex == NULL)
    {
        fprintf(stderr, "ERROR %lu: CreateMutex Filed\n", GetLastError());

        return;
    }

    thread = CreateThread(NULL, 0, UDP_communication, NULL, 0, &thread_id);

    if(thread == NULL)
    {
        fprintf(stderr, "ERROR %lu: CreateThread Filed", GetLastError());

        return;
    }

    is_connected = 0;
    running      = 1;
}

int get_lidar_data(LidarData* buffer)
{
    int error;

    if(!is_connected) return 1;

//    WaitForSingleObject(mutex, INFINITE);
    error = memcpy_s(buffer, sizeof(LidarData), &lidar_data, sizeof(LidarData));
//    ReleaseMutex(mutex);

    return error;
}

void set_geo_coord(GeoPos data)
{
    if(is_connected) UDP_send(socket_info, (char*)&data, sizeof(data));
}

void clear_remote()
{
    running = 0;

    socket_close(socket_info);

    if(thread)
    {
        CloseHandle(thread);
        WaitForSingleObject(thread, INFINITE);
    }

    if(mutex) CloseHandle(mutex);
}
