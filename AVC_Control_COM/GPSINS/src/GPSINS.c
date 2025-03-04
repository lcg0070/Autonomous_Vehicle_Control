// Created by jarry_goon on 24. 5. 29.

#include "car_control.h"
#include "GPSINS.h"

#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>

GPSINSData* GPSINS_DATA = NULL;

static HANDLE memory_map_handle = NULL;
static LPBYTE memory_map = NULL;

void init_GPSINS()
{
    GPSINS_DATA = (GPSINSData*)malloc(sizeof(GPSINSData));
    if (GPSINS_DATA == NULL)
    {
        fprintf(stderr, "ERROR: Failed to allocate memory for GPSINS_DATA.\n");
        CAR_STATE.running = 0;
        return;
    }

    memset(GPSINS_DATA, 0, sizeof(*GPSINS_DATA));

    memory_map_handle = OpenFileMappingA(FILE_MAP_READ, FALSE, "mmf_server");  // Use multibyte string

    if (memory_map_handle == NULL)
    {
        fprintf(stderr, "ERROR(%lu): Failed to open shared memory.\n", GetLastError());

        free(GPSINS_DATA);
        GPSINS_DATA = NULL;

        CAR_STATE.running = 0;

        return;
    }

    memory_map = (BYTE*)MapViewOfFile(memory_map_handle, FILE_MAP_READ, 0, 0, sizeof(SensorData));

    if (memory_map == NULL)
    {
        CloseHandle(memory_map_handle);
        fprintf(stderr, "ERROR(%lu): Failed to map view of file.\n", GetLastError());

        free(GPSINS_DATA);
        GPSINS_DATA = NULL;

        CAR_STATE.running = 0;

        return;
    }

    GPSINS_DATA = (GPSINSData*)memory_map;

    printf("Shared Memory Data:\n");
    printf("Packet ID: %d\n", GPSINS_DATA->packetId);
    printf("Roll: %.12f, Pitch: %.12f, Yaw: %.12f\n", GPSINS_DATA->roll, GPSINS_DATA->pitch, GPSINS_DATA->yaw);
    printf("Lat: %.12f, Lon: %.12f, Alt: %.12f\n", GPSINS_DATA->lat, GPSINS_DATA->lon, GPSINS_DATA->alt);
    printf("VelE: %.12f, VelN: %.12f, VelU: %.12f\n", GPSINS_DATA->velE, GPSINS_DATA->velN, GPSINS_DATA->velU);

    printf("GPS/INS Initialize Finish\n");
}

void clear_GPSINS()
{
    if (memory_map)
        UnmapViewOfFile(memory_map);

    if (memory_map_handle)
        CloseHandle(memory_map_handle);
}
