//
// Created by jarry_goon on 24. 5. 29.
//

#ifndef GPS_H
#define GPS_H

#include <Windows.h>  // Add this line to include Windows-specific types

typedef struct
{
    int packetId;
    char gps_quality;
    double roll;
    double pitch;
    double yaw;
    double lat;
    double lon;
    double alt;
    double velE;
    double velN;
    double velU;
    double geo_DOP;
    double pos_DOP;
    double vertical_DOP;
    double horizontal_DOP;
    double N_DOP;
    double E_DOP;
    char filter_state;
    char RTK;
    char exitFlag;
} SensorData;  // Add SensorData struct definition

typedef SensorData GPSINSData;

extern GPSINSData* GPSINS_DATA;
extern HANDLE memory_map_handle;  // Add this line
extern LPBYTE memory_map;         // Add this line

void init_GPSINS();

void clear_GPSINS();

void update_yaw(double velN, double yaw);

#endif //GPS_H
