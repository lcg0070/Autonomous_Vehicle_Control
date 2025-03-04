//
// Created by jarry_goon on 24. 6. 30.
//

#ifndef REOMOTE_RECOG_COM_H
#define REOMOTE_RECOG_COM_H

#define OFF_SIZE 122

typedef struct lidar_data
{
    short off[OFF_SIZE];
    short flag;
} LidarData;

typedef struct geo_pos
{
    double N;
    double E;
    double yaw;
} GeoPos;

void init_remote();

int get_lidar_data(LidarData* buffer);

void set_geo_coord(GeoPos data);

void clear_remote();

#endif //REOMOTE_RECOG_COM_H
