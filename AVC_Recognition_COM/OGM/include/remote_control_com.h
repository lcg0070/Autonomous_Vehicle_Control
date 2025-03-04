//
// Created by jarry_goon on 24. 7. 1.
//

#ifndef REMOTE_CONTROL_COM_H
#define REMOTE_CONTROL_COM_H

#ifdef __cplusplus
extern "C"{
#endif

#define OGM_WIDTH   400  // [-] 120m x 0.3 (widht x resolution)
#define OGM_HEIGHT  500  // [-] 150m x 0.3 (width x resolution)
#define OFF_SIZE 122

#define SERVER_PORT 5873
#define SERVER_ADDR "192.168.0.101"

typedef struct send_buffer
{
    short off[OFF_SIZE];
    short         flag;
} SendBuffer;

typedef struct recieve_buffer
{
    double N;
    double E;
    double yaw;
} RecieveBuffer;

void init_remote();

RecieveBuffer get_geo_data();

void set_lidar_data(SendBuffer* lidar);

void clear_remote();

#ifdef __cplusplus
}
#endif

#endif //REMOTE_CONTROL_COM_H
