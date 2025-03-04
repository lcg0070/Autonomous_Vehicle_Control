//
// Created by jarry_goon on 24. 6. 27.
//

#ifndef LIDAR_PROCESSING_H
#define LIDAR_PROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Function.h"
#include "SensorLidar.h"

// Margin
#define DEFAULT_MARGIN 0.50
#define MAX_MARGIN 0.60
#define RUBBER_MARGIN 0.21
#define STATIC_MARGIN 0.05
#define LONG_STATIC_MARGIN 0.0

//translation
#define LIDAR2CAMERA_TRANSLATION    0.292
#define CAMERA2GPS_TRANSLATION      0.261
#define ORIGINAL_INDEX              ((int)1441)
#define INTERESTED_INDEX            ((int)561)
#define LIDAR_MAXRANGE              15.0
#define OBSTALCE_THRESHOLD          0.5

//    INTERESTED_INDEX/7
#define OBSTACLE_INDEX_LIDAR (int)130

#define HISTORY_MAX (int)5


typedef struct lidar_info
{
    double x;
    double y;
}LidarBodyData;

typedef struct
{
    short range[INTERESTED_INDEX];
    short flag;
}BUFFER;

extern double rubber_flag;
extern REAL obstacle_xy[OBSTACLE_INDEX_LIDAR * 3];
extern LidarBodyData LIDAR_CORD[ORIGINAL_INDEX];


void init_lidar();

/*-----------------------------------------------------------------------------------*/
/* Lidar Preprocessing                                                               */
/*-----------------------------------------------------------------------------------*/
void median_filter(LidarData* dst, int median_size);
void lidardata(LidarData* dst, int obstacle_num);
int lidar_preprocessing();

void clear_socket();

#ifdef __cplusplus
}
#endif

#endif //LIDAR_PROCESSING_H
