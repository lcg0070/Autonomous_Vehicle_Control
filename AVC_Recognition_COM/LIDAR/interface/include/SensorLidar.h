#ifndef _SENSORLIDAR_H
#define _SENSORLIDAR_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "urg_sensor.h"
#include "Function.h"


/*-----------------------------------------------------------------------------------*/
/* Macro                                                                             */
/*-----------------------------------------------------------------------------------*/

#define NMAX_PC				((NSIZE)2161)

typedef enum
{
    START_35  = 800,
    START_60  = 600,
    START_90  = 360,
    START_100 = 280,
    START_105 = 105,
    START_110 = 110,
} StartAngle;

typedef enum
{
    END_35  = 1361,
    END_60  = 1561,
    END_90  = 1801,
    END_100 = 1881,
    END_105 = 1921,
    END_110 = 1961,
} EndAngle;

typedef struct
{
    float range;
    REAL angle;
} LidarData;

/*-----------------------------------------------------------------------------------*/
/* Function Declaration                                                              */
/*-----------------------------------------------------------------------------------*/
int Lidar_Initialize(void);

int Lidar_importData(LidarData* dst, REAL lidar_max_range, StartAngle start, EndAngle end);

void Lidar_Terminate(void);

#endif
