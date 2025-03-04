#include "urg_utils.h"
#include "open_urg_sensor.h"

#include "Function.h"
#include "SensorLidar.h"

static urg_t   urg;
static LIDATA* data;
static LIDINT* intensity;
static LIDATA time_stamp;

int Lidar_Initialize(void)
{
    int max_data_size;

    if(open_urg_sensor(&urg) < 0)
        return NOK;

    max_data_size = urg_max_data_size(&urg);
    data          = (long*)malloc(max_data_size * sizeof(data[0]));
    if(!data)
    {
        perror("urg_max_index()");
        return NOK;
    }

    intensity = (LIDINT*)malloc(max_data_size * sizeof(intensity[0]));
    if(!intensity)
    {
        perror("urg_max_index()");
        return NOK;
    }

    urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, URG_SCAN_INFINITY, 0, 1);

    return YOK;
}

/*
int Lidar_importData(LidarData* dst, const REAL lidar_max_range, const StartAngle start, const EndAngle end)
{
    COUNT n_pc;
    int   index_range;
    int   i;

    REAL   rng;
    REAL   ang;
    LIDINT intense;

    double temp_rng = 0.0;

    n_pc = urg_get_distance_intensity(&urg, data, intensity, &time_stamp);
    if(n_pc <= 0)
    {
        Lidar_Terminate();
        Lidar_Initialize();
        return 1;
    }

    index_range = end - start;

    LOOP(i, index_range)
    {
        rng     = data[i + START_90] / 1000.0;
        ang     = urg_index2rad(&urg, i + start);
        intense = intensity[i + start];

        if(rng < 0)
            rng = 0;

        if(rng > lidar_max_range)
            rng = lidar_max_range;

        if(intense) temp_rng = rng;
        else rng             = temp_rng;

        dst[i].range = rng;
        dst[i].angle = -ang;
    }
    return 0;
}
*/

int Lidar_importData(LidarData* dst, const REAL lidar_max_range, const StartAngle start, const EndAngle end)
{
    COUNT n_pc;
    int   index_range;
    int   i;
    int count = 0;
    REAL   rng;
    REAL   ang;
    LIDINT intense;

    n_pc = urg_get_distance_intensity(&urg, data, intensity, &time_stamp);
    if(n_pc <= 0)
    {
        Lidar_Terminate();
        Lidar_Initialize();
        return 1;
    }

    index_range = end - start;

    LOOP(i, index_range)
    {
        rng     = data[i + START_90] / 1000.0;
        ang     = urg_index2rad(&urg, i + start);
        intense = intensity[i + start];

        if(rng < 0)
            continue;

        if(rng > lidar_max_range)
            continue;

        if(!intense) continue;

        dst[count].range = rng;
        dst[count].angle = -ang;
        count++;
    }

//    last data
    dst[count].range = -1.0;
    return 0;
}


void Lidar_Terminate(void)
{
    if(intensity) free(intensity);
    if(data) free(data);
    urg_close(&urg);
}
