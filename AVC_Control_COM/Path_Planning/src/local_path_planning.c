//
// Created by jarry_goon on 24. 6. 10.
//

#include "local_path_planning.h"
#include "car_control.h"
#include "remote_recog_com.h"

#include <windows.h>
#include <time.h>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

//SFF
#define SIGMA 0.5
#define WEIGHT 0.9
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define ANGLE_RESOL 0.01      // [rad]

#define RS       10.0
#define OFF_SIZE 122

#define SQ(X) ((X) * (X))

void init_local_planning()
{
    init_remote();
}

static void steer_command(const double off[OFF_SIZE])
{
    double mean_angle;
    double steer_angle;
    double max_iff;

    steer_angle = CAR_STATE.steer * DEG2RAD;
    mean_angle  = CAR_STATE.steer * DEG2RAD;
    max_iff     = 0.0;

    for(int i = 0; i < OFF_SIZE; i++)
    {
        double angle;
        double sff;
        double iff;

        angle = (double)(i - (OFF_SIZE >> 1)) * ANGLE_RESOL;
        sff   = RS * exp(-SQUARE(angle - mean_angle) / (2.0 * SQUARE(SIGMA)));

        iff = WEIGHT * sff + (1.0 - WEIGHT) * off[i];

        if(iff > max_iff)
        {
            max_iff     = iff;
            steer_angle = angle;
        }
    }

    CAR_STATE.steer = steer_angle * RAD2DEG;
}

void local_planning(const double N, const double E, const double yaw)
{
    LidarData     lidar_data;
    double off[OFF_SIZE];

    if(get_lidar_data(&lidar_data)) return;

    for(int i = 0; i < OFF_SIZE; i++)
        off[i] = (double)lidar_data.off[i] * 0.001;

    steer_command(off);
}

void clear_local_planning()
{
    clear_remote();
}
