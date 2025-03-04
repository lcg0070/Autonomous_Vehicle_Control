//
// Created by jarry_goon on 24. 5. 29.
//

#include "path_planning.h"
#include "car_control.h"
#include "global_path_planning.h"
#include "local_path_planning.h"
#include "GPSINS.h"
#include "remote_recog_com.h"

#include <stdio.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

PathPlanningInfo GUIDANCE_INFO;

void init_path_planning()
{
    memset(&GUIDANCE_INFO, 0, sizeof(GUIDANCE_INFO));

    init_global_planning();
    init_local_planning();

    printf("Path Planning Initialize Finish\n");
}

void guidance()
{
    double geo_pos[2];  // {lat [deg], lon[deg]}
    double ned_pos[2];  // {N [m], E[m]}

    static double yaw;

    // 1. Read GPS/INS data
    if(fabs(GPSINS_DATA->velE) >= 0.5 || fabs(GPSINS_DATA->velN) >= 0.5)
        yaw = atan2(GPSINS_DATA->velE, GPSINS_DATA->velN);

//    GUIDANCE_INFO.yaw = -GPSINS_DATA->yaw * DEG2RAD;
    GUIDANCE_INFO.yaw = yaw;

    geo_pos[0] = GPSINS_DATA->lat;
    geo_pos[1] = GPSINS_DATA->lon;

    geodetic2ned(geo_pos, ned_pos);

    if(GPSINS_DATA->RTK == 1)
    {
        ned_pos[0] += N_BIAS;
        ned_pos[1] += E_BIAS;
    }

    GUIDANCE_INFO.N = ned_pos[0];
    GUIDANCE_INFO.E = ned_pos[1];

    const GeoPos geo = {ned_pos[0], ned_pos[1], yaw};

    set_geo_coord(geo);

    if(CAR_STATE.drive_mode == MANUAL_MODE) return;

    CAR_STATE.speed = 2.0;

    global_guidance(ned_pos[0], ned_pos[1]);
    local_planning(ned_pos[0], ned_pos[1], yaw);
}

void clear_guidance()
{
    clear_global_guidance();
    clear_local_planning();
}
