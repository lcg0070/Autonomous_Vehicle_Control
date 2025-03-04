//
// Created by jarry_goon on 24. 5. 28.
//

#include "path_planning.h"
#include "global_path_planning.h"
#include "GPSINS.h"
#include "car_control.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

#include <stdio.h>

static int     num_waypoint;
static int     idx_waypoint;
static double* waypoints_N;
static double* waypoints_E;

static double dst_N;  // [deg] Destination latitude
static double dst_E;  // [deg] Destination longitude

void init_global_planning()
{
    FILE* file;

    double dst_geo_pos[2];
    double dst_ned_pos[2];

    // 1. File read
    if(fopen_s(&file, "../waypoints.txt", "r"))
    {
        fprintf(stderr, "ERROR: Fail open file\n");
        CAR_STATE.running = 0;

        return;
    }

    // 2. Count number of waypoints
    num_waypoint = 0;
    for(char c = getc(file); c != EOF; c = getc(file))
        if(c == '\n') num_waypoint++;

    rewind(file);

    if(num_waypoint == 0)
    {
        fprintf(stderr, "ERROR: There are no waypoints entered.\n");
        fclose(file);
        CAR_STATE.running = 0;

        return;
    }

    // 3. Allocation waypoints array
    waypoints_N = (double*)malloc(sizeof(double) * num_waypoint);
    waypoints_E = (double*)malloc(sizeof(double) * num_waypoint);

    // 4. Configuration waypoints
    for(int i = 0; i < num_waypoint; i++)
    {
        fscanf_s(file, "%lf\t%lf\n", &dst_geo_pos[0], &dst_geo_pos[1]);

        geodetic2ned(dst_geo_pos, dst_ned_pos);

        waypoints_N[i] = dst_ned_pos[0];
        waypoints_E[i] = dst_ned_pos[1];
    }

    fclose(file);

    // 5. Set First destination
    idx_waypoint   = 0;

    dst_N = waypoints_N[0];
    dst_E = waypoints_E[0];

    GUIDANCE_INFO.dst_N = dst_N;
    GUIDANCE_INFO.dst_E = dst_E;
}

void clear_global_guidance()
{
    if(waypoints_N) free(waypoints_N);
    if(waypoints_E) free(waypoints_E);
}

static void vehicle_model(const double dot_gamma)
{
    double beta;
    double delta_w;

    double velocity;

    velocity = CAR_STATE.speed;

    // 1. Compute Beta
    beta = asin(VEHICLE_WHEELBASE * 0.5 * dot_gamma / velocity);

    // 2. Compute delta W
    delta_w = atan((VEHICLE_WHEELBASE * 0.5 * dot_gamma + velocity * sin(beta)) / (velocity * cos(beta)));
    delta_w = atan(2.0 * tan(beta));
    //delta_w = atan2(dot_gamma*VEHICLE_WHEELBASE,velocity);
    // 4. Update data
    GUIDANCE_INFO.delta_w = delta_w * RAD2DEG;

    CAR_STATE.steer = delta_w * RAD2DEG;
}


void global_guidance(const double N, const double E)
{
    double dot_gamma;
    double gamma;

    double d_N;
    double d_E;

    double lambda;  // [rad] GPS에서 받아온 정보로 lambda 구하기
    double r;       // [m]   GPS에서 받아온 정보로 거리 구하기
    double mod_r;

    double eta;

    gamma = GUIDANCE_INFO.yaw;

    // 3. Compute Lambda & distance to waypoint
    d_N = dst_N - N;
    d_E = dst_E - E;

    lambda = atan2(d_E, d_N);
    r      = PYTHAGORAS(d_E, d_N);


    if(r < MIN_DISTANCE) mod_r = MIN_DISTANCE;
    else if(r > MAX_DISTANCE) mod_r = MAX_DISTANCE;
    else mod_r = r;

    // 4. Waypoint change
    if(r < DISTANCE_THRESHOLD)
    {
        idx_waypoint++;

        if(idx_waypoint >= num_waypoint)
        {
            CAR_STATE.drive_mode = MANUAL_MODE;
            CAR_STATE.speed      = 0.0;

            return;
        }

        // Waypoint update
        dst_N = waypoints_N[idx_waypoint];
        dst_E = waypoints_E[idx_waypoint];

        GUIDANCE_INFO.dst_N = dst_N;
        GUIDANCE_INFO.dst_E = dst_E;
    }

    // 5. Update dot_gamma
    eta       = atan2(sin(gamma - lambda), cos(gamma - lambda));
    dot_gamma = -CONTROL_GAIN * CAR_STATE.speed * sin(eta) / mod_r;

    GUIDANCE_INFO.dot_gamma = dot_gamma;
    GUIDANCE_INFO.distance  = r;

    vehicle_model(dot_gamma);
}


static void cartesian(double XYZ[3], const double lat, const double lon)
{
    double N;

    N      = SEMIMAJOR_AXIS / sqrt(1.0 - FLATTENING * (2.0 - FLATTENING) * sin(lat * DEG2RAD) * sin(lat * DEG2RAD));
    XYZ[0] = (N + ALT0) * cos(lat * DEG2RAD) * cos(lon * DEG2RAD);
    XYZ[1] = (N + ALT0) * cos(lat * DEG2RAD) * sin(lon * DEG2RAD);
    XYZ[2] = (N * SQUARE(1.0 - FLATTENING) + ALT0) * sin(lat * DEG2RAD);
}

void geodetic2ned(const double geo_pos[2], double ned[2])
{
    static char   is_init = 0;
    static double lat0;
    //0.586661084243476

    static double sin_lat;
    static double cos_lat;
    static double sin_lon;
    static double cos_lon;

    if(!is_init)
    {
        lat0    = atan2(SQUARE(SEMIMINOR_AXIS) * tan(LAT0 * DEG2RAD), SQUARE(SEMIMAJOR_AXIS));
        sin_lat = sin(lat0);
        cos_lat = cos(lat0);
        sin_lon = sin(LON0 * DEG2RAD);
        cos_lon = cos(LON0 * DEG2RAD);

        is_init = 1;
    }

    const double R[3][3] = {
            {-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat},
            {-sin_lon, cos_lon, 0},
            {-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat}
            };

    /*
    double R[3][3] = {
            0.372289802178929,   -0.453399967974530,    0.809832558146602,
            -0.772848208534589,   -0.634590928523940,         0.0,
            0.513912395023169,   -0.625877641776585,   -0.586661084243476
    };
     */

    double XYZ0[3] = {0.0,};
    cartesian(XYZ0, LAT0, LON0);

    double XYZ[3]      = {0.0};
    double distance[3] = {0.0};

    cartesian(XYZ, geo_pos[0], geo_pos[1]);

    distance[0] = XYZ[0] - XYZ0[0];
    distance[1] = XYZ[1] - XYZ0[1];
    distance[2] = XYZ[2] - XYZ0[2];

    ned[0] = R[0][0] * distance[0] + R[0][1] * distance[1] + R[0][2] * distance[2];
    ned[1] = R[1][0] * distance[0] + R[1][1] * distance[1] + R[1][2] * distance[2];
}
