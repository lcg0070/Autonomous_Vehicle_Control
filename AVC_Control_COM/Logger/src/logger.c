//
// Created by jarry_goon on 24. 5. 30.
//

#include "logger.h"
#include "idle_time.h"

#include "car_control.h"
#include "GPSINS.h"
#include "path_planning.h"
#include "idle_time.h"

#include <stdio.h>
#include <time.h>

#define NAME_SIZE 100
#define ERROR_BUFFER_SIZE 100

static FILE* file;

void init_logger()
{
    errno_t   error;
    time_t    current_time;
    struct tm time_info;

    char file_name[NAME_SIZE];
    char error_buffer[ERROR_BUFFER_SIZE];

    time(&current_time);
    error = localtime_s(&time_info, &current_time);
    if(error)
    {
        strerror_s(error_buffer, sizeof(error_buffer), error);
        fprintf(stderr, "Error occurred in \"localtime_s\": %s\n", error_buffer);
        CAR_STATE.running = 0;

        return;
    }

    strftime(file_name, NAME_SIZE, "%Y%m%d_%H%M%S.csv", &time_info);

    error = fopen_s(&file, file_name, "w+t");
    if(error || file == NULL)
    {
        strerror_s(error_buffer, sizeof(error_buffer), error);
        fprintf(stderr, "Error occurred in \"fopen_s\": %s\n", error_buffer);
        fprintf(stderr, "ERROR: Fail Open %s\n", file_name);
        CAR_STATE.running = 0;

        return;
    }

    fprintf(file, "Time[s], Speed Cmd[m/s], Steer Cmd[deg], Steer[deg], ");
    fprintf(file, "Roll[deg], Pitch[deg], Yaw[deg], ");
    fprintf(file, "Lat[deg], Lon[deg], Alt[m], ");
    fprintf(file, "Vel E[m/s], Vel N[m/s], Vel U[m/s], ");
    fprintf(file, "N[m], E[m], GPS Quality, Filter State, RTK, ");
    fprintf(file, "dot gamma, delta_w, distance");
    fprintf(file, "\n");
}

void logger()
{
    static int count = 0;

    count++;
    if(count > 1000)
    {
        count = 0;
        return;
    }

    printf("\e[0K\e[1B\e[0K\e[1B\e[0K\e[2F");

    printf("Speed: %.1f[-],  Steer: %.1f[deg],  Current Steer: %.1f[deg]\n",
           CAR_STATE.speed, CAR_STATE.steer, CAR_STATE.current_steer);
    if(GPSINS_DATA)
    {
        printf("N: %.3f[m], E: %.3f[m], Yaw: %.3f[deg], GPS State: %d, Filter State: %d, RTK: %d\n",
               GUIDANCE_INFO.N, GUIDANCE_INFO.E, GUIDANCE_INFO.yaw, GPSINS_DATA->gps_quality,
               GPSINS_DATA->filter_state, GPSINS_DATA->RTK);
        printf("N DOP: %.2f, E DOP: %.2f, Dst N: %.2f, Dst E: %.2f",
               GPSINS_DATA->N_DOP, GPSINS_DATA->E_DOP, GUIDANCE_INFO.dst_N, GUIDANCE_INFO.dst_E);
        printf("\e[2F");
    }
    printf("\e[1F");

    if(file == NULL) return;

    fprintf(file, "%.3f, %.1f, %1.f, %1.f, ",
            TIME.elapsed, CAR_STATE.speed, CAR_STATE.steer, CAR_STATE.current_steer);

    if(GPSINS_DATA)
    {
        fprintf(file, "%.2f, %.2f, %.2f, ",
                GPSINS_DATA->roll, GPSINS_DATA->pitch, GUIDANCE_INFO.yaw);
        fprintf(file, "%.10f, %.10f, %.2f, ",
                GPSINS_DATA->lat, GPSINS_DATA->lon, GPSINS_DATA->alt);
        fprintf(file, "%.2f, %.2f, %.2f, ",
                GPSINS_DATA->velE, GPSINS_DATA->velN, GPSINS_DATA->velU);
        fprintf(file, "%.2f, %.2f, %d, %d, %d, ",
                GUIDANCE_INFO.N, GUIDANCE_INFO.E, GPSINS_DATA->gps_quality,
                GPSINS_DATA->filter_state, GPSINS_DATA->RTK);
        fprintf(file, "%.2f, %.2f, %.2f ",
                GUIDANCE_INFO.dot_gamma, GUIDANCE_INFO.delta_w, GUIDANCE_INFO.distance);

    }
    fprintf(file, "\n");
}

void clear_logger()
{
    if(file) fclose(file);
}
