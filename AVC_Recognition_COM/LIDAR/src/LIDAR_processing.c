//
// Created by jarry_goon on 24. 6. 27.
//

#include "LIDAR_processing.h"
#include "SensorLidar.h"
#include "communication.h"


// modified by LCK
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)
#define SQUARE(x) ((x) * (x))
#define PYTHAGORAS(X, Y) (sqrt(SQUARE(X) + SQUARE(Y)))


LidarBodyData LIDAR_CORD[ORIGINAL_INDEX];

static REAL out_array[ORIGINAL_INDEX];

void init_lidar(){
    Lidar_Initialize();
}

/*-----------------------------------------------------------------------------------*/
/* Lidar Preprocessing                                                               */
/*-----------------------------------------------------------------------------------*/
static int compare(const void* a, const void* b)
{
    LidarData double_a = *(LidarData*)a;
    LidarData double_b = *(LidarData*)b;

    if(double_a.range < double_b.range) return -1;
    if(double_a.range > double_b.range) return 1;
    return 0;
}

static void median_filter(LidarData* dst, const int median_size)
{
    LidarData* temp;
    int     kernel_size;
    int     ancker_point;
    register int i;

    kernel_size = sizeof(LidarData) * median_size;
    temp        = (LidarData*)malloc(kernel_size);

    ancker_point = median_size >> 1;

    LOOP(i, ancker_point){
        out_array[i]                      = dst[i].range;
        out_array[ORIGINAL_INDEX - i - 1] = dst[ORIGINAL_INDEX - i - 1].range;
    }

    LOOP(i, ORIGINAL_INDEX - median_size) {
        // Copy the range values into the temp array
        memcpy_s(temp, median_size * sizeof(LidarData), dst + i, median_size * sizeof(LidarData));
        qsort(temp, median_size, sizeof(double), compare);
        out_array[i + ancker_point] = temp[ancker_point].range;
    }
     free(temp);
}

/*
static char check_obstacle(const double range,  const double before_range,
                           const int    idx,    const int    start_idx
)
{
    char is_object;
    char is_last_idx;
    char is_max_range;
    char is_not_continuous;

    is_object         = idx - start_idx >= MIN_OBJECT_SIZE;
    is_last_idx       = idx == ORIGINAL_INDEX - 1;
    is_max_range      = range == LIDAR_MAXRANGE;
    is_not_continuous = fabs(range - before_range) > OBSTALCE_THRESHOLD;

    return (is_object << 3) | (is_not_continuous << 2) | (is_max_range << 1) | is_last_idx;
}
*/

/*
static int obstacle_index(LidarData* dst)
{
    char obstacle_criteria;

    int    obstacle_num  = 0;
    int    obstacle_flag = 0;
    double before_data   = out_array[0];

    register int i;

    // Loop through all the LIDAR data points
    LOOP(i, ORIGINAL_INDEX)
    {
        obstacle_criteria = check_obstacle(out_array[i], before_data,
                                           i, starting_idx[obstacle_num]);
        if((obstacle_criteria > 8) && obstacle_flag)
        {
            ending_angle[obstacle_num] = dst[i-1].angle;
            ending_idx[obstacle_num]   = i - 1;
            obstacle_num++;
        }

        obstacle_flag = obstacle_flag ^ IS_MAX_RANGE(obstacle_criteria);

        // If no obstacle is being tracked, start a new one
        if(!obstacle_flag || IS_NOT_CONTINUOUS(obstacle_criteria))
        {
            starting_angle[obstacle_num] = dst[i].angle;
            starting_idx[obstacle_num]   = i;
            obstacle_flag                = 1;  // Mark that an obstacle is being tracked
        }
        before_data = out_array[i];  // Update the previous data point
    }
    return obstacle_num;
}
 */

/*
static void obstacle_coordinates(LidarData* dst, const int obstacle_num)
{
    double       obstacle_margin = 0;
    register int i;

    LOOP(i, obstacle_num)
    {
        //        3 points or 2points
        int case_type = 0;

        int    start_idx = starting_idx[i];
        double x1        = cos(dst[start_idx].angle) * out_array[start_idx];
        double y1        = sin(dst[start_idx].angle) * out_array[start_idx];

        int    end_idx = ending_idx[i];
        double x2      = cos(dst[end_idx].angle) * out_array[end_idx];
        double y2      = sin(dst[end_idx].angle) * out_array[end_idx];

        int    min_index = start_idx;
        double min_data  = LIDAR_MAXRANGE;

        for(int temp = start_idx; temp <= end_idx; ++temp)
        {
            if(min_data > out_array[temp])
            {
                min_index = temp;
                min_data  = out_array[temp];
            }
        }

        double x3 = 0;
        double y3 = 0;

        if(min_index == start_idx || min_index == end_idx)
        {
            case_type = 1;
        }
        else
        {
            x3 = cos(dst[min_index].angle) * min_data;
            y3 = sin(dst[min_index].angle) * min_data;
        }
        double circle_x    = 0;
        double circle_y    = 0;
        double circle_r    = 0;
        double temp_margin = 0;

        //        circle_case 0,1
        switch(case_type)
        {
            case 0:
                double den = 2. * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2));
                double num1 = (y3 - y2) * (SQUARE(x2) - SQUARE(x1) + SQUARE(y2) - SQUARE(y1)) + (y1 - y2) * (
                                  SQUARE(x3) - SQUARE(x2) + SQUARE(y3) - SQUARE(y2));
                double num2 = (x2 - x3) * (SQUARE(x2) - SQUARE(x1) + SQUARE(y2) - SQUARE(y1)) + (x2 - x1) * (
                                  SQUARE(x3) - SQUARE(x2) + SQUARE(y3) - SQUARE(y2));

                circle_x = num1 / den;
                circle_y = num2 / den;

                break;
            case 1:
                circle_x = (x1 + x2) * 0.5;
                circle_y = (y1 + y2) * 0.5;

                break;
            default:
                break;
        }

        obstacle_xy[3 * i]     = circle_x + LIDAR2CAMERA_TRANSLATION + CAMERA2GPS_TRANSLATION;
        obstacle_xy[3 * i + 1] = circle_y;
        // (-max + min)* |deg|/35 + max
        temp_margin = (DEFAULT_MARGIN - MAX_MARGIN) * fabs(RAD2DEG * atan2(circle_y, obstacle_xy[3 * i] - 35) ) / 35.0 +
                      MAX_MARGIN;
        circle_r               = PYTHAGORAS(circle_x - x1, circle_y - y1) + temp_margin;
        obstacle_xy[3 * i + 2] = circle_r;
    }
}
*/

/*
void lidardata(LidarData* dst, int obstacle_num)
{
    register int i;
    register int j;

    //    initialize output
    LOOP(i, obstacle_num)
    {
        double obs_centerrange = PYTHAGORAS(obstacle_xy[3 * i], obstacle_xy[3 * i + 1]);
        double obs_minrange    = obs_centerrange - obstacle_xy[3 * i + 2];

        //        over_range
        if(obs_minrange > LIDAR_MAXRANGE)
        {
            continue;
        }
        //        car_crashes to obstacle
        if(obs_minrange < 0)
        {
            continue;
        }

        //        radian
        double obs_angle       = asin(obstacle_xy[3 * i + 2] / obs_centerrange);
        double obs_centerangle = atan2(obstacle_xy[3 * i + 1], obstacle_xy[3 * i]);

        double obs_minangle = obs_centerangle - obs_angle;
        double obs_maxangle = obs_centerangle + obs_angle;

        LOOP(j, ORIGINAL_INDEX)
        {
            if(dst[j].angle >= obs_maxangle)
            {
                continue;
            }
            if(dst[j].angle < obs_minangle)
            {
                break;
            }
            double temp_x = obstacle_xy[3 * i];
            double temp_y = obstacle_xy[3 * i + 1];
            double temp_r = obstacle_xy[3 * i + 2];

            double Tan       = tan(dst[j].angle);
            double sqrt_term =
                    sqrt(SQUARE(temp_x + temp_y * Tan) - (SQUARE(Tan) + 1) * (
                             SQUARE(temp_x) + SQUARE(temp_y) - SQUARE(temp_r)));
            double x1 = (temp_x + temp_y * Tan + sqrt_term) / (SQUARE(Tan) + 1);
            double y1 = Tan * x1;
            double r1 = PYTHAGORAS(x1, y1);
            double x2 = (temp_x + temp_y * Tan - sqrt_term) / (SQUARE(Tan) + 1);
            double y2 = Tan * x2;
            double r2 = PYTHAGORAS(x2, y2);

            if(out_array[j] > r1)
            {
                out_array[j] = r1;
            }
            if(out_array[j] > r2)
            {
                out_array[j] = r2;
            }
        }
    }
}
*/

/*
int lidar_preprocessing()
{
    // static FILE *file;
    // fopen_s(&file, "lidar.csv", "w+t");
    static LidarData lidarinput[ORIGINAL_INDEX];
    BUFFER      buffer;

    if(Lidar_importData(lidarinput, 15, START_90, END_90))
        return 1;

    median_filter(lidarinput, 15);

    int obstacle_num;
    //    obstacle indexing, count
    obstacle_num = obstacle_index(lidarinput);

    obstacle_coordinates(lidarinput, obstacle_num);
    lidardata(lidarinput, obstacle_num);

    int i = 0;
    LOOP(i, INTERESTED_INDEX) {
        buffer.range[i] = out_array[ORIGINAL_INDEX - (i + START_35 - START_90)] * 1000.;
    }

    buffer.flag = 1;
    UDP_send(socket_info, (char*)&buffer, sizeof(buffer));

    // UDP_recive(&socket_info, (char*)&buffer, sizeof(buffer));

    return 0;
}
*/

void lidar_gridcoord(LidarData* src){
    static int i;
    LOOP(i, ORIGINAL_INDEX){
        if(src[i].range == -1.0){
            LIDAR_CORD[i].x = -1.0;
            break;
        }
        LIDAR_CORD[i].x = src[i].range * cos(src[i].angle) + LIDAR2CAMERA_TRANSLATION;
        LIDAR_CORD[i].y = src[i].range * sin(src[i].angle);
    }
}

int lidar_preprocessing()
{
    LidarData lidarinput[ORIGINAL_INDEX];

    if(Lidar_importData(lidarinput, 15, START_90, END_90))
        return 1;
    median_filter(lidarinput, 15);
    lidar_gridcoord(lidarinput);

    return 0;
}
