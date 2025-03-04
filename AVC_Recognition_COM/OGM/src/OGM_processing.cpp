//
// Created by User on 2024-06-29.
//

#include "OGM_processing.hpp"
#include "LIDAR_processing.h"
#include "calibration.hpp"
#include "remote_control_com.h"

#include <cmath>

#define OGM_WIDTH   400  // [pixel] 120m x 0.3 (widht x resolution)
#define OGM_HEIGHT  500  // [pixel] 150m x 0.3 (width x resolution)
#define OGM_RESOL   0.3  // [pixel / m]

#define BODY_WIDTH  100  // [-] 30m x 0.3 (width x resolution)
#define BODY_HEIGHT 50   // [-] 15m x 0.3 (width x resolution)
#define BODY_GRID   100  // [-] 30m x 0.3 (width x resolution)

#define SQUARE(x) ((x) * (x))
#define PYTHAGORAS(X, Y) (sqrt(SQUARE(X) + SQUARE(Y)))

#define RS       12.0
#define OFF_SIZE 122
#define RADIUS   0.6

#define MIN_ANGLE   (-0.61)   // [rad]
#define MAX_ANGLE   0.61      // [rad]
#define ANGLE_RESOL 0.01      // [rad]

// SERVER
#define BUFFER_LEN  1922

static unsigned char OGM[OGM_HEIGHT][OGM_WIDTH];

static cv::VideoWriter video;

void init_OGM()
{
    cv::Mat grid_map(OGM_HEIGHT, OGM_WIDTH, CV_8U, &OGM);

    std::vector<cv::Point> points(4);

    cv::Point circle1;
    cv::Point circle2;
    int       circle_radius;

    memset(OGM, 0, sizeof(OGM));
    init_remote();

    points[0] = cv::Point((OGM_WIDTH >> 1) - 55, (OGM_HEIGHT >> 1) - 176);
    points[1] = cv::Point((OGM_WIDTH >> 1) + 148, (OGM_HEIGHT >> 1) - 110);
    points[2] = cv::Point((OGM_WIDTH >> 1) + 55, (OGM_HEIGHT >> 1) + 176);
    points[3] = cv::Point((OGM_WIDTH >> 1) - 148, (OGM_HEIGHT >> 1) + 110);

    circle1 = cv::Point((OGM_WIDTH >> 1) + 52, (OGM_HEIGHT >> 1) - 143);
    circle2 = cv::Point((OGM_WIDTH >> 1) - 52, (OGM_HEIGHT >> 1) + 143);

    circle_radius = 100;

    cv::fillPoly(grid_map, points, 255);
    // cv::circle(grid_map, circle1, circle_radius, 255, -1);
    // cv::circle(grid_map, circle2, circle_radius, 255, -1);

    video = cv::VideoWriter("OGM.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
                            30, cv::Size(OGM_WIDTH, OGM_HEIGHT));
}

inline void body2ned(double pos_n[2], const double yaw, const double B_x, const double B_y)
{
    pos_n[0] = cos(-yaw) * B_x + sin(-yaw) * B_y;
    pos_n[1] = -sin(-yaw) * B_x + cos(-yaw) * B_y;
}

static void ned2body(double N, double E, double yaw, double body[2])
{
    body[0] = cos(yaw) * N + sin(yaw) * E;
    body[1] = -sin(yaw) * N + cos(yaw) * E;
}

static void cal_off(const double   yaw,
                    const cv::Mat& local_ogm, double off[OFF_SIZE]
)
{
    cv::imshow("Local OGM", local_ogm);

    double object_N;
    double object_E;

    // I. Init OFF
    for(int i  = 0; i < OFF_SIZE; i++)
        off[i] = RS;

    // II. OFF Update
    for(int E = 0; E < local_ogm.cols; E++)
    {
        for(int N = 0; N < local_ogm.rows; N++)
        {
            double body[2];
            double slope1;
            double slope2;
            double max_slope;
            double min_slope;

            // 1. Check occupancy.
            const uchar occupancy = local_ogm.at<uchar>(N, E);

            // If occupancy is small, assum empty the this point.
            if(occupancy < 150) continue;

            // 2. Convert I-frame to Body-frame
            object_N = static_cast<double>((local_ogm.rows >> 1) - N) * OGM_RESOL;
            object_E = static_cast<double>(E - (local_ogm.cols >> 1)) * OGM_RESOL;

            ned2body(object_N, object_E, yaw, body);

            if(body[0] < 0) continue;

            double r = RADIUS;
            if(SQUARE(body[0]) + SQUARE(body[1]) < SQUARE(RADIUS)) r = sqrt(SQUARE(body[0]) + SQUARE(body[1]));

            // 3. Calculate max/min slope
            double integer_part;
            double real_part;
            double denomi_part;

            integer_part = body[0] * body[1];
            real_part    = SQUARE(body[0]) + SQUARE(body[1]) - SQUARE(r);
            real_part    = r * sqrt(real_part);
            denomi_part  = SQUARE(body[1]) - SQUARE(r);

            slope1 = (integer_part - real_part) / denomi_part;
            slope2 = (integer_part + real_part) / denomi_part;

            max_slope = std::max(slope1, slope2);
            min_slope = std::min(slope1, slope2);

            if(max_slope < tan(MIN_ANGLE) || min_slope > tan(MAX_ANGLE)) continue;

            // 4. Compute max/min azimuth
            double min_azimuth;
            double max_azimuth;

            min_azimuth = static_cast<int>(atan(min_slope) * 1e2) * 1e-2;

            max_azimuth = static_cast<int>(atan(max_slope) * 1e2) * 1e-2;

            if(min_azimuth < MIN_ANGLE) min_azimuth = MIN_ANGLE;
            if(max_azimuth > MAX_ANGLE) max_azimuth = MAX_ANGLE;

            int min_idx;
            int idx_len;

            min_idx = static_cast<int>(min_azimuth * 1e2);

            min_idx += OFF_SIZE >> 1;
            idx_len = static_cast<int>((max_azimuth - min_azimuth) * 1e2);

            for(int idx = 0; idx < idx_len; idx++)
            {
                double slope;
                double reflect_x;
                double reflect_y;

                double range;
                double azimuth = min_azimuth + idx * 1e-2;

                slope = tan(azimuth);

                integer_part = body[0] + slope * body[1];
                denomi_part  = SQUARE(slope) + 1.0;
                real_part    = SQUARE(integer_part);
                real_part -= denomi_part * (SQUARE(body[0]) + SQUARE(body[1]) - SQUARE(r));
                real_part = sqrt(real_part);

                reflect_x = (integer_part - real_part) / denomi_part;
                reflect_y = slope * reflect_x;

                range = sqrt(SQUARE(reflect_x) + SQUARE(reflect_y));

                if(off[min_idx + idx] > range) off[min_idx + idx] = range;
            }
        }
    }

    static int count = 0;

    FILE* file;
    char  file_name[100];

    sprintf_s(file_name, 100, "OFF%d.txt", count);
    fopen_s(&file, file_name, "w+t");

    for(int i = 0; i < OFF_SIZE; i++)
    {
        fprintf(file, "%f\n", off[i]);
    }

    fclose(file);

    count++;
}

void OGM_calculate()
{
    double x;
    double y;

    SendBuffer    buffer_sen;
    RecieveBuffer buffer_rec;
    cv::Mat       ogm(OGM_HEIGHT, OGM_WIDTH, CV_8U, &OGM);

    double off[OFF_SIZE];

    char flag = 1;

    buffer_rec = get_geo_data();
    double N   = buffer_rec.N;
    double E   = buffer_rec.E;
    double yaw = buffer_rec.yaw;

    if(N > 75.0 || N < -75.0 || E > 60.0 || E < -60.0) return;

    double position_i[2];

    for(int i = 0; i < ORIGINAL_INDEX; i++)
    {
        double position_n[2];

        x = LIDAR_CORD[i].x;
        y = LIDAR_CORD[i].y;

        if(x == -1) break;

        x += LIDAR2CAMERA_TRANSLATION;

        x += CAMERA2GPS_TRANSLATION;

        body2ned(position_n, yaw, x, y);

        position_i[0] = position_n[0] + N;
        position_i[1] = position_n[1] + E;

        // NED_x / GRID_SIZE + GRIDMAP_WIDTH/2.
        int cord_N = (OGM_HEIGHT >> 1) - static_cast<int>(position_i[0] / GRID_SIZE);
        int cord_E = (OGM_WIDTH >> 1) + static_cast<int>(position_i[1] / GRID_SIZE);

        if(cord_N >= 500 || cord_N < 0 || cord_E >= 400 || cord_E < 0) continue;

        if(OGM[cord_N][cord_E] != 255) OGM[cord_N][cord_E] += 1;
    }

    int N_idx = (OGM_HEIGHT >> 1) - static_cast<int>(N / GRID_SIZE);
    int E_idx = (OGM_WIDTH >> 1) + static_cast<int>(E / GRID_SIZE);

    int      box_x1;
    int      box_y1;
    int      box_x2;
    int      box_y2;
    int      box_w;
    int      box_h;
    cv::Rect roi;

    box_x1 = E_idx - 16;
    if(box_x1 < 0) box_x1 = 0;
    box_x2 = E_idx + 17;
    if(box_x2 > OGM_WIDTH) box_x2 = OGM_WIDTH - 1;

    box_y1 = N_idx - 16;
    if(box_y1 < 0) box_y1 = 0;
    box_y2 = N_idx + 17;
    if(box_y2 > OGM_HEIGHT) box_y2 = OGM_HEIGHT - 1;

    box_w = box_x2 - box_x1;
    box_h = box_y2 - box_y1;

    roi               = cv::Rect(box_x1, box_y1, box_w, box_h);
    cv::Mat local_OGM = ogm(roi);

    cal_off(yaw, local_OGM, off);

    for(int i             = 0; i < OFF_SIZE; i++)
        buffer_sen.off[i] = static_cast<short>(off[i] * 1000.0);

    video.write(ogm);
    cv::imshow("OGM", ogm);
    cv::waitKey(1);

    set_lidar_data(&buffer_sen);
}

void clear_OGM()
{
    video.release();
    clear_remote();
}
