//
// Created by 이찬근 on 2024/05/28.
//

#include "calibration.hpp"
#include "LIDAR_processing.h"
#include "YOLOv8.cuh"

#include <cmath>

#define DEG2RAD (M_PI / 180.0)
#define CUBE(X) ((X) * (X) * (X))
#define SQR(X) ((X) * (X))

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define FPS 30

static cv::VideoCapture cap(0, cv::CAP_DSHOW);
static cv::VideoWriter  video;

static cv::Mat map1, map2, out;
static cv::Mat frame;
static YOLO    model("../best.engine");

CameraInfo CAMERA_CORD[OBSTACLE_INDEX];

//initialize
void init_camera()
{
    if(!cap.isOpened())
    {
        std::cerr << "Error: Could not open camera" << std::endl;
        return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, FPS);

    video = cv::VideoWriter("Cam.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
                            30, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

    //    calibration
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat distCoeffs   = cv::Mat::zeros(4, 1, CV_32F);
    CameraParameter(cameraMatrix, distCoeffs);

    cv::Mat  transform_P = cv::Mat::eye(4, 2, CV_32F);
    cv::Size imageSize(FRAME_WIDTH, FRAME_HEIGHT);
    out = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), out, imageSize, CV_32F, map1, map2);
}

void camera_processing()
{
    cap.read(frame);

    cv::Mat replace_img = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1, cv::Scalar(0));
    cv::remap(frame, replace_img, map1, map2, cv::INTER_NEAREST);

    video.write(frame);

    static Labels label;

    static int    camera_angleidx;
    static double temp_angle;

    //    initialize
    CAMERA_CORD[0].angle = -100.0;

    if(model.predict(replace_img))
    {
        camera_angleidx = 0;
        auto objects    = model.get_objects();

        for(const auto& [labels, score, bbox, _]: objects)
        {
            label                              = static_cast<Labels>(labels);
            CAMERA_CORD[camera_angleidx].label = label;

            int pixel_x = bbox.x + bbox.width >> 1;
            int pixel_y = bbox.y + bbox.height >> 1;

            switch(label)
            {
                case STATIC:
                    temp_angle = static_angle(pixel_x, pixel_y);
                    if(fabs(temp_angle) > 30.0) break;
                    CAMERA_CORD[camera_angleidx].angle = temp_angle * DEG2RAD;
                    camera_angleidx++;
                    break;

                case LONG_STATIC:
                    temp_angle = static_angle(pixel_x, pixel_y);
                    if(fabs(temp_angle) > 30.0) break;
                    CAMERA_CORD[camera_angleidx].angle = temp_angle * DEG2RAD;
                    camera_angleidx++;
                    break;

                case RUBBER:
                    temp_angle = rubber_angle(pixel_x, pixel_y);
                    if(fabs(temp_angle) > 30.0) break;
                    CAMERA_CORD[camera_angleidx].angle = temp_angle * DEG2RAD;
                    camera_angleidx++;
                    break;

                default:
                    temp_angle = sign_angle(pixel_x, pixel_y);
                    if(fabs(temp_angle) > 30.0) break;
                    CAMERA_CORD[camera_angleidx].angle = temp_angle * DEG2RAD;
                    camera_angleidx++;
                    break;
            }

            //            visualize
            cv::rectangle(frame, bbox, cv::Scalar(0, 0, 255), 2);
            int         text_x     = bbox.x + bbox.width + 5;
            int         text_y     = bbox.y + 20;
            std::string label_text = std::to_string(label);
            cv::putText(frame, label_text, cv::Point(text_x, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                        cv::Scalar(0, 255, 0), 2);
        }
        CAMERA_CORD[camera_angleidx].angle = -100.0;
    }

    cv::imshow("frame", frame);
    cv::waitKey(1);
    /* line tracing

    // roi img
    cv::Mat b_channel;
    std::vector<cv::Mat> bgr_channels;
    cv::Mat replace_img(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1, cv::Scalar(0));
    cv::Rect roi(FRAME_WIDTH/2 - 150,FRAME_HEIGHT - 150,300,150);
    cv::Mat frame;

    cv::split(frame, bgr_channels);
    b_channel = bgr_channels[0];
    b_channel = (b_channel > 200)*255;
    cv::Mat roi_image= b_channel(roi);

    roi_image.copyTo(replace_img(roi));

    // Hough 변환을 이용한 선 검출
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(replace_img, lines, 1, CV_PI / 180, 50);
    line_tracing(lines);
     */
}

void clear_camera()
{
    cap.release();
    video.release();
}

// calibration
void CameraParameter(cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
    constexpr float fx = 762.6861660432412f; // Intrinsic Matrix Coefficient
    constexpr float fy = 759.4522224865120f; // Intrinsic Matrix Coefficient

    constexpr float cx = 634.6184968164251f;
    constexpr float cy = 360.1121000493944f;

    constexpr float k1 = 0.025783630673902f; // Radial Distortion
    constexpr float k2 = -0.080964143500310f; // Radial Distortion
    constexpr float p1 = 0.0f; // Tangential Distortion
    constexpr float p2 = 0.0f; // Tangential Distortion

    // Camera Intrinsic Matrix

    //eye = [1 00 ; 010 ; 001]
    cameraMatrix.at<float>(0, 0) = fx;
    cameraMatrix.at<float>(1, 1) = fy;
    cameraMatrix.at<float>(0, 2) = cx;
    cameraMatrix.at<float>(1, 2) = cy;

    distCoeffs.at<float>(0, 0) = k1;
    distCoeffs.at<float>(1, 0) = k2;
    distCoeffs.at<float>(2, 0) = p1;
    distCoeffs.at<float>(3, 0) = p2;
}

double line_tracing(std::vector<cv::Vec2f>& lines)
{
    double right_rho   = 0, left_rho    = 0;
    double right_theta = 0, left_theta  = 0;
    double right_slope = 0, left_slope  = 0;
    int    left_count  = 0, right_count = 0;
    double y_ref       = FRAME_HEIGHT / 2;

    double left_x  = 0;
    double right_x = 0;

    for(size_t i = 0; i < lines.size(); i++)
    {
        float     rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1,               pt2;
        double    a  = cos(theta),   b  = sin(theta);
        double    x0 = a * rho,      y0 = b * rho;
        pt1.x        = cvRound(x0 + 1000 * (-b));
        pt1.y        = cvRound(y0 + 1000 * (a));
        pt2.x        = cvRound(x0 - 1000 * (-b));
        pt2.y        = cvRound(y0 - 1000 * (a));

        double slope = (pt2.y - pt1.y) / (pt2.x - pt1.x);
        if(theta > 1.57 && theta < 3.0)
        {
            right_rho += rho;
            right_theta += theta;
            right_slope += slope;
            right_count++;
        }
        else if(theta < 1.57 && theta > 0.3)
        {
            left_rho += rho;
            left_theta += theta;
            left_slope += slope;
            left_count++;
        }
    }
    if(left_count > 0)
    {
        left_rho /= left_count;
        left_theta /= left_count;
        left_slope /= left_count;
        //        y_ref = left_slope(x - left_rho* cos(left_theta) + left_rho * sin(left_theta);
        left_x = (y_ref - left_rho * sin(left_theta)) / left_slope + left_rho * cos(left_theta);
    }
    if(right_count > 0)
    {
        right_rho /= right_count;
        right_theta /= right_count;
        right_slope /= right_count;
        right_x = (y_ref - right_rho * sin(right_theta)) / right_slope + right_rho * cos(right_theta);
    }

    if(left_x && right_x) return pixel_angle((left_x + right_x) / 2) - FRAME_WIDTH / 2;
    if(left_x) return pixel_angle(left_x - FRAME_WIDTH / 2);
    if(right_x) return pixel_angle(right_x - FRAME_WIDTH / 2);
    return 0.;
}


double rubber_angle(double pixel_x, double pixel_y)
{
    double a = -268.058021915204e-009;
    double b = -22.5659443977558e-009;

    double c = -413.799951340194e-009;
    double d = 8.33692250646199e-006;

    double e = 267.807341458657e-006;
    double f = 221.259231065033e-006;

    double g = -4.72473186045047e-003;
    double h = 46.1831671922636e-003;

    double i = 864.269149865466e-003;
    double j = -92.2752539829180e+000;

    double angle = a * CUBE(pixel_x) + b * SQR(pixel_x) * pixel_y + c * pixel_x * SQR(pixel_y) + d * CUBE(pixel_y) + e *
                   SQR(pixel_x) + f * pixel_x * pixel_y + g * SQR(pixel_y) + h * pixel_x + i * pixel_y + j;
    return angle;
}

double sign_angle(double pixel_x, double pixel_y)
{
    double a = -285.504563961612e-009;
    double b = -312.220926806805e-009;

    double c = -3.22979746026529e-006;
    double d = -18.9002409706087e-006;

    double e = 331.411340708658e-006;
    double f = 1.20675862835053e-003;

    double g = 9.92960841148824e-003;
    double h = -44.2408825424042e-003;
    double i = -1.74106660363485e+000;
    double j = 61.5781778841101e+000;

    double angle = a * CUBE(pixel_x) + b * SQR(pixel_x) * pixel_y + c * pixel_x * SQR(pixel_y) + d * CUBE(pixel_y) + e *
                   SQR(pixel_x) + f * pixel_x * pixel_y + g * SQR(pixel_y) + h * pixel_x + i * pixel_y + j;
    return angle;
}

double static_angle(double pixel_x, double pixel_y)
{
    constexpr double a = -284.379903383106e-009;
    constexpr double b = 176.437595468379e-009;
    constexpr double c = -479.603280438410e-009;
    constexpr double d = 13.6826217232490e-006;

    constexpr double e = 245.685290901302e-006;
    constexpr double f = 81.3112456719713e-006;
    constexpr double g = -8.85776187848484e-003;
    constexpr double h = 70.5747956659590e-003;
    constexpr double i = 1.91164361198659e+000;
    constexpr double j = -178.256011284006e+000;

    double angle = a * CUBE(pixel_x) + b * SQR(pixel_x) * pixel_y + c * pixel_x * SQR(pixel_y) + d * CUBE(pixel_y) + e *
                   SQR(pixel_x) + f * pixel_x * pixel_y + g * SQR(pixel_y) + h * pixel_x + i * pixel_y + j;
    return angle;
}

double pixel_angle(double pixel)
{
    double angle = 65.1450836026326e+000 * atan(2.28772200201605e-003 * pixel + 11.9951211835658e-003) -
                   627.360615448695e-003;
    if(angle > 35) angle = 35;
    if(angle < -35) angle = -35;

    return angle;
}
