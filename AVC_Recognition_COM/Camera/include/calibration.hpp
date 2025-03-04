//
// LCK
//

#ifndef AVC_CAMERA_CAMERAFUNC_HPP
#define AVC_CAMERA_CAMERAFUNC_HPP

#define OBSTACLE_INDEX  130

#include <opencv2/opencv.hpp>

typedef enum labels
{
    BICYCLE,
    CROSS,
    DISABLED,
    LONG_STATIC,
    NONE,
    PARK,
    RUBBER,
    SLOW,
    STATIC,
    STOP,
} Labels;

typedef struct camera_info
{
    Labels label;
    double angle;
} CameraInfo;

extern CameraInfo CAMERA_CORD[OBSTACLE_INDEX];

void init_camera();

void camera_processing();

void clear_camera();

//calibration
void CameraParameter(cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

// SteeringAngle
double line_tracing(std::vector<cv::Vec2f>& lines);

// angle calculate
double rubber_angle(double pixel_x, double pixel_y);
double sign_angle(double pixel_x, double pixel_y);
double static_angle(double pixel_x, double pixel_y);
double pixel_angle(double x);


#endif //AVC_CAMERA_CAMERAFUNC_HPP
