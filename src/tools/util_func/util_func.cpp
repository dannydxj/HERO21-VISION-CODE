#include "util_func.h"

#include <cmath>

using namespace cv;
using namespace std;

/// 自瞄pitch补偿量，分辨率 1/10 度
double Util::pitch_skew_on_armor = -7.0;

/// 自瞄yaw补偿量， 分辨率 1/10度
double Util::yaw_skew_on_armor = 0.7;

/// 能量机关pitch补偿量，分辨率 1/10 度
double Util::pitch_skew_on_rune = -0.5;

/// 能量机关yaw补偿量， 分辨率 1/10度
double Util::yaw_skew_on_rune = -1.3;


// 空间坐标系下点间距离
double Util::distance(const Point3d &point1, const Point3d &point2) {
    double distance = sqrt((point2.x - point1.x) * (point2.x - point1.x) +
                           (point2.y - point1.y) * (point2.y - point1.y) +
                           (point2.z - point1.z) * (point2.z - point1.z));
    return distance;
}

// 图像坐标系下点间距离
double Util::distance(const Point2d &point1, const Point2d &point2) 
{
    double distance = sqrt((point2.x - point1.x) * (point2.x - point1.x) + (point2.y - point1.y) * (point2.y - point1.y));
    return distance;
}

cv::Mat Util::convertTo3Channels(const Mat &binImg) 
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++) 
    {
        channels.push_back(binImg);
    }
    merge(channels, three_channel);
    return three_channel;
}

bool Util::equalZero(double x) { return abs(x) <= EXP; }

double Util::getpolarAngle(const cv::Point2d &origin_point, const cv::Point2d &temp_point) 
{
    double angle, rad;
    // x轴从左向右，y轴从下到上  0 - 2 PI
    cv::Point2d new_point = cv::Point2d(temp_point.x - origin_point.x, temp_point.y - origin_point.y);
    rad = Util::distance(temp_point, origin_point);
    if (new_point.y >= 0)
    {
        angle = acos(new_point.x / rad) * 180.0 / Util::PI;
    }
    else
    {
        angle = (2 * 3.1415926 - acos(new_point.x / rad)) * 180.0 / Util::PI;
    }
    return angle;
}

void Util::transAngle(double &x1, double &x2, double theta) {
    theta = theta * 3.1415926 / 180.0;
    double cosdat, sindat;
    double x, y;
    //计算
    cosdat = cos(theta);  // theta需为弧度
    sindat = sin(theta);
    x = x1 * cosdat + x2 * sindat;
    y = x2 * cosdat - x1 * sindat;
    x1 = x;
    x2 = y;
}

void Util::wheel2land(Target &target, const ImageClass &image_object) 
{
    target.x = target.x;
    double temp = target.y;
    target.y = target.z;
    target.z = -temp;

    Util::transAngle(target.y, target.z, -image_object.pose.ptz_pitch);
    Util::transAngle(target.x, target.z, image_object.pose.ptz_roll);
    Util::transAngle(target.x, target.y, -image_object.pose.ptz_yaw);
}

void Util::extractPoints(cv::RotatedRect &rect, std::vector<cv::Point> &points) 
{
    cv::Point2f mid_points[4];
    rect.points(mid_points);
    for (int i = 0; i < 4; i++) {
        points.emplace_back(mid_points[i]);
    }
}

void Util::drawRotatedRect(cv::Mat &image, const cv::RotatedRect &rect,
                           const cv::Scalar &scalar, int thickness) {
    Point2f pt[4];
    rect.points(pt);
    for (int i = 0; i < 4; i++) {
        line(image, pt[i % 4], pt[(i + 1) % 4], scalar, thickness);
    }
}


double Util::computFlightTime(const Target &target, double v) {
    double x = target.x;
    double y = target.y;
    double z = target.z;
    double s = sqrt(x * x + y * y);
    double lamda = (exp(k*s/m) - 1);

    // 二次曲线 (ax^2 + bx + c)
    double a = (G * m * m * lamda * lamda) / (2 * k * k * v * v);
    double b =  - m * lamda / k;
    double c = (G * m * m * lamda * lamda) / (2 * k * k * v * v) + z ; 

    double square_delta = b * b - 4 * a * c;

    if (square_delta < 0) {
        // std::cout << "warning" << std::endl;
        return ERROR; 
    }
    double delta = pow(square_delta, 0.5);
    double theta = atan2((-b - delta), (2 * a));
    double flight_time = (m * lamda) / (k * v * cos(theta)); 
    if (flight_time > 3){
        return ERROR;
    }else{
        return flight_time;
    }
}

double Util::getPredYaw(const Target &target)
{
    double y = target.y;
    double x = target.x;
    double angle_rad = atan2(y , x) - Util::PI/2;
    return (angle_rad * 180 / Util::PI);
}

double Util::getPredPitch(const Target &target, double v)
{
    double x = target.x;
    double y = target.y;
    double z = target.z;
    double s = sqrt(x * x + y * y);
    double lamda = (exp(k*s/m) - 1);

    // 二次曲线 (ax^2 + bx + c)
    double a = (G * m * m * lamda * lamda) / (2 * k * k * v * v);
    double b =  - m * lamda / k;
    double c = (G * m * m * lamda * lamda) / (2 * k * k * v * v) + z ; 

    double square_delta = b * b - 4 * a * c;

    if (square_delta < 0) 
    {
        LOGW("get flight time failed");
        return 0; 
    }
    double delta = pow(square_delta, 0.5);
    double pitch = atan2((-b - delta), (2 * a));
    return (pitch * 180.0 / PI);
}

void Util::setByteFlags(unsigned char &byte, int start_index,
              int len, bool flag) {
    // 必要保护
    if (start_index + len - 1 > 7 || len <= 0) {
        return;
    } else {
        unsigned char temp = static_cast<unsigned char>(pow(2, len) - 1);
        if (flag) {
            // 置 1
            byte = byte | (temp << start_index);
        } else {
            // 置 0
            byte = byte & (~(temp << start_index));
        }
    }
}

// 获取字节中指定的几位
unsigned char Util::readByteFlags(unsigned char byte, int start_index, int len) {
    // 如果索引和长度有问题，返回 0
    if (start_index + len - 1 > 7 || len <= 0) {
        return 0;
    } else {
        unsigned char temp = static_cast<unsigned char>(pow(2, len) - 1);
        unsigned char result = byte & (temp << start_index);
        return result;
    }
}

double Util::getAbsolute3Ddistance(const Target &target)
{
    double tmp_dis = sqrt(pow(target.x, 2) + pow(target.y, 2) + pow(target.z, 2));
    return tmp_dis;
}