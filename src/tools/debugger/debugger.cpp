#include "debugger.h"

#include <iostream>
#include <sstream>
#include <typeinfo>

#include "base.h"

using namespace std;
using namespace cv;

void Debugger::warning(const std::string &description,
                       const std::string &file_name,
                       const std::string &func_name, int line) 
{
    cerr << "[WARNING] In function '" << func_name << "':\n"
         << file_name << ":" << line << ": " << description << endl;
}

void Debugger::error(const std::string &description,
                     const std::string &file_name, const std::string &func_name,
                     int line) {
    cerr << "[ERROR] In function '" << func_name << "':\n"
         << file_name << ":" << line << ": " << description << endl;
    exit(1);
}

void Debugger::Cameratrackbar(MVCamera *camera,
                              const string &window_name_target) {
    /* Trackbar for function to adjust camera */
#ifdef USE_FIX_EXP
    static int exposure_time = static_cast<int>(camera->exposure_time);
    createTrackbar("exposure_time", window_name_target, &exposure_time, 20000, 0, 0);
    CameraSetExposureTime(camera->hCamera, exposure_time);  // 曝光
    createTrackbar("analog_gain", window_name_target, &(camera->analog_gain), 256, 0, 0);
    CameraSetAnalogGain(camera->hCamera, camera->analog_gain);  // 模拟增益
#else
    createTrackbar("target_brightness", window_name_target, &(camera->auto_target_brightness), 255, 0, 0);
    CameraSetAeTarget(camera->hCamera, camera->auto_target_brightness);  // 自动曝光目标亮度
#endif

    // createTrackbar("ifEnalbeNoiseFilter", window_name_target,
    // &(camera->enableNoiseFilter), 1, 0, 0);
    // CameraSetNoiseFilter(camera->hCamera, camera->enableNoiseFilter); //
    // 降噪使能 createTrackbar("sharpness", window_name_target,
    // &(camera->sharpness), 100, 0, 0); CameraSetSharpness(camera->hCamera,
    // camera->sharpness); // 锐化
    createTrackbar("gamma", window_name_target, &(camera->gamma), 400, 0, 0);
    CameraSetGamma(camera->hCamera, camera->gamma);  // gamma
    createTrackbar("contrast", window_name_target, &(camera->contrast), 200, 0,
                   0);
    CameraSetContrast(camera->hCamera, camera->contrast);  // 对比度
}

void Debugger::Armortrackbar(AimBot &detector, MVCamera *camera,
                             const string &window_name_target,
                             const string &window_name_proc) {
    /* Trackbar for function to adjust camera */
    Debugger::Cameratrackbar(camera, window_name_target);

    /* Trackbar for function `Preprocess()`. */
#ifdef ARMOR_OLD
    createTrackbar("old_blue_subtract_thres", window_name_proc, &detector.BLUE_SUBTRACT_THRES, 255, 0, 0);
    createTrackbar("old_red_subtract_thres", window_name_proc, &detector.RED_SUBTRACT_THRES, 255, 0, 0);
#elif defined ARMOR_NEW
    createTrackbar("new_blue_channel_thres", window_name_proc, &detector.BLUE_CHANNEL_THRESH, 255, 0, 0);
    createTrackbar("new_red_channel_thres", window_name_proc, &detector.RED_CHANNEL_THRESH, 255, 0, 0);
#endif
    createTrackbar("blue_gray_thres", window_name_proc, &detector.BLUE_BINARY_THRES, 255, 0, 0);
    createTrackbar("red_gray_thres", window_name_proc, &detector.RED_BINARY_THRES, 255, 0, 0);
    createTrackbar("kernel_size", window_name_proc, &detector.KERNEL_SIZE, 20, 0, 0);
#ifdef COMPILE_WITH_CUDA
    detector.kernel = cv::cuda::createMorphologyFilter( MORPH_CLOSE, CV_8U, 
                                                        getStructuringElement(MORPH_RECT, Size(detector.KERNEL_SIZE, detector.KERNEL_SIZE)));
#else
    detector.kernel = getStructuringElement(MORPH_RECT, Size(detector.KERNEL_SIZE, detector.KERNEL_SIZE));
#endif

    /* Trackbar for function selectTarget() */
    static int max_lightbar_angle = static_cast<int>(detector.MAX_LIGHTBAR_ANGLE);
    createTrackbar("max_llightbar_angle", window_name_proc, &max_lightbar_angle, 90, 0, 0);
    detector.MAX_LIGHTBAR_ANGLE = static_cast<double>(max_lightbar_angle);

    static int max_lb_delta = static_cast<int>(detector.MAX_LIGHTBAR_DELTA);
    createTrackbar("max_lightbar_delta", window_name_proc, &max_lb_delta, 90, 0, 0);
    detector.MAX_LIGHTBAR_DELTA = static_cast<double>(max_lb_delta);

    static int max_am_angle = static_cast<int>(detector.MAX_ARMOR_ANGLE);
    createTrackbar("max_armor_angle", window_name_proc, &max_am_angle, 90, 0, 0);
    detector.MAX_ARMOR_ANGLE = static_cast<double>(max_am_angle);
}

void Debugger::Runetrackbar(RuneDetector &solver, MVCamera *camera,
                            const string &window_name_target,
                            const std::string &window_name_proc) {
    /* Trackbar for function to adjust camera */
    Debugger::Cameratrackbar(camera, window_name_target);

    /* Trackbar for imgaeProcess */
    createTrackbar("rune_gray_thres", window_name_proc, &solver.RUNE_GRAY_THRES, 255, 0, 0);
    createTrackbar("rune_blue_subtract_thres", window_name_proc, &solver.RUNE_BLUE_THRESH, 255, 0, 0);
    createTrackbar("rune_red_subtract_thres", window_name_proc, &solver.RUNE_RED_THRESH, 255, 0, 0);
    createTrackbar("rune_blue_channel_thres", window_name_proc, &solver.RUNE_BLUE_CHANNEL_THRESH, 255, 0, 0);
    createTrackbar("rune_red_channel_thres", window_name_proc, &solver.RUNE_RED_CHANNEL_THRESH, 255, 0, 0);
    // createTrackbar("gauss_kernel_size", window_name_proc,
    // &solver.gauss_kernel_size, 20, 0, 0);
    createTrackbar("dilate_kernel_size", window_name_proc, &solver.dilate_kernel_size, 20, 0, 0);
    // createTrackbar("close_kernel_size", window_name_proc,
    // &solver.close_kernel_size, 20, 0, 0); createTrackbar("open_kernel_size",
    // window_name_proc, &solver.open_kernel_size, 20, 0, 0);

    solver.dilate_kernel = cv::getStructuringElement(
        MORPH_RECT, cv::Size(2 * solver.dilate_kernel_size + 1,
                             2 * solver.dilate_kernel_size + 1));
    solver.close_kernel = cv::getStructuringElement(
        MORPH_RECT, cv::Size(2 * solver.close_kernel_size + 1,
                             2 * solver.close_kernel_size + 1));
    solver.open_kernel = cv::getStructuringElement(
        MORPH_RECT, cv::Size(2 * solver.open_kernel_size + 1,
                             2 * solver.open_kernel_size + 1));

    /* Trackbar for function selectTarget() */
    createTrackbar("min_child_contour_area", window_name_proc,
                   &solver.RUNE_MIN_CHILD_CONTOUR_AREA, 3000, 0, 0);
    createTrackbar("min_dad_contour_area", window_name_proc,
                   &solver.RUNE_MIN_DAD_CONTOUR_AREA, 20000, 0, 0);
    // createTrackbar("max_child_contour_area", window_name_proc,
    // &solver.RUNE_MAX_CHILD_CONTOUR_AREA, 3000, 0, 0);
    // createTrackbar("max_dad_contour_area", window_name_proc,
    // &solver.RUNE_MAX_DAD_CONTOUR_AREA, 20000, 0, 0);
}

void Debugger::Utiltrackbar(int &value, std::string& window_name_target)
{
    static int tmp_value = value;
    createTrackbar("synchronize param", window_name_target, &tmp_value, 30, 0, 0);    
    value = tmp_value - 15; // 实现正负
}


template <typename T>
void Debugger::drawValue(cv::Mat &image, cv::Point point, std::string name,
                         const T &value) 
{
    static stringstream sstr;
    sstr << name << ": " << value;
    putText(image, sstr.str(), point, QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
    sstr.str("");
}

void Debugger::drawTypeValue(Target &target, cv::Mat &image) 
{
    drawValue(image, cv::Point(20, 80), "x: ", target.x);
    drawValue(image, cv::Point(20, 100), "y: ", target.y);
    drawValue(image, cv::Point(20, 120), "z: ", target.z);
}

void Debugger::drawArmor(const Armor &armor, Mat &image) {
    drawRotatedRect(image, armor.armor_rect, cv::Scalar(255, 0, 0));
    if (armor.burstAim) {
        cv::putText(image, "burst", armor.armor_rect.center, QT_FONT_NORMAL, 1, Scalar(0, 255, 0));
    }
}

void Debugger::drawRotatedRect(cv::Mat &image, const cv::RotatedRect &rect,
                               const cv::Scalar &scalar, int thickness) {
    Point2f pt[4];
    rect.points(pt);
    for (int i = 0; i < 4; i++) {
        line(image, pt[i % 4], pt[(i + 1) % 4], scalar, thickness);
    }
}

