#include "aimbot.h"

void AimBot::processImage(const cv::Mat &src, const int enemy_color) 
{
    /* 浅拷贝原图，方便debug */
    debug_image = src;

    /* 获得roi图像 */
    if (!roi_rect.empty() && ROI_ENABLE) 
    {
        src(roi_rect).copyTo(roi_image);
        // src.copyTo(roi_image);
    } else {
        src.copyTo(roi_image);
    }

#ifndef COMPILE_WITH_CUDA
// 畸变矫正
#ifdef DISTORTION_CORRECT
    static Mat map1, map2;
    initUndistortRectifyMap(
        camera_matrix, distortion_coeff, Mat(),
        getOptimalNewCameraMatrix(camera_matrix, distortion_coeff,
                                  roi_image.size(), 1, roi_image.size(), 0),
        roi_image.size(), CV_16SC2, map1, map2);
    remap(roi_image, roi_image, map1, map2, INTER_LINEAR);
#endif                                                    // DISTORTION_CORRECT
    cvtColor(roi_image, gray_image, cv::COLOR_BGR2GRAY);  // 转灰度图
    if (enemy_color == COLOR_BLUE) {
        threshold(gray_image, binary_image, BLUE_BINARY_THRES, 255, cv::THRESH_BINARY);
    } else {
        threshold(gray_image, binary_image, RED_BINARY_THRES, 255, cv::THRESH_BINARY);
    }

    std::vector<cv::Mat> channels;
    // 通道分离相减
    split(roi_image, channels);
#ifdef ARMOR_NEW
    if (enemy_color == COLOR_BLUE) {
        channel_thresh_image = channels[0];
        threshold(channel_thresh_image, channel_thresh_image, BLUE_CHANNEL_THRESH, 255, cv::THRESH_BINARY);
    } else {
        channel_thresh_image = channels[2];
        threshold(channel_thresh_image, channel_thresh_image, RED_CHANNEL_THRESH, 255, cv::THRESH_BINARY);
    }
    // 取图像交集
    processed_image = binary_image & channel_thresh_image;
#elif defined ARMOR_OLD
    if (enemy_color == COLOR_BLUE) {
        subtract(channels[0], channels[2], subtract_image);
        threshold(subtract_image, subtract_image, BLUE_SUBTRACT_THRES, 255, THRESH_BINARY);
    } else {
        subtract(channels[2], channels[0], subtract_image);
        threshold(subtract_image, subtract_image, RED_SUBTRACT_THRES, 255, THRESH_BINARY);
    }
    // 取图像交集
    processed_image = binary_image & subtract_image;
#else
    LOGE("armor preprocess method error!");
    exit(1);
#endif
    // 膨胀操作
    dilate(processed_image, processed_image, kernel);
#else
// 畸变矫正
#ifdef DISTORTION_CORRECT
    static Mat map1, map2;
    static cv::cuda::GpuMat gpu_map1, gpu_map2;
    initUndistortRectifyMap(
        camera_matrix, distortion_coeff, Mat(),
        getOptimalNewCameraMatrix(camera_matrix, distortion_coeff,
                                  roi_image.size(), 1, roi_image.size(), 0),
        roi_image.size(), CV_16SC2, map1, map2);
    gpu_map1.upload(map1);
    gpu_map2.upload(map2);
    cv::cuda::remap(roi_image, roi_image, gpu_map1, gpu_map2, INTER_LINEAR);
#endif  // DISTORTION_CORRECT
    static cv::cuda::GpuMat gpu_src, gpu_dst;
    gpu_src.upload(roi_image);
    // 与CPU处理步骤相同
    static cv::cuda::GpuMat gpu_gray, gpu_binary, gpu_subtract, gpu_channel;
    static vector<cv::cuda::GpuMat> gpu_channels;

    cv::cuda::cvtColor(gpu_src, gpu_gray, COLOR_BGR2GRAY);
    if (enemy_color == COLOR_BLUE) {
        cv::cuda::threshold(gpu_gray, gpu_binary, BLUE_BINARY_THRES, 255,
                            cv::THRESH_BINARY);
    } else {
        cv::cuda::threshold(gpu_gray, gpu_binary, RED_BINARY_THRES, 255,
                            cv::THRESH_BINARY);
    }
    cv::cuda::split(gpu_src, gpu_channels);
#ifdef ARMOR_NEW
    if (enemy_color == COLOR_BLUE) {
        cv::cuda::threshold(gpu_channels[0], gpu_channel, BLUE_CHANNEL_THRESH,
                            255, cv::THRESH_BINARY);
    } else {
        cv::cuda::threshold(gpu_channels[2], gpu_channel, RED_CHANNEL_THRESH,
                            255, cv::THRESH_BINARY);
    }
    cv::cuda::bitwise_and(gpu_binary, gpu_channel, gpu_dst);
#elif defined ARMOR_OLD
    if (enemy_color == COLOR_BLUE) {
        cv::cuda::subtract(gpu_channels[0], gpu_channels[2], gpu_subtract);
        cv::cuda::threshold(gpu_subtract, gpu_subtract, BLUE_SUBTRACT_THRES,
                            255, cv::THRESH_BINARY);
    } else {
        cv::cuda::subtract(gpu_channels[2], gpu_channels[0], gpu_subtract);
        cv::cuda::threshold(gpu_subtract, gpu_subtract, RED_SUBTRACT_THRES, 255,
                            cv::THRESH_BINARY);
    }
    cv::cuda::bitwise_and(gpu_binary, gpu_subtract, gpu_dst);
#else
    LOGE("armor preprocess method error!");
    exit(1);
#endif
    kernel->apply(gpu_dst, gpu_dst);
    gpu_dst.download(processed_image);
#endif  // COMPILE_WITH_CUDA
}