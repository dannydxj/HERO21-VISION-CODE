#include "aimbot.h"

AimBot::AimBot() : classifier_sj((char *)"../src/aimbot/classifier/classifier_sj/para/"),
                   classifier_dk((char *)"../src/aimbot/classifier/classifier_dk/darknet/cfg/heronet.cfg",
                                 (char *)"../src/aimbot/classifier/classifier_dk/darknet/backup/heronet_4.weights",
                                 (char *)"../src/aimbot/classifier/classifier_dk/darknet/data/names.list") 
{
    this->curr_state = this->next_state = AimState::SEARCH_STATE;
    this->targetTracked = false;
}

AimBot::~AimBot() = default;

void AimBot::init(const cv::FileStorage &file_storage) 
{
    loadParam(file_storage);
    roi_rect = cv::Rect();
    armors.reserve(30);  // 因为armor比较大，为避免拷贝开销
}

void AimBot::loadParam(const cv::FileStorage &file_storage) {
    // 相机内参矩阵参数fy，用于小孔成像测距
    file_storage["camera_matrix"] >> CAMERA_MATRIX;
    camera_fy = CAMERA_MATRIX.at<double>(1, 1);

    // 装甲板识别
    cv::FileNode arm_detect = file_storage["armor_detect"];

    // 是否使用数字分类器
    USE_MODEL = arm_detect["USE_MODEL"];
    // 是否使用ROI
    ROI_ENABLE = arm_detect["ARMOR_ROI_ENABLE"];

    // 图像预处理相关参数传入
    BLUE_BINARY_THRES = arm_detect["BLUE_GREY_THRES"];
    BLUE_SUBTRACT_THRES = arm_detect["BLUE_SUBTRACT_THRES"];
    BLUE_CHANNEL_THRESH = arm_detect["BLUE_CHANNEL_THRESH"];

    RED_BINARY_THRES = arm_detect["RED_GREY_THRES"];
    RED_SUBTRACT_THRES = arm_detect["RED_SUBTRACT_THRES"];
    RED_CHANNEL_THRESH = arm_detect["RED_CHANNEL_THRESH"];

    KERNEL_SIZE = arm_detect["KERNEL_SIZE"];

    // 最大预选数量相关参数传入
    MAX_CANDIDATE_NUM = arm_detect["MAX_CANDIDATE_NUM"];

    // 装甲板筛选限定条件相关参数传入
    MAX_LIGHTBAR_AREA = arm_detect["MAX_LIGHTBAR_AREA"];
    MIN_LIGHTBAR_AREA = arm_detect["MIN_LIGHTBAR_AREA"];
    MIN_LIGHTBAR_RATIO = arm_detect["MIN_LIGHTBAR_RATIO"];
    MIN_ASPECT_BIG_RATIO = arm_detect["MIN_ASPECT_BIG_RATIO"];
    MAX_ASPECT_BIG_RATIO = arm_detect["MAX_ASPECT_BIG_RATIO"];
    MIN_ASPECT_SMALL_RATIO = arm_detect["MIN_ASPECT_SMALL_RATIO"];
    MAX_ASPECT_SMALL_RATIO = arm_detect["MAX_ASPECT_SMALL_RATIO"];
    MAX_LENGTH_RATIO = arm_detect["MAX_LENGTH_RATIO"];
    MAX_LIGHTBAR_ANGLE = arm_detect["MAX_LIGHTBAR_ANGLE"];
    MAX_LIGHTBAR_DELTA = arm_detect["MAX_LIGHTBAR_DELTA"];
    MAX_ARMOR_ANGLE = arm_detect["MAX_ARMOR_ANGLE"];

#ifdef DISTORTION_CORRECT
    camera_matrix = file_storage["camera_matrix"];
    distortion_coeff = file_storage["distortion_coeff"];
#endif  // DISTORTION_CORRECT

#ifndef COMPILE_WITH_CUDA
    // 取合适的核大小
    kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                       cv::Size(KERNEL_SIZE, KERNEL_SIZE));
#else
    kernel = cv::cuda::createMorphologyFilter(
        MORPH_CLOSE, CV_8U,
        getStructuringElement(MORPH_RECT, Size(KERNEL_SIZE, KERNEL_SIZE)));
#endif
}