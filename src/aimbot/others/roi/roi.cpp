#include "aimbot.h"

const cv::Rect &AimBot::getRoiRect() const { return roi_rect; }

void AimBot::setRoiRect(const cv::Rect &roiRect) 
{
    int detect_x, detect_y;
    int detect_width, detect_height;

    detect_width = static_cast<int>(roiRect.width * 3.0);    //  宽 * 3
    detect_height = static_cast<int>(roiRect.height * 4.0);  //  高 * 4
    detect_x = roiRect.x - (detect_width - roiRect.width) / 2;  // x成员， 左上角
    detect_y = roiRect.y - (detect_height - roiRect.height) / 2;

    preventROIExceed(detect_x, detect_y, detect_width, detect_height);
    roi_rect = cv::Rect(detect_x, detect_y, detect_width, detect_height);  //左上角， 宽度、 高度
}

void AimBot::preventROIExceed(int &x, int &y, int &width, int &height) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + width > debug_image.cols) width = debug_image.cols - x;
    if (y + height > debug_image.rows) height = debug_image.rows - y;
}