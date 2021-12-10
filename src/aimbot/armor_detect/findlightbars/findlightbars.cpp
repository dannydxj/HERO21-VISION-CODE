#include "aimbot.h"

static void adjustLightBarRect(cv::RotatedRect &rect);
static void adjustLightBarAngle(float elli_angle, double &bar_angle);
static void judgeLightbarColor(Lightbar &lightbar, const cv::Mat &src);

void AimBot::findLightbars(const cv::Mat &processed_image, const int mcu_color) 
{
    // 找出所有轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    if (ROI_ENABLE && !roi_rect.empty())
    {
        findContours(processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(roi_rect.x, roi_rect.y));  // 寻找最外层轮廓
    }else{
        findContours(processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  // 寻找最外层轮廓
    }
    for (auto &contour : contours) 
    {
        if (contour.size() < 6) 
            continue;
        Lightbar lightbar;
        lightbar.rect = cv::minAreaRect(contour);
        lightbar.elli_rect = cv::fitEllipse(contour);
        // 面积筛选
        if (lightbar.rect.size.area() < MAX_LIGHTBAR_AREA && lightbar.rect.size.area() > MIN_LIGHTBAR_AREA) 
        {
            //矫正灯条minAreaRect，保证 height > width
            adjustLightBarRect(lightbar.rect);
            adjustLightBarAngle(lightbar.elli_rect.angle, lightbar.angle);

            if (lightbar.rect.size.height / lightbar.rect.size.width < 1.5)
                continue;
            //角度、颜色筛选灯条
            if (abs(lightbar.angle) < MAX_LIGHTBAR_ANGLE) 
            {
                judgeLightbarColor(lightbar, debug_image);
                if (lightbar.color == mcu_color) 
                {
                    lightbars.emplace_back(lightbar);
                }
            }
        }
    }
    /* 对候选灯条进行排序 */
    sort(lightbars.begin(), lightbars.end(), Lightbar::coordinateComparator);
}

static void adjustLightBarRect(cv::RotatedRect &rect) 
{
    if (rect.size.width > rect.size.height) {
        rect = cv::RotatedRect(rect.center, cv::Size2f(rect.size.height, rect.size.width), rect.angle + 90);
    }
}

static void adjustLightBarAngle(float elli_angle, double &bar_angle) 
{
    if (elli_angle > 90.0) {
        bar_angle = elli_angle - 180.0;
    } else {
        bar_angle = elli_angle;
    }
}

static void judgeLightbarColor(Lightbar &lightbar, const cv::Mat &src) 
{
    auto region = lightbar.rect.boundingRect();

    region.x -= fmax(3, region.width * 0.1);
    region.y -= fmax(3, region.height * 0.05);
    region.width += 2 * fmax(3, region.width * 0.1);
    region.height += 2 * fmax(3, region.height * 0.05);
    // roi
    region &= cv::Rect(0, 0, src.cols, src.rows);
    cv::Mat roi = src(region);

    int red_cnt = 0, blue_cnt = 0;

    for (int row = 0; row < roi.rows; row++) {
        for (int col = 0; col < roi.cols; col++) {
            red_cnt += roi.at<cv::Vec3b>(row, col)[2];
            blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
        }
    }
    if (red_cnt > blue_cnt) {
        lightbar.color = EnemyColor::COLOR_RED;
    } else {
        lightbar.color = EnemyColor::COLOR_BLUE;
    }
}

static void judgeEdge() {}