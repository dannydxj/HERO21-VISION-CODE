#include "armor.h"

#include "timer.h"

using namespace cv;
using namespace std;

static void adjustArmorRect(cv::RotatedRect &rect);

Armor::Armor() = default;
Armor::~Armor() = default;

void Armor::setNumberImg(int mcu_number) 
{
    cv::Rect2d rect_left = lbar_rect.boundingRect();
    cv::Rect2d rect_right = rbar_rect.boundingRect();
    int min_x, min_y, max_x, max_y;
    min_x = int(fmin(rect_left.x, rect_right.x) - 8);
    max_x = int(fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 8);
    min_y = int(fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0);
    max_y = int(fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) + 0.5 * (rect_left.height + rect_right.height) / 2.0);
    int temp_width = max_x - min_x;
    int temp_height = max_y - min_y;
    if (mcu_number != 6)  // 这里需要注意一下，因为判断是数字区域是否超出ROI区域，而不是原图区域
    {  
        preventExceed(min_x, min_y, temp_width, temp_height, src);
        number_img = src(cv::Rect(min_x, min_y, temp_width, temp_height));
        resize(number_img, number_img, cv::Size(48, 36), INTER_AREA);
    } else {
        min_x += 8;
        temp_width -= 16;
        preventExceed(min_x, min_y, temp_width, temp_height, src);
        number_img = src(cv::Rect(min_x, min_y, temp_width, temp_height));
        resize(number_img, number_img, cv::Size(56, 56), INTER_AREA);
        cvtColor(number_img, number_img, COLOR_BGR2GRAY);
    }
}

int Armor::getNumber() { return classifier_num; }

void Armor::setPriority(int classifier_num, int mcu_num) 
{
    this->classifier_num = classifier_num;  // 给数字赋值

    // 根据数字编号设置打击优先级
    if (classifier_num != 2 || classifier_num != 9) {
        priority = 1;
    } else {
        priority = 2;
    }
    if (mcu_num != EnemyNumber::USE_MODEL_NO_TRACK) {
        if (classifier_num == mcu_num) {
            priority = 0;
        }
    }
}

bool Armor::scoreComparator(const Armor &a, const Armor &b) 
{
    return a.sum_error_score < b.sum_error_score;
}

bool Armor::priorityComparator(const Armor &a, const Armor &b) {
    // 结合数字和优先级排序，统一进行升序排列
    if (a.priority == b.priority)
        return a.sum_error_score < b.sum_error_score;
    else
        return a.priority < b.priority;
}

bool Armor::setOffsetDistance(const Armor &a, const Armor &b) {
    return a.offset_distance < b.offset_distance;
}

cv::Rect Armor::rect() { return armor_rect.boundingRect(); }

void Armor::preventExceed(int &x, int &y, int &width, int &height,
                          const cv::Mat &src) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + width > src.cols) width = src.cols - x;
    if (y + height > src.rows) height = src.rows - y;
}

void Armor::assignMemberVariables(const Lightbar &lbar, const Lightbar &rbar) 
{
    float delta_width = fabs(rbar.rect.center.x - lbar.rect.center.x);
    float delta_height = fabs(rbar.rect.center.y - lbar.rect.center.y);
    double armor_width =
        sqrt(delta_width * delta_width + delta_height * delta_height);
    double armor_height = (lbar.rect.size.height + rbar.rect.size.height) / 2;
    double armor_ratio = armor_width / armor_height;
    double armor_bar_len_ratio = lbar.rect.size.height / rbar.rect.size.height;
    armor_bar_len_ratio =
        ((armor_bar_len_ratio > 1.0) ? armor_bar_len_ratio
                                     : (1.0 / armor_bar_len_ratio));

    this->lbar_rect = lbar.rect;
    this->rbar_rect = rbar.rect;
    std::vector<cv::Point> armor_vertices;
    Util::extractPoints(this->lbar_rect, armor_vertices);
    Util::extractPoints(this->rbar_rect, armor_vertices);
    this->armor_rect = cv::minAreaRect(armor_vertices);
    adjustArmorRect(armor_rect);
    this->ratio = armor_ratio;
    this->bar_len_ratio = armor_bar_len_ratio;
    this->bar_delta_angle = abs(lbar.angle - rbar.angle);
    this->isBigArmor = armor_ratio > 3.7 ? true : false;
    this->armor_angle = fabs(atan2(delta_height, delta_width) * 180 / Util::PI);
}

static void adjustArmorRect(cv::RotatedRect &rect) {
    if (rect.size.width < rect.size.height)
        rect = cv::RotatedRect(rect.center,
                               cv::Size2f(rect.size.height, rect.size.width),
                               rect.angle + 90);
}


void Armor::listInfo()
{
    std::cout << "amor_rect_area: " << this->armor_rect.size.area() << std::endl;
    std::cout << "armor_ratio: " << this->ratio << std::endl;
    std::cout << "bar_delta_angle; " << this->bar_delta_angle << std::endl;
    std::cout << "armor_angle: " << this->armor_angle << std::endl;
}