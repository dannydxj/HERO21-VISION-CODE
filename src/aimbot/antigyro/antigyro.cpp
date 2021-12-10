#include "aimbot.h"

#define MIN_RADIUS 0.1
#define MAX_RADIUS 2.0

static double Otsu(const CircleQueue<Target, TOP_NUM> &his_targets);
static void split(const CircleQueue<Target, TOP_NUM> &his_targets,
                  const double thresh, std::vector<Target> &low,
                  std::vector<Target> &high);
static std::vector<Target> chooseGroup(std::vector<Target> &low, std::vector<Target> &high);
static bool fitCircle(std::vector<Target> points, GyroData &gyro_data);

bool AimBot::judgeGyro(const CircleQueue<Target, TOP_NUM> &his_targets, GyroData &gyro_data) 
{
    double thresh = Otsu(his_targets); /* 计算阈值 */
    std::vector<Target> low, high;
    split(his_targets, thresh, low, high); /* 根据阈值分割点集并去掉单向最值点 */
    if (fitCircle(chooseGroup(low, high), gyro_data)) 
    {
        return true;
    } else 
    {
        return false;
    }
}

double Otsu(const CircleQueue<Target, TOP_NUM> &his_targets) 
{
    if (his_targets.size < 5) return 0;

    // 首先寻找最低值与最高值
    double min, max = his_targets.values[his_targets.head].z;
    for (int for_i = his_targets.head; for_i != his_targets.tail;
         for_i = (for_i + 1) % his_targets.size) {
        if (his_targets.values[for_i].z > max) {
            max = his_targets.values[for_i].z;
        }
        if (his_targets.values[for_i].z < min) {
            min = his_targets.values[for_i].z;
        }
    }
    // 高度分辨率1cm
    int delta = static_cast<int>((max - min) * 100);

    // determine threshold
    double w0 = 0, w1 = 0;
    double m0 = 0, m1 = 0;
    double max_sb = 0, sb = 0;
    int th = 0;
    int val;

    for (int for_i = 0; for_i++; for_i <= delta) {
        w0 = w1 = m0 = m1 = 0;
        for (int for_j = his_targets.head; for_j != his_targets.tail;
             for_j = (for_j + 1) % his_targets.size) {
            val = his_targets.values[for_j].z;
            if (val < min + for_i * 0.01) {
                w0++;
                m0 += val;
            } else {
                w1++;
                m1 += val;
            }
        }
        m0 /= w0;
        m1 /= w1;
        w0 /= his_targets.size;
        w1 /= his_targets.size;
        sb = w0 * w1 * pow((m0 - m1), 2);
        if (sb > max_sb) {
            max_sb = sb;
            th = for_i;
        }
    }
    double thresh = min + 0.01 * th;
    return thresh;
}

void split(const CircleQueue<Target, TOP_NUM> &his_targets,
           const double thresh, std::vector<Target> &low,
           std::vector<Target> &high) {
    for (int i = his_targets.head; i != his_targets.tail;
         i = (i + 1) % his_targets.size) {
        if (his_targets.values[i].z <= thresh) {
            low.emplace_back(his_targets.values[i]);
        } else {
            high.emplace_back(his_targets.values[i]);
        }
    }
}

std::vector<Target> chooseGroup(std::vector<Target> &low,
                                std::vector<Target> &high) {
    if (low.size() > high.size() && low.size() > 4) {
        std::sort(low.begin(), low.end(),
                  [](const Target &a, const Target &b) { return a.z < b.z; });
        low.pop_back();
    } else if (high.size() > low.size() && high.size() > 4) {
        std::sort(high.begin(), high.end(),
                  [](const Target &a, const Target &b) { return a.z < b.z; });
        high.erase(high.begin());
    }
}

bool fitCircle(std::vector<Target> points, GyroData &gyro_data) {
    int iNum = (int)points.size();
    double X1 = 0.0;
    double Y1 = 0.0;
    double X2 = 0.0;
    double Y2 = 0.0;
    double X3 = 0.0;
    double Y3 = 0.0;
    double X1Y1 = 0.0;
    double X1Y2 = 0.0;
    double X2Y1 = 0.0;
    std::vector<Target>::iterator iter;
    std::vector<Target>::iterator end = points.end();
    for (iter = points.begin(); iter != end; ++iter) {
        X1 = X1 + (*iter).x;
        Y1 = Y1 + (*iter).y;
        X2 = X2 + (*iter).x * (*iter).x;
        Y2 = Y2 + (*iter).y * (*iter).y;
        X3 = X3 + (*iter).x * (*iter).x * (*iter).x;
        Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
        X1Y1 = X1Y1 + (*iter).x * (*iter).y;
        X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
        X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
    }
    double C = 0.0;
    double D = 0.0;
    double E = 0.0;
    double G = 0.0;
    double H = 0.0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    C = iNum * X2 - X1 * X1;
    D = iNum * X1Y1 - X1 * Y1;
    E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
    G = iNum * Y2 - Y1 * Y1;
    H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    if (std::isnan(a) || std::isnan(b)) {
        return false;
    }
    c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
    double A = 0.0;
    double B = 0.0;
    double R = 0.0;
    A = a / (-2);
    B = b / (-2);
    R = double(sqrt(a * a + b * b - 4 * c) / 2);
    if (std::isnan(R)) {
        return false;
    }
    /* 限定R的半径范围，因为拟合的点数较少，且点并不准确，条件宽松 */
    if (R > MIN_RADIUS && R < MAX_RADIUS) {
        gyro_data.center.x = A;
        gyro_data.center.y = B;
        gyro_data.radius = R;
    } else {
        return false;
    }
}