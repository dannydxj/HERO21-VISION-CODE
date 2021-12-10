#include "workspace.h"

Workspace workspace;

static void init();
static void loadParam();
static void openCamera();

int main() {
    init();
    workspace.run();
    return 0;
}

/**
 * @brief 初始化函数,加载参数，打开相机
 */
static void init() 
{
    loadParam();
    if (workspace.USE_CAMERA) 
    {
        openCamera();       
    }
}

/**
 * @brief 读入配置文件
 */
static void loadParam() 
{
    // workspace参数传入
    cv::FileStorage file_storage(PARAM_PATH, cv::FileStorage::READ);
    workspace.init(file_storage);
    // 其它文件参数传入及初始化
    workspace.armor_detector.init(file_storage);
    workspace.target_solver.init(file_storage);
    workspace.rune_detector.init(file_storage);
    workspace.rune_descriptior.init(file_storage);
    workspace.sentinel_hunter.init(file_storage);
    workspace.camera->init(file_storage);
    file_storage.release();
}

/**
 * @brief 打开相机
 */
static void openCamera() 
{
    try 
    {
        (workspace.camera)->open();
    } catch (CameraException &e1) 
    {
        Debugger::warning(e1.what(), __FILE__, __FUNCTION__, __LINE__);
        (workspace.camera)->close();
        sleep(1);
        for (int i = 0; i < 5; ++i) {
            try 
            {
                (workspace.camera)->open();
                if ((workspace.camera)->isOpen()) 
                {
                    break;
                }
            } catch (CameraException &e2) 
            {
                Debugger::warning(e2.what(), __FILE__, __FUNCTION__, __LINE__);
                sleep(1);
            }
        }
        if (!(workspace.camera)->isOpen()) 
        {
            exit(1);
        }
    }
}
