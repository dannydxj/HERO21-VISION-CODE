// 发信息
// 运动预测在发送信息的线程中去做，每10ms发送一次信息

#include "workspace.h"

static void setRuneOKPack(SendPack &send_pack, Target &target1, Target &target2, double v, double energy_yaw);
static void setCenteringPack(SendPack &send_pack, double yaw, double pitch);
static void func_trans(Target &target, double yaw);


void Workspace::sendCommunicatingFunc()
{
    if (!USE_CAN) 
        return;
    static int send_cnt = 0;
    while (true)
    {
        while (can_flag == 0)
        {   // TODO 可能会有bug,在云台单片机一段时间不工作后又重新工作
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while (!judgeIfApproSendWindowPhase())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        int64_t clock_now = workspace_timer.getMCUTime(); 
        switch (work_msg.mode)
        {
        case Mode::MODE_ARMOR2:
            setArmorPredictdata(send_pack, clock_now);
            break;
        case Mode::MODE_SENTINEL:
            setSentinelPredictdata(send_pack, clock_now);
            break;
        case Mode::MODE_SMALLRUNE:
            setSmallRunePredData(send_pack, clock_now);
            break;
        case Mode::MODE_BIGRUNE:
            setBigRunePredData(send_pack, clock_now);
        default:
            break;
        }
        can_node.send(send_pack);
        send_cnt = (send_cnt + 1) % 100;
        if (send_cnt == 0)
        {
            can_flag = 0; // 每发送100次置0，防止电控端重启
        }

    }
}

bool Workspace::judgeIfApproSendWindowPhase()
{
    int64_t clock_now = workspace_timer.getMCUTime();
    if (clockAtsend == 0)
    {
        clockAtsend = static_cast<int64_t>(clock_now / 10) * 10 + 9;
        return false; // 不发送。只是初始化一下
    }
    else
    {
        if (clock_now > clockAtsend)
        {
            clockAtsend = static_cast<int64_t>(clock_now / 10) * 10 + 9;
            return true;
        } else if ((clockAtsend - clock_now) > 20)  // mcu restart
        {
            clockAtsend = 0;
            return true;
        } else 
        {
            return false;
        }
    }
}

void Workspace::setArmorPredictdata(SendPack &send_pack, int64_t clock_now)
{
    send_pack.mode = Mode::MODE_ARMOR2; // TODO
    send_pack.clock = clock_now;
    // 判断是否远距离不起用运动预测
    if (!his_coords.isEmpty())
    {
        Target tmp_last_target;
        his_coords.back(tmp_last_target, 1);
        double tmp_dis = Util::getAbsolute3Ddistance(tmp_last_target);
        if (tmp_dis > 4.0)
        {
            armor_predictor.setPredErrorSendPack(his_coords, send_pack, work_msg.bullet_speed);
        }
        else
        {
            // std::unique_lock<mutex> locker(pred_mutex);
            if (!armor_predictor.getPredtarget(work_msg.bullet_speed, clock_now)) // 直线已经过时 
            {
                if (his_coords.length != 0) //该情况出现在有新的坐标点，但是新的坐标点不足以拟合有效的直线
                {   
                    armor_predictor.setPredErrorSendPack(his_coords, send_pack, work_msg.bullet_speed);
                }
                else
                { // 没有新的坐标点，直线也已经过时
                    send_pack.target_found = false;
                }
            } 
            else
            {
                armor_predictor.setPredOKSendPack(send_pack);
            }
        }
    }

}

void Workspace::setSentinelPredictdata(SendPack &send_pack, int64_t clock_now)
{
    if (USE_CAN){  // 因为遥控器还没有加入哨兵模式
        send_pack.mode = Mode::MODE_SENTINEL; 
    }else{
        send_pack.mode = Mode::MODE_ARMOR2; 
    }
    send_pack.clock = clock_now;
    {// 上锁
        std::unique_lock<mutex> locker(pred_mutex);
        if (!armor_predictor.getPredtarget(work_msg.bullet_speed, clock_now)) // 直线已经过时 
        {
            if (his_coords.length != 0) //该情况出现在有新的坐标点，但是新的坐标点不足以拟合有效的直线
            {   
                armor_predictor.setPredErrorSendPack(his_coords, send_pack, work_msg.bullet_speed);
            }
            else
            { // 没有新的坐标点，直线也已经过时
                send_pack.target_found = false;
            }
        } else
        {
            armor_predictor.setPredOKSendPack(send_pack);
        }
    }// 解锁
}



void Workspace::setSmallRunePredData(SendPack &send_pack, int64_t clock_now)
{
    send_pack.mode = Mode::MODE_SMALLRUNE;
    if (rune_detector.flag_start == 0)
    {
        send_pack.target_found = false; // 不控制云台
    }else
    {
        /* 首先进行对心判断 */
        if (rune_detector.flag_center_aligning)
        {
            setCenteringPack(send_pack, rune_detector.central_yaw, rune_detector.central_pitch);
            return;
        }
        /* 判断是否拟合成功圆 */
        else if (rune_descriptior.flag_fit_circle_success == 1 && rune_descriptior.rotate_dir != 0) /* 圆拟合成功 */
        {
            Target pred_target;
            Target five_ms_late_pred_target; // 预测点5ms之后的坐标值
            bool cond1 = rune_predictor.PredConstVelocity(rune_descriptior.const_rune_data, clock_now, work_msg.bullet_speed, pred_target);
            bool cond2 = rune_predictor.PredConstVelocity(rune_descriptior.const_rune_data, clock_now+5, work_msg.bullet_speed, five_ms_late_pred_target);
            if( cond1 && cond2 )
            {
                setRuneOKPack(send_pack, pred_target, five_ms_late_pred_target, work_msg.bullet_speed, rune_detector.energy_yaw);
            }else{
                setCenteringPack(send_pack, rune_detector.central_yaw, rune_detector.central_pitch);
            }

        }else{ /* 没拟合成功自然要对心拟合 */
            setCenteringPack(send_pack, rune_detector.central_yaw, rune_detector.central_pitch);
            return;
        }
    }
}

void Workspace::setBigRunePredData(SendPack &send_pack, int64_t clock_now)
{
    send_pack.mode = Mode::MODE_SMALLRUNE;
    if (rune_detector.flag_start == 0){
        send_pack.target_found = false; // 不控制云台
    }else{
        /* 首先进行对心判断 */
        if (rune_detector.flag_center_aligning)
        {
            setCenteringPack(send_pack, rune_detector.central_yaw, rune_detector.central_pitch);
            return;
        }
        /* 判断是否寻找速度零点成功 */
        else if (rune_descriptior.flag_speed_clock == 1) /* 寻找速度零点成功 */
        {
            Target pred_target;
            Target five_ms_late_pred_target; // 预测点5ms之后的坐标值
            bool cond1 = rune_predictor.PredPivVelocity(rune_descriptior.piv_rune_data, clock_now, work_msg.bullet_speed, pred_target);
            bool cond2 = rune_predictor.PredPivVelocity(rune_descriptior.piv_rune_data, clock_now+5, work_msg.bullet_speed, five_ms_late_pred_target);
            if( cond1 && cond2 )
            {
                setRuneOKPack(send_pack, pred_target, five_ms_late_pred_target, work_msg.bullet_speed, rune_detector.energy_yaw);
            }else{
                setCenteringPack(send_pack, rune_detector.central_yaw, rune_detector.central_pitch);
            }

        }else{ /* 没拟合成功自然要对心拟合 */
            setCenteringPack(send_pack, rune_detector.central_yaw, rune_detector.central_pitch);
            return;
        }
    }
}

void setCenteringPack(SendPack &send_pack, double yaw, double pitch)
{
    send_pack.target_found = true;
    send_pack.pred_yaw = yaw;
    send_pack.pred_pitch = pitch;
    send_pack.pitch_palstance = 0.0;
    send_pack.yaw_palstance = 0.0;
}

void setRuneOKPack(SendPack &send_pack, Target &target1, Target &target2, double v, double energy_yaw)
{
    send_pack.target_found = true;
    func_trans(target1, energy_yaw); /* MARK */
    func_trans(target2, energy_yaw);
    send_pack.pred_yaw = Util::getPredYaw(target1);
    send_pack.pred_pitch = Util::getPredPitch(target1, v);
    send_pack.yaw_palstance = ((Util::getPredYaw(target2) - Util::getPredYaw(target1)) / 0.005);
    send_pack.pitch_palstance = ((Util::getPredPitch(target2, v) - Util::getPredPitch(target1, v)) / 0.005);
}

static void func_trans(Target &target, double yaw)
{
    Util::transAngle(target.x, target.y, -(yaw-90));
}
