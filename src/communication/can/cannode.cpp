#include "cannode.h"

static void Flag_Ctrl(char *FlagChar, unsigned char Num, char Flag);
static char Flag_Read(char Flags_IN, unsigned char Num);


CanNode::CanNode() {}

CanNode::~CanNode() {
    if (skt_) {
        close(skt_);
    }
}

bool CanNode::init() {
    bool status = true;

    if (CAN_COM == 0) {
        dev_name = "can0";
        system("ip link set down can0");
        system(
            "ip link set can0 type can bitrate 1000000 dbitrate 2000000 "
            "berr-reporting on fd on");
        system("ip link set up can0");
    } else if (CAN_COM == 1) {
        dev_name = "can1";
        system("ip link set down can1");
        system(
            "ip link set can1 type can bitrate 1000000 dbitrate 2000000 "
            "berr-reporting on fd on");
        system("ip link set up can1");
    }

    if ((skt_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cerr << dev_name << " socket error.\n";
        status = false;
    }
    strcpy(ifr_.ifr_name, dev_name.c_str());

    if (ioctl(skt_, SIOCGIFINDEX, &ifr_) < 0) {
        std::cerr << dev_name << " ioctl error.\n";
        status = false;
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    if (bind(skt_, (struct sockaddr *)&addr_, sizeof(addr_)) != 0)  // bind socket and device can
    {
        std::cout << dev_name << "bind error.\n";
        status = false;
    }

    for (int i = 0; i < REC_ID_NUM; ++i) {
        cfilter_[i].can_id = id_rcv_[i];      // receive frame id
        cfilter_[i].can_mask = CAN_SFF_MASK;  // succeed when (reveived_id) &
                                              // mask == (can_id) & mask
    }
    if (setsockopt(skt_, SOL_CAN_RAW, CAN_RAW_FILTER, &cfilter_, sizeof(cfilter_)) != 0) 
    {
        std::cout << dev_name << " sockopt error.\n";
        status = false;
    }
    return status;
}

bool CanNode::send(SendPack &send_pack) {
    bool res = false;
    Frame frame[2] = {0};
    frame[0].can_dlc = 7;
    frame[1].can_dlc = 8;
    frame[0].can_id = id_snd_[0];
    frame[1].can_id = id_snd_[1];
    {
        /* 第一个数据包 */
        /* 第一个字节 */
        if (send_pack.burstAim){
            Util::setByteFlags(frame[0].data[0], 2, 1, 1);
        }else {
            Util::setByteFlags(frame[0].data[0], 2, 1, 0);
        }
        if (send_pack.fire){
            Util::setByteFlags(frame[0].data[0], 1, 1, 1);
        }else {
            Util::setByteFlags(frame[0].data[0], 1, 1, 0);
        }
        if (send_pack.target_found){
            Util::setByteFlags(frame[0].data[0], 0, 1, 1);
        }else {
            Util::setByteFlags(frame[0].data[0], 0, 1, 0);
        }
        /* 第二个字节，工作模式 */
        if (send_pack.mode == Mode::MODE_ARMOR2){
            frame[0].data[1] = 0x01;
        }else if (send_pack.mode == Mode::MODE_SMALLRUNE){
            frame[0].data[1] = 0x02;
        }else if (send_pack.mode == Mode::MODE_BIGRUNE){
            frame[0].data[1] = 0x03;
        }else if (send_pack.mode == Mode::MODE_SENTINEL){
            frame[0].data[1] = 0x01; // TODO 
        }else{
            LOGE("send error mode!");
            return false;
        }
        /* 控制精度 */
        frame[0].data[2] = static_cast<int16_t>(send_pack.pitch_resolution * 180) >> 8;
        frame[0].data[3] = static_cast<int16_t>(send_pack.pitch_resolution * 180);
        frame[0].data[4] = static_cast<int16_t>(send_pack.yaw_resolution * 180) >> 8;
        frame[0].data[5] = static_cast<int16_t>(send_pack.yaw_resolution * 180);// 精确到
        /* 检校和 */
        for (int i = 0; i < 6; ++i){
            frame[0].data[6] += frame[0].data[i];
        }
    }
    {
        if (send_pack.pred_yaw > 180.0)
        {
            send_pack.pred_yaw -= 360.0;
        } else if(send_pack.pred_yaw < -180.0)
        {
            send_pack.pred_yaw += 360.0;
        }
        /* 第二个数据包 */
        int percentile = send_pack.clock % 1000; 
        int units_digit = send_pack.clock % 10;
        int send_digit = (percentile - units_digit) / 10;
        frame[1].data[0] = static_cast<int8_t>(send_digit); /* 当前时钟的十位数 */
        frame[1].data[1] = (static_cast<int16_t>(send_pack.pred_pitch * 180)) >> 8;
        frame[1].data[2] = (static_cast<int16_t>(send_pack.pred_pitch * 180));
        frame[1].data[3] = static_cast<int16_t>(send_pack.pred_yaw * 180) >> 8;
        frame[1].data[4] = static_cast<int16_t>(send_pack.pred_yaw * 180);
        frame[1].data[5] = static_cast<int8_t>(send_pack.pitch_palstance); // 精确到度/s
        frame[1].data[6] = static_cast<int8_t>(send_pack.yaw_palstance);
        for (int i = 0; i < 7; ++i)
        {
            frame[1].data[7] += frame[1].data[i];
        }
    }
    for (int i = 0; i < 2; i++)
    {
        if (i == 1 && !send_pack.target_found){ // 若没有找到目标，不发送坐标设定值
            break;
        }
        int nbytes = write(skt_, &frame[i], sizeof(Frame));
        if (nbytes < 0) {
            std::cerr << "can raw socket write.\n";
            res = false;
        } else if (nbytes < sizeof(Frame)) {
            std::cerr << "write: incomplete frame.\n";
            res = false;
        } else if (nbytes == sizeof(Frame)) {
            for (int i = 0; i < 8; ++i)
                // can_printf("SEND data[%d]: %x\n", i, frame.data[i]);
            res = true;
        }
    }
    return res;
}

bool CanNode::send(float value, int id) 
{
    bool res = false;
    Frame frame;
    uint8_t *p;
    p = (uint8_t*)&value;
    frame.can_dlc = 5;
    frame.can_id = id_snd_[0];
    frame.data[0] = static_cast<uint8_t>(id);
    frame.data[1] = (uint8_t)p[3];
    frame.data[2] = (uint8_t)p[2];
    frame.data[3] = (uint8_t)p[1];
    frame.data[4] = (uint8_t)p[0];
    int nbytes = write(skt_, &frame, sizeof(Frame));
    if (nbytes < 0) {
        std::cerr << "can raw socket write.\n";
        res = false;
    } else if (nbytes < sizeof(Frame)) {
        std::cerr << "write: incomplete frame.\n";
        res = false;
    } else if (nbytes == sizeof(Frame)) {
        for (int i = 0; i < 8; ++i)
            can_printf("SEND data[%d]: %x\n", i, frame.data[i]);
        res = true;
    }
    return res;
}


bool CanNode::receive(ReadPack &read_pack) {
    bool res = false;

    Frame frame;

    //
    int nbytes = read(skt_, &frame, sizeof(Frame));

    if (nbytes < 0) {
        std::cerr << "can raw socket read.\n";
        res = false;
    } else if (nbytes < sizeof(Frame)) {
        std::cerr << "read: incomplete frame.\n";
        res = false;
    } else if (nbytes == sizeof(Frame)) {
        unpack(frame, read_pack);
        res = true;
    }

    return res;
}

bool CanNode::unpack(const Frame &frame, ReadPack &read_pack) 
{
    // 根据 id 分配包的类型
    switch (frame.can_id) 
    {
        // 对时包
        case 0x011:
            read_pack.pack_type = CommunicationPackType::READ_PACK_FOR_TIME;
            break;
        // 姿态包
        case 0x012:
            read_pack.pack_type = CommunicationPackType::READ_PACK_FOR_POSE;
            break;
        // 处理发送的 id 错误的情况
        default:
            LOGE("can_id invalid: %X", frame.can_id);
            return false;
    }

    // 根据报文的字节数，输出报文内容
    can_printf("READ frame.can_id: %X\n", frame.can_id);
    for (int i = 0; i < frame.can_dlc; ++i) 
    {
        can_printf("READ data[%d]: %X\n", i, frame.data[i]);
    }

    // 根据报文的字节数进行和校验
    uint8_t checksum = 0;
    for (int i = 0; i < frame.can_dlc - 1; ++i) 
    {
        checksum += frame.data[i];
    }
    if (frame.data[frame.can_dlc-1] != checksum) 
    {
        printf("READ data checksum error: %X\n", checksum);
        return false;
    }

    if (read_pack.pack_type == CommunicationPackType::READ_PACK_FOR_TIME)
    {
        // 当前电控基准时刻
        int64_t temp_base_mcu_time =
            (static_cast<int64_t>(frame.data[2]) << 16) +
            (static_cast<int64_t>(frame.data[1]) << 8) +
            static_cast<int64_t>(frame.data[0]);
        // 转成 ms
        read_pack.base_mcu_time = temp_base_mcu_time * 100;

        // 当前自瞄模式
        int8_t temp_mode = static_cast<int8_t>(frame.data[3]);
        temp_mode = static_cast<int8_t>( temp_mode & 0xF); // & 1 即可保留；& 0 即可清 0
        switch(temp_mode)
        {
            case 0x01:
                read_pack.mode = Mode::MODE_ARMOR2;
                break;
            case 0x02:
                read_pack.mode = Mode::MODE_SMALLRUNE;
                break;
            case 0x03:
                read_pack.mode = Mode::MODE_BIGRUNE;
                break;
            case 0x04:
                read_pack.mode = Mode::MODE_SENTINEL;
            default:
                return false;
        }

        // 当前弹速
        read_pack.bullet_speed = (((static_cast<int16_t>(frame.data[3] & (0xF0))) << 4) +
                                 (static_cast<int16_t>(frame.data[4]))) * 0.01;

        // 处理标志位字节
        int8_t temp_flag = static_cast<int8_t>(frame.data[5]);
        // 是否按下了鼠标右键. TODO待测试
        bool last_status = read_pack.isRightMouseButtonPressing;
        if ((temp_flag & 1) == 1) {
            read_pack.isRightMouseButtonPressing = true;
        } else {
            read_pack.isRightMouseButtonPressing = false;
        }
        if (!last_status && read_pack.isRightMouseButtonPressing)
        {
            read_pack.isRightMouseButtonClicked = true;
        }else{
            read_pack.isRightMouseButtonClicked = false;
        }

        // 敌方颜色
        if (((temp_flag >> 1) & 1) == 1) 
        {
            read_pack.enemy_color = EnemyColor::COLOR_RED;
        } else 
        {
            read_pack.enemy_color = EnemyColor::COLOR_BLUE;
        }
        // 敌方数字
        read_pack.enemy_number = static_cast<int>((temp_flag >> 2) & 0xF);  // 采用 & 运算是为了将无关高位也清 0
    }
    else if (read_pack.pack_type == CommunicationPackType::READ_PACK_FOR_POSE)
    {
        // 记录当前云台 pitch
        int16_t temp_pitch = (static_cast<int16_t>(frame.data[0]) << 8) +
                                static_cast<int16_t>(frame.data[1]);
        read_pack.ptz_pitch = static_cast<double>(temp_pitch) * 1 / 180.0;
        // 记录当前云台 yaw
        int16_t temp_yaw = (static_cast<int16_t>(frame.data[2]) << 8) +
                            static_cast<int16_t>(frame.data[3]);
        read_pack.ptz_yaw = static_cast<double>(temp_yaw) * 1 / 180.0;
        // 记录当前云台 roll
        int16_t temp_roll = (static_cast<int16_t>(frame.data[4]) << 8) +
                            static_cast<int16_t>(frame.data[5]);
        read_pack.ptz_roll = static_cast<double>(temp_roll) * 1 / 180.0;
        // 记录当前姿态的电控时刻（未补全）
        int64_t temp_pose_mcu_time = static_cast<int64_t>(frame.data[6]);
        read_pack.pose_mcu_time = temp_pose_mcu_time * 5;
    } else {
        LOGE("error pack type!");
    }

    can_printf("READ pack_type: %d\n", read_pack.pack_type);
    can_printf("READ base_mcu_time: %u\n", read_pack.base_mcu_time);
    can_printf("READ pose_mcu_time: %u\n", read_pack.pose_mcu_time);
    can_printf("READ mode:%d own_color:%d\n", read_pack.mode, read_pack.enemy_color);
    can_printf("READ enemy_number: %d\n", read_pack.enemy_number);
    can_printf("READ ptz_pitch: %lf ptz_yaw: %lf\n ptz_roll: %lf\n", read_pack.ptz_pitch, read_pack.ptz_yaw, read_pack.ptz_roll);
    can_printf("READ bullet_speed: %lf\n", read_pack.bullet_speed);
    can_printf("READ isRightMouseButtonClicked: %d\n", read_pack.isRightMouseButtonClicked);    
    return true;
}


// 更新一个字节中的指定位
static void Flag_Ctrl(char *FlagChar, unsigned char Num, char Flag)
{
    if(Num>7)
    {
        return;
    }
    else
    {
        if(Flag)   
        {
            // 置位
            *FlagChar = *FlagChar | (1 << Num);
        }
        else
        {
            // 清零
            *FlagChar = *FlagChar & (~(1 << Num));
        }
    }
}

// 字节中指定的位
static char Flag_Read(char Flags_IN, unsigned char Num)
{
    if (Num > 7)
    {
        // 清零
        return 0;
    }
    else
    {
        if (Flags_IN & (1 << Num))
        {
            // 置位
            return 1;
        }
        else
        {
            // 清零
            return 0;
        }
    }
}