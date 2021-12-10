#include "can_to_debug.h"

DebugCAN::DebugCAN() {}

DebugCAN::~DebugCAN() {
    if (skt_) {
        close(skt_);
    }
}

bool DebugCAN::init() {
    bool status = true;

    dev_name = "can1";
    system("ip link set down can1");
    system(
        "ip link set can1 type can bitrate 1000000 dbitrate 2000000 "
        "berr-reporting on fd on");
    system("ip link set up can1");
    // dev_name = "can0";
    // system("ip link set down can0");
    // system(
    //     "ip link set can0 type can bitrate 1000000 dbitrate 2000000 "
    //     "berr-reporting on fd on");
    // system("ip link set up can0");


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
    for (int i = 0; i < DEBUG_REC_ID_NUM; ++i) {
        cfilter_[i].can_id = id_rcv_[i];      // receive frame id
        cfilter_[i].can_mask = CAN_SFF_MASK;  // succeed when (reveived_id) & / mask == (can_id) & mask
    }
    if (setsockopt(skt_, SOL_CAN_RAW, CAN_RAW_FILTER, &cfilter_, sizeof(cfilter_)) != 0) 
    {
        std::cout << dev_name << " sockopt error.\n";
        status = false;
    }
    return status;
}

bool DebugCAN::send(float value, int id) 
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
