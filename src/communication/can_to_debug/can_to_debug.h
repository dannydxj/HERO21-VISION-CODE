#ifndef DEBUG_CANNODE_H
#define DEBUG_CANNODE_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "base.h"
#include "log.h"
#include "macro_switch.h"
#include "types.h"
#include "util_func.h"

#define DEBUG_REC_ID_NUM 2
#define DEBUG_SEND_ID_NUM 1

class DebugCAN {
   public:
    typedef struct sockaddr_can SockAddr_Can;
    typedef struct ifreq Ifreq;
    typedef struct can_filter Filter;
    typedef struct can_frame Frame;  // send frame

   private:
    SockAddr_Can addr_;
    Ifreq ifr_;
    Filter cfilter_[2];
    int skt_;  // socket
    std::string dev_name;

    // remained for multiple ids
    unsigned int id_rcv_[DEBUG_REC_ID_NUM] = {0x301, 0x030};
    unsigned int id_snd_[DEBUG_SEND_ID_NUM] = {0x001};

   public:
    DebugCAN();
    ~DebugCAN();
    bool init();
    bool send(float value, int id);

};

#endif
