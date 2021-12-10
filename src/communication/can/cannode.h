#ifndef CANNODE_H
#define CANNODE_H

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

#define REC_ID_NUM 2
#define SEND_ID_NUM 3

class CanNode {
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
    unsigned int id_rcv_[REC_ID_NUM] = {0x011, 0x012};
    unsigned int id_snd_[SEND_ID_NUM] = {0x021, 0x022, 0x001}; // TODO 0x001 to debug

   public:
    CanNode();
    ~CanNode();
    bool init();
    bool send(SendPack &send_pack);
    bool send(float value, int id); /* 重载函数,用来debug的函数 */
    bool receive(ReadPack &read_pack);

   private:
    bool unpack(const Frame &frame, ReadPack &read_pack);
};

#endif
