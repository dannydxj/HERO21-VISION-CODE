#ifndef CIRCLEQUEUE_H
#define CIRCLEQUEUE_H

#include "log.h"

template <typename valueType, int queue_size>
class CircleQueue {
public:
    valueType values[queue_size];
    int size = queue_size; /* 循环队列容量 */
    int length; /* 记录存储数据的长度 */
    int head;
    int tail;

public:
    CircleQueue() : head(0), tail(0), length(0){};

    ~CircleQueue() = default;

    inline bool isEmpty() const  
    { 
        return (head == tail); 
    }

    void push(const valueType &value) 
    {
        values[tail] = value;
        tail = (tail + 1) % queue_size;
        if (head == tail) 
        {
            head = (head + 1) % queue_size;
        }
        length = (tail - head + queue_size) % queue_size;
    }

    /* 删掉最旧的数据 */
    bool popHead() 
    {
        if (isEmpty()) 
        {
            LOGW("warning at popHead(): queue empty!");
            return false; // 为空，不删除数据
        }else
        {
            head = (head + 1) % queue_size;
            length = (tail - head + queue_size) % queue_size;
            return true;
        }
    }

    /* 获取倒数第n个数据 */
    bool back(valueType &value, int n) const
    { 
        // 保护
        if (isEmpty()) 
        {
            LOGW("warning at back(value, n): queue empty!");
            return false;
        }
        if (n > this->length) 
        {
            LOGW("warning at back(value, n): ask too much data!");
            return false;
        }
        int end = (tail + size - n) % size;
        value = values[end];
        return true;
    }

    void clear() { head = tail = length = 0;}

};

#endif