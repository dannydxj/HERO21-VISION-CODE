//
// 专门用于取图的循环队列
//

#ifndef QUEUE_FOR_IMAGE_H
#define QUEUE_FOR_IMAGE_H

#include <mutex>

template<typename ValType, int size>
class Queue 
{
private:
    ValType val[size];
    int head, tail;
    std::mutex reading[size];

public:
    Queue():head(0), tail(0){};

    ~Queue() = default;

    inline bool empty(){
        return head == tail;
    }

    void push(const ValType &v) 
    {
        std::unique_lock<std::mutex> lock(reading[tail]);
        val[tail] = v;
        tail = (tail + 1) % size;
        if (head == tail) 
        {
            head = (head + 1) % size;
        }
    }

    void pop(ValType &v)
    {
        while(head == tail);
        std::unique_lock<std::mutex> lock(reading[head]);
        auto tmp = head;
        head = (head + 1) % size;
        v = val[tmp];
    }
};


#endif
