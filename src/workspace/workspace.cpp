#include "workspace.h"

Workspace::Workspace() = default;

Workspace::~Workspace() 
{
    if (camera != nullptr) 
        delete camera;
}

void Workspace::run() 
{
    std::thread image_receiving_thread(&Workspace::imageReceivingFunc, this);
    std::thread image_processing_thread(&Workspace::imageProcessingFunc, this);
    std::thread read_communicating_thread(&Workspace::readCommunicatingFunc, this);
    std::thread send_communicating_thread(&Workspace::sendCommunicatingFunc, this);

    image_receiving_thread.join();
    image_processing_thread.join();
    read_communicating_thread.join();
    send_communicating_thread.join();
}






