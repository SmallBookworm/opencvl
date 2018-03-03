//
// Created by peng on 18-3-3.
//

#include "protonect.h"

using namespace std;
using namespace cv;

int Protonect::connect() {
    //! [context]
    //! [context]

    //! [discovery]
    if (this->freenect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    string serial = this->freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;
    //! [discovery]

    int depthProcessor = Processor::cl;

    if (depthProcessor == Processor::cpu) {
        if (!this->pipeline)
            //! [this->pipeline]
            this->pipeline = new libfreenect2::CpuPacketPipeline();
        //! [this->pipeline]
    } else if (depthProcessor == Processor::gl) {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if (!this->pipeline)
            this->pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        std::cout << "OpenGL this->pipeline is not supported!" << std::endl;
#endif
    } else if (depthProcessor == Processor::cl) {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if (!this->pipeline)
            this->pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
        std::cout << "OpenCL this->pipeline is not supported!" << std::endl;
#endif
    }

    if (this->pipeline) {
        //! [open]
        this->dev = this->freenect2.openDevice(serial, this->pipeline);
        //! [open]
    } else {
        this->dev = this->freenect2.openDevice(serial);
    }

    if (this->dev == 0) {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    //signal(SIGINT, this->sigint_handler);
    //this->protonect_shutdown = false;

    //! [listeners]
    this->listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color |
                                                              libfreenect2::Frame::Depth |
                                                              libfreenect2::Frame::Ir);

    this->dev->setColorFrameListener(this->listener);
    this->dev->setIrAndDepthFrameListener(this->listener);
    //! [listeners]

    return 0;
}

void Protonect::start() {
    //! [start]
    this->dev->start();

    std::cout << "device serial: " << this->dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << this->dev->getFirmwareVersion() << std::endl;
    //! [start]

    //! [registration setup]
    this->registration = new libfreenect2::Registration(this->dev->getIrCameraParams(),
                                                        this->dev->getColorCameraParams());

}

void Protonect::stop() {
    //! [stop]
    dev->stop();
    dev->close();
    //! [stop]

    delete this->registration;
}