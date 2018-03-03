//
// Created by peng on 18-3-3.
//

#ifndef LKINECT_PROTONECT_H
#define LKINECT_PROTONECT_H

#include <iostream>
#include <iomanip>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

enum Processor {
    cl, gl, cpu
};

class Protonect {
public:
    bool protonect_shutdown = false;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::PacketPipeline *pipeline = nullptr;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::Registration *registration;
public:
    int connect();

    void start();

    void stop();
};


#endif //LKINECT_PROTONECT_H
