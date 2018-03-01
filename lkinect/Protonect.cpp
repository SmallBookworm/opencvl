//! [headers]
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
//! [headers]

using namespace std;
using namespace cv;

enum Processor { cl, gl, cpu };

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
    protonect_shutdown = true;
}

int main()
{
    std::cout << "Hello World!" << std::endl;

    //! [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::PacketPipeline *pipeline = nullptr;
    //! [context]

    //! [discovery]
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;
    //! [discovery]

    int depthProcessor = Processor::cl;

    if(depthProcessor == Processor::cpu)
    {
        if(!pipeline)
            //! [pipeline]
            pipeline = new libfreenect2::CpuPacketPipeline();
        //! [pipeline]
    } else if (depthProcessor == Processor::gl) {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if(!pipeline)
            pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    } else if (depthProcessor == Processor::cl) {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!pipeline)
            pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
        std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }

    if(pipeline)
    {
        //! [open]
        dev = freenect2.openDevice(serial, pipeline);
        //! [open]
    } else {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;

    //! [listeners]
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                  libfreenect2::Frame::Depth |
                                                  libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //! [listeners]

    //! [start]
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    //! [start]

    //! [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4); // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger
    //! [registration setup]

    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

    //! [loop start]
    while(!protonect_shutdown)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        //! [loop start]

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        cv::imshow("rgb", rgbmat);
        cv::imshow("ir", irmat / 4500.0f);
        cv::imshow("depth", depthmat / 4500.0f);

        //! [registration]
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        //! [registration]

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);


        cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
        cv::imshow("registered", rgbd);
        cv::imshow("depth2RGB", rgbd2 / 4500.0f);

        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        //! [loop end]
        listener.release(frames);
    }
    //! [loop end]

    //! [stop]
    dev->stop();
    dev->close();
    //! [stop]

    delete registration;

    std::cout << "Goodbye World!" << std::endl;
    return 0;
}