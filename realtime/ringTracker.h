//
// Created by peng on 18-3-28.
//

#ifndef REALTIME_RINGTRACKER_H
#define REALTIME_RINGTRACKER_H

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <zconf.h>
#include "coordinate.h"
#include "cv-helpers.hpp"

class RingTracker {
private:
    double distance;

    cv::Vec3f getPoleRange(cv::Mat grayFrame);

    //-1 fail
    int getData(cv::Mat &result);

public:
    int operator()(Coordinate &coordinate);
};


#endif //REALTIME_RINGTRACKER_H
