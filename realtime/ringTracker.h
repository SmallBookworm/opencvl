//
// Created by peng on 18-3-28.
//

#ifndef REALTIME_RINGTRACKER_H
#define REALTIME_RINGTRACKER_H

#define SENSEANGLE (0.0 / 180 * M_PI)
#define HANGLE (64.0 / 180 * M_PI)
#define VANGLE (41.0 / 180 * M_PI)

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <zconf.h>
#include "coordinate.h"

class RingTracker {
private:
    double distance;

    cv::Point2f absoluteCoordinate(float x, float y, float angle);

    cv::Vec3f getCoordinate(float x, float y, float z, int wWidth, int wHeight);

    cv::Vec3f getPoleRange(cv::Mat grayFrame);

    //-1 fail
    int getData(cv::Mat &result, Coordinate &coordinate);

public:
    int operator()(Coordinate &coordinate);
};


#endif //REALTIME_RINGTRACKER_H
