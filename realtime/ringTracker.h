//
// Created by peng on 18-3-28.
//

#ifndef REALTIME_RINGTRACKER_H
#define REALTIME_RINGTRACKER_H

#include <opencv2/opencv.hpp>
#include <mutex>
#include <zconf.h>

class RingTracker {
public:
    explicit RingTracker() {
        this->coordinate.x = 0.0;
        this->coordinate.y = 0.0;
    };
private:
    cv::Point2f coordinate;
    std::mutex coor_mutex;
public:
    void operator()();

    cv::Point2f getCoordinate();
};


#endif //REALTIME_RINGTRACKER_H
