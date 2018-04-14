//
// Created by peng on 18-3-31.
//

#ifndef REALTIME_COORDINATE_H
#define REALTIME_COORDINATE_H

#include <opencv2/opencv.hpp>
#include <mutex>

class Coordinate {
private:
    //output coordinate
    cv::Point2f coordinate;
    std::mutex coor_mutex;
    //input angle
    float dAngle;
    std::mutex angle_mutex;
public:
    explicit Coordinate() {
        this->coordinate.x = 0.0;
        this->coordinate.y = 0.0;
        this->dAngle = 0;
    };

    cv::Point2f get();

    //only one thread use this function
    void set(cv::Point2f point);

    float getDAngle();

    void setDAngle(float dAngle);
};


#endif //REALTIME_COORDINATE_H
