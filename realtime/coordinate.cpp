//
// Created by peng on 18-3-31.
//

#include "coordinate.h"

using namespace std;
cv::Point2f Coordinate::get() {
    lock_guard <mutex> l(coor_mutex);
    return coordinate;
}
void Coordinate::set(cv::Point2f point) {
    lock_guard <mutex> l(coor_mutex);
    this->coordinate=point;
}

float Coordinate::getDAngle() {
    lock_guard <mutex> l(angle_mutex);
    return dAngle;
}

void Coordinate::setDAngle(float dAngle) {
    lock_guard <mutex> l(angle_mutex);
    Coordinate::dAngle = dAngle;
}
