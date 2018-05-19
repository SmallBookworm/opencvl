//
// Created by peng on 18-4-6.
//

#include "deviationPosition.h"

using namespace std;

int DeviationPosition::getPoint(cv::Point2f &out) {
    lock_guard<mutex> l(coor_mutex);
    if (used) {
        return -1;
    } else {
        used = true;
        out = point;
        return state;
    }
}

void DeviationPosition::setPoint(cv::Point2f point, int res) {
    lock_guard<mutex> l(coor_mutex);
    state = res;
    DeviationPosition::point = point;
    used = false;
}

bool DeviationPosition::getStop() {
    lock_guard<mutex> l(stop_mutex);
    return stop;
}

void DeviationPosition::setStop(bool f) {
    lock_guard<mutex> l(stop_mutex);
    stop = f;
}

bool DeviationPosition::getStby() {
    lock_guard<mutex> l(stby_mutex);
    return standby;
}

void DeviationPosition::setStby(bool s) {
    lock_guard<mutex> l(stby_mutex);
    standby = s;
}

cv::Vec4f DeviationPosition::getRing() {
    lock_guard<mutex> l(ring_mutex);
    return this->ring;
}

void DeviationPosition::setRing(cv::Vec4f in) {
    lock_guard<mutex> l(ring_mutex);
    this->ring = in;
}


void DeviationPosition::init(cv::Vec4f in) {
    lock_guard<mutex> l(coor_mutex);
    lock_guard<mutex> s(stop_mutex);
    lock_guard<mutex> m(stby_mutex);
    lock_guard<mutex> k(ring_mutex);
    state = -1;
    used = true;
    stop = false;
    standby= false;
    ring = in;
}

void DeviationPosition::await() {
    lock_guard<mutex> s(stop_mutex);
    lock_guard<mutex> m(stby_mutex);
    stop = false;
    standby= true;
}