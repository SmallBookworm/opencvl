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

void DeviationPosition::init() {
    lock_guard<mutex> l(coor_mutex);
    lock_guard<mutex> s(stop_mutex);
    state = -1;
    used = true;
    stop = false;
}
