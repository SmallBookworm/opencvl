//
// Created by peng on 18-3-28.
//

#include "ringTracker.h"

using namespace cv;
using namespace std;

cv::Point2f RingTracker::getCoordinate() {
    lock_guard<mutex> l(coor_mutex);
    return coordinate;
}

void RingTracker::operator()() {
    usleep(100000);
    lock_guard<mutex> lockGuard(coor_mutex);
    coordinate.x += 1;

}