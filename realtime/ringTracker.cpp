//
// Created by peng on 18-3-28.
//

#include "ringTracker.h"

using namespace cv;
using namespace std;

void RingTracker::operator()(Coordinate &coordinate) {
    int x = 0;
    while (true) {
        usleep(100000);
        coordinate.set(Point2f(x, 0));
        x++;
    }

}