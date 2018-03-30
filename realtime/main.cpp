#include <iostream>
#include <thread>

#include "ringTracker.h"

using namespace std;
using namespace cv;

int main() {
    RingTracker ringTracker;
    thread thread1(ringTracker);
    thread1.detach();
    while (true) {
        Point2f point2f = ringTracker.getCoordinate();
        cout << point2f << endl;
        usleep(1000000);
    }

    return 0;
}