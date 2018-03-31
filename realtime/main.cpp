#include <iostream>
#include <thread>

#include "ringTracker.h"

using namespace std;
using namespace cv;

int main() {
    Coordinate coordinate;
    RingTracker ringTracker;
    thread thread1(ringTracker,ref(coordinate));
    thread1.detach();
    while (true) {
        Point2f point2f = coordinate.get();
        cout << point2f << endl;
        usleep(1000000);
    }

    return 0;
}