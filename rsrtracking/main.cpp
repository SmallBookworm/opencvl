#include <iostream>
#include <thread>
#include <future>
#include "ball_tracker.h"

using namespace std;
using namespace cv;

int main() {
    DeviationPosition position;
    position.await();
    Tracker tracker;
    thread thread1(tracker, ref(position));
    thread1.detach();
    //tracker.test();
    getchar();
    position.init(Vec3f(0.314, 1.138, 5.334));
    getchar();
    Point2f point2f;
    if (position.getPoint(point2f) > 0)
        cout << point2f << endl;
    getchar();
    position.setStop(true);
    return 0;
}