#include <iostream>
#include <thread>
#include <future>
#include "ball_tracker.h"

using namespace std;
using namespace cv;

int main() {
    promise<int> prom;
    future<int> fut = prom.get_future();
    DeviationPosition position;
    Tracker tracker;
    thread thread1(tracker, ref(fut),ref(position));
    thread1.detach();
    //tracker.test();
    getchar();
    Point2f point2f;
    if (position.getPoint(point2f) > 0)
        cout << point2f << endl;
    getchar();
    prom.set_value(10);
    return 0;
}