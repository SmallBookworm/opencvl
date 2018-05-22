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
    short x = -3175, y = 4490, angle = 90;
    //the Global Positioning System's height is  340 mm
    Vec4f ring(y - 500, 2400 - 310, x + 3175, angle / 180.0 * M_PI);
    //change coordinate system
    float c1 = ring[0], c2 = ring[2];
    ring[0] = static_cast<float>(cos(ring[3]) * c1 - sin(ring[3]) * c2);
    ring[2] = static_cast<float>(cos(ring[3]) * c2 + sin(ring[3]) * c1);
    //camera relative position
    ring[2] -= 0;
    //mm -> m
    ring[0] /= 1000;
    ring[1] /= 1000;
    ring[2] /= 1000;
    cout << ring << endl;
    position.init(ring);
    getchar();
    Point2f point2f;
    if (position.getPoint(point2f) > 0)
        cout << point2f << endl;
    getchar();
    position.setStop(true);
    return 0;
}