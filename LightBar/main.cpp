#include "lineTest.h"


using namespace std;
using namespace cv;

int main() {
    bool init = false;
    promise<int> prom;
    LineInfo info;
    while (true) {
        if (!init) {
            future<int> fut = prom.get_future();
            info.fut = &fut;
            LineTest tracker;
            thread thread1(tracker, ref(fut), ref(info));
            thread1.detach();
            init = true;
        } else {
            float res[3];
            if (info.get(res) > 0)
                cout << res << endl;
            if (waitKey(1) == 27)
                prom.set_value(10);
            else
                break;
        }
    }


    return 0;
}