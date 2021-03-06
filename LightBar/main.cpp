#include <thread>
#include "lineTest.h"


using namespace std;
using namespace cv;

int main() {
    bool init = false;
    LineInfo lineInfo;
    while (true) {
        if (!init) {
            lineInfo.init();
            LineTest tracker;
            thread thread1(tracker, ref(lineInfo));
            thread1.detach();
            init = true;
        } else {
            float res[3];
            if (lineInfo.get(res) > 0)
                cout << res[0] <<"  "<< res[1] << endl;
            //can't get 'A' once more
//            if (getchar() == 65){
//                info.setStop(true);
//                cout << info.getStop() << endl;
//            }
//            else if (getchar() == 97)
//                init = false;
        }
    }


    return 0;
}