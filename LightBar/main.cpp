#include "lineTest.h"


using namespace std;
using namespace cv;

int main() {

    VideoCapture capture;
    capture.open("/home/peng/下载/realse/1.avi");

    Mat srcImage;
    if (!capture.isOpened()) {
        std::cout << "fail to open video!" << std::endl;
        return -1;
    }

    LineTest lineTest;
    while (capture.isOpened()) {
        capture >> srcImage;
        if (srcImage.empty())
            break;
        cout << lineTest.watch(srcImage) << endl;
        imshow("show", srcImage);

        if (waitKey(1) == 27) {
            break;
        }
    }
    waitKey(0);
    capture.release();
    return 0;
}