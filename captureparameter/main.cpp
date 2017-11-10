#include <iostream>
#include <vector>
#include <opencv/cv.hpp>


using namespace cv;

int main() {
    int deviceID = 1;
    int apiID = CAP_ANY;
    VideoCapture capture(deviceID + apiID);
    if (!capture.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    if (!capture.set(CV_CAP_PROP_FPS, 120)) {
        std::cerr << "ERROR! Unable to change camera FPS\n";
    }//帧数
//    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);//宽度
//    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);//高度

    printf("width = %.2f\n", capture.get(CV_CAP_PROP_FRAME_WIDTH));
    printf("height = %.2f\n", capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    printf("fbs = %.2f\n", capture.get(CV_CAP_PROP_FPS));
    printf("brightness = %.2f\n", capture.get(CV_CAP_PROP_BRIGHTNESS));
    printf("contrast = %.2f\n", capture.get(CV_CAP_PROP_CONTRAST));
    printf("saturation = %.2f\n", capture.get(CV_CAP_PROP_SATURATION));
    printf("hue = %.2f\n", capture.get(CV_CAP_PROP_HUE));
    printf("exposure = %.2f\n", capture.get(CV_CAP_PROP_EXPOSURE));


    Mat frame;
    double fps;
    char string[10];
    int nCount = 0;
    TickMeter tm;

    for (;;) {
        if(nCount==0)
            tm.start();
        capture >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR!blank frame grabbed\n";
            break;
        }
        nCount++;

        if (nCount==60) {
            tm.stop();
            fps = nCount / tm.getTimeSec();
            tm.reset();
            nCount=0;
            sprintf(string, "%.2f", fps);
        }
        std::string fpsString("FPS:");
        fpsString += string;
        putText(frame, fpsString, Point(5, 20), FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 0));

        imshow("LIVE", frame);
        if (waitKey(5) >= 0)
            break;
    }
    return 0;
}