#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>

using namespace std;
using namespace cv;

int main() {
    // declares all required variables
    Rect2d roi;
    Mat frame;
    // create a tracker object
    Ptr<Tracker> tracker = TrackerKCF::create();
    // set input video
    VideoCapture cap("/home/peng/下载/机器学习视频/IMG_2382.MOV");
    if (!cap.isOpened()) {
        std::cout << "fail to open video!" << std::endl;
        return -1;
    }
    // get bounding box
    while (cap.isOpened()){
        cap >> frame;
        namedWindow("tracker", CV_WINDOW_NORMAL);
        resizeWindow("tracker", 1080, 720);
        imshow("tracker", frame);
        if (waitKey(1) == 27){
            roi = selectROI("tracker", frame);
            //quit if ROI was not selected
            if (roi.width == 0 || roi.height == 0)
                return 0;
            else
                break;
        }
    }

    // initialize the tracker
    tracker->init(frame, roi);
    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");
    for (;;) {
        // get frame from the video
        cap >> frame;
        // stop the program if no more images
        if (frame.rows == 0 || frame.cols == 0)
            break;
        // update the tracking result
        tracker->update(frame, roi);
        // draw the tracked object
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
        // show image with the tracked object
        imshow("tracker", frame);
        //quit on ESC button
        if (waitKey(1) == 27)break;
    }
    return 0;
}