//
// Created by peng on 18-3-1.
//


#include "ball_tracker.h"

using namespace std;
using namespace cv;

Tracker::Tracker() {
    this->ring[0] = -1;
}

vector<vector<Point>> Tracker::findAllContours(Mat &input) {
    Mat frame = input.clone();

    Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
    dilate(frame, frame, element);
    cvtColor(frame, frame, CV_BGR2GRAY);
    //高斯平滑
    GaussianBlur(frame, frame, Size(9, 9), 0, 0);
    imshow("GaussianBlur", frame);

    Canny(frame, frame, 30, 60, 3);
    imshow("Canny", frame);

    vector<vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy,
                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    return contours;
}

std::vector<std::vector<Point>> Tracker::findForegroundContours(
        const Ptr<BackgroundSubtractorMOG2> &pBackgroundKnn,
        Mat scene, double scale) {
    Mat img;
    resize(scene, img, Size(0, 0), scale, scale);

    Mat fgmask, fgimg, bgimg;
    pBackgroundKnn->apply(img, fgmask);

    medianBlur(fgmask, fgmask, 5);

    morphologyEx(fgmask, fgmask, MORPH_CLOSE, Mat::ones(15, 3, CV_8UC1));
    imshow("MORPH_CLOSE", fgmask);
    std::vector<std::vector<Point>> region_contours;
    findContours(fgmask, region_contours, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    return region_contours;
}

std::vector<RotatedRect> Tracker::getRotatedRect(std::vector<std::vector<Point>> region_contours) {
    std::vector<RotatedRect> objects;
    for (size_t i = 0; i != region_contours.size(); ++i) {
        RotatedRect rect = minAreaRect(region_contours[i]);
        objects.push_back(rect);
    }
    return objects;
}

Vec4f Tracker::getEdgeCircle(std::vector<Point> contour) {
    Vec4f circle;
    Point2f center;
    float radius = 0;
    minEnclosingCircle(contour, center, radius);
    circle[0] = center.x;
    circle[1] = center.y;
    circle[2] = radius;
    circle[3] = 0;
    for (auto &point:contour) {
        circle[3] += abs(sqrt(pow(center.x - point.x, 2) + pow(center.y - point.y, 2)) - radius);
    }
    circle[3] /= contour.size();
    return circle;
}

cv::Vec4f Tracker::getBall(std::vector<std::vector<cv::Point>> contours, Mat &resultImage) {
    bool minI = false;
    Vec4f minC;
    int minx, miny, max;

    if (this->ballCoordinates.empty()) {
        miny = resultImage.rows * 2 / 3;
    }

    for (auto &contour : contours) {
//        if (contour.size() < 7)
//            continue;
        Vec4f circle = this->getEdgeCircle(contour);
//        if (circle[1] > miny)
//            continue;
//        if (circle[2] < 10 || circle[2] > 30)
//            continue;
//        if (circle[3] < 5)
//            continue;
        Point center(round(circle[0]), round(circle[1]));
        int radius = round(circle[2]);
        cv::circle(resultImage, center, radius, Scalar(0, 255, 0), 1);
        cout << circle << endl;
        minC = circle;
        if (!minI) {
            minI = true;
        }
    }
    if (minI) {
        minC[0] = -1;
    }
    return minC;
}

int Tracker::isPassed(cv::Mat &frame) {
    Ptr<BackgroundSubtractorMOG2> pBackgroundKnn = createBackgroundSubtractorMOG2();
    vector<vector<Point>> contours = this->findForegroundContours(pBackgroundKnn,frame,1);
    if (this->ring[0] < 0) {
    }
    Mat result = frame.clone();
    Vec4f circle = getBall(contours, result);
    imshow("ball", result);
    usleep(300000);
}

void Tracker::test() {
    VideoCapture videoCapture("/home/peng/下载/ball_pass_ring(5)/depth(fail).avi");
    Mat frame;
    int i = 0;
    while (videoCapture.isOpened()) {
        videoCapture >> frame;
        if (frame.empty())
            break;
        ++i;
        if (i < 70 || i > 100)
            continue;
        this->isPassed(frame);
        cout << i << endl;
        if (waitKey(1) == 27)
            break;
    }

}

// move-constructible function object (i.e., an object whose class defines operator(), including closures and function objects).
void Tracker::operator()(std::future<int> &fut) {
    int i = 0;
    future_status status;

    do {

        status = fut.wait_for(chrono::milliseconds(1));
    } while (status != future_status::ready);
}

