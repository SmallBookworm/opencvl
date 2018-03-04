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
        Mat scene, double scale) {
    Mat img;
    resize(scene, img, Size(0, 0), scale, scale);

    Mat fgmask, fgimg, bgimg;
    this->pBackgroundKnn->apply(img, fgmask);

    medianBlur(fgmask, fgmask, 5);

    morphologyEx(fgmask, fgmask, MORPH_CLOSE, Mat::ones(15, 3, CV_8UC1));
    //test
    namedWindow("MORPH_CLOSE", CV_WINDOW_NORMAL);
    resizeWindow("MORPH_CLOSE", 1080, 720);
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
template <class T>
float Tracker::getCircleDepth(cv::Vec4f circle, cv::Mat &depthMat) {
    float result = 0;
    int count=0;
    for (int i = static_cast<int>(ceil(circle[1] - circle[2])); i < circle[1] + circle[2]; ++i) {
        double squareX = pow(circle[2], 2) - pow(i - circle[1], 2);
        int maxX = static_cast<int>(sqrt(squareX) + circle[0]);
        int minX = static_cast<int>(ceil(circle[0] - sqrt(squareX)));
        for (int j = minX; j < maxX; ++j) {
            T depth = depthMat.at<T>(i, j);
            //test
            //cout << "point" << i << "," << j << ":" << depth << endl;
            result += depth[0];
            count++;
        }
    }
    return result/count;
}

cv::Vec4f Tracker::getBall(std::vector<std::vector<cv::Point>> contours, Mat &resultImage) {
    bool minI = false;
    Vec4f minC;
    float cSize;
    float minSizes, minX, minY, maxX, maxY, minR, maxR, maxC;
    minSizes = 60;
    if (this->ballCoordinates.empty()) {
        maxY = resultImage.rows * 3 / 4;
        minY = 0;
        maxX = resultImage.cols / 3;
        minX = 0;
        maxR = resultImage.rows / 10;
        minR = resultImage.rows / 20;
        maxC = 8;
    } else {
        Vec4f before = this->ballCoordinates.back();
        Vec3f info = this->ballInfo.back();
        //speed 50
        maxX = before[0] + 100 * (this->frameI - info[0]);
        minX = before[0];
        maxY = before[1] + 50 * (this->frameI - info[0]);
        minY = before[1] - 50 * (this->frameI - info[0]);
        maxR = static_cast<float>(before[2] * 1.2);
        minR = before[2] / (2 * (this->frameI - info[0]));

        minSizes = info[1] / (2 * (this->frameI - info[0]));
        maxC = before[3] * 2;
    }

    for (auto &contour : contours) {
        if (contour.size() < minSizes)
            continue;
        Vec4f circle = this->getEdgeCircle(contour);
        //test
        //cout << circle << endl;
        if (circle[0] > maxX || circle[0] < minX)
            continue;
        if (circle[1] > maxY || circle[1] < minY)
            continue;
        if (circle[2] > maxR || circle[2] < minR)
            continue;
        if (circle[3] > maxC)
            continue;

        if (!minI) {
            minC = circle;
            cSize = contour.size();
            minI = true;
        } else if (circle[3] < minC[3]) {
            minC = circle;
            cSize = contour.size();
        }
    }
    if (!minI) {
        minC[0] = -1;
        return minC;
    }
    this->ballCoordinates.push_back(minC);
    this->ballInfo.emplace_back(this->frameI, cSize, this->getCircleDepth<Vec3b>(minC, resultImage));
    //test
    Point center(round(minC[0]), round(minC[1]));
    int radius = round(minC[2]);
    cv::circle(resultImage, center, radius, Scalar(0, 255, 0), 1);
    cerr << minC << endl;
    cerr << this->ballInfo.back() << endl;
    return minC;
}

int Tracker::isPassed(cv::Mat &frame) {
    vector<vector<Point>> contours = this->findForegroundContours(frame, 1);
    if (this->ring[0] < 0) {
    }
    Mat result = frame.clone();
    Vec4f circle = getBall(contours, result);
    if (circle[0] < 0) {
        return -1;
    }
    namedWindow("ball", 0);
    resizeWindow("ball", 640, 480);
    imshow("ball", result);
    usleep(100000);
    return 0;
}

void Tracker::test() {
    VideoCapture videoCapture("/home/peng/下载/ball_pass_ring(5)/depth(pass).avi");
    if (!videoCapture.isOpened()) {
        perror("open video fail!");
        return;
    }

    Mat frame;
    this->frameI = 0;
    while (videoCapture.isOpened()) {
        videoCapture >> frame;
        if (frame.empty())
            break;
        ++this->frameI;
        if (this->frameI < 30 || this->frameI > 100)
            continue;
        cout << this->frameI << endl;
        if(this->frameI==54)
        imshow("86", frame);
        cout << this->isPassed(frame) << endl;

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

