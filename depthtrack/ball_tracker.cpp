//
// Created by peng on 18-3-1.
//


#include "ball_tracker.h"

using namespace std;
using namespace cv;

Tracker::Tracker() {
    this->frameI = 0;
    this->ring[0] = -1;
}

vector<vector<Point>> Tracker::findAllContours(Mat &input) {
    Mat frame = input.clone();

    Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
    dilate(frame, frame, element);
    cvtColor(frame, frame, CV_BGR2GRAY);
    //高斯平滑
    GaussianBlur(frame, frame, Size(9, 9), 0, 0);
//    imshow("GaussianBlur", frame);

    Canny(frame, frame, 30, 60, 3);
    imshow("Canny", frame);

    vector<vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy,
                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());

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
//    namedWindow("MORPH_CLOSE", CV_WINDOW_NORMAL);
//    resizeWindow("MORPH_CLOSE", 1080, 720);
//    imshow("MORPH_CLOSE", fgmask);
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

template<class T>
float Tracker::getCircleDepth(cv::Vec4f circle, cv::Mat &depthMat) {
    float result = 0;
    int count = 0;
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
    return result / count;
}

cv::RotatedRect Tracker::getRingPole(std::vector<cv::Point> contours) {
    vector<cv::Point> pole;
    RotatedRect rect = minAreaRect(contours);
    float minY = 1000, maxY = 0;
    Point2f points[4];
    rect.points(points);
    for (auto &point:points) {
        if (point.y < minY)
            minY = point.y;
        else if (point.y > maxY)
            maxY = point.y;
    }
    float min = minY + (maxY - minY) / 3, max = maxY - (maxY - minY) / 3;
    for (auto &pt:contours) {
        if (pt.y > min && pt.y < max)
            pole.push_back(pt);
    }
    return minAreaRect(pole);
}

template<typename T>
float Tracker::selectROIDepth(std::string windowName, cv::Mat &depthMat) {
    Rect rect = selectROI(windowName, depthMat);
    cout << "tl:" << rect.tl() << endl;
    return depthMat.at<T>(rect.tl())[0];
}

cv::Vec4f Tracker::getBall(std::vector<std::vector<cv::Point>> contours, Mat &resultImage) {
    bool minI = false;
    Vec4f minC;
    float cSize;
    float minSizes, minX, minY, maxX, maxY, minR, maxR, maxC;
    minSizes = 20;
    if (this->ballCoordinates.empty()) {
        maxY = resultImage.rows * 3 / 4;
        minY = 0;
        maxX = resultImage.cols / 2;
        minX = 0;
        maxR = resultImage.rows / 10;
        minR = resultImage.rows / 35;
        maxC = 3;
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

int Tracker::getRing(std::vector<std::vector<cv::Point>> contours, cv::Mat &result) {
    vector<RotatedRect> rects = this->getRotatedRect(contours);
    for (int i = 0; i < rects.size(); ++i) {
        if (rects[i].size.height < 150)
            continue;
        if (rects[i].size.width > rects[i].size.height / 2)
            continue;
        if (rects[i].center.x < 200 || rects[i].center.x > 300)
            continue;
        if (rects[i].center.y < result.rows / 2)
            continue;

        this->ring[0] = 1;
        RotatedRect ringPole = this->getRingPole(contours[i]);
        //test
        Point2f points[4];
        ringPole.points(points);
        for (int j = 0; j < 4; ++j) {
            line(result, points[j], points[(j + 1) % 4], Scalar(0, 255, rects[i].center.y), 1, CV_AA);
        }
        break;
    }
    return 1;
}

//-1 no ball,0 ball run,1 pass,2 not pass
int Tracker::isPassed(cv::Mat &frame) {
    vector<vector<Point>> contours = this->findForegroundContours(frame, 1);
    if (this->ring[0] < 0) {
//        Mat ringR = frame.clone();
//            imshow("ring", ringR);
//
//        cout<<"depth:"<<this->selectROIDepth<Vec3b>("ring", ringR)<<endl;
        this->ring = Vec4f(240, 234, 51, 180);
    }
    Mat result = frame.clone();
    Vec4f circle = getBall(contours, result);
    // restart when no ball in 10 frames
    if (circle[0] < 0) {
        if (!this->ballInfo.empty() && this->frameI - this->ballInfo.back()[0] > 10) {
            this->ballInfo.clear();
            this->ballCoordinates.clear();
        }
        return -1;
    }
    namedWindow("ball", 0);
    resizeWindow("ball", 640, 480);
    imshow("ball", result);
    usleep(100000);
    Vec3f info0 = this->ballInfo.back();
    if (info0[2] >= this->ring[3]) {
        if (this->frameI - this->ballInfo.back()[0] > 10) {
            this->ballInfo.clear();
            this->ballCoordinates.clear();
        }
        if (sqrt(pow((info0[0] - this->ring[0]), 2) + pow(info0[1] - this->ring[1], 2)) < this->ring[2])
            return 1;
        else
            return 2;
    }

    return 0;
}

void Tracker::test() {
    VideoCapture videoCapture("/home/peng/下载/ball_pass_ring(5)/depth(fail).avi");
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
        if (this->frameI < 30 || this->frameI > 1000)
            continue;
        cout << this->frameI << endl;
        if (this->frameI == 93)
            imshow("97", frame);
        int pas = this->isPassed(frame);
        switch (pas) {
            case -1:
                cerr << "no ball!" << endl;
                break;
            case 0:
                cout << "\033[33m" << "run" << "\033[0m" << endl;
                break;
            case 1:
                cout << "\033[32m" << "success!" << "\033[0m" << endl;
                break;
            case 2:
                cout << "\033[32m" << "fail!" << "\033[0m" << endl;
                break;
        }

        if (waitKey(1) == 27)
            break;
    }

}

bool Protonect::protonect_shutdown = false;

// move-constructible function object (i.e., an object whose class defines operator(), including closures and function objects).
void Tracker::operator()(std::future<int> &fut) {

    if (this->protonect.connect() < 0) {
        return;
    }
    this->protonect.start();
    libfreenect2::FrameMap frames;
    Mat depthmat;
    future_status status;
    do {
        protonect.listener->waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        //main
        ++this->frameI;
        cout << this->frameI << endl;
        int pas = this->isPassed(depthmat);
        switch (pas) {
            case -1:
                cerr << "no ball!" << endl;
                break;
            case 0:
                cout << "\033[33m" << "run" << "\033[0m" << endl;
                break;
            case 1:
                cout << "\033[32m" << "success!" << "\033[0m" << endl;
                break;
            case 2:
                cout << "\033[32m" << "fail!" << "\033[0m" << endl;
                break;
        }
        //main
        //test
        cv::imshow("depth", depthmat / 4500.0f);
        int key = cv::waitKey(1);
        Protonect::protonect_shutdown =
                Protonect::protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        protonect.listener->release(frames);
        status = fut.wait_for(chrono::milliseconds(1));
    } while (!Protonect::protonect_shutdown && (status != future_status::ready));
    protonect.stop();
}

