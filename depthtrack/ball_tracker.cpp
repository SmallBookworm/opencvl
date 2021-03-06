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

template<typename T>
double Tracker::distance(T x1, T x2, T y1, T y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double Tracker::realDistance(cv::Vec3f point1, cv::Vec3f point2) {
    return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2) + pow(point1[2] - point2[2], 2));
}

Mat Tracker::leastSquares(cv::Mat inMat, cv::Mat outMat) {
    Mat res = inMat.t() * inMat;
    res = res.inv();
    res *= inMat.t();
    res *= outMat;
    return res;

}

std::vector<float> Tracker::curveFitting(std::vector<float> x, std::vector<float> y, int dimension) {
    int size = static_cast<int>(x.size());
    Mat inMat(size, dimension + 1, CV_32FC1), outMat(size, 1, CV_32FC1);
    for (int i = 0; i < size; ++i) {
        auto *pxvec = inMat.ptr<float>(i);
        int j;
        for (j = 0; j < dimension + 1; ++j) {
            pxvec[j] = static_cast<float>(pow(x[i], j));
        }
        auto *pyvec = outMat.ptr<float>(i);
        pyvec[0] = y[i];
    }
    Mat res = this->leastSquares(inMat, outMat);
    vector<float> ds;
    for (int k = 0; k < dimension + 1; ++k) {
        ds.push_back(res.at<float>(0, k));
    }
    return ds;
}

Vec3f Tracker::x2curveFitting(std::vector<float> x, std::vector<float> y) {
    int size = static_cast<int>(x.size());
    Mat inMat(size, 3, CV_32FC1), outMat(size, 1, CV_32FC1);
    for (int i = 0; i < size; ++i) {
        auto *pxvec = inMat.ptr<float>(i);
        int j;
        for (j = 0; j < 3; ++j) {
            pxvec[j] = static_cast<float>(pow(x[i], j));
        }
        auto *pyvec = outMat.ptr<float>(i);
        pyvec[0] = y[i];
    }
    Mat res = this->leastSquares(inMat, outMat);
    return Vec3f(res.at<float>(0, 0), res.at<float>(0, 1), res.at<float>(0, 2));
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

cv::Vec3f Tracker::getCircleCoordinate(cv::Vec4f circle, cv::Vec3f info, int wWidth, int wHeight) {
    Vec3f coordinate;
    coordinate[2] = info[2];
    coordinate[0] = static_cast<float>(info[2] * tan((57.5 / 2) / 180 * M_PI) * (wWidth / 2 - circle[0]) /
                                       (wWidth / 2));
    coordinate[1] = static_cast<float>(info[2] * tan((43.5 / 2) / 180 * M_PI) * (wHeight / 2 - circle[1]) /
                                       (wHeight / 2));
    return coordinate;
}

float Tracker::getCircleDepth(cv::Vec4f circle, cv::Mat &depthMat) {
    float result = 0;
    int count = 0;
    float a = circle[2] / 2;
    if ((circle[0] + circle[2]) >= depthMat.cols || (circle[1] + circle[2]) >= depthMat.rows ||
        (circle[0] - circle[2]) < 0 || (circle[1] - circle[2]) < 0)
        return -1;
    for (int i = static_cast<int>(ceil(circle[1] - a)); i < circle[1] + a; ++i) {
        double squareX = pow(a, 2) - pow(i - circle[1], 2);
        int maxX = static_cast<int>(sqrt(squareX) + circle[0]);
        int minX = static_cast<int>(ceil(circle[0] - sqrt(squareX)));
        for (int j = minX; j < maxX; ++j) {
            float depth = depthMat.at<float>(i, j);
            if (depth <= 0)
                continue;
            //test
            //cout << "point" << i << "," << j << ":" << depth << endl;
            result += depth;
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
    return depthMat.at<T>(rect.tl());
}

cv::Vec4f Tracker::getBall(std::vector<std::vector<cv::Point>> contours, Mat &resultImage) {
    bool minI = false;
    Vec4f minC;
    Vec3f realC;
    float cDepth;
    float cSize;
    float minSizes, maxsizes, minX, minY, maxX, maxY, minDi, maxDi, minD, maxD, maxC, minR;
    minSizes = 20;
    maxsizes = 200;
    if (this->realCoordinates.empty()) {
        maxY = resultImage.rows;
        minY = 0;
        maxX = resultImage.cols;
        minX = resultImage.cols * 3 / 4;
        minR = 2;

        minD = 500;
        maxD = 2000;
        maxC = 30;
    } else {
        Vec3f info = this->ballInfo.back();
        //speed 50
//        maxX = before[0] + 100 * (this->frameI - info[0]);
//        minX = before[0];
//        maxY = before[1] + 50 * (this->frameI - info[0]);
//        minY = before[1] - 50 * (this->frameI - info[0]);

        maxDi = 4000 * (this->frameI - info[0]);
        minDi = 0;
        maxD = info[2] + 1000 * (this->frameI - info[0]) + 1200;
        minD = info[2] + 1000 * (this->frameI - info[0]) - 1200;

        minSizes = static_cast<float>(info[1] / (2 * (this->frameI - info[0])));
        Vec4f before = this->ballCoordinates.back();
        minR = before[2] / 4;
        maxC = before[3] * 2;
        maxC = 1 > maxC ? 1 : maxC;
    }
    for (auto &contour : contours) {
        //cout<<contour.size()<<endl;
        if (contour.size() < minSizes || contour.size() > maxsizes)
            continue;

        Vec4f circle = this->getEdgeCircle(contour);
        if (circle[2] < minR)
            continue;

        float depth = this->getCircleDepth(circle, resultImage);
        if (isnan(depth) || depth < 0)
            continue;

        Vec3f coor = this->getCircleCoordinate(circle, Vec3f(0, 0, depth));
        if (depth > maxD || depth < minD)
            continue;
        if (circle[3] > maxC)
            continue;
        //test
        cout << "depth:" << depth << endl;
        cout << circle << endl;
        if (this->realCoordinates.empty()) {
            if (circle[0] > maxX || circle[0] < minX)
                continue;
            if (circle[1] > maxY || circle[1] < minY)
                continue;
        } else {
            double dis = this->realDistance(coor, this->realCoordinates.back());
            cout << "distance:" << dis << endl;
            if (dis > maxDi || dis < minDi)
                continue;
        }


        if (!minI) {
            minC = circle;
            cSize = contour.size();
            cDepth = depth;
            realC = coor;
            minI = true;
        } else if (circle[3] < minC[3]) {
            minC = circle;
            cSize = contour.size();
            cDepth = depth;
            realC = coor;
        }
    }
    if (!minI) {
        minC[0] = -1;
        return minC;
    }
    this->ballCoordinates.push_back(minC);
    this->ballInfo.emplace_back(this->frameI, cSize, cDepth);
    this->realCoordinates.push_back(realC);
    //test
    Point center(round(minC[0]), round(minC[1]));
    int radius = round(minC[2]);
    cv::circle(resultImage, center, radius, Scalar(0, 255, 0), 1);
    cerr << minC << endl;
    cerr << this->ballInfo.back() << endl;
    cerr << realC << endl;
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

int Tracker::passCF() {
    unsigned long size = this->ballInfo.size();
    if (size > 2) {
        //ball coordinates in ring's plane (x,y,z)
        Vec3f point;
        point[2] = this->ring[3];

        vector<float> xs, ys;
        for (int i = 0; i < size; ++i) {
            //x
            xs.push_back(this->realCoordinates[i][0]);
            //z
            ys.push_back(this->realCoordinates[i][2]);
        }
        vector<float> func1 = this->curveFitting(xs, ys, 1);
        point[0] = (point[2] - func1[0]) / func1[1];

        float b = func1[1];
        double bc = sqrt(pow(1 / b, 2) + 1);
        xs.clear();
        ys.clear();
        for (int j = 0; j < size; ++j) {
            xs.push_back(static_cast<float &&>(this->realCoordinates[j][2] * bc));
            ys.push_back(this->realCoordinates[j][1]);
        }
        Vec3f func2 = this->x2curveFitting(xs, ys);
        point[1] = static_cast<float>(func2[0] + func2[1] * point[2] * bc + func2[2] * pow(point[2] * bc, 2));
        double dis = this->realDistance(this->ringCoordinate, point);

        float br = this->ballCoordinates.back()[2];
        float bdepth = this->ballInfo.back()[2];
        //512x424,compute ball real radius
        double realR = br / 256 * bdepth * tan((57.5 / 2) / 180 * M_PI);
        if (realR + dis < this->rRingR)
            return 1;
        else
            return 2;
    } else
        return -1;
}

//-1 no ball,0 ball run,1 pass,2 not pass
int Tracker::isPassed(cv::Mat &frame) {
    vector<vector<Point>> contours = this->findForegroundContours(frame, 1);
    //get ring data
    if (this->ring[0] < 0) {
//       Mat ringR = frame.clone();
//            imshow("ring", ringR);
//        cout<<"depth:"<<this->selectROIDepth<float>("ring", ringR)<<endl;

        this->ring = Vec4f(357, 159, 35, 4200);
        this->ringCoordinate = this->getCircleCoordinate(this->ring, Vec3f(0, 0, this->ring[3]));
        this->rRingR = static_cast<float>(this->ring[2] / 256 * this->ring[3] * tan((57.5 / 2) / 180 * M_PI));
    }
    Mat result = frame.clone();
    Vec4f circle = getBall(contours, result);
    // restart when no ball in 5 frames
    if (circle[0] < 0) {
        int res = -1;
        if (!this->ballInfo.empty() && this->frameI - this->ballInfo.back()[0] > 5) {
            res = this->passCF();
            this->ballInfo.clear();
            this->ballCoordinates.clear();
            this->realCoordinates.clear();
        }
        return res;
    }
    //test
    namedWindow("ball", 0);
    resizeWindow("ball", 640, 480);
    imshow("ball", result);
    usleep(100000);
    //judge result when ball passed ring's plane
    Vec3f info0 = this->ballInfo.back();
    if (info0[2] >= this->ring[3]) {
        //double dis = this->distance<float>(info0[0], this->ring[0], info0[1], this->ring[1]);
        //bool flag = (dis + info0[2]) < this->ring[2];
        int res = this->passCF();
        this->ballInfo.clear();
        this->ballCoordinates.clear();
        this->realCoordinates.clear();
        return res;
    }

    return 0;
}

void Tracker::test() {
    vector<float> x = {0, 1, 2}, y = {2, 1, -2};
    cout << this->x2curveFitting(x, y) << endl;

    VideoCapture videoCapture("/home/peng/下载/ball_pass_ring(5)/depth(pass3).avi");
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
        if (this->frameI < 0 || this->frameI > 1000)
            continue;
        cout << this->frameI << endl;
        if (this->frameI == 99)
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
    Protonect protonect;
    //open kinect
    if (protonect.connect() < 0) {
        return;
    }
    protonect.start();

    libfreenect2::FrameMap frames;
    Mat depthmat;
    future_status status;
    do {
        //get frame data
        protonect.listener->waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        ++this->frameI;
        cout << "frame:" << this->frameI << endl;
        //compute result
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
        //test
        cv::imshow("depth", depthmat / 4500.0f);
        int key = cv::waitKey(1);
        //shutdown when kinect error or main thread request
        Protonect::protonect_shutdown =
                Protonect::protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
        protonect.listener->release(frames);
        status = fut.wait_for(chrono::milliseconds(1));
    } while (!Protonect::protonect_shutdown && (status != future_status::ready));
    protonect.stop();
}

