//
// Created by peng on 18-3-1.
//


#include "ball_tracker.h"
#include "cv-helpers.hpp"

using namespace std;
using namespace cv;

Tracker::Tracker() {
    this->frameI = 0;
    reboundTest = false;
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

//1,x,x^2,x^3...
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

vector<vector<Point>> Tracker::findAllContours(Mat &input, bool isDepth) {
    Mat frame = input.clone();
    //expansive working
    Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
    dilate(frame, frame, element);

    if (!isDepth)
        cvtColor(frame, frame, CV_BGR2GRAY);
    //高斯平滑
    GaussianBlur(frame, frame, Size(9, 9), 0, 0);
//    imshow("GaussianBlur", frame);

    Canny(frame, frame, 30, 60, 3);
    //imshow("Canny", frame);

    vector<vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy,
                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());

    return contours;
}

std::vector<std::vector<Point>> Tracker::findForegroundContours(
        Mat &img, double scale) {
    resize(img, img, Size(0, 0), scale, scale);

    Mat fgmask, fgimg, bgimg;
    this->pBackgroundKnn->apply(img, fgmask);

    medianBlur(fgmask, fgmask, 5);

    morphologyEx(fgmask, fgmask, MORPH_CLOSE, Mat::ones(15, 3, CV_8UC1));
    //test
//    namedWindow("MORPH_CLOSE", CV_WINDOW_NORMAL);
//    resizeWindow("MORPH_CLOSE", 1080, 720);
    img = fgmask;

    std::vector<std::vector<Point>> region_contours;
    findContours(fgmask, region_contours, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    return region_contours;
}

int Tracker::pSum(cv::Mat gray) {
    int counter = 0;
    //迭代器访问像素点
    Mat_<uchar>::iterator it = gray.begin<uchar>();
    Mat_<uchar>::iterator itend = gray.end<uchar>();
    for (; it != itend; ++it) {
        if ((*it) > 0) counter += 1;//二值化后，像素点是0或者255
    }
    return counter;
}

std::vector<RotatedRect> Tracker::getRotatedRect(std::vector<std::vector<Point>> region_contours) {
    std::vector<RotatedRect> objects;
    for (size_t i = 0; i != region_contours.size(); ++i) {
        RotatedRect rect = minAreaRect(region_contours[i]);
        objects.push_back(rect);
    }
    return objects;
}

Vec4f Tracker::getEdgeCircle(cv::Mat &foreground, std::vector<Point> contour) {
    Vec4f circle;
    Point2f center;
    float radius = 0;
    Moments mu = moments(contour, false);
    center = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
    radius = static_cast<float>(sqrt(contourArea(contour) / M_PI));
    circle[0] = center.x;
    circle[1] = center.y;
    circle[2] = radius;

    float result = 0;
    int count = 0;
    float a = circle[2];
    if ((circle[0] + circle[2]) >= foreground.cols || (circle[1] + circle[2]) >= foreground.rows ||
        (circle[0] - circle[2]) < 0 || (circle[1] - circle[2]) < 0)
        return -1;
    for (int i = static_cast<int>(ceil(circle[1] - a)); i < circle[1] + a; ++i) {
        double squareX = pow(a, 2) - pow(i - circle[1], 2);
        int maxX = static_cast<int>(sqrt(squareX) + circle[0]);
        int minX = static_cast<int>(ceil(circle[0] - sqrt(squareX)));
        for (int j = minX; j < maxX; ++j) {
            uchar value = foreground.at<uchar>(i, j);
            if (value > 0)
                result++;
            count++;
        }
    }
    circle[3] = result / count;
    return circle;
}

cv::Vec3f Tracker::getCircleCoordinate(cv::Vec4f circle, cv::Vec3f info, int wWidth, int wHeight) {
    Vec3f coordinate;
    coordinate[2] = info[2];
    coordinate[0] = static_cast<float>(info[2] * tan(HANGLE / 2) * (wWidth / 2 - circle[0]) /
                                       (wWidth / 2));
    coordinate[1] = static_cast<float>(info[2] * tan(VANGLE / 2) * (wHeight / 2 - circle[1]) /
                                       (wHeight / 2));
    //change coordinate system
    float c1 = coordinate[1], c2 = coordinate[2];
    coordinate[1] = static_cast<float>(cos(SENSEANGLE) * c1 + sin(SENSEANGLE) * c2);
    coordinate[2] = static_cast<float>(cos(SENSEANGLE) * c2 - sin(SENSEANGLE) * c1);
    return coordinate;
}

float Tracker::getCircleDepth(cv::Vec4f circle, rs2::depth_frame depthFrame) {
    float result = 0;
    int count = 0;
    float a = circle[2] / 2;
    if ((circle[0] + circle[2]) >= depthFrame.get_width() || (circle[1] + circle[2]) >= depthFrame.get_height() ||
        (circle[0] - circle[2]) < 0 || (circle[1] - circle[2]) < 0)
        return -1;
    for (int i = static_cast<int>(ceil(circle[1] - a)); i < circle[1] + a; ++i) {
        double squareX = pow(a, 2) - pow(i - circle[1], 2);
        int maxX = static_cast<int>(sqrt(squareX) + circle[0]);
        int minX = static_cast<int>(ceil(circle[0] - sqrt(squareX)));
        for (int j = minX; j < maxX; ++j) {
            float depth = depthFrame.get_distance(j, i);
            if (depth <= 0)
                continue;
            //test
            //cout << "point" << i << "," << j << ":" << depth << endl;
            result += depth;
            count++;
        }
    }
    return result / count + 0.03;
}


Rect Tracker::selectROIDepth(std::string windowName, cv::Mat &depthMat) {
    Rect rect = selectROI(windowName, depthMat);
    cout << "tl:" << rect.tl() << endl;
    return rect;
}

cv::Vec4f
Tracker::getBall(cv::Mat &foreground, std::vector<std::vector<cv::Point>> contours, Mat &resultImage,
                 rs2::depth_frame depthFrame) {
    bool minI = false;
    Vec4f minC;
    Vec3f realC;
    float cDepth;
    float cSize;
    float minSizes, maxsizes, minX, minY, maxX, maxY, minDi, maxDi, minZ, maxZ, minP, minR, maxR;
    minSizes = 20;
    maxsizes = 70;

    minR = 0.07;
    maxR = 0.2;

    minP = 0.8;
    if (this->realCoordinates.empty()) {
        //initial region
        maxY = resultImage.rows;
        minY = 0;
        maxX = resultImage.cols * 2 / 3;
        minX = resultImage.cols / 3;
        minZ = 1.000;
        maxZ = ringWatcher.coordinate[2];
    } else {
        Vec3f info = this->ballInfo.back();
        Vec3f realCI = this->realCoordinates.back();
        //speed 50
//        maxX = before[0] + 100 * (this->frameI - info[0]);
//        minX = before[0];
//        maxY = before[1] + 50 * (this->frameI - info[0]);
//        minY = before[1] - 50 * (this->frameI - info[0]);

        maxDi = 2 * (this->frameI - info[0]);
        minDi = 0;
        //make sure that ball goes far away.
        maxZ = realCI[2] + 1.00 * (this->frameI - info[0]) + 1;
        minZ = realCI[2] + 1.00 * (this->frameI - info[0]) - 1;

        minSizes = static_cast<float>(info[1] / (2 * (this->frameI - info[0])));
    }
    for (auto &contour : contours) {
        //1 point's number
        //cout<<contour.size()<<endl;
        if (contour.size() < minSizes || contour.size() > maxsizes)
            continue;

        Vec4f circle = this->getEdgeCircle(foreground, contour);
        //2 grade of circle
        if (circle[3] < minP)
            continue;
        cout << circle << endl;

        float depth = this->getCircleDepth(circle, depthFrame);
        if (isnan(depth) || depth <= 0)
            continue;
        cout << "depth:" << depth << endl;

        //3 ball radius is not changed
        //Of course,ball's radius don't change when coordinate system is changed.
        // Horizontal FOV (HD 16:9): 64; Vertical FOV (HD 16:9): 41
        double realR = circle[2] / (resultImage.cols / 2) * depth * tan(HANGLE / 2);
        //test
        cout << realR << endl;
        if (realR < minR || realR > maxR)
            continue;

        Vec3f coor = this->getCircleCoordinate(circle, Vec3f(0, 0, depth), depthFrame.get_width(),
                                               depthFrame.get_height());

        //test
        cout << coor << endl;
        //judge zone when empty.if not,distance.
        if (this->ballInfo.empty()) {
            //4 initial region
            if (coor[2] > maxZ || coor[2] < minZ)
                continue;
            if (circle[0] > maxX || circle[0] < minX)
                continue;
            if (circle[1] > maxY || circle[1] < minY)
                continue;
        } else {
            if (coor[2] > maxZ || coor[2] < minZ)
                continue;
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
        } else if (circle[3] > minC[3]) {
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


int Tracker::passCF(cv::Mat &frame) {
    unsigned long size = this->ballInfo.size();
    if (size >= 2) {
        //ball coordinates in ring's plane (x,y,z)
        Vec3f point;
        //point[2] = ringWatcher.coordinate[2];
        //x
        vector<float> xs, ys;
        for (int i = 0; i < size; ++i) {
            //x
            xs.push_back(this->realCoordinates[i][0]);
            //z
            ys.push_back(this->realCoordinates[i][2]);
        }
        vector<float> func1 = this->curveFitting(xs, ys, 1);
        //point[0] = (point[2] - func1[0]) / func1[1];
        point[0] = static_cast<float>((ringWatcher.func[0] - func1[0]) / (func1[1] - ringWatcher.func[1]));
        point[2] = func1[0] + point[0] * func1[1];
        xs.clear();
        ys.clear();
        //when size is 2,fit line
        if (size == 2) {
            for (int i = 0; i < size; ++i) {
                //y
                xs.push_back(this->realCoordinates[i][1]);
                //z
                ys.push_back(this->realCoordinates[i][2]);
            }
            vector<float> func2 = this->curveFitting(xs, ys, 1);
            point[1] = (point[2] - func2[0]) / func2[1];
        } else {
            //y
            float b = func1[1];
            //1/sin(a)
            double bc = sqrt(pow(1 / b, 2) + 1);
            for (int j = 0; j < size; ++j) {
                xs.push_back(static_cast<float &&>(this->realCoordinates[j][2] * bc));
                ys.push_back(this->realCoordinates[j][1]);
            }
            Vec3f func2 = this->x2curveFitting(xs, ys);
            point[1] = static_cast<float>(func2[0] + func2[1] * point[2] * bc + func2[2] * pow(point[2] * bc, 2));
        }

        double dis = this->realDistance(ringWatcher.coordinate, point);
        //cout << "bp:" << point <<"  dis:"<<dis<< endl;
        //d-value( right hand coordinate system)
        Vec3f dv = point - ringWatcher.coordinate;
        this->dValue.x = dv[0];
        this->dValue.y = dv[1];

        float br = this->ballCoordinates.back()[2];
        float bdepth = this->ballInfo.back()[2];
        //Of course,ball's radius don't change when coordinate system is changed.
        // Horizontal FOV (HD 16:9): 64; Vertical FOV (HD 16:9): 41
        double realR = br / (frame.cols / 2) * bdepth * tan(HANGLE / 2);
        if (realR + dis < ringWatcher.r)
            return 1;
        else if (dis < ringWatcher.r)
            return 3;
        else
            return 2;
    } else {
        //restart
        this->clearInfo();
        return -1;
    }
}

int Tracker::isPassed(cv::Mat &frame, rs2::depth_frame depthFrame) {
    Mat foreground = frame.clone();
    vector<vector<Point>> contours = this->findForegroundContours(foreground, 1);
    //test
    //imshow("MORPH_CLOSE", forground);
    if (frameI < 30)
        imwrite("/home/peng/文档/test/f" + to_string(frameI) + ".jpg", foreground);
    //debouncing
    double sum = foreground.cols * foreground.rows;
    cout << "s:" << (pSum(foreground) / sum) << endl;
    if ((pSum(foreground) / sum) > 0.04)
        return -1;
//test ring-wather
//    Mat clo = frame.clone();
//    ringWatcher.getRing(clo);
//    imshow("fuck?", clo);

    Mat result = frame.clone();
    Vec4f circle = getBall(foreground, contours, result, depthFrame);
    // get result or restart when no ball in 5 frames
    if (circle[0] < 0) {
        int res = -1;
        if (!this->ballInfo.empty() && this->frameI - this->ballInfo.back()[0] > 4) {
            res = this->passCF(frame);
        }
        return res;
    }
    //test
    //namedWindow("ball", WINDOW_AUTOSIZE);
    //resizeWindow("ball", 848, 480);
    //imshow("ball", result);
    if (frameI < 30)
        imwrite("/home/peng/文档/test/ball" + to_string(frameI) + ".jpg", result);
    //usleep(100000);
    //judge result when ball passed ring's plane
    Vec3f info0 = this->realCoordinates.back();
    if (info0[2] >= ringWatcher.coordinate[2]) {
        int res = this->passCF(frame);
        return res;
    }

    return 0;
}

cv::Vec4f Tracker::getReBall(cv::Mat &foreground, std::vector<std::vector<cv::Point>> contours, cv::Mat &resultImage,
                             rs2::depth_frame depthFrame) {
    bool minI = false;
    Vec4f minC;
    Vec3f realC;
    float cDepth;
    float cSize;
    float minSizes, maxsizes, minX, minY, maxX, maxY, minDi, maxDi, minP, minR, maxR;
    minSizes = 20;
    maxsizes = 70;

    minR = 0.07;
    maxR = 0.2;

    minP = 0.8;
    if (this->reBall.empty()) {
        //initial region
        maxY = resultImage.rows * 0;
        minY = resultImage.rows * 0.5;
        maxX = resultImage.cols * 0.75;
        minX = resultImage.cols * 0.25;
        //prevent shaking ring
        maxDi = 2;
        minDi = 0.5;
    } else {
        Vec3f info = this->reBallInfo.back();
        Vec3f realCI = this->reRealCoordinates.back();

        maxDi = 2 * (this->frameI - info[0]);
        minDi = 0;

        minSizes = info[1] / (2 * (this->frameI - info[0]));
    }
    for (auto &contour : contours) {
        //1 point's number
        //cout<<contour.size()<<endl;
        if (contour.size() < minSizes || contour.size() > maxsizes)
            continue;

        Vec4f circle = this->getEdgeCircle(foreground, contour);
        //2 grade of circle
        if (circle[3] < minP)
            continue;
        cout << circle << endl;

        float depth = this->getCircleDepth(circle, depthFrame);
        if (isnan(depth) || depth < 0)
            continue;
        cout << "depth:" << depth << endl;

        //3 ball radius is not changed
        //Of course,ball's radius don't change when coordinate system is changed.
        // Horizontal FOV (HD 16:9): 64; Vertical FOV (HD 16:9): 41
        double realR = circle[2] / (resultImage.cols / 2) * depth * tan(HANGLE / 2);
        //test
        cout << realR << endl;
        if (realR < minR || realR > maxR)
            continue;

        Vec3f coor = this->getCircleCoordinate(circle, Vec3f(0, 0, depth), depthFrame.get_width(),
                                               depthFrame.get_height());

        //test
        cout << coor << endl;
        //judge zone when empty.if not,distance.
        if (this->reBallInfo.empty()) {
            if (circle[0] > maxX || circle[0] < minX)
                continue;
            if (circle[1] > maxY || circle[1] < minY)
                continue;
            double dis = this->realDistance(coor, this->realCoordinates.back());
            cout << "distance:" << dis << endl;
            if (dis > maxDi || dis < minDi)
                continue;
        } else {
            double dis = this->realDistance(coor, this->reRealCoordinates.back());
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
        } else if (circle[3] > minC[3]) {
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
    this->reBall.push_back(minC);
    this->reBallInfo.emplace_back(this->frameI, cSize, cDepth);
    this->reRealCoordinates.push_back(realC);
    //test
    Point center(round(minC[0]), round(minC[1]));
    int radius = round(minC[2]);
    cv::circle(resultImage, center, radius, Scalar(0, 255, 0), 1);
    cerr << minC << endl;
    cerr << this->reBallInfo.back() << endl;
    cerr << realC << endl;
    return minC;
}

int Tracker::surePassed(cv::Mat &frame, rs2::depth_frame depthFrame) {
    Mat foreground = frame.clone();
    vector<vector<Point>> contours = this->findForegroundContours(foreground, 1);
    Mat result = frame.clone();
    Vec4f circle = getReBall(foreground, contours, result, depthFrame);
    //test
    //namedWindow("ball", WINDOW_AUTOSIZE);
    //resizeWindow("ball", 848, 480);
    //imshow("ball", result);
    //usleep(100000);

    //judge result when ball passed 5 frames
    if (this->frameI - this->ballInfo.back()[0] > 4) {
        if (this->reBall.size() < 2) {
            this->clearInfo();
            return 1;
        } else {
            float dDep = 0;
            dDep = this->realCoordinates.back()[2] - this->reRealCoordinates.front()[2];
            if (dDep < 0) {
                this->clearInfo();
                return 1;
            }
            //fail when  ball closes in 5 frames
            this->clearInfo();
            return 2;
        }
    }

    if (circle[0] < 0) {
        return -1;
    } else
        return 0;
}

void Tracker::clearInfo() {
    reboundTest = false;
    this->ballInfo.clear();
    this->ballCoordinates.clear();
    this->realCoordinates.clear();
    this->reBall.clear();
    this->reRealCoordinates.clear();
    this->reBallInfo.clear();
}

int Tracker::test() {
}


// move-constructible function object (i.e., an object whose class defines operator(), including closures and function objects).
int Tracker::operator()(DeviationPosition &position) try {
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // align processing-blocks
    rs2::align align_to(RS2_STREAM_COLOR);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    //Add desired streams to configuration
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 60);
    //cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 60);
    // Start streaming with configuration
    pipe.start(cfg);

    bool status = position.getStop();
    while (!status) {
        status = position.getStop();
        if (position.getStby()) {
            if (this->frameI != 0) {
                this->frameI = 0;
                clearInfo();
            }
            //standby
            continue;
        }
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        // Make sure the frameset is spatialy aligned
        // (each pixel in depth image corresponds to the same pixel in the color image)
        rs2::frameset aligned_set = align_to.process(data);
        rs2::depth_frame depthFrame = aligned_set.get_depth_frame();
        rs2::frame depth = color_map(depthFrame);
        //rs2::frame ir = data.get_infrared_frame();
        ++this->frameI;
        cout << "frame:" << this->frameI << endl;
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image = frame_to_mat(aligned_set.get_color_frame());

        //get ring data
        if (ringWatcher.r < 0 && this->frameI > 0) {
//        Mat ringR = image.clone();
//        imshow("ring", ringR);
//        Rect rect = this->selectROIDepth("ring", ringR);
//        cout << "rdepth:" << depthFrame.get_distance(rect.tl().x, rect.tl().y) << endl;

//            ringWatcher.ring = Vec4f(328, 335, 60, 4.069);
//            ringWatcher.coordinate = this->getCircleCoordinate(ringWatcher.ring, Vec3f(0, 0, ringWatcher.ring[3]),
//                                                               depthFrame.get_width(), depthFrame.get_height());
            //calculate radius ,it is wrong when camera doesn't look at the front horizontally.In fact,it is known.
//            ringWatcher.r = static_cast<float>(ringWatcher.ring[2] / (depthFrame.get_width() / 2) *
//                                               ringWatcher.ring[3] *
//                                               tan(HANGLE / 2));
//
//            cout << "r:" << ringWatcher.r << endl;
//            cout << "coor" << ringWatcher.coordinate << endl;
            ringWatcher.r = 0.4;
            Vec4f coor = position.getRing();
            ringWatcher.coordinate = Vec3f(coor[0], coor[1], coor[2]);
            if (coor[3] < (M_PI / 2) && coor[3] > 0) {
                ringWatcher.func[1] = -tan(M_PI / 2 - coor[3]);
                ringWatcher.func[0] = coor[2] - ringWatcher.func[1] * coor[0];
            } else {
                ringWatcher.func[0] = coor[2];
                ringWatcher.func[1] = 0;
            }
            cout << "rwc:" << ringWatcher.coordinate << endl;
            cout << "rfc:" << ringWatcher.func[0] << "  " << ringWatcher.func[1] << endl;
        }

        //compute result
        if (reboundTest) {
            int sure = this->surePassed(image, depthFrame);
            //test
            switch (sure) {
                case -1:
                    cerr << "s:no ball!" << endl;
                    break;
                case 0:
                    cout << "\033[33m" << "s:run" << "\033[0m" << endl;
                    break;
                case 1:
                    cout << "\033[32m" << "sure!" << "\033[0m" << endl;
                    position.setPoint(this->dValue, 1);
                    break;
                case 2:
                    cout << "\033[32m" << "s:fail!" << "\033[0m" << endl;
                    position.setPoint(this->dValue, 0);
                    break;
            }
        } else {
            int pas = this->isPassed(image, depthFrame);
            //test
            switch (pas) {
                case -1:
                    cerr << "no ball!" << endl;
                    break;
                case 0:
                    cout << "\033[33m" << "run" << "\033[0m" << endl;
                    break;
                case 1:
                    cout << "\033[32m" << "success!" << "\033[0m" << endl;
                    position.setPoint(this->dValue, 1);
                    cout << dValue << endl;
                    this->clearInfo();
                    break;
                case 2:
                    cout << "\033[32m" << "fail!" << "\033[0m" << endl;
                    position.setPoint(this->dValue, 0);
                    cout << dValue << endl;
                    this->clearInfo();
                    break;
                case 3:
                    cout << "\033[32m" << "no sure!" << "\033[0m" << endl;
                    reboundTest = true;
                    cout << dValue << endl;
                    break;
            }
        }
        // Update the window with new data
        //test
        //imshow("display", image);
        if (frameI < 30) {
            imwrite("/home/peng/文档/test/" + to_string(frameI) + ".jpg", image);
            imwrite("/home/peng/文档/test/dep" + to_string(frameI) + ".jpg", frame_to_mat(depth));
        }
//        if (waitKey(1) == 27)
//            break;

    };

    return EXIT_SUCCESS;
}
catch (const rs2::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
              << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

