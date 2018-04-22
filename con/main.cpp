#include <iostream>
#include <vector>
#include <opencv/cv.hpp>
#include <fcntl.h>
#include <linux/videodev2.h>
#include<sys/ioctl.h>

using namespace cv;
using namespace std;

Vec4f getEdgeCircle(std::vector<Point> contour);

std::vector<RotatedRect>
get_foreground_object(Ptr<BackgroundSubtractorMOG2> pBackgroundKnn, Mat scene, double scale, bool isFlag);

int main() {

    VideoCapture capture(1);
    //capture.open("/home/peng/下载/realse/1.avi");
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    int fd = open("/dev/video1", O_RDWR);
    if (fd >= 0) {
        struct v4l2_control ctrl;
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        ctrl.value = V4L2_EXPOSURE_MANUAL;
        int ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

        struct v4l2_control ctrl1;
        ctrl1.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        ctrl1.value=1;
        ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl1);
        if (ret < 0) {
            printf("Get exposure failed (%d)\n", ret);
        } else
            printf("\nGet Exposure :[%d]\n", ctrl1.value);
    }

    Mat srcImage;
    if (!capture.isOpened()) {
        std::cout << "fail to open video!" << std::endl;
        return -1;
    }

    Ptr<BackgroundSubtractorMOG2> pBackgroundKnn = createBackgroundSubtractorMOG2();
    long nCount = 0;
    std::vector<std::vector<Point2f>> data;
    while (capture.isOpened()) {
        capture >> srcImage;
        nCount++;
        if (srcImage.empty())
            break;
        imshow("show", srcImage);
        Mat resultImage = srcImage.clone();

//        //中值滤波
//        medianBlur(srcImage, srcImage, 3);
//        Mat hsvImage;
//        cvtColor(srcImage, hsvImage, COLOR_BGR2HSV);
//        //imshow("hsvImage",hsvImage);
//
//        Mat lowTempMat;
//        Mat upperTempMat;
//
//        inRange(hsvImage, Scalar(25, 10, 30), Scalar(40, 30, 50), lowTempMat);
//        imshow("red", lowTempMat);
//        inRange(hsvImage, Scalar(60, 0, 30), Scalar(75, 20, 60), upperTempMat);
//        Mat redTempMat;
//        addWeighted(lowTempMat, 1.0, upperTempMat, 1.0, 0.0, redTempMat);
//
//        //高斯平滑
//        GaussianBlur(redTempMat, redTempMat, Size(9, 9), 2, 2);
//
//        Canny(srcImage, redTempMat, 0, 150, 3);
//        imshow("red", redTempMat);

        std::vector<RotatedRect> regions = get_foreground_object(pBackgroundKnn, srcImage, 1, true);
        for (int i = 0; i < regions.size(); ++i) {
            if (regions[i].size.area() < 1000 || regions[i].size.area() > 7000 || regions[i].center.x > 1700)
                continue;
            Point2f points[4];
            regions[i].points(points);
            Point2f maxP[2] = {Point2f(99999, 99999), Point2f(99999, 99999)};
            std::vector<Point2f> rsPoints;
            data.push_back(rsPoints);
            for (int j = 0; j < 4; ++j) {
                line(resultImage, points[j], points[(j + 1) % 4], Scalar(0, 255, regions[i].center.y), 3, 8);
                if (points[j].x < maxP[0].x) {
                    maxP[1] = maxP[0];
                    maxP[0] = points[j];
                } else if (points[j].x < maxP[1].x)
                    maxP[1] = points[j];
            }
            Point2f cen = (maxP[0] + maxP[1]) / 2;
            circle(resultImage, cen, 4, Scalar(255, 0, 0), CV_FILLED);
            std::cout << cen << std::endl;
            rsPoints.push_back(cen);
        }
        std::cout << nCount << std::endl;
//        if (nCount == 143)
//            imshow("12", resultImage);
//        namedWindow("resultImage",CV_WINDOW_NORMAL);
//        resizeWindow("resultImage",1080,720);
//        imshow("resultImage", resultImage);

//        std::vector<std::vector<Point>> contours;
//        std::vector<Vec4i> hierarchy;
//        findContours(redTempMat, contours, hierarchy,
//                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
//        int minI = -1;
//        Vec4f minC;
//        for (int i = 0; i < contours.size(); ++i) {
//            if (contours[i].size() < 3)
//                continue;
//            Scalar color = Scalar(0, 0, 255);
//            Vec4f circle = getEdgeCircle(contours[i]);
//            if (circle[2] < 10)
//                continue;
//            if (minI == -1) {
//                minC = circle;
//                minI = i;
//            } else if (circle[3] < minC[3]) {
//                std::cout << circle[2] << std::endl;
//                minC = circle;
//                minI = i;
//            }
//            drawContours(resultImage, contours, i, color, 2, 8, hierarchy, 0, Point());
//        }
//        if (minI != -1) {
//            Point center(round(minC[0]), round(minC[1]));
//            int radius = round(minC[2]);
//            cv::circle(resultImage, center, radius, Scalar(0, 255, 0), 5);
//        }
//
//        imshow("resultImage", resultImage);

        if (waitKey(1) == 27) {
            break;
        }
    }
    waitKey(0);
    capture.release();
    return 0;
}

std::vector<Point2f>
getContinuePoints(vector<std::vector<Point2f>> points, int continueFrames, int promise, int minFrames) {
    std::vector<Point2f> result;
}

vector<int[2]> getCloseNumber(vector<Point2f> p1, vector<Point2f> p2) {

}

Vec4f getEdgeCircle(std::vector<Point> contour) {
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

std::vector<RotatedRect> get_foreground_object(
        Ptr<BackgroundSubtractorMOG2> pBackgroundKnn,
        Mat scene, double scale, bool isFlag) {
    if (!isFlag) {
        std::vector<RotatedRect> one_rect;
        Rect whole;
        whole.x = whole.y = 0;
        whole.height = scene.rows;
        whole.width = scene.cols;

        one_rect.emplace_back(whole.br(), whole.tl(),
                              Point2f(whole.tl().x, whole.tl().y + whole.height));
        return one_rect;
    }
    Mat img;
    resize(scene, img, Size(0, 0), scale, scale);

    Mat fgmask, fgimg, bgimg;
    pBackgroundKnn->apply(img, fgmask);

    medianBlur(fgmask, fgmask, 5);

    morphologyEx(fgmask, fgmask, MORPH_CLOSE, Mat::ones(15, 3, CV_8UC1));
    namedWindow("MORPH_CLOSE", CV_WINDOW_NORMAL);
    resizeWindow("MORPH_CLOSE", 1080, 720);
    imshow("MORPH_CLOSE", fgmask);
    std::vector<std::vector<Point>> region_contours;
    findContours(fgmask, region_contours, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    std::vector<RotatedRect> objects;
    for (size_t i = 0; i != region_contours.size(); ++i) {
        RotatedRect rect = minAreaRect(region_contours[i]);
        objects.push_back(rect);



//        rect.y += rect.height * 1 / 4;
//        rect.height *= 3 / 4.0;
//
//        rect.x /= scale;
//        rect.width /= scale;
//        rect.y /= scale;
//        rect.height /= scale;
//
//        if (rect.area() < scene.total() / 400) {
//            objects.push_back(rect);
//        }

    }
    return objects;
}