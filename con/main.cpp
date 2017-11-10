#include <iostream>
#include <vector>
#include <opencv/cv.hpp>

using namespace cv;

Vec4f getEdgeCircle(std::vector<Point> contour);

std::vector<RotatedRect>
get_foreground_object(Ptr<BackgroundSubtractorMOG2> pBackgroundKnn, Mat scene, double scale, bool isFlag);

int main() {

    VideoCapture capture;
    capture.open("/home/peng/下载/blur.mp4");

    Mat srcImage;
    if (!capture.isOpened()) {
        std::cout << "fail to open video!" << std::endl;
        return -1;
    }

    Ptr<BackgroundSubtractorMOG2> pBackgroundKnn = createBackgroundSubtractorMOG2();
    long nCount = 0;
    while (capture.isOpened()) {
        capture >> srcImage;
        nCount++;
        if (srcImage.empty())
            break;
        //imshow("show",srcImage);
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
            if (regions[i].size.area() < 200 || regions[i].size.area() > 700)
                continue;
            if (nCount < 150 && (regions[i].center.x > 200 || regions[i].center.y < 170))
                continue;
            if (nCount > 150 && (regions[i].center.y > 170 || regions[i].center.y < 120))
                continue;
            Point2f points[4];
            regions[i].points(points);
            Point2f maxP[2] = {Point2f(0, 0), Point2f(0, 0)};
            for (int j = 0; j < 4; ++j) {
                line(resultImage, points[j], points[(j + 1) % 4], Scalar(0, 255, 0), 3, 8);
                if (points[j].x > maxP[0].x){
                        maxP[1]=maxP[0];
                    maxP[0] = points[j];
                }
                else if (points[j].x > maxP[1].x)
                    maxP[1] = points[j];
            }
            std::cout << regions[i].center << std::endl;
        }
        std::cout << nCount << std::endl;
        imshow("resultImage", resultImage);
//        std::vector<std::vector<Point>> contours;
//        std::vector<Vec4i> hierarchy;
//        findContours(redTempMat, contours, hierarchy,
//                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
//        int minI = -1;
//        Vec4f minC;
//        for (int i = 0; i < contours.size(); ++i) {
//            if (contours[i].size() < 3)
//                break;
//            Scalar color = Scalar(0, 0, 255);
//            Vec4f circle = getEdgeCircle(contours[i]);
//            if (circle[2] < 10)
//                break;
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
    capture.release();
    return 0;
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