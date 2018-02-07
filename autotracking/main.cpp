#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

Mat get_foreground_object(
        Ptr<BackgroundSubtractorMOG2> pBackgroundKnn,
        Mat scene, double scale);

vector<RotatedRect> trackingObject(Mat frame, RotatedRect rect, double maxD, double maxR, double minR);

vector<RotatedRect> trackingObjectR2(Mat frame, Rect2d rect, double maxD, double maxR, double minR);

RotatedRect bestRect(vector<RotatedRect> rects, double x, double y);

vector<RotatedRect> findObject(Mat frame, double maxR, double minR, double minX, double maxY);

Point2f getLeftCenter(Point2f points[4]) {
    Point2f minP[2] = {Point2f(99999, 99999), Point2f(99999, 99999)};;
    for (int i = 0; i < 4; ++i) {
        if (points[i].x < minP[0].x) {
            minP[1] = minP[0];
            minP[0] = points[i];
        } else if (points[i].x < minP[1].x)
            minP[1] = points[i];
    }
    return (minP[0] + minP[1]) / 2;
}

int main() {
    // declares all required variables
    Mat frame;
    // set input video
    string path = "/home/peng/下载/数据2/";
    for (int nameN = 81; nameN < 157; ++nameN) {
        string name = to_string(nameN);
        VideoCapture cap(path + name + ".MOV");
        if (!cap.isOpened()) {
            std::cout << "fail to open video!" << std::endl;
            return -1;
        }
        double height = cap.get(CAP_PROP_FRAME_HEIGHT);
        double width = cap.get(CAP_PROP_FRAME_WIDTH);

        // get bounding box
        Ptr<BackgroundSubtractorMOG2> pBackgroundKnn = createBackgroundSubtractorMOG2();
        int num = 0;
        while (cap.isOpened()) {
            cap >> frame;
            num++;
            // stop the program if no more images
            if (frame.rows == 0 || frame.cols == 0)
                break;
            //ignore unimportant frames
            if (num > 40) {
                Mat nnframe = get_foreground_object(pBackgroundKnn, frame, 1);
                vector<RotatedRect> enableRects = findObject(nnframe, 7000, 3000, width * 0.8, height * 0.5);
                if (enableRects.empty())
                    continue;
                RotatedRect rect = enableRects[0];
                vector<RotatedRect> rects;
                VideoCapture cap1(path + name + ".MOV");
                int cap1Num = 0;
                ofstream ofile;
                ofile.open(path + "resa/" + name);
                cout << num << endl;
                ofile << num << endl;
                Point2f ps[4];
                rect.points(ps);
                cout << getLeftCenter(ps) << endl;
                ofile << getLeftCenter(ps) << endl;

                int empty = 0;
                int success = 0;
                //get points
                while (cap1.isOpened()) {
                    // get frame from the video
                    cap1 >> frame;
                    cap1Num++;
                    if (cap1Num <= num)
                        continue;
                    cout << cap1Num << endl;
                    ofile << cap1Num << endl;
                    // stop the program if no more images
                    if (frame.rows == 0 || frame.cols == 0)
                        break;
                    // update the tracking result
                    Mat nframe = get_foreground_object(pBackgroundKnn, frame, 1);
                    vector<RotatedRect> trackRect;
                    if (rects.empty()) {
                        trackRect = trackingObject(nframe, rect, rect.size.width * 2, rect.size.area() * 3,
                                                   rect.size.area() / 3);
                    } else {
                        trackRect = trackingObject(nframe, rects.back(), rect.size.width * 2, rect.size.area() * 3,
                                                   rect.size.area() / 3);
                    }
                    if (!trackRect.empty()) {
                        success++;
                        RotatedRect frect;
                        if (rects.empty())
                            frect = bestRect(trackRect, rect.center.x, rect.center.y);
                        else
                            frect = bestRect(trackRect, rects.back().center.x, rects.back().center.y);
                        rects.push_back(frect);
                        // draw the tracked object
                        Point2f points[4];
                        frect.points(points);
                        for (int j = 0; j < 4; ++j) {
                            line(frame, points[j], points[(j + 1) % 4], Scalar(0, 0, 255), 3, 8);
                        }
                        cout << getLeftCenter(points) << endl;
                        ofile << getLeftCenter(points) << endl;
                        // show image with the tracked object
                        imshow("tracker", frame);
                    } else {
                        empty++;
                        if (empty > 1 && success < 20) {
                            break;
                        }
                    }
                    //quit on ESC button
                    //if (waitKey(1) == 27)break;
                }
                cap1.release();
                ofile.close();
                if (success>50)
                    break;
            }
        }

        cap.release();
    }
    return 0;
}

Mat get_foreground_object(
        Ptr<BackgroundSubtractorMOG2> pBackgroundKnn,
        Mat scene, double scale) {
    Mat img;
    resize(scene, img, Size(0, 0), scale, scale);

    Mat fgmask, fgimg, bgimg;
    pBackgroundKnn->apply(img, fgmask);

    medianBlur(fgmask, fgmask, 5);

    morphologyEx(fgmask, fgmask, MORPH_CLOSE, Mat::ones(15, 3, CV_8UC1));
//    namedWindow("MORPH_CLOSE", CV_WINDOW_NORMAL);
//    resizeWindow("MORPH_CLOSE", 1080, 720);
//    imshow("MORPH_CLOSE", fgmask);

    return fgmask;
}

vector<RotatedRect> trackingObject(Mat frame, RotatedRect rect, double maxD, double maxR, double minR) {
    std::vector<std::vector<Point>> region_contours;
    findContours(frame, region_contours, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    std::vector<RotatedRect> objects;
    for (size_t i = 0; i < region_contours.size(); ++i) {
        RotatedRect newRect = minAreaRect(region_contours[i]);
        double distant = sqrt(pow(rect.center.x - newRect.center.x, 2) + pow(rect.center.y - newRect.center.y, 2));
        if (distant < maxD && newRect.size.area() < maxR &&
            newRect.size.area() > minR && newRect.center.x < rect.center.x) {
            objects.push_back(newRect);
        }

    }
    return objects;
}

vector<RotatedRect> findObject(Mat frame, double maxR, double minR, double minX, double maxY) {
    std::vector<std::vector<Point>> region_contours;
    findContours(frame, region_contours, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    std::vector<RotatedRect> objects;
    for (size_t i = 0; i < region_contours.size(); ++i) {
        RotatedRect newRect = minAreaRect(region_contours[i]);
        if (newRect.size.area() < maxR && newRect.size.area() > minR
            && newRect.center.x > minX && newRect.center.y < maxY) {
            objects.push_back(newRect);
        }
    }
    return objects;
}

vector<RotatedRect> trackingObjectR2(Mat frame, Rect2d rect, double maxD, double maxR, double minR) {
    std::vector<std::vector<Point>> region_contours;
    findContours(frame, region_contours, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    std::vector<RotatedRect> objects;
    for (size_t i = 0; i < region_contours.size(); ++i) {
        RotatedRect newRect = minAreaRect(region_contours[i]);
        if (sqrt(pow(rect.x + rect.width / 2 - newRect.center.x, 2) +
                 pow(rect.y + rect.height / 2 - newRect.center.y, 2)) < maxD &&
            newRect.size.area() < maxR && newRect.size.area() > minR &&
            (rect.x + rect.width / 2) > newRect.center.x)
            objects.push_back(newRect);
    }
    return objects;
}

RotatedRect bestRect(vector<RotatedRect> rects, double x, double y) {
    RotatedRect rect;
    double count = -1;
    for (int i = 0; i < rects.size(); ++i) {
        double distant = sqrt(pow(x - rects[i].center.x, 2) + pow(y - rects[i].center.y, 2));
        if (count == -1 || distant < count) {
            count = distant;
            rect = rects[i];
        }
    }
    return rect;
}