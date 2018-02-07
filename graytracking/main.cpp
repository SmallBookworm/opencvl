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
    Rect2d roi;
    Mat frame;
    // set input video
    string a = "51";
    VideoCapture cap("/home/peng/下载/数据1/" + a + ".MOV");
    if (!cap.isOpened()) {
        std::cout << "fail to open video!" << std::endl;
        return -1;
    }
    ofstream ofile;
    ofile.open("/home/peng/下载/数据1/res/" + a);
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
        if (num > 0) {
            Mat nframe = get_foreground_object(pBackgroundKnn, frame, 1);
            namedWindow("tracker", CV_WINDOW_NORMAL);
            resizeWindow("tracker", 1080, 720);
            imshow("tracker", nframe);
            if (waitKey(20) == 27) {
                roi = selectROI("tracker", nframe);
                //quit if ROI was not selected
                if (roi.width == 0 || roi.height == 0)
                    return 0;
                else
                    break;
            }
        }
    }
    cout<<roi.area()<<endl;
    vector<RotatedRect> rects;
    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");
    cout << num << endl;
    ofile << num << endl;
    cout << roi.tl() << endl;
    Point2f po(0, static_cast<float>(roi.height / 2));
    Point2f pr = po + Point2f(roi.tl().x, roi.tl().y);
    cout << pr << endl;
    ofile << pr << endl;
    while (cap.isOpened()) {
        // get frame from the video
        cap >> frame;
        num++;
        cout << num << endl;
        ofile << num << endl;
        // stop the program if no more images
        if (frame.rows == 0 || frame.cols == 0)
            break;
        // update the tracking result
        Mat nframe = get_foreground_object(pBackgroundKnn, frame, 1);
        vector<RotatedRect> trackRect;
        if (rects.empty()) {
            trackRect = trackingObjectR2(nframe, roi, roi.width * 2, roi.area() * 3, roi.area() / 3);
        } else {
            trackRect = trackingObject(nframe, rects.back(), roi.width * 2, roi.area() * 3, roi.area() / 3);
        }
        if (!trackRect.empty()) {
            RotatedRect frect;
            if (rects.empty())
                frect = bestRect(trackRect, roi.x + roi.width / 2, roi.y + roi.height / 2);
            else
                frect = bestRect(trackRect, rects.back().center.x, rects.back().center.y);
            rects.push_back(frect);
            // draw the tracked object
            Point2f points[4];
            frect.points(points);
            for (int j = 0; j < 4; ++j) {
                line(frame, points[j], points[(j + 1) % 4], Scalar(0, 0, 255), 3, 8);
            }
            cout << frect.center << endl;
            cout << getLeftCenter(points) << endl;
            ofile << getLeftCenter(points) << endl;
            // show image with the tracked object
            imshow("tracker", frame);
        }
        //quit on ESC button
        if (waitKey(1) == 27)break;
    }
    waitKey(0);
    cap.release();
    ofile.close();
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