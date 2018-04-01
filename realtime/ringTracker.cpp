//
// Created by peng on 18-3-28.
//

#include "ringTracker.h"

using namespace cv;
using namespace std;

cv::Vec3f RingTracker::getPoleRange(Mat depthMat) {
    //binaryzation
    Mat grayFrame;
    inRange(depthMat, 5.5, 6.3, grayFrame);
    float max = 0.75;
    float min = 0.4;
    float colsS[grayFrame.cols]{0};
    vector<vector<Vec2i>> valueRange;

    for (int i = 0; i < grayFrame.cols; ++i) {
        vector<Vec2i> range;
        int start = -1;
        int end = -1;
        for (int j = 0; j < grayFrame.rows; ++j) {
            if (grayFrame.at<uchar>(j, i) > 0) {
                colsS[i] += 1.0;
                if (start < 0)
                    start = j;
                end = j;
            } else if (start > 0) {
                range.emplace_back(start, end);
                start = end = -1;
            }
        }
        colsS[i] /= grayFrame.rows;
        valueRange.push_back(range);
    }
    int minC = 1;
    int maxC = grayFrame.cols / 40;
    int count = 0;
    int k;
    for (k = 0; k < grayFrame.cols; ++k) {
        if (colsS[k] > min && colsS[k] < max)
            count++;
        else if (count > minC && count < maxC) {
            break;
        } else
            count = 0;
    }
    Vec3f res;
    if (count == 0) {
        res[0] = -1;
        return res;
    }
    int endY = 0;
    int endX = 0;
    double depth = 0;
    int noiseDis = grayFrame.rows / 10;
    for (int l = 1; l <= count; ++l) {
        endX += k - l;
        vector<Vec2i> col = valueRange[k - l];
        int rStart = -1;
        int rEnd = -1;
        for (auto i = col.rbegin(); i != col.rend(); i++) {
            if (((*i)[1] - (*i)[0]) < noiseDis)
                continue;
            if ((rStart - (*i)[0]) > noiseDis)
                break;
            rStart = (*i)[0];
            rEnd = (*i)[1];
        }
        if (rStart > 0) {
            cout << rStart << endl;
            endY += rStart;
            //get depth
            for (int i = rStart; i < rEnd; ++i) {

            }
        }

    }
    res[0] = endX / count;
    res[1] = endY / count;
    return res;
}

int RingTracker::getData(cv::Mat &result) {
    int x0 = result.cols / 4;
    int y0 = 0;
    Rect roi(x0, y0, result.cols / 2, result.rows);
    Vec3f res = this->getPoleRange(Mat(result, roi));
    if (res[0] < 0)
        return -1;
    circle(result, Point(x0 + res[0], y0 + res[1]), 10, Scalar(255));
}

int RingTracker::operator()(Coordinate &coordinate) {
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    //Add desired streams to configuration
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    //cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    bool contFlag = true;
    while (contFlag) {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::depth_frame depthFrame = data.get_depth_frame();
        Mat depthMat = depth_frame_to_meters(pipe, depthFrame);

        //compute result
        getData(depthMat);

        coordinate.set(Point2f(0, 0));
        // Update the window with new data
        imshow(window_name, depthMat);
        contFlag = waitKey(1) < 0;
    }

    return EXIT_SUCCESS;
}