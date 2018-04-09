//
// Created by peng on 18-3-14.
//

#ifndef CROSS_LINEFINDER_H
#define CROSS_LINEFINDER_H

#include <opencv2/opencv.hpp>
#include <vector>


class LineFinder {
private:
    cv::Mat img; // original image
    std::vector <cv::Vec4i> lines;
    double deltaRho;
    double deltaTheta;
    int minVote;
    double minLength; // min length for a line
    double maxGap; // max allowed gap along the line

public:
    // Default accumulator resolution is 1 pixel by 1 degree
    // no gap, no mimimum length
    LineFinder() : deltaRho(1),
                   deltaTheta(M_PI / 180),
                   minVote(10),
                   minLength(30),
                   maxGap(20) {}

    // Set the resolution of the accumulator
    void setAccResolution(double dRho, double dTheta) {
        deltaRho = dRho;
        deltaTheta = dTheta;
    }

    // Set the minimum number of votes
    void setMinVote(int minv) {
        minVote = minv;
    }

    // Set line length and gap
    void setLineLengthAndGap(double length, double gap) {
        minLength = length;
        maxGap = gap;
    }

    // Apply probabilistic Hough Transform
    std::vector <cv::Vec4i> findLines(cv::Mat &binary, cv::Mat &dstImage);
};


#endif //CROSS_LINEFINDER_H
