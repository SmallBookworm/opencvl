#include <iostream>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "detector.h"


using namespace cv;

int main(int argc, char **argv) {

    VideoCapture capture(0);
    Mat srcImage;
    Detector detector(srcImage);
    if(!capture.isOpened()){
         std::cout<<"fail to open video!"<<std::endl;
        return -1;
    }
    while (capture.isOpened()){
        capture>>srcImage;
        if(srcImage.empty())
            break;
        imshow("show",srcImage);
        detector.setSrcImage(srcImage);
        Mat resultImage = srcImage.clone();
        std::vector<Vec3f> circles;
        circles = detector.getCircles(srcImage);
        if (circles.empty())
            printf("No circle \n");
        else{
            for (auto &circle : circles) {
                //std::cout << circle << std::endl;
                Point center(round(circle[0]), round(circle[1]));
                int radius = round(circle[2]);
                cv::circle(resultImage, center, radius, Scalar(0, 255, 0), 5);
            }

            detector.getScore( circles);
        }
        imshow("resultImage", resultImage);

        if(waitKey(1)==27){
            break;
        }
    }




    waitKey(0);
    return 0;
}