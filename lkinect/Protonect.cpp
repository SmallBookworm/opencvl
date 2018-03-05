//! [headers]
#include "protonect.h"


//! [headers]

using namespace std;
using namespace cv;


Mat mergeC3(Mat depthmat) {
    Mat df(depthmat.rows, depthmat.cols, CV_32FC3, Scalar(0, 0, 0));
    vector<Mat> cg;
    vector<Mat> channels;
    split(depthmat, channels);
    cg.push_back(channels.at(0));
    cg.push_back(Mat(depthmat.rows, depthmat.cols, CV_32FC1, Scalar(0.0)));
    cg.push_back(Mat(depthmat.rows, depthmat.cols, CV_32FC1, Scalar(0.0)));
    merge(cg, df);
    return df;
}

bool Protonect::protonect_shutdown = false;

int main() {
    Protonect protonect;
    if (protonect.connect() < 0) {
        return -1;
    }

    protonect.start();
    libfreenect2::FrameMap frames;

    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2,
                                                                                     4); // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger
    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
    //open writer
    VideoWriter videoWriter, videoWriter0, videoWriter1, videoWriter2, videoWriter3;
    string videoID = "6";
    //! [loop start]
    while (!Protonect::protonect_shutdown) {
        protonect.listener->waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        //! [loop start]

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        if (!videoWriter0.isOpened()) {
            videoWriter0.open(
                    "/home/peng/下载/test/" + videoID + ".avi",
                    CV_FOURCC('M', 'J', 'P', 'G'),
                    30,
                    Size(depth->width, depth->height),
                    true
            );
        }
        Mat df = mergeC3(depthmat);
        videoWriter0 << df;

        if (!videoWriter1.isOpened()) {
            videoWriter1.open(
                    "/home/peng/下载/test/ir" + videoID + ".avi",
                    CV_FOURCC('M', 'J', 'P', 'G'),
                    30,
                    Size(ir->width, ir->height),
                    true
            );
        }
        Mat irdf = mergeC3(irmat);
        videoWriter1 << irdf;
        if (!videoWriter3.isOpened()) {
            videoWriter3.open(
                    "/home/peng/下载/test/rgb" + videoID + ".avi",
                    CV_FOURCC('M', 'J', 'P', 'G'),
                    30,
                    Size(rgbmat.cols, rgbmat.rows),
                    true
            );
        }
        videoWriter3 << rgbmat;
        //show normal in opencv
        cv::imshow("depth", df / 4500.0f);
        cv::imshow("ir", irmat / 4500.0f);
        cv::imshow("rgb", rgbmat);

        //! [registration]
        protonect.registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        //! [registration]

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

        if (!videoWriter.isOpened()) {
            videoWriter.open(
                    "/home/peng/下载/test/rgbd2" + videoID + ".avi",
                    CV_FOURCC('M', 'J', 'P', 'G'),
                    30,
                    Size(rgbd2.cols, rgbd2.rows),
                    true
            );
        }
        Mat df1 = mergeC3(rgbd2);
        videoWriter << df1;

        if (!videoWriter2.isOpened()) {
            videoWriter2.open(
                    "/home/peng/下载/test/rgbd" + videoID + ".avi",
                    CV_FOURCC('M', 'J', 'P', 'G'),
                    30,
                    Size(rgbd.cols, rgbd.rows),
                    true
            );
        }
        videoWriter2 << rgbd;
        //cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
        cv::imshow("registered", rgbd);
        cv::imshow("depth2RGB", rgbd2 / 4500.0f);

        int key = cv::waitKey(1);
        protonect.protonect_shutdown =
                protonect.protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        //! [loop end]
        protonect.listener->release(frames);
    }
    //! [loop end]

    protonect.stop();

    std::cout << "Fuxk World!" << std::endl;
    return 0;
}