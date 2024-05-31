#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <chrono>
#include <thread>

class CameraRecorder {
public:
    CameraRecorder(int cam1, int cam2, int fps);
    ~CameraRecorder();

    void init();
    void recordFrame();
    void finalize();

private:
    int camIndex1;
    int camIndex2;
    int fps;
    double timeForEachFrame;
    cv::VideoCapture cap1;
    cv::VideoCapture cap2;
    std::vector<cv::Mat> frames1;
    std::vector<cv::Mat> frames2;
    cv::VideoWriter out1;
    cv::VideoWriter out2;
    std::string videoFolder;
    std::string imageFolder;
};
