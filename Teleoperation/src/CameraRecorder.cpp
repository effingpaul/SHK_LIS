#include "CameraRecorder.h"

CameraRecorder::CameraRecorder(int cam1, int cam2, int fps) 
    : camIndex1(cam1), camIndex2(cam2), fps(fps), timeForEachFrame(1.0 / fps) {
    videoFolder = "videos";
    imageFolder = "images";
}

CameraRecorder::~CameraRecorder() {
    cap1.release();
    cap2.release();
    cv::destroyAllWindows();
}

void CameraRecorder::init() {
    cap1.open(camIndex1);
    cap2.open(camIndex2);

    if (!cap1.isOpened() || !cap2.isOpened()) {
        throw std::runtime_error("Error: Unable to open the cameras.");
    }

    std::filesystem::create_directory(videoFolder);
    std::filesystem::create_directory(imageFolder);
    for (const auto& entry : std::filesystem::directory_iterator(videoFolder)) {
        std::filesystem::remove_all(entry);
    }
    for (const auto& entry : std::filesystem::directory_iterator(imageFolder)) {
        std::filesystem::remove_all(entry);
    }

    int frameWidth = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_WIDTH));
    int frameHeight = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_HEIGHT));
    out1.open(videoFolder + "/output1.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(frameWidth, frameHeight));
    out2.open(videoFolder + "/output2.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(frameWidth, frameHeight));
}

void CameraRecorder::recordFrame() {
    cv::Mat frame1, frame2;
    cap1 >> frame1;
    cap2 >> frame2;

    if (frame1.empty() || frame2.empty()) {
        throw std::runtime_error("Error: Empty frame captured.");
    }

    cv::imshow("frame1", frame1);
    cv::imshow("frame2", frame2);

    frames1.push_back(frame1);
    frames2.push_back(frame2);

    out1.write(frame1);
    out2.write(frame2);
}

void CameraRecorder::finalize() {
    for (size_t i = 0; i < frames1.size(); ++i) {
        std::string filename = imageFolder + "/frame1_" + std::to_string(i) + ".jpg";
        cv::imwrite(filename, frames1[i]);
    }

    for (size_t i = 0; i < frames2.size(); ++i) {
        std::string filename = imageFolder + "/frame2_" + std::to_string(i) + ".jpg";
        cv::imwrite(filename, frames2[i]);
    }

    out1.release();
    out2.release();
}
