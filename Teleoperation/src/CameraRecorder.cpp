#include "include/CameraRecorder.h"

CameraRecorder::CameraRecorder(int cam1, int cam2 = -1, int fps = 10, std::string folderName = "test") 
    : camIndex1(cam1), camIndex2(cam2), fps(fps), timeForEachFrame(1.0 / fps) {
    videoFolder = "videos";
    imageFolder = "images";
    this->folderName = folderName;
}

CameraRecorder::~CameraRecorder() {
    cap1.release();
    if (camIndex2 != -1){
        cap2.release();
    }
    cv::destroyAllWindows();
}

void CameraRecorder::init() {
    cap1.open(camIndex1);
    if (camIndex2 != -1){
        cap2.open(camIndex2);
    }
    if (!cap1.isOpened() || (camIndex2 != -1 && !cap2.isOpened())) {
        throw std::runtime_error("Error: Unable to open the cameras.");
    }

    // create video and image subfolder in folderName
    //videoFolder = folderName + "/" + videoFolder;
    imageFolder = folderName + "/" + imageFolder;
    //std::filesystem::create_directory(videoFolder);
    std::filesystem::create_directory(imageFolder);

    // int frameWidth = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_WIDTH));
    // int frameHeight = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_HEIGHT));
    // out1.open(videoFolder + "/output1.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(frameWidth, frameHeight));
    // if (camIndex2 != -1) {
    //     out2.open(videoFolder + "/output2.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(frameWidth, frameHeight));
    // }
}

void CameraRecorder::recordFrame(int index, bool saveImg = true) {
    cv::Mat frame1, frame2;
    cap1 >> frame1;
    if (camIndex2 != -1){
        cap2 >> frame2;
    }

    if (frame1.empty() || (camIndex2 != -1 && frame2.empty())) {
        throw std::runtime_error("Error: Empty frame captured.");
    }

    cv::imshow("frame1", frame1);
    frames1.push_back(frame1);

    if (saveImg) {
        cv::imwrite(imageFolder + "/cam1_" + std::to_string(index) + ".jpg", frame1);
    }
    // write frame to jpeg in images folder
    //outFrame.open(imageFolder + "/cam1_" + std::to_string(index) + ".jpg", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(frame1.cols, frame1.rows));
    //outFrame.write(frame1);
    //outFrame.release();
    //out1.write(frame1);

    if (camIndex2 != -1) {
        cv::imshow("frame2", frame2);
        frames2.push_back(frame2);
        out2.write(frame2);
    }
}

void CameraRecorder::finalize() {
    return;
    // for (size_t i = 0; i < frames1.size(); ++i) {
    //     std::string filename = imageFolder + "/frame1_" + std::to_string(i) + ".jpg";
    //     cv::imwrite(filename, frames1[i]);
    // }

    // if (camIndex2 == -1) {
    //     out1.release();
    //     return;
    // }else{
    //     for (size_t i = 0; i < frames2.size(); ++i) {
    //         std::string filename = imageFolder + "/frame2_" + std::to_string(i) + ".jpg";
    //         cv::imwrite(filename, frames2[i]);
    //     }
// 
    // out2.release();
    // }
}
