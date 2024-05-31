#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

const int FPS = 20;
const double TIME_FOR_EACH_FRAME = 1.0 / FPS;

int main() {
    VideoCapture cap(4);
    VideoCapture cap2(10);

    if (!cap.isOpened() || !cap2.isOpened()) {
        cerr << "Error: Unable to open the cameras." << endl;
        return -1;
    }

    vector<Mat> frames;
    vector<Mat> frames2;

    while (true) {
        auto start = chrono::high_resolution_clock::now();

        Mat frame, frame2;
        cap >> frame;
        cap2 >> frame2;

        if (frame.empty() || frame2.empty()) {
            cerr << "Error: Empty frame captured." << endl;
            break;
        }

        imshow("frame", frame);
        imshow("frame2", frame2);

        frames.push_back(frame);
        frames2.push_back(frame2);

        // Placeholder for additional computations
        // This is where you perform your additional tasks
        // simulate some computation time
        //this_thread::sleep_for(chrono::milliseconds(50));

        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = end - start;
        double elapsed_time = elapsed.count();

        double sleep_time = TIME_FOR_EACH_FRAME - elapsed_time;
        if (sleep_time > 0) {
            this_thread::sleep_for(chrono::duration<double>(sleep_time));
        }

        if (waitKey(1) == 'q') {
            break;
        }
    }

    cap.release();
    cap2.release();
    destroyAllWindows();

    filesystem::create_directory("videos");
    filesystem::create_directory("images");
    for (const auto& entry : filesystem::directory_iterator("videos")) {
        filesystem::remove_all(entry);
    }
    for (const auto& entry : filesystem::directory_iterator("images")) {
        filesystem::remove_all(entry);
    }

    VideoWriter out("videos/output.avi", VideoWriter::fourcc('X', 'V', 'I', 'D'), 20.0, Size(640, 480));
    for (const auto& frame : frames) {
        out.write(frame);
    }
    out.release();

    VideoWriter out2("videos/output2.avi", VideoWriter::fourcc('X', 'V', 'I', 'D'), 20.0, Size(640, 480));
    for (const auto& frame : frames2) {
        out2.write(frame);
    }
    out2.release();

    for (size_t i = 0; i < frames.size(); ++i) {
        string filename = "images/frame_" + to_string(i) + ".jpg";
        imwrite(filename, frames[i]);
    }

    for (size_t i = 0; i < frames2.size(); ++i) {
        string filename = "images/frame2_" + to_string(i) + ".jpg";
        imwrite(filename, frames2[i]);
    }

    return 0;
}
