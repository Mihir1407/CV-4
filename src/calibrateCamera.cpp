#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include "functions.h"

int main() {
    cv::VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Size patternSize(9, 6);
    float squareSize = 1.0;
    std::vector<cv::Point2f> corner_set;
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>> point_list;

    while (true) {
        cv::Mat frame;
        capture >> frame;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, patternSize, corner_set, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(frame, patternSize, cv::Mat(corner_set), found);
        }

        cv::imshow("Frame", frame);
        char key = (char)cv::waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 's' && found) {
            corner_list.push_back(corner_set);
            point_list.push_back(Create3DChessboardCorners(patternSize, squareSize));
            std::cout << "Calibration frame captured." << std::endl;
            if (corner_list.size() >= 5) {
                RunCalibrationAndSave(corner_list, point_list, frame.size());
            }
        }
    }

    capture.release();
    cv::destroyAllWindows();
    return 0;
}
