#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include "functions.h"

int main() {
    cv::VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Mat cameraMatrix, distCoeffs;
    ReadCameraParameters("camera_calibration.csv", cameraMatrix, distCoeffs);

    cv::Size patternSize(9, 6);
    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Vec3f> objectPoints = Create3DChessboardCorners(patternSize, 1.0);

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

            cv::Mat rvec, tvec;
            solvePnP(objectPoints, corner_set, cameraMatrix, distCoeffs, rvec, tvec);

            std::cout << "Rotation Vector:\n" << rvec << std::endl;
            std::cout << "Translation Vector:\n" << tvec << std::endl;
        }

        cv::imshow("Frame", frame);
        if (cv::waitKey(30) == 'q') {
            break;
        }
    }

    capture.release();
    cv::destroyAllWindows();
    return 0;
}
