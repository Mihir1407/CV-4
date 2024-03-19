// selectCalibrationImages.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This program is designed to assist in the collection of data necessary for camera calibration. It captures live video feed from a webcam, detects chessboard patterns within the frames,
//              and allows the user to save these frames along with their corresponding chessboard corner coordinates and 3D point representations. The saved images and data are stored in PNG format and CSV files, respectively. 
//              This tool streamlines the process of gathering calibration data, facilitating accurate camera calibration.
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include "functions.h"

/*
   Function: main
   Purpose: Serves as the entry point for the program. It initializes video capture, continuously processes frames to detect chessboard corners, and saves the calibration data upon user request. The program supports camera calibration by providing a means to collect and save necessary image data and associated chessboard corner detections.
   Returns: 1 if an error occurs (e.g., camera cannot be opened), 0 upon successful execution and orderly program termination.
*/
int main() {
    cv::VideoCapture capture(1);
    if (!capture.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    std::ofstream cornerFile("corner_list.csv"), pointFile("point_list.csv");
    cv::Size patternSize(9, 6);
    float squareSize = 1.0;  
    std::vector<cv::Point2f> corner_set;
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>> point_list;
    std::vector<cv::Vec3f> point_set = Create3DChessboardCorners(patternSize, squareSize);

    int saveCount = 0;
    while (true) {
        cv::Mat frame;
        capture >> frame;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, patternSize, corner_set, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(frame, patternSize, cv::Mat(corner_set), found);
        }

        cv::imshow("Frame", frame);
        char key = (char)cv::waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 's' && found) {
            std::string filename = "calibrationImage_" + std::to_string(++saveCount) + ".png";
            cv::imwrite(filename, frame); 
            corner_list.push_back(corner_set);
            point_list.push_back(point_set);

            cornerFile << "Image " << saveCount << std::endl;
            WriteToCSV(cornerFile, corner_set);
            pointFile << "Image " << saveCount << std::endl;
            WriteToCSV(pointFile, point_set);
            std::cout << "Saved " << filename << " with corresponding corner points." << std::endl;
        }
    }
    cornerFile.close();
    pointFile.close();
    capture.release();
    cv::destroyAllWindows();
    return 0;
}
