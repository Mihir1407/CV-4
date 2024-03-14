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

    std::ofstream cornerFile("corner_list.csv"), pointFile("point_list.csv");
    cv::Size patternSize(9, 6);
    float squareSize = 1.0;  // Units are arbitrary, consistency is key.
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
