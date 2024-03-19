// projectAxes.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This program captures video from a webcam, detects a chessboard pattern within the video frames, and visualizes 3D axes anchored at the corner of the chessboard. 
//              It uses the camera calibration parameters read from a CSV file to accurately project the 3D axes onto the 2D image. 
//              The axes are drawn in different colors for easy differentiation: X-axis in red, Y-axis in green, and Z-axis in blue. This visualization helps in understanding the orientation and position of the chessboard in the camera's viewpoint, serving as a practical tool for testing camera calibration and pose estimation algorithms.

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include "functions.h"

/*
   Function: main
   Purpose: Entry point for the program. It continuously captures frames from the webcam, detects chessboard corners, estimates the pose of the chessboard using solvePnP, and projects 3D axes onto the chessboard. The axes visualization provides a clear representation of the camera's perspective relative to the chessboard.
   Returns: 1 if an error occurs (e.g., the camera cannot be opened), 0 upon successful execution and orderly shutdown of the program.
*/
int main()
{
    cv::VideoCapture capture(1);
    if (!capture.isOpened())
    {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Mat cameraMatrix, distCoeffs;
    ReadCameraParameters("camera_calibration.csv", cameraMatrix, distCoeffs);

    cv::Size patternSize(9, 6); 
    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Vec3f> objectPoints = Create3DChessboardCorners(patternSize, 1.0); 

    std::vector<cv::Vec3f> axisPoints = {
        cv::Vec3f(0, 0, 0),    
        cv::Vec3f(3, 0, 0), 
        cv::Vec3f(0, 3, 0), 
        cv::Vec3f(0, 0, -3) 
    };

    while (true)
    {
        cv::Mat frame;
        capture >> frame;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, patternSize, corner_set,
                                               cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (found)
        {
            cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            cv::Mat rvec, tvec;
            solvePnP(objectPoints, corner_set, cameraMatrix, distCoeffs, rvec, tvec);

            std::vector<cv::Point2f> imagePoints;
            projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

            // Draw the axes.
            cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3); 
            cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3); 
            cv::line(frame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3); 

            drawChessboardCorners(frame, patternSize, cv::Mat(corner_set), found);
        }

        cv::imshow("Frame", frame);

        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    capture.release();
    cv::destroyAllWindows();

    return 0;
}
