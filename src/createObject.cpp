// createObject.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This program captures video from a webcam and uses camera calibration parameters to augment 3D objects onto a detected chessboard pattern in real-time.
//              It processes captured frames to detect a chessboard and uses the detected corners to estimate the pose of the chessboard. Using this pose, it projects
//              3D models (blocks and a cylinder) onto the chessboard, creating an augmented reality effect. 

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include "functions.h"

/*
   Function: main
   Purpose: It captures video frames, detects a chessboard within these frames, and augments 3D objects on the chessboard by projecting
            them onto the 2D video frames. The program utilizes camera calibration parameters to accurately compute the pose of the chessboard and render the 3D
            objects in accordance with the perspective and distortion of the camera.
   Returns: 0 on successful execution, 1 if an error occurs (e.g., camera not opening).
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
    float squareSize = 1.0;
    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Vec3f> objectPoints = Create3DChessboardCorners(patternSize, squareSize);

    cv::Vec3f baseCenter = objectPoints.at(4 * patternSize.width + 2) + cv::Vec3f(squareSize * 1.5, 1.5 * squareSize, squareSize * 2);
    std::vector<std::vector<cv::Vec3f>> blocks = CreateBlocks(baseCenter, 4 * squareSize, 6 * squareSize, 1 * squareSize, 3); // Three blocks

    while (true)
    {
        cv::Mat frame;
        capture >> frame;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, patternSize, corner_set, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            cv::Mat rvec, tvec;
            solvePnP(objectPoints, corner_set, cameraMatrix, distCoeffs, rvec, tvec);

            for (const auto &block : blocks)
            {
                std::vector<cv::Point2f> imagePoints;
                projectPoints(block, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

                for (size_t i = 0; i < 4; ++i)
                {
                    cv::line(frame, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(235, 52, 52), 2);
                    cv::line(frame, imagePoints[i + 4], imagePoints[((i + 1) % 4) + 4], cv::Scalar(235, 52, 52), 2);
                    cv::line(frame, imagePoints[i], imagePoints[i + 4], cv::Scalar(235, 52, 52), 2);
                }
            }

            cv::Vec3f topLeft = blocks.back()[4];     
            cv::Vec3f topRight = blocks.back()[5];    
            cv::Vec3f bottomRight = blocks.back()[6]; 
            cv::Vec3f bottomLeft = blocks.back()[7];  

            cv::Vec3f topBlockCenter(
                (topLeft[0] + topRight[0] + bottomRight[0] + bottomLeft[0]) / 4.0f, 
                (topLeft[1] + topRight[1] + bottomRight[1] + bottomLeft[1]) / 4.0f, 
                topLeft[2]                                                         
            );

            float cylinderRadius = 0.75 * squareSize; 
            float cylinderHeight = 3.0 * squareSize;  
            int slices = 36;                          
            std::vector<cv::Vec3f> cylinderPoints = CreateCylinder(topBlockCenter, cylinderRadius, cylinderHeight, slices);

            std::vector<cv::Point2f> projectedCylinderPoints;
            projectPoints(cylinderPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedCylinderPoints);

            for (int i = 0; i < slices; i++)
            {
                cv::line(frame, projectedCylinderPoints[i * 2], projectedCylinderPoints[((i + 1) % slices) * 2], cv::Scalar(235, 52, 52), 2);
                cv::line(frame, projectedCylinderPoints[i * 2 + 1], projectedCylinderPoints[((i + 1) % slices) * 2 + 1], cv::Scalar(235, 52, 52), 2);
                cv::line(frame, projectedCylinderPoints[i * 2], projectedCylinderPoints[i * 2 + 1], cv::Scalar(235, 52, 52), 2);
            }
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