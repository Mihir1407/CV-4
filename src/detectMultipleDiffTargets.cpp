// detectMultipleDiffTargets.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This program demonstrates augmented reality (AR) techniques by detecting QR codes and chessboard patterns in real-time video from a webcam. Upon detection,
//              it augments the video stream with 3D shapes: a pyramid over QR codes and a combination of cubes and cylinders over the chessboard. It utilizes OpenCV's
//              features for QR code detection, chessboard corner detection, and camera calibration to accurately overlay 3D shapes on these detected patterns, showcasing
//              basic AR visualization techniques.
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include "functions.h"

/*
   Function: drawLine
   Purpose: Draws a line on an image from a start point to an end point with a specified color.
   Arguments:
       - img: Reference to the image matrix on which the line is to be drawn (cv::Mat &).
       - start: The starting point of the line (const cv::Point &).
       - end: The ending point of the line (const cv::Point &).
       - color: The color of the line (const cv::Scalar &).
   Returns: None.
*/
void drawLine(cv::Mat &img, const cv::Point &start, const cv::Point &end, const cv::Scalar &color) {
    cv::line(img, start, end, color, 2, cv::LINE_AA);
}

/*
   Function: drawCube
   Purpose: Draws a cube on an image centered at a specified point, with given size and rotation angle.
   Arguments:
       - img: Reference to the image matrix on which the cube is to be drawn (cv::Mat &).
       - center: The center point of the cube's base (const cv::Point &).
       - size: The length of the cube's edges (double).
       - angle: The rotation angle of the cube around its center in radians (double).
   Returns: None.
*/
void drawCube(cv::Mat &img, const cv::Point &center, double size, double angle) {
    std::vector<cv::Point> front, back;
    for (int i = 0; i < 4; i++) {
        double offsetAngle = angle + i * CV_PI / 2;
        front.push_back(cv::Point(center.x + size * cos(offsetAngle), center.y + size * sin(offsetAngle)));
    }
    
    for (int i = 0; i < 4; i++) {
        back.push_back(cv::Point(front[i].x + size * cos(angle - CV_PI / 4) / 2, front[i].y + size * sin(angle - CV_PI / 4) / 2));
    }

    cv::Scalar colorFront(0, 255, 0), colorBack(255, 0, 0), colorSides(0, 0, 255);
    for (int i = 0; i < 4; i++) {
        drawLine(img, front[i], front[(i + 1) % 4], colorFront);
        drawLine(img, back[i], back[(i + 1) % 4], colorBack);
        drawLine(img, front[i], back[i], colorSides);
    }
}

/*
   Function: drawPyramid
   Purpose: Draws a pyramid on an image with the apex pointing upwards. The base is centered at a specified point and has a given size.
   Arguments:
       - img: Reference to the image matrix on which the pyramid is to be drawn (cv::Mat &).
       - center: The center point of the pyramid's base (const cv::Point &).
       - size: The half-length of the pyramid's base edges (double), affecting the overall size of the pyramid.
   Returns: None.
*/
void drawPyramid(cv::Mat &img, const cv::Point &center, double size) {
    std::vector<cv::Point> base;
    for (int i = 0; i < 4; i++) {
        double angle = CV_PI / 4 + i * CV_PI / 2; 
        base.push_back(cv::Point(center.x + size * cos(angle), center.y + size * sin(angle)));
    }

    cv::Point apex(center.x, center.y - size); 
    for (int i = 0; i < 4; i++) {
        drawLine(img, base[i], base[(i + 1) % 4], cv::Scalar(255, 255, 0)); 
    }

    for (int i = 0; i < 4; i++) {
        drawLine(img, base[i], apex, cv::Scalar(255, 0, 0)); 
    }
}

/*
   Function: main
   Purpose: Serves as the entry point of the application. It continuously captures video frames from a webcam, detects QR codes and chessboard patterns,
            and augments these detections with 3D shapes rendered on the video stream. The program uses camera calibration data for accurate 3D projection.
            Pressing the 'Esc' key terminates the program.
   Returns: 0 on successful execution, -1 if an error occurs, such as failing to open the camera.
*/
int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return -1;
    }

    cv::namedWindow("AR Targets Detection", cv::WINDOW_AUTOSIZE);

    cv::QRCodeDetector qrDetector;

    cv::Mat cameraMatrix, distCoeffs;
    ReadCameraParameters("camera_calibration.csv", cameraMatrix, distCoeffs);

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        std::vector<cv::Point> points;
        std::string decodedInfo = qrDetector.detectAndDecode(frame, points);
        if (!decodedInfo.empty() && points.size() == 4) {
            std::vector<cv::Point> points;
            std::string decodedInfo = qrDetector.detectAndDecode(frame, points);
            if (!decodedInfo.empty() && points.size() == 4) {
                cv::Point qrCenter(0, 0);
                for (const auto &point : points) {
                    qrCenter += point;
                }
                qrCenter.x /= 4;
                qrCenter.y /= 4;

                drawPyramid(frame, qrCenter, 50); 
            }
        
        }
        cv::Mat cameraMatrix, distCoeffs;
        ReadCameraParameters("camera_calibration.csv", cameraMatrix, distCoeffs);

        cv::Size patternSize(9, 6);
        float squareSize = 1.0;
        std::vector<cv::Point2f> corner_set;
        std::vector<cv::Vec3f> objectPoints = Create3DChessboardCorners(patternSize, squareSize);

        cv::Vec3f baseCenter = objectPoints.at(4 * patternSize.width + 2) + cv::Vec3f(squareSize * 1.5, 1.5 * squareSize, squareSize * 2);
        std::vector<std::vector<cv::Vec3f>> blocks = CreateBlocks(baseCenter, 4 * squareSize, 6 * squareSize, 1 * squareSize, 3); // Three blocks

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


        cv::imshow("AR Targets Detection", frame);

        if (cv::waitKey(1) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
