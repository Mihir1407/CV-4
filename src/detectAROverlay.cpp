// detectAROverlay.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This program captures live video feed from a camera and implements augmented reality (AR) by detecting QR codes within the video frames. 
//              Upon detection of a QR code, it overlays a 3D cube that rotates based on specific gestures identified in the video feed. 
//              The gesture detection is primarily focused on identifying a green object within the frame, which triggers the rotation of the 3D cube. 

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>

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
   Function: detectGesture
   Purpose: Detects a specific gesture within the frame, defined by the presence of a green object.
   Arguments:
       - frame: The input image from which the gesture is to be detected (const cv::Mat &).
   Returns: A boolean value indicating whether the gesture has been detected (true) or not (false).
            The gesture is considered detected if there is a significant amount of green color present in the image.
*/
bool detectGesture(const cv::Mat &frame) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(70,51,102), cv::Scalar(109, 255, 153), mask); 
    return cv::countNonZero(mask) > 500; 
}

/*
   Function: main
   Purpose: Serves as the entry point for the program. It captures video from a camera, detects QR codes within the video frames, 
            and renders a rotating cube centered on the detected QR codes. Additionally, it checks for a specific gesture (green object)
            to control the rotation of the cube. The program exits when the ESC key is pressed.
   Arguments: None.
   Returns: An integer indicating the exit status of the program. Returns 0 on successful execution and -1 if the camera could not be opened.
*/
int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return -1;
    }

    cv::namedWindow("QR Code Detection with Rotating Cube", cv::WINDOW_AUTOSIZE);
    cv::QRCodeDetector qrDetector;
    bool isGestureDetected = false;
    double angle = 0.0;

    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) break;

        std::vector<cv::Point> points;
        std::string decodedInfo = qrDetector.detectAndDecode(frame, points);

        bool currentGestureState = detectGesture(frame);
        if (currentGestureState) {
            isGestureDetected = true;
        } else {
            isGestureDetected = false;
        }

        if (!decodedInfo.empty() && points.size() == 4) {
            cv::Point qrCenter(0, 0);
            for (const auto &point : points) {
                qrCenter += point;
            }
            qrCenter.x /= points.size();
            qrCenter.y /= points.size();

            if (isGestureDetected) {
                angle += CV_PI / 60; 
            }

            drawCube(frame, qrCenter, 100, angle);
        }

        cv::imshow("QR Code Detection with Rotating Cube", frame);

        if (cv::waitKey(1) == 27) break; 
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
