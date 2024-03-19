// detectMultipleTargets.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This program captures video from a webcam and utilizes OpenCV's QR code detection capabilities to identify multiple QR codes within each frame. 
//              Upon detection, it outlines each QR code with a green rectangle and draws blue lines from the corners of each QR code to a common apex point above the QR code, creating a pyramid-like visual effect. 
//              Thisvisualization helps in easily identifying and highlighting the presence and position of QR codes in real-time video. The program also displays the total count of detected QR codes on the video feed, updating dynamically with each frame.

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

/*
   Function: main
   Purpose: Entry point for the program. It continuously captures frames from the webcam, detects QR codes within these frames, and visually highlights each detected QR code. Additionally, it calculates and displays the total number of QR codes detected in each frame.
   Returns: 1 if an error occurs (e.g., the camera cannot be opened), 0 upon successful execution and orderly shutdown of the program.
*/
int main() {
    cv::VideoCapture cap(0); 

    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::namedWindow("Multiple QR Code Detection", cv::WINDOW_AUTOSIZE);
    cv::QRCodeDetector qrDetector;

    while (true) {
        cv::Mat frame;
        cap >> frame; 

        if (frame.empty()) {
            std::cerr << "ERROR: Couldn't grab a camera frame." << std::endl;
            break;
        }

        std::vector<cv::Point> points;
        std::vector<std::string> decodedObjects;
        bool detected = qrDetector.detectAndDecodeMulti(frame, decodedObjects, points);

        if (detected) {
            int qrCodeCount = 0;
            for (size_t i = 0; i < points.size(); i += 4) {
                qrCodeCount++;
                for (int j = 0; j < 4; j++) {
                    cv::line(frame, points[i + j], points[i + (j + 1) % 4], cv::Scalar(0, 255, 0), 4);
                }

                cv::Point center(0, 0);
                for (int j = 0; j < 4; j++) {
                    center += points[i + j];
                }
                center.x /= 4;
                center.y /= 4;

                cv::Point pyramidApex(center.x, center.y - 100); 

                for (int j = 0; j < 4; j++) {
                    cv::line(frame, points[i + j], pyramidApex, cv::Scalar(255, 0, 0), 2);
                }
            }

            std::string label = "QR Codes detected: " + std::to_string(qrCodeCount);
            cv::putText(frame, label, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("Multiple QR Code Detection", frame);

        if (cv::waitKey(1) == 27) break; 
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
