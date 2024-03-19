// detectCorners.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This program captures video from a webcam and detects chessboard corners in real-time. It is designed for camera calibration, using a chessboard pattern detection method. 
//              The program converts each frame to grayscale, identifies the chessboard corners, refines their accuracy, and visually displays these corners on the output feed. 
//              It provides an effective tool for collecting calibration data by allowing users to capture the necessary geometric patterns through a simple and interactive interface.


#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

/*
   Function: main
   Purpose: Serves as the entry point for the program. It captures frames from the webcam, converts them to grayscale, detects and refines chessboard corners, and displays the corners overlaid on the original frame. 
            The process continues until the user exits by pressing 'q'. This functionality supports camera calibration by facilitating the collection of various chessboard views.
   Returns: 0 on successful execution and completion, 1 if an error occurs, specifically if the camera cannot be opened.
*/
int main()
{
    cv::VideoCapture capture(1); 

    if (!capture.isOpened())
    {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Size patternSize(9, 6);  
    std::vector<cv::Point2f> corner_set; 

    while (true)
    {
        cv::Mat frame;
        capture >> frame; 

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, patternSize, corner_set,
                                               cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            drawChessboardCorners(frame, patternSize, cv::Mat(corner_set), found);

            std::cout << "Number of corners: " << corner_set.size() << std::endl;
            std::cout << "First corner: " << corner_set[0].x << ", " << corner_set[0].y << std::endl;
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
