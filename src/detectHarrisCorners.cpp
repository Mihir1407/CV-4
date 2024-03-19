// detectHarrisCorners.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This program captures live video from a webcam and applies the Harris corner detection algorithm to identify corner points in each frame. 
//              It further refines the corner detection using non-maximum suppression to ensure that only the most prominent corners are marked. 
//              The identified corners are visualized by drawing small circles around them. 

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

/*
   Function: nonMaximumSuppression
   Purpose: Applies non-maximum suppression to the result of Harris corner detection to filter out less prominent corners based on a specified threshold.
   Arguments:
       - src: Source image obtained from Harris corner detection (const cv::Mat&).
       - dst: Destination image where the result of non-maximum suppression is stored (cv::Mat&).
       - sz: Size of the neighborhood considered for suppression (const int). [Parameter not used in current implementation]
       - threshold: The threshold value for selecting prominent corners (const float).
   Returns: None.
*/
void nonMaximumSuppression(const cv::Mat& src, cv::Mat& dst, const int sz, const float threshold) {
    cv::Mat dilate_dst;
    dst = cv::Mat::zeros(src.size(), CV_8U); 

    cv::dilate(src, dilate_dst, cv::Mat());

    for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
            if (src.at<float>(i, j) == dilate_dst.at<float>(i, j) && src.at<float>(i, j) > threshold) {
                dst.at<uchar>(i, j) = 255; 
            }
        }
    }
}

/*
   Function: main
   Purpose: Entry point of the program. It captures video from a webcam, processes each frame to detect corners using the Harris corner detection algorithm, and visualizes these corners after applying non-maximum suppression.
   Returns: -1 if an error occurs (e.g., video stream cannot be opened), 0 on successful execution and completion.
*/
int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream" << std::endl;
        return -1;
    }

    cv::Size patternSize(9, 6); 

    cv::namedWindow("Harris Corners", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame, gray;
        cap >> frame; 
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat dst, dst_norm, dst_norm_scaled, corners_nms;
        cv::cornerHarris(gray, dst, 2, 3, 0.04, cv::BORDER_DEFAULT);
        cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        cv::convertScaleAbs(dst_norm, dst_norm_scaled);

        nonMaximumSuppression(dst_norm, corners_nms, 2, 200); 

        for (int i = 0; i < corners_nms.rows; i++) {
            for (int j = 0; j < corners_nms.cols; j++) {
                if (corners_nms.at<uchar>(i, j)) {
                    cv::circle(frame, cv::Point(j, i), 5, cv::Scalar(255, 0, 0), 1); 
                }
            }
        }

        cv::imshow("Harris Corners", frame);

        if (cv::waitKey(30) >= 0) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
