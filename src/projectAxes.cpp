#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include "functions.h"

int main()
{
    cv::VideoCapture capture(0);
    if (!capture.isOpened())
    {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Mat cameraMatrix, distCoeffs;
    ReadCameraParameters("camera_calibration.csv", cameraMatrix, distCoeffs);

    cv::Size patternSize(9, 6); // Should match the calibration pattern size.
    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Vec3f> objectPoints = Create3DChessboardCorners(patternSize, 1.0); // Consistent with calibration.

    // Define 3D axis points for visualization.
    std::vector<cv::Vec3f> axisPoints = {
        cv::Vec3f(0, 0, 0),    // Origin
        cv::Vec3f(3, 0, 0), // X axis
        cv::Vec3f(0, 3, 0), // Y axis
        cv::Vec3f(0, 0, -3) // Z axis
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
            cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3); // X-axis in red.
            cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3); // Y-axis in green.
            cv::line(frame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3); // Z-axis in blue.

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
