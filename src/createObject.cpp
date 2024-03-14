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

    cv::Size patternSize(9, 6);
    float squareSize = 1.0;
    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Vec3f> objectPoints = Create3DChessboardCorners(patternSize, squareSize);

    // Define parameters for the blocks
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

            // Project 3D points to image plane and draw each block
            for (const auto &block : blocks)
            {
                std::vector<cv::Point2f> imagePoints;
                projectPoints(block, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

                // Draw the top and bottom faces of the block
                for (size_t i = 0; i < 4; ++i)
                {
                    cv::line(frame, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(235, 52, 52), 2);
                    cv::line(frame, imagePoints[i + 4], imagePoints[((i + 1) % 4) + 4], cv::Scalar(235, 52, 52), 2);
                    cv::line(frame, imagePoints[i], imagePoints[i + 4], cv::Scalar(235, 52, 52), 2);
                }
            }

            // drawChessboardCorners(frame, patternSize, cv::Mat(corner_set), found);

            // Get the top face corners of the uppermost block
            cv::Vec3f topLeft = blocks.back()[4];     // Top front-left corner
            cv::Vec3f topRight = blocks.back()[5];    // Top front-right corner
            cv::Vec3f bottomRight = blocks.back()[6]; // Top back-right corner
            cv::Vec3f bottomLeft = blocks.back()[7];  // Top back-left corner

            // Calculate the center of the top face of the upper block
            cv::Vec3f topBlockCenter(
                (topLeft[0] + topRight[0] + bottomRight[0] + bottomLeft[0]) / 4.0f, // average x
                (topLeft[1] + topRight[1] + bottomRight[1] + bottomLeft[1]) / 4.0f, // average y
                topLeft[2]                                                          // z is the same for all top corners
            );

            // Create the cylinder points
            float cylinderRadius = 0.75 * squareSize; // Define a suitable radius for your cylinder
            float cylinderHeight = 3.0 * squareSize;  // Define a suitable height for your cylinder
            int slices = 36;                          // Number of slices to approximate the cylinder
            std::vector<cv::Vec3f> cylinderPoints = CreateCylinder(topBlockCenter, cylinderRadius, cylinderHeight, slices);

            // Project the cylinder points
            std::vector<cv::Point2f> projectedCylinderPoints;
            projectPoints(cylinderPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedCylinderPoints);

            // Draw the cylinder
            for (int i = 0; i < slices; i++)
            {
                // Draw the bottom circle
                cv::line(frame, projectedCylinderPoints[i * 2], projectedCylinderPoints[((i + 1) % slices) * 2], cv::Scalar(235, 52, 52), 2);
                // Draw the top circle
                cv::line(frame, projectedCylinderPoints[i * 2 + 1], projectedCylinderPoints[((i + 1) % slices) * 2 + 1], cv::Scalar(235, 52, 52), 2);
                // Draw the sides of the cylinder
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