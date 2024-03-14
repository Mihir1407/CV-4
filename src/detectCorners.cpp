#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main()
{
    cv::VideoCapture capture(0); // Open the default camera

    if (!capture.isOpened())
    {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Size patternSize(9, 6);  // Number of inner corners per a chessboard row and column (9x6 board)
    std::vector<cv::Point2f> corner_set; // This will be filled by the detected corners

    // Main loop
    while (true)
    {
        cv::Mat frame;
        capture >> frame; // Capture a new image frame

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Find the chessboard corners
        bool found = cv::findChessboardCorners(gray, patternSize, corner_set,
                                               cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

        // If found, refine the corner positions
        if (found)
        {
            cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            // Draw the corners on the frame
            drawChessboardCorners(frame, patternSize, cv::Mat(corner_set), found);

            // Example of accessing the first corner point
            std::cout << "Number of corners: " << corner_set.size() << std::endl;
            std::cout << "First corner: " << corner_set[0].x << ", " << corner_set[0].y << std::endl;
        }

        // Display the resulting frame
        cv::imshow("Frame", frame);

        // Break the loop when 'q' is pressed
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    // When everything done, release the video capture object
    capture.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}
