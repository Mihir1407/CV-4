#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>

std::vector<cv::Vec3f> Create3DChessboardCorners(cv::Size patternSize, float squareSize) {
    std::vector<cv::Vec3f> corners;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            corners.push_back(cv::Vec3f(j * squareSize, -i * squareSize, 0.0f));
        }
    }
    return corners;
}

void WriteToCSV(std::ofstream &file, const std::vector<cv::Vec3f> &points) {
    for (const auto &p : points) {
        file << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    }
}

void WriteToCSV(std::ofstream &file, const std::vector<cv::Point2f> &corners) {
    for (const auto &c : corners) {
        file << c.x << ", " << c.y << std::endl;
    }
}

void RunCalibrationAndSave(const std::vector<std::vector<cv::Point2f>> &corner_list,
                           const std::vector<std::vector<cv::Vec3f>> &point_list,
                           cv::Size &image_size) {
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 2) = image_size.width / 2.0;
    camera_matrix.at<double>(1, 2) = image_size.height / 2.0;

    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // Use 5-parameter model by default
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(point_list, corner_list, image_size, camera_matrix, dist_coeffs, rvecs, tvecs);

    std::cout << "Re-projection error: " << rms << std::endl;
    std::cout << "Camera matrix: " << camera_matrix << std::endl;
    std::cout << "Distortion coefficients: " << dist_coeffs << std::endl;

    std::ofstream outFile("camera_calibration.csv");

    outFile << std::fixed << std::setprecision(15);

    outFile << "Camera_Matrix";
    outFile << "," << camera_matrix.at<double>(0, 0) << "," << camera_matrix.at<double>(1, 1)
            << "," << camera_matrix.at<double>(0, 2) << "," << camera_matrix.at<double>(1, 2) << std::endl;
    outFile << "Distortion_Coefficients";
    for (int i = 0; i < dist_coeffs.rows; i++) {
        outFile << "," << dist_coeffs.at<double>(i);
    }
    outFile << std::endl;
    outFile.close();
}

void ReadCameraParameters(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    std::ifstream inFile(filename);
    
    if (!inFile) {
        std::cerr << "Error: Could not open " << filename << std::endl;
        throw std::runtime_error("Failed to open calibration file.");
    }

    std::string line, token;
    
    // Reading camera matrix
    std::getline(inFile, line); // Read the camera matrix values
    std::istringstream cm(line);

    std::getline(cm, token, ','); // Skip "Camera_Matrix" label
    
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    // fx
    std::getline(cm, token, ',');
    cameraMatrix.at<double>(0, 0) = std::stod(token);

    // fy
    std::getline(cm, token, ',');
    cameraMatrix.at<double>(1, 1) = std::stod(token);

    // cx
    std::getline(cm, token, ',');
    cameraMatrix.at<double>(0, 2) = std::stod(token);

    // cy
    std::getline(cm, token, ',');
    cameraMatrix.at<double>(1, 2) = std::stod(token);

    // Reading distortion coefficients
    std::getline(inFile, line); // Read the distortion coefficients values
    std::istringstream dc(line);
    
    std::getline(dc, token, ','); // Skip "Distortion_Coefficients" label

    distCoeffs = cv::Mat::zeros(5, 1, CV_64F); // Assuming 5 distortion coefficients
    
    for (int i = 0; i < 5; ++i) {
        if (!std::getline(dc, token, ',')) {
            std::cerr << "Error: Unexpected format in distortion coefficients data." << std::endl;
            throw std::runtime_error("Format error in distortion coefficients data.");
        }
        distCoeffs.at<double>(i) = std::stod(token);
    }

    std::cout << "Camera matrix: " << cameraMatrix << std::endl;
    std::cout << "Distortion coefficients: " << distCoeffs << std::endl;
}

std::vector<std::vector<cv::Vec3f>> CreateBlocks(cv::Vec3f baseCenter, float baseWidth, float baseDepth, float height, int numBlocks)
{
    std::vector<std::vector<cv::Vec3f>> blocks;
    float decrementFactor = 0.8; // Decrease size by 20% for each upper block
    float currentZOffset = 0.0;  // Initialize the current height offset at the base level on the chessboard

    for (int i = 0; i < numBlocks; ++i)
    {
        float currentWidth = baseWidth * std::pow(decrementFactor, i);
        float currentDepth = baseDepth * std::pow(decrementFactor, i);

        // Update the z-coordinate to stack the blocks along the z-axis (towards the viewer)
        float currentBaseZ = baseCenter[2] + currentZOffset + height; // Move outwards by one block height for each new block

        std::vector<cv::Vec3f> block = {
            cv::Vec3f(baseCenter[0] - currentWidth / 2, baseCenter[1] - currentDepth / 2, currentBaseZ),
            cv::Vec3f(baseCenter[0] + currentWidth / 2, baseCenter[1] - currentDepth / 2, currentBaseZ),
            cv::Vec3f(baseCenter[0] + currentWidth / 2, baseCenter[1] + currentDepth / 2, currentBaseZ),
            cv::Vec3f(baseCenter[0] - currentWidth / 2, baseCenter[1] + currentDepth / 2, currentBaseZ),
            cv::Vec3f(baseCenter[0] - currentWidth / 2, baseCenter[1] - currentDepth / 2, currentBaseZ - height),
            cv::Vec3f(baseCenter[0] + currentWidth / 2, baseCenter[1] - currentDepth / 2, currentBaseZ - height),
            cv::Vec3f(baseCenter[0] + currentWidth / 2, baseCenter[1] + currentDepth / 2, currentBaseZ - height),
            cv::Vec3f(baseCenter[0] - currentWidth / 2, baseCenter[1] + currentDepth / 2, currentBaseZ - height)};

        blocks.push_back(block);

        // Update the z-offset for the next block to stack on top of this one
        currentZOffset += height;
    }

    return blocks;
}

std::vector<cv::Vec3f> CreateCylinder(cv::Vec3f baseCenter, float radius, float height, int slices)
{
    std::vector<cv::Vec3f> cylinderPoints;

    // Generate points for the top and bottom circles of the cylinder
    for (int i = 0; i < slices; i++)
    {
        float angle = (2 * CV_PI * i) / slices;
        float x = radius * cos(angle);
        float y = radius * sin(angle);

        // Bottom circle point
        cylinderPoints.push_back(cv::Vec3f(baseCenter[0] + x, baseCenter[1] + y, baseCenter[2]));
        // Corresponding top circle point
        cylinderPoints.push_back(cv::Vec3f(baseCenter[0] + x, baseCenter[1] + y, baseCenter[2] + height));
    }

    return cylinderPoints;
}