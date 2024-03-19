// functions.cpp
// Author: Aditya Gurnani, Mihir Chitre
// Date: 03/19/2024
// Description: This utility program consists of several functions designed to support camera calibration and 3D object creation for augmented reality (AR) applications. 
//              It includes methods for generating 3D representations of chessboard corners, writing 3D point data to a CSV file, performing camera calibration with saved points, 
//              reading calibration parameters from a file, and creating 3D blocks and cylinders. These functionalities are crucial for developing AR applications that require accurate spatial mapping and object placement within a real-world environment.
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>

/*
   Function: Create3DChessboardCorners
   Purpose: Generates a vector of 3D points representing the corners of a chessboard, which are used for camera calibration.
   Arguments:
       - patternSize: Dimensions of the chessboard pattern (cv::Size).
       - squareSize: The size of a single square on the chessboard (float).
   Returns: A vector of 3D points (std::vector<cv::Vec3f>).
*/
std::vector<cv::Vec3f> Create3DChessboardCorners(cv::Size patternSize, float squareSize) {
    std::vector<cv::Vec3f> corners;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            corners.push_back(cv::Vec3f(j * squareSize, -i * squareSize, 0.0f));
        }
    }
    return corners;
}

/*
   Function: WriteToCSV
   Purpose: Writes a vector of 3D points or 2D corners to a CSV file.
   Arguments:
       - file: Reference to the output file stream (std::ofstream&).
       - points/corners: Vector of 3D points or 2D corners to be written (const std::vector<cv::Vec3f>& or const std::vector<cv::Point2f>&).
   Returns: None.
*/
void WriteToCSV(std::ofstream &file, const std::vector<cv::Vec3f> &points) {
    for (const auto &p : points) {
        file << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    }
}

/*
   Function: WriteToCSV
   Purpose: Writes a vector of 3D points or 2D corners to a CSV file.
   Arguments:
       - file: Reference to the output file stream (std::ofstream&).
       - points/corners: Vector of 3D points or 2D corners to be written (const std::vector<cv::Vec3f>& or const std::vector<cv::Point2f>&).
   Returns: None.
*/
void WriteToCSV(std::ofstream &file, const std::vector<cv::Point2f> &corners) {
    for (const auto &c : corners) {
        file << c.x << ", " << c.y << std::endl;
    }
}

/*
   Function: RunCalibrationAndSave
   Purpose: Performs camera calibration using the detected chessboard corners and the corresponding 3D points, then saves the camera matrix and distortion coefficients to a CSV file.
   Arguments:
       - corner_list: A list of detected chessboard corners for each calibration image (const std::vector<std::vector<cv::Point2f>>&).
       - point_list: A list of corresponding 3D points for each chessboard corner (const std::vector<std::vector<cv::Vec3f>>&).
       - image_size: The size of the calibration images (cv::Size&).
   Returns: None.
*/
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

/*
   Function: ReadCameraParameters
   Purpose: Reads camera calibration parameters (camera matrix and distortion coefficients) from a CSV file.
   Arguments:
       - filename: Name of the file from which to read the parameters (const std::string &).
       - cameraMatrix: Output camera matrix (cv::Mat &).
       - distCoeffs: Output distortion coefficients (cv::Mat &).
   Returns: None.
*/
void ReadCameraParameters(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    std::ifstream inFile(filename);
    
    if (!inFile) {
        std::cerr << "Error: Could not open " << filename << std::endl;
        throw std::runtime_error("Failed to open calibration file.");
    }

    std::string line, token;
    
    std::getline(inFile, line); 
    std::istringstream cm(line);

    std::getline(cm, token, ','); 
    
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    std::getline(cm, token, ',');
    cameraMatrix.at<double>(0, 0) = std::stod(token);

    std::getline(cm, token, ',');
    cameraMatrix.at<double>(1, 1) = std::stod(token);

    std::getline(cm, token, ',');
    cameraMatrix.at<double>(0, 2) = std::stod(token);

    std::getline(cm, token, ',');
    cameraMatrix.at<double>(1, 2) = std::stod(token);

    std::getline(inFile, line); 
    std::istringstream dc(line);
    
    std::getline(dc, token, ',');

    distCoeffs = cv::Mat::zeros(5, 1, CV_64F); 
    
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

/*
   Function: CreateBlocks
   Purpose: Generates 3D models for a set of blocks positioned on a base, typically used for creating stacked block structures in AR applications.
   Arguments:
       - baseCenter: The center position of the base block (cv::Vec3f).
       - baseWidth, baseDepth, height: Dimensions of the blocks (float).
       - numBlocks: Number of blocks to generate (int).
   Returns: A vector of vectors, each containing the 3D points of a block (std::vector<std::vector<cv::Vec3f>>).
*/
std::vector<std::vector<cv::Vec3f>> CreateBlocks(cv::Vec3f baseCenter, float baseWidth, float baseDepth, float height, int numBlocks)
{
    std::vector<std::vector<cv::Vec3f>> blocks;
    float decrementFactor = 0.8;
    float currentZOffset = 0.0; 
    for (int i = 0; i < numBlocks; ++i)
    {
        float currentWidth = baseWidth * std::pow(decrementFactor, i);
        float currentDepth = baseDepth * std::pow(decrementFactor, i);

        float currentBaseZ = baseCenter[2] + currentZOffset + height; 

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

        currentZOffset += height;
    }

    return blocks;
}

/*
   Function: CreateCylinder
   Purpose: Generates 3D points for a cylinder, useful for AR visualizations.
   Arguments:
       - baseCenter: The center position of the cylinder's base (cv::Vec3f).
       - radius: Radius of the cylinder (float).
       - height: Height of the cylinder (float).
       - slices: Number of slices to approximate the cylinder's curvature (int).
   Returns: A vector containing the 3D points of the cylinder (std::vector<cv::Vec3f>).
*/
std::vector<cv::Vec3f> CreateCylinder(cv::Vec3f baseCenter, float radius, float height, int slices)
{
    std::vector<cv::Vec3f> cylinderPoints;

    for (int i = 0; i < slices; i++)
    {
        float angle = (2 * CV_PI * i) / slices;
        float x = radius * cos(angle);
        float y = radius * sin(angle);

        cylinderPoints.push_back(cv::Vec3f(baseCenter[0] + x, baseCenter[1] + y, baseCenter[2]));
        cylinderPoints.push_back(cv::Vec3f(baseCenter[0] + x, baseCenter[1] + y, baseCenter[2] + height));
    }

    return cylinderPoints;
}