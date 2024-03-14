#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

std::vector<cv::Vec3f> Create3DChessboardCorners(cv::Size patternSize, float squareSize);

void WriteToCSV(std::ofstream &file, const std::vector<cv::Vec3f> &points);

void WriteToCSV(std::ofstream &file, const std::vector<cv::Point2f> &corners);

std::vector<cv::Vec3f> Create3DChessboardCorners(cv::Size patternSize, float squareSize);

void RunCalibrationAndSave(const std::vector<std::vector<cv::Point2f>> &corner_list,
                           const std::vector<std::vector<cv::Vec3f>> &point_list,
                           cv::Size &image_size);

void ReadCameraParameters(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

std::vector<std::vector<cv::Vec3f>> CreateBlocks(cv::Vec3f baseCenter, float baseWidth, float baseDepth, float height, int numBlocks);

std::vector<cv::Vec3f> CreateCylinder(cv::Vec3f baseCenter, float radius, float height, int slices);

#endif