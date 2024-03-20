#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

namespace fs = boost::filesystem;
using namespace Eigen;
using namespace std;

struct GridMap{
    int rows;
    int cols;
    vector<uint8_t> gridMapArray;
    vector<vector<uint8_t>> gridMapMatrix;
    cv::Mat gridMapImage;
    Matrix<float, Dynamic, Dynamic> distanceMap;
    double minDist, maxDist;
    pair<int, int> start;
    pair<int, int> goal;

    GridMap();

    void loadFromImage(const fs::path& folderName);
    bool isImageFile(const fs::path& filePath);
}