#include "grid_map.h"
#include <boost/filesystem.hpp>
using namespace std;
namespace fs = boost::filesystem;
using namespace Eigen;

GridMap::GridMap(){
    cout << "Initializing the GridMap..." << endl;
};

void GridMap::cv2vec(const cv::Mat& src, std::vector<uint8_t>& dst) {
        // Assuming src is a grayscale image (single channel)

        dst.resize(cols*rows);

        // Convert each pixel from cv::Mat to std::vector<uint8_t>
        for (int i = 0; i < src.rows; ++i) {
            for (int j = 0; j < src.cols; ++j) {
                // Setting to 0 or 255 the gridmap[i,j] if there is an obstacle or not in cell i j
                if (src.at<uchar>(i, j) > 230) {
                    dst[i * src.cols + j] = 255;
                } else {
                    dst[i * src.cols + j] = 0;
                }
            }
        }
}

void GridMap::vec2cv(const std::vector<uint8_t>& src, cv::Mat& dst){
    // Assuming dst is a grayscale image (single channel)
    dst.create(rows, cols, CV_8UC1);

    // Convert each pixel from std::vector<uint8_t> to cv::Mat
    for (int i = 0; i < dst.rows; ++i) {
        for (int j = 0; j < dst.cols; ++j) {
            // Copying the pixel value from src to the corresponding location in dst
            dst.at<uchar>(i, j) = src[i * dst.cols + j];
        }
    }
}

bool GridMap::isImageFile(const fs::path& filePath) {

    std::string extension = filePath.extension().string();
    return (extension == ".png" || extension == ".jpeg" || extension == ".jpg" || extension == ".bmp" || extension == ".gif");
};


void GridMap::loadImage(const fs::path& folder){
    fs::path imgPath = findImage(folder);
    if(!imgPath.empty()){
        cout << "Image found: " << imgPath <<endl;
        cv::Mat loadedImg = cv::imread(imgPath.string(), cv::IMREAD_GRAYSCALE);
        if(loadedImg.empty()){
            throw runtime_error("Error loading the image");
        }else{
            cout << "Image loaded successfully" << endl;
            processImage(loadedImg);
        }
    } else {
        cerr << "No image found in the specified folder" << endl;
    }
   
}

fs::path GridMap::findImage(const fs::path& folder){
    for (const auto& entry : fs::directory_iterator(folder)) {
        if (isImageFile(entry.path()) && fs::is_regular_file(entry.path())) {
            return entry.path();
        }
    }
    return fs::path();
}
 
void GridMap::processImage(const cv::Mat& img) {
    rows = img.rows;
    cols = img.cols;
    cv2vec(img, gridMapArray);
}

void GridMap::loadFromVec(std::vector<uint8_t> src, int Rows,int Cols){
    gridMapArray = src;
    rows = Rows;
    cols = Cols;
    vec2cv(gridMapArray,gridMapImage);
}

bool GridMap::checkValidStartAndGoal(pair<int, int> start, pair<int, int> goal) {
    if (gridMapArray[start.first * cols + start.second] != 255) {
        cerr << "Start position is not traversable. Value: ";
        cerr << gridMapArray[start.first * cols + start.second];
        return false;
    }
    if (gridMapArray[goal.first * cols + goal.second] != 255) {
        cerr << "Goal position (" << goal.first << ", " << goal.second << ") is not traversable. Value: ";
        cerr << to_string(gridMapArray[goal.first * cols + goal.second]);
        return false;
    }
    return true;
}


void GridMap::computeDistanceMap() {
    // Converti la griglia della mappa in una matrice OpenCV
    cv::Mat gridMapCV(rows, cols, CV_8U);
    std::copy(gridMapArray.begin(), gridMapArray.end(), gridMapCV.data);

    // Calcola la trasformata di distanza
    cv::Mat distanceTransform;
    cv::distanceTransform(gridMapCV, distanceTransform, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::minMaxLoc(distanceTransform, &minDist, &maxDist);

    // Converti la trasformata di distanza nel formato Eigen
    convertDistanceMap(distanceTransform);
}
 
void GridMap::convertDistanceMap(const cv::Mat& distanceTransform) {
    distanceMap.resize(distanceTransform.rows, distanceTransform.cols);
    for (int i = 0; i < distanceTransform.rows; ++i) {
        for (int j = 0; j < distanceTransform.cols; ++j) {
            distanceMap(i, j) = distanceTransform.at<float>(i, j);
        }
    }
}



