#ifndef RADAR_CALIBRATOR__COMMON_HPP_
#define RADAR_CALIBRATOR__COMMON_HPP_

#include <opencv2/opencv.hpp>

constexpr int enemey_color = 1;

inline bool inImage(const cv::Mat &image, const cv::Point2f &p) {
  return p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows;
}

#endif