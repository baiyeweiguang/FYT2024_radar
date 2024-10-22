#ifndef RADAR_CALIBRATOR__COMMON_HPP_
#define RADAR_CALIBRATOR__COMMON_HPP_

#include <opencv2/opencv.hpp>
constexpr int RED = 0;
constexpr int BLUE = 1;
constexpr int ENEMY_COLOR = BLUE;

inline bool inImage(const cv::Mat &image, const cv::Point2f &p) {
  return p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows;
}

struct RobotPosition {
  bool sent = true;
  float score = 0;
  uint16_t robot_id = 0;
  float x = 0;
  float y = 0;
};

#endif