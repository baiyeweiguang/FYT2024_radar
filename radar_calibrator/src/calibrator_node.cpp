#include "radar_calibrator/calibrator_node.hpp"
#include "radar_calibrator/calibrator_widget.hpp"
#include "radar_calibrator/common.hpp"
#include <chrono>

CalibratorNode::CalibratorNode(std::shared_ptr<CalibratorWidget> window,
                               const rclcpp::NodeOptions &options)
    : Node("calibrator_node", options), window_(window) {
  // lru_decider_ = std::make_unique<LruDecider>(this->get_clock());
  marks_.resize(12);
  // 0-5: red robots, 6-11: blue robots
  for (int i = 0; i < 6; i++) {
    marks_[i].robot_id = i + 1;
    marks_[i + 6].robot_id = i + 1 + 100;
  }

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/debug_img", 1,
      std::bind(&CalibratorNode::imageCallback, this, std::placeholders::_1));

  detection_sub_ = create_subscription<radar_interfaces::msg::DetectionArray>(
      "/detections", 1,
      std::bind(&CalibratorNode::detectionCallback, this,
                std::placeholders::_1));

  mark_info_pub_ =
      create_publisher<radar_interfaces::msg::ClientMapReceiveData>("/rm_radar",
                                                                    10);

  int freq = 10;
  int period = static_cast<int>(1000 / freq);
  timer_ = create_wall_timer(std::chrono::milliseconds(period),
                             std::bind(&CalibratorNode::timerCallback, this));
}

void CalibratorNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    window_->setCameraImage(cv_ptr->image);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void CalibratorNode::detectionCallback(
    const radar_interfaces::msg::DetectionArray::SharedPtr msg) {

  for (auto &mark : marks_) {
    mark.x = 0;
    mark.y = 0;
  }

  std::vector<radar_interfaces::msg::ClientMapReceiveData> targets;
  for (const auto &detection : msg->detections) {
    cv::Rect box(detection.bbox.top_left.x, detection.bbox.top_left.y,
                 detection.bbox.bottom_right.x - detection.bbox.top_left.x,
                 detection.bbox.bottom_right.y - detection.bbox.top_left.y);

    cv::Point2f projected = window_->projectToMap(box);

    int offset = detection.color == 0 ? 0 : 6;
    marks_[detection.class_id - 1 + offset].x = projected.x;
    marks_[detection.class_id - 1 + offset].y = projected.y;
  }

  window_->map_widget->setTargets(marks_);
}

void CalibratorNode::timerCallback() {
  radar_interfaces::msg::ClientMapReceiveData marks_msg;
  marks_msg.header.stamp = this->now();
  marks_msg.header.frame_id = "map";
  int offset = enemey_color == 0 ? 0 : 6;

  marks_msg.hero_position_x = marks_[0 + offset].x * 100;
  marks_msg.hero_position_y = marks_[0 + offset].y * 100;
  marks_msg.engineer_position_x = marks_[1 + offset].x * 100;
  marks_msg.engineer_position_y = marks_[1 + offset].y * 100;
  marks_msg.infantry_3_position_x = marks_[2 + offset].x * 100;
  marks_msg.infantry_3_position_y = marks_[2 + offset].y * 100;
  marks_msg.infantry_4_position_x = marks_[3 + offset].x * 100;
  marks_msg.infantry_4_position_y = marks_[3 + offset].y * 100;
  marks_msg.infantry_5_position_x = marks_[4 + offset].x * 100;
  marks_msg.infantry_5_position_y = marks_[4 + offset].y * 100;
  marks_msg.sentry_position_x = marks_[5 + offset].x * 100;
  marks_msg.sentry_position_y = marks_[5 + offset].y * 100;
  mark_info_pub_->publish(marks_msg);
}