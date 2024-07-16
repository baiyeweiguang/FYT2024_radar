#include "radar_calibrator/calibrator_node.hpp"
#include "radar_calibrator/calibrator_widget.hpp"
#include <chrono>

CalibratorNode::CalibratorNode(std::shared_ptr<CalibratorWidget> window,
                               const rclcpp::NodeOptions &options)
    : Node("calibrator_node", options), window_(window) {
  lru_decider_ = std::make_unique<LruDecider>(this->get_clock());

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

  std::vector<radar_interfaces::msg::ClientMapReceiveData> targets;
  for (const auto &detection : msg->detections) {
    cv::Rect box(detection.bbox.top_left.x, detection.bbox.top_left.y,
                 detection.bbox.bottom_right.x - detection.bbox.top_left.x,
                 detection.bbox.bottom_right.y - detection.bbox.top_left.y);

    cv::Point2f projected = window_->projectToMap(box);

    radar_interfaces::msg::ClientMapReceiveData mark_info;
    mark_info.target_robot_id = detection.class_id + detection.color * 100;
    mark_info.target_position_x = projected.x;
    mark_info.target_position_y = projected.y;

    lru_decider_->update(mark_info);
    targets.push_back(std::move(mark_info));
  }

  window_->map_widget->setTargets(targets);
}

void CalibratorNode::timerCallback() {
  auto mark_info = lru_decider_->getMarkInfo();
  if (mark_info) {
    mark_info_pub_->publish(*mark_info);
  }
}