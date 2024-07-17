#ifndef RADAR_CALIBRATOR__CALIBRATOR_NODE_HPP_
#define RADAR_CALIBRATOR__CALIBRATOR_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "radar_calibrator/calibrator_widget.hpp"
#include "radar_calibrator/common.hpp"
#include "radar_calibrator/lru_decider.hpp"
#include "radar_interfaces/msg/client_map_receive_data.hpp"
#include "radar_interfaces/msg/detection_array.hpp"

class CalibratorNode : public rclcpp::Node {
public:
  CalibratorNode(std::shared_ptr<CalibratorWidget> window,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  void
  detectionCallback(const radar_interfaces::msg::DetectionArray::SharedPtr msg);

  void timerCallback();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<radar_interfaces::msg::DetectionArray>::SharedPtr
      detection_sub_;
  rclcpp::Publisher<radar_interfaces::msg::ClientMapReceiveData>::SharedPtr
      mark_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<CalibratorWidget> window_;

  std::vector<RobotPosition> marks_;
  // std::unique_ptr<LruDecider> lru_decider_;
};
#endif