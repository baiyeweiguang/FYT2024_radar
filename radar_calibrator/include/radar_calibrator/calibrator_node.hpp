#ifndef RADAR_CALIBRATOR__CALIBRATOR_NODE_HPP_
#define RADAR_CALIBRATOR__CALIBRATOR_NODE_HPP_

#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "radar_calibrator/calibrator_widget.hpp"
#include "radar_calibrator/common.hpp"
#include "radar_interfaces/msg/client_map_receive_data.hpp"
#include "radar_interfaces/msg/detection_array.hpp"
#include "radar_interfaces/msg/radar_info.hpp"
#include "radar_interfaces/msg/radar_cmd.hpp"
class CalibratorNode : public rclcpp::Node {
public:
  CalibratorNode(std::shared_ptr<CalibratorWidget> window,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  void
  detectionCallback(const radar_interfaces::msg::DetectionArray::SharedPtr msg);

  void radarInfoCallback(const radar_interfaces::msg::RadarInfo::SharedPtr msg);

  void timerCallback();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<radar_interfaces::msg::DetectionArray>::SharedPtr
      detection_sub_;
  rclcpp::Publisher<radar_interfaces::msg::ClientMapReceiveData>::SharedPtr
      mark_info_pub_;

  rclcpp::Subscription<radar_interfaces::msg::RadarInfo>::SharedPtr radar_info_sub_;
  rclcpp::Publisher<radar_interfaces::msg::RadarCmd>::SharedPtr radar_cmd_pub_;
  radar_interfaces::msg::RadarCmd cmd_msg_;

  uint8_t buff_num_ = 0;
  rclcpp::Time last_buff_time_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr decision_;

  std::shared_ptr<CalibratorWidget> window_;

  std::vector<RobotPosition> marks_;
};
#endif