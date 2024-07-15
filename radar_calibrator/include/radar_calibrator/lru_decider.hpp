#ifndef RADAR_CALIBRATOR__LRU_DECIDER_HPP_
#define RADAR_CALIBRATOR__LRU_DECIDER_HPP_

#include <optional>

#include <rclcpp/clock.hpp>

#include <opencv2/opencv.hpp>

#include "radar_interfaces/msg/client_map_receive_data.hpp"
#include "radar_interfaces/msg/detection_array.hpp"

class LruDecider {

public:
  LruDecider(rclcpp::Clock::SharedPtr clock);

  void update(const radar_interfaces::msg::ClientMapReceiveData &msg);

  std::optional<radar_interfaces::msg::ClientMapReceiveData> getMarkInfo();

private:
  rclcpp::Clock::SharedPtr clock_;

  struct MarkStamped {
    radar_interfaces::msg::ClientMapReceiveData mark_info;
    bool detected = false;
    rclcpp::Time last_send_time;
  };
  std::vector<MarkStamped> marks_;
};

#endif