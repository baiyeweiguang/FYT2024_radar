// Copyright (C) Qiayuan Liao.
// Copyright (C) FYT Vision Grpup. All rights reserved.

#ifndef RADAR_REFEREE_UI_BASE_HPP_
#define RADAR_REFEREE_UI_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <deque>

#include "radar_referee/common/data.hpp"

namespace radar_referee
{
class Sender
{
public:
  explicit Sender(rclcpp::Clock::SharedPtr clock, SerialBase& base) : clock_(clock), base_(base){};

  ~Sender() = default;

  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);
  void sendRadarInteractiveData(const radar_referee::ClientMapReceiveData& data);
  void sendMapSentryData(const radar_referee::MapSentryData& data);
  void sendRadarCmdData(const radar_referee::RadarCmd& data);
  // void sendCurrentSentryData(const radar_interfaces::msg::CurrentSentryPosDataConstPtr& data);
  void sendCustomInfoData(std::wstring data);

  void sendSerial(const rclcpp::Time& time, int data_len);
  void clearTxBuffer();

  void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const;

  uint8_t tx_buffer_[128];
  int tx_len_;

protected:
  rclcpp::Clock::SharedPtr clock_;
  SerialBase& base_;
  static int id_;
  rclcpp::Time last_send_;
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
};
}  // namespace radar_referee
#endif
