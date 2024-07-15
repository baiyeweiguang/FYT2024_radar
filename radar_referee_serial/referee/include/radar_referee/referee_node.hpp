// Copyright (C) Qiayuan Liao.
// Copyright (C) FYT Vision Grpup. All rights reserved.

#ifndef RADAR_REFEREE_REFEREE_NODE_HPP_
#define RADAR_REFEREE_REFEREE_NODE_HPP_

#include <cstdint>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include "radar_referee/referee_comm.hpp"
#include "radar_referee/common/data.hpp"

namespace radar_referee
{
class RefereeNode : public rclcpp::Node
{
public:
  explicit RefereeNode(const rclcpp::NodeOptions& options)
    : Node("refree_node", options), last_get_data_time_(this->now())
  {
    RCLCPP_INFO(this->get_logger(), "New serial protocol loading.");
    // pub

    game_robot_status_pub_ = this->create_publisher<radar_interfaces::msg::GameRobotStatus>("game_robot_status", 1);
    game_status_pub_ = this->create_publisher<radar_interfaces::msg::GameStatus>("game_status", 1);
    power_heat_data_pub_ = this->create_publisher<radar_interfaces::msg::PowerHeatData>("power_heat_data", 1);
    game_robot_hp_pub_ = this->create_publisher<radar_interfaces::msg::GameRobotHp>("game_robot_hp", 1);
    current_sentry_pos_pub_ =
        this->create_publisher<radar_interfaces::msg::CurrentSentryPosData>("current_sentry_pos", 1);
    buff_pub_ = this->create_publisher<radar_interfaces::msg::Buff>("robot_buff", 1);
    event_data_pub_ = this->create_publisher<radar_interfaces::msg::EventData>("event_data", 1);
    dart_status_pub_ = this->create_publisher<radar_interfaces::msg::DartStatus>("dart_status_data", 1);
    icra_buff_debuff_zone_status_pub_ =
        this->create_publisher<radar_interfaces::msg::IcraBuffDebuffZoneStatus>("icra_buff_debuff_zone_status_data", 1);
    supply_projectile_action_pub_ =
        this->create_publisher<radar_interfaces::msg::SupplyProjectileAction>("supply_projectile_action_data", 1);
    dart_remaining_time_pub_ =
        this->create_publisher<radar_interfaces::msg::DartRemainingTime>("dart_remaining_time_data", 1);
    robot_hurt_pub_ = this->create_publisher<radar_interfaces::msg::RobotHurt>("robot_hurt_data", 1);
    shoot_data_pub_ = this->create_publisher<radar_interfaces::msg::ShootData>("shoot_data", 1);
    bullet_allowance_pub_ = this->create_publisher<radar_interfaces::msg::BulletAllowance>("bullet_allowance_data", 1);
    rfid_status_pub_ = this->create_publisher<radar_interfaces::msg::RfidStatus>("rfid_status_data", 1);
    dart_client_cmd_pub_ = this->create_publisher<radar_interfaces::msg::DartClientCmd>("dart_client_cmd_data", 1);
    client_map_receive_pub_ =
        this->create_publisher<radar_interfaces::msg::ClientMapReceiveData>("client_map_receive", 1);
    robots_position_pub_ = this->create_publisher<radar_interfaces::msg::RobotsPositionData>("robot_position", 1);
    radar_mark_pub_ = this->create_publisher<radar_interfaces::msg::RadarMarkData>("radar_mark", 1);
    client_map_send_data_pub_ =
        this->create_publisher<radar_interfaces::msg::ClientMapSendData>("client_map_send_data", 1);
    game_robot_pos_pub_ = this->create_publisher<radar_interfaces::msg::GameRobotPosData>("game_robot_pos", 1);
    sentry_info_pub_ = this->create_publisher<radar_interfaces::msg::SentryInfo>("sentry_info", 1);
    radar_info_pub_ = this->create_publisher<radar_interfaces::msg::RadarInfo>("radar_info", 1);

    power_management_sample_and_status_data_pub_ =
        this->create_publisher<radar_interfaces::msg::PowerManagementSampleAndStatusData>(
            "power_manager/sample_and_status", 1);
    power_management_initialization_exception_pub_ =
        this->create_publisher<radar_interfaces::msg::PowerManagementInitializationExceptionData>(
            "initialization_exception", 1);
    power_management_system_exception_data_ =
        this->create_publisher<radar_interfaces::msg::PowerManagementSystemExceptionData>(
            "power_manager/system_exception", 1);
    power_management_process_stack_overflow_pub_ =
        this->create_publisher<radar_interfaces::msg::PowerManagementProcessStackOverflowData>(
            "power_manager/stack_overflow", 1);
    power_management_unknown_exception_pub_ =
        this->create_publisher<radar_interfaces::msg::PowerManagementUnknownExceptionData>(
            "power_manager/unknown_exception", 1);
    // initSerial
    base_.initSerial();

    read_thread_ = std::thread([this]() -> void {
      rclcpp::Rate rate(40);
      while (rclcpp::ok())
      {
        this->read();
        rate.sleep();
      }
    });
    // referee_comm
    referee_comm_ = std::make_unique<radar_referee::RefereeComm>(this->create_sub_node("referee_comm"), base_);
  };
  void read();
  void clearRxBuffer()
  {
    rx_buffer_.clear();
    rx_len_ = 0;
  }
  rclcpp::Publisher<radar_interfaces::msg::GameRobotStatus>::SharedPtr game_robot_status_pub_;
  rclcpp::Publisher<radar_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<radar_interfaces::msg::PowerHeatData>::SharedPtr power_heat_data_pub_;
  rclcpp::Publisher<radar_interfaces::msg::GameRobotHp>::SharedPtr game_robot_hp_pub_;
  rclcpp::Publisher<radar_interfaces::msg::CurrentSentryPosData>::SharedPtr current_sentry_pos_pub_;
  rclcpp::Publisher<radar_interfaces::msg::Buff>::SharedPtr buff_pub_;
  rclcpp::Publisher<radar_interfaces::msg::EventData>::SharedPtr event_data_pub_;
  rclcpp::Publisher<radar_interfaces::msg::DartStatus>::SharedPtr dart_status_pub_;
  rclcpp::Publisher<radar_interfaces::msg::IcraBuffDebuffZoneStatus>::SharedPtr icra_buff_debuff_zone_status_pub_;
  rclcpp::Publisher<radar_interfaces::msg::SupplyProjectileAction>::SharedPtr supply_projectile_action_pub_;
  rclcpp::Publisher<radar_interfaces::msg::DartRemainingTime>::SharedPtr dart_remaining_time_pub_;
  rclcpp::Publisher<radar_interfaces::msg::RobotHurt>::SharedPtr robot_hurt_pub_;
  rclcpp::Publisher<radar_interfaces::msg::ShootData>::SharedPtr shoot_data_pub_;
  rclcpp::Publisher<radar_interfaces::msg::BulletAllowance>::SharedPtr bullet_allowance_pub_;
  rclcpp::Publisher<radar_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  rclcpp::Publisher<radar_interfaces::msg::DartClientCmd>::SharedPtr dart_client_cmd_pub_;
  rclcpp::Publisher<radar_interfaces::msg::ClientMapReceiveData>::SharedPtr client_map_receive_pub_;
  rclcpp::Publisher<radar_interfaces::msg::RobotsPositionData>::SharedPtr robots_position_pub_;
  rclcpp::Publisher<radar_interfaces::msg::RadarMarkData>::SharedPtr radar_mark_pub_;
  rclcpp::Publisher<radar_interfaces::msg::GameRobotPosData>::SharedPtr game_robot_pos_pub_;
  rclcpp::Publisher<radar_interfaces::msg::SentryInfo>::SharedPtr sentry_info_pub_;
  rclcpp::Publisher<radar_interfaces::msg::RadarInfo>::SharedPtr radar_info_pub_;
  rclcpp::Publisher<radar_interfaces::msg::ClientMapSendData>::SharedPtr client_map_send_data_pub_;
  rclcpp::Publisher<radar_interfaces::msg::PowerManagementSampleAndStatusData>::SharedPtr
      power_management_sample_and_status_data_pub_;
  rclcpp::Publisher<radar_interfaces::msg::PowerManagementInitializationExceptionData>::SharedPtr
      power_management_initialization_exception_pub_;
  rclcpp::Publisher<radar_interfaces::msg::PowerManagementSystemExceptionData>::SharedPtr
      power_management_system_exception_data_;
  rclcpp::Publisher<radar_interfaces::msg::PowerManagementProcessStackOverflowData>::SharedPtr
      power_management_process_stack_overflow_pub_;
  rclcpp::Publisher<radar_interfaces::msg::PowerManagementUnknownExceptionData>::SharedPtr
      power_management_unknown_exception_pub_;

  SerialBase base_;
  std::vector<uint8_t> rx_buffer_;

  std::unique_ptr<radar_referee::RefereeComm> referee_comm_;
  int rx_len_;

private:
  int unpack(uint8_t* rx_data);
  void getRobotInfo();
  void publishCapacityData();

  rclcpp::Time last_get_data_time_;
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  const int k_unpack_buffer_length_ = 256;
  uint8_t unpack_buffer_[256]{};

  std::thread read_thread_;
};

}  // namespace radar_referee
#endif
