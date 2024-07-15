// Copyright (C) Qiayuan Liao.
// Copyright (C) FYT Vision Grpup. All rights reserved.

#ifndef RADAR_REFEREE_DATA_HPP_
#define RADAR_REFEREE_DATA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
// #include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "radar_referee/common/protocol.hpp"
#include "serial/serial.h"

#include <radar_interfaces/msg/state_cmd.hpp>
#include <radar_interfaces/msg/event_data.hpp>
#include <radar_interfaces/msg/robot_hurt.hpp>
#include <radar_interfaces/msg/shoot_data.hpp>
#include <radar_interfaces/msg/dart_status.hpp>
#include <radar_interfaces/msg/game_status.hpp>
#include <radar_interfaces/msg/rfid_status.hpp>
#include <radar_interfaces/msg/engineer_ui.hpp>
#include <radar_interfaces/msg/game_robot_hp.hpp>
#include <radar_interfaces/msg/dart_client_cmd.hpp>
#include <radar_interfaces/msg/map_sentry_data.hpp>
#include <radar_interfaces/msg/radar_mark_data.hpp>
#include <radar_interfaces/msg/power_heat_data.hpp>
#include <radar_interfaces/msg/bullet_allowance.hpp>
#include <radar_interfaces/msg/game_robot_status.hpp>
#include <radar_interfaces/msg/manual_to_referee.hpp>
#include <radar_interfaces/msg/client_map_send_data.hpp>
#include <radar_interfaces/msg/robots_position_data.hpp>
#include <radar_interfaces/msg/dart_remaining_time.hpp>
#include <radar_interfaces/msg/client_map_receive_data.hpp>
#include <radar_interfaces/msg/supply_projectile_action.hpp>
#include <radar_interfaces/msg/icra_buff_debuff_zone_status.hpp>
#include <radar_interfaces/msg/current_sentry_pos_data.hpp>
#include <radar_interfaces/msg/game_robot_pos_data.hpp>
#include "radar_interfaces/msg/sentry_info.hpp"
#include "radar_interfaces/msg/radar_info.hpp"
#include "radar_interfaces/msg/buff.hpp"
#include <radar_interfaces/msg/power_management_sample_and_status_data.hpp>
#include <radar_interfaces/msg/power_management_system_exception_data.hpp>
#include <radar_interfaces/msg/power_management_initialization_exception_data.hpp>
#include <radar_interfaces/msg/power_management_process_stack_overflow_data.hpp>
#include <radar_interfaces/msg/power_management_unknown_exception_data.hpp>

namespace radar_referee
{
struct CapacityData
{
  double chassis_power;
  double limit_power;
  double buffer_power;
  double cap_power;
  bool is_online = false;
};

class SerialBase
{
public:
  serial::Serial serial_;

  int client_id_ = 0;  // recipient's id
  int robot_id_ = 0;   // recent  robot's id
  int capacity_recent_mode_, capacity_expect_mode_;
  std::string robot_color_;
  bool referee_data_is_online_ = false;

  void initSerial()
  {
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setPort("/dev/ttyReferee");
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
    if (serial_.isOpen())
      return;
    try
    {
      serial_.open();
    }
    catch (serial::IOException& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cannot open referee port, what: %s", e.what());
    }
  }

  // CRC check
  uint8_t getCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length, unsigned char uc_crc_8)
  {
    unsigned char uc_index;
    while (dw_length--)
    {
      uc_index = uc_crc_8 ^ (*pch_message++);
      uc_crc_8 = radar_referee::kCrc8Table[uc_index];
    }
    return (uc_crc_8);
  }

  uint32_t verifyCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length)
  {
    unsigned char uc_expected;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return 0;
    uc_expected = getCRC8CheckSum(pch_message, dw_length - 1, radar_referee::kCrc8Init);
    return (uc_expected == pch_message[dw_length - 1]);
  }

  void appendCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length)
  {
    unsigned char uc_crc;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return;
    uc_crc = getCRC8CheckSum((unsigned char*)pch_message, dw_length - 1, radar_referee::kCrc8Init);
    pch_message[dw_length - 1] = uc_crc;
  }

  uint16_t getCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length, uint16_t w_crc)
  {
    uint8_t chData;
    if (pch_message == nullptr)
      return 0xFFFF;
    while (dw_length--)
    {
      chData = *pch_message++;
      (w_crc) = (static_cast<uint16_t>(w_crc) >> 8) ^
                radar_referee::wCRC_table[(static_cast<uint16_t>(w_crc) ^ static_cast<uint16_t>(chData)) & 0x00ff];
    }
    return w_crc;
  }

  uint32_t verifyCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length)
  {
    uint16_t w_expected;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return 0;
    w_expected = getCRC16CheckSum(pch_message, dw_length - 2, radar_referee::kCrc16Init);
    return ((w_expected & 0xff) == pch_message[dw_length - 2] &&
            ((w_expected >> 8) & 0xff) == pch_message[dw_length - 1]);
  }

  void appendCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length)
  {
    uint16_t wCRC;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return;
    wCRC = getCRC16CheckSum(static_cast<uint8_t*>(pch_message), dw_length - 2, radar_referee::kCrc16Init);
    pch_message[dw_length - 2] = static_cast<uint8_t>((wCRC & 0x00ff));
    pch_message[dw_length - 1] = static_cast<uint8_t>(((wCRC >> 8) & 0x00ff));
  }
};
}  // namespace radar_referee

#endif
