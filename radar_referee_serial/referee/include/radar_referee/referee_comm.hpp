// Copyright (C) Qiayuan Liao.
// Copyright (C) FYT Vision Grpup. All rights reserved.

#ifndef RADAR_REFEREE_REFEREE_COMM_HPP_
#define RADAR_REFEREE_REFEREE_COMM_HPP_

#include <sys/types.h>
#include <cstdint>
#include <deque>

#include <radar_interfaces/msg/detail/manual_to_referee__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "radar_interfaces/msg/target_info_array.hpp"
#include "radar_interfaces/msg/detection_array.hpp"
#include "radar_referee/common/data.hpp"
#include "radar_referee/common/sender.hpp"
namespace radar_referee
{
class RefereeComm
{
public:
  explicit RefereeComm(rclcpp::Node::SharedPtr node, SerialBase& base);

  // unpack call back
  virtual void robotStatusDataCallBack(const radar_interfaces::msg::GameRobotStatus& game_robot_status_data,
                                       const rclcpp::Time& last_get_data_time);
  virtual void updateEnemyHeroState(const radar_interfaces::msg::GameRobotHp& game_robot_hp_data,
                                    const rclcpp::Time& last_get_data_time);
  virtual void gameStatusDataCallBack(const radar_interfaces::msg::GameStatus& game_status_data,
                                      const rclcpp::Time& last_get_data_time);
  virtual void capacityDataCallBack(const radar_interfaces::msg::PowerManagementSampleAndStatusData& data,
                                    rclcpp::Time& last_get_data_time);
  virtual void powerHeatDataCallBack(const radar_interfaces::msg::PowerHeatData& power_heat_data,
                                     const rclcpp::Time& last_get_data_time);
  virtual void robotHurtDataCallBack(const radar_interfaces::msg::RobotHurt& robot_hurt_data,
                                     const rclcpp::Time& last_get_data_time);
  virtual void bulletRemainDataCallBack(const radar_interfaces::msg::BulletAllowance& bullet_allowance,
                                        const rclcpp::Time& last_get_data_time);
  virtual void heroHpDataCallBack(const radar_interfaces::msg::GameRobotHp& game_robot_hp_data,
                                  const rclcpp::Time& last_get_data_time);
  virtual void interactiveDataCallBack(const radar_referee::InteractiveData& interactive_data,
                                       const rclcpp::Time& last_get_data_time);
  virtual void eventDataCallBack(const radar_interfaces::msg::EventData& event_data,
                                 const rclcpp::Time& last_get_data_time);

  // sub call back
  virtual void manualDataCallBack(const radar_interfaces::msg::ManualToReferee::ConstSharedPtr data);
  virtual void radarDataCallBack(const radar_interfaces::msg::TargetInfoArray data);
  virtual void radarReceiveCallback(const radar_interfaces::msg::ClientMapReceiveData::ConstSharedPtr data);
  // virtual void mapSentryCallback(const radar_interfaces::msg::MapSentryDataConstPtr& data);
  // virtual void sentryDeviateCallback(const radar_interfaces::msg::SentryDeviateConstPtr& data);
  // virtual void sendCurrentSentryCallback(const radar_interfaces::msg::CurrentSentryPosDataConstPtr& data);
private:
  rclcpp::Subscription<radar_interfaces::msg::ManualToReferee>::SharedPtr manual_sub_;
  rclcpp::Subscription<radar_interfaces::msg::ClientMapReceiveData>::SharedPtr radar_receive_sub_;
  // rclcpp::Subscription<radar_interfaces::msg::SentryDeviate>::SharedPtr sentry_deviate_sub_;
  // rclcpp::Subscription<radar_interfaces::msg::MapSentryData>::SharedPtr map_sentry_sub_;

  rclcpp::Time radar_interactive_data_last_send_{0, 0, RCL_ROS_TIME};
  rclcpp::Time sentry_interactive_data_last_send_;
  Sender* interactive_data_sender_{};

  SerialBase& base_;
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace radar_referee
#endif
