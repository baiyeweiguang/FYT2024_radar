//
// Created by ljq on 2021/12/3.
//

#pragma once
// #include <rm_common/decision/command_sender.h>

// #include "rm_referee/ui/ui_base.h"
// #include "rm_referee/ui/trigger_change_ui.h"
// #include "rm_referee/ui/time_change_ui.h"
// #include "rm_referee/ui/flash_ui.h"

#include "radar_referee/common/data.hpp"

namespace radar_referee
{
class RefereeBase
{
public:
  explicit RefereeBase(rclcpp::Node& nh, SerialBase& base);
  virtual void addUi();

  // unpack call back
  virtual void robotStatusDataCallBack(const radar_interfaces::msg::GameRobotStatus& game_robot_status_data,
                                       const rclcpp::Time& last_get_data_time);
  virtual void updateEnemyHeroState(const radar_interfaces::msg::GameRobotHp& game_robot_hp_data, const rclcpp::Time& last_get_data_time);
  virtual void gameStatusDataCallBack(const radar_interfaces::msg::GameStatus& game_status_data, const rclcpp::Time& last_get_data_time);
  virtual void capacityDataCallBack(const radar_interfaces::msg::PowerManagementSampleAndStatusData& data,
                                    rclcpp::Time& last_get_data_time);
  virtual void powerHeatDataCallBack(const radar_interfaces::msg::PowerHeatData& power_heat_data, const rclcpp::Time& last_get_data_time);
  virtual void robotHurtDataCallBack(const radar_interfaces::msg::RobotHurt& robot_hurt_data, const rclcpp::Time& last_get_data_time);
  virtual void bulletRemainDataCallBack(const radar_interfaces::msg::BulletAllowance& bullet_allowance,
                                        const rclcpp::Time& last_get_data_time);
  virtual void interactiveDataCallBack(const rm_referee::InteractiveData& interactive_data,
                                       const rclcpp::Time& last_get_data_time);
  virtual void eventDataCallBack(const radar_interfaces::msg::EventData& event_data, const rclcpp::Time& last_get_data_time);
  virtual void updateHeroHitDataCallBack(const radar_interfaces::msg::GameRobotHp& game_robot_hp_data);

  // sub call back
//   virtual void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state);
//   virtual void actuatorStateCallback(const radar_interfaces::msg::ActuatorState::ConstPtr& data);
//   virtual void dbusDataCallback(const radar_interfaces::msg::DbusData::ConstPtr& data);
//   virtual void chassisCmdDataCallback(const radar_interfaces::msg::ChassisCmd::ConstPtr& data);
//   virtual void vel2DCmdDataCallback(const geometry_msgs::Twist::ConstPtr& data);
//   virtual void shootStateCallback(const radar_interfaces::msg::ShootState::ConstPtr& data);
//   virtual void gimbalCmdDataCallback(const radar_interfaces::msg::GimbalCmd::ConstPtr& data);
//   virtual void cardCmdDataCallback(const radar_interfaces::msg::StateCmd::ConstPtr& data);
//   virtual void engineerUiDataCallback(const radar_interfaces::msg::EngineerUi::ConstPtr& data);
//   virtual void manualDataCallBack(const radar_interfaces::msg::ManualToReferee::ConstPtr& data);
//   virtual void radarDataCallBack(const std_msgs::Int8MultiArrayConstPtr& data);
//   virtual void cameraNameCallBack(const std_msgs::StringConstPtr& data);
//   virtual void trackCallBack(const radar_interfaces::msg::TrackDataConstPtr& data);
//   virtual void balanceStateCallback(const radar_interfaces::msg::BalanceStateConstPtr& data);
//   virtual void radarReceiveCallback(const radar_interfaces::msg::ClientMapReceiveData::ConstPtr& data);
//   virtual void mapSentryCallback(const radar_interfaces::msg::MapSentryDataConstPtr& data);
//   virtual void sentryDeviateCallback(const radar_interfaces::msg::SentryDeviateConstPtr& data);
//   virtual void sendCurrentSentryCallback(const radar_interfaces::msg::CurrentSentryPosDataConstPtr& data);
//   virtual void sendSentryCmdCallback(const radar_interfaces::msg::SentryInfoConstPtr& data);
//   virtual void sendRadarCmdCallback(const radar_interfaces::msg::RadarInfoConstPtr& data);
//   virtual void sendSentryStateCallback(const std_msgs::StringConstPtr& data);

  // send  ui
  void sendSerialDataCallback();
  void sendQueue();

//   rclcpp::Subscriber joint_state_sub_;
//   rclcpp::Subscriber actuator_state_sub_;
//   rclcpp::Subscriber dbus_sub_;
//   rclcpp::Subscriber chassis_cmd_sub_;
//   rclcpp::Subscriber vel2D_cmd_sub_;
//   rclcpp::Subscriber shoot_state_sub_;
//   rclcpp::Subscriber gimbal_cmd_sub_;
//   rclcpp::Subscriber detection_status_sub_;
//   rclcpp::Subscriber card_cmd_sub_;
//   rclcpp::Subscriber calibration_status_sub_;
//   rclcpp::Subscriber engineer_cmd_sub_;
//   rclcpp::Subscriber radar_date_sub_;
//   rclcpp::Subscriber manual_data_sub_;
//   rclcpp::Subscriber camera_name_sub_;
//   rclcpp::Subscriber track_sub_;
//   rclcpp::Subscriber balance_state_sub_;
//   rclcpp::Subscriber radar_receive_sub_;
//   rclcpp::Subscriber map_sentry_sub_;
//   rclcpp::Subscriber radar_to_sentry_sub_;
//   rclcpp::Subscriber sentry_cmd_sub_;
//   rclcpp::Subscriber radar_cmd_sub_;
//   rclcpp::Subscriber sentry_state_sub_;

//   ChassisTriggerChangeUi* chassis_trigger_change_ui_{};
//   ShooterTriggerChangeUi* shooter_trigger_change_ui_{};
//   GimbalTriggerChangeUi* gimbal_trigger_change_ui_{};
//   TargetTriggerChangeUi* target_trigger_change_ui_{};
//   TargetViewAngleTriggerChangeUi* target_view_angle_trigger_change_ui_{};
//   CameraTriggerChangeUi* camera_trigger_change_ui_{};
//   BulletTimeChangeUi* bullet_time_change_ui_{};

//   CapacitorTimeChangeUi* capacitor_time_change_ui_{};
//   EffortTimeChangeUi* effort_time_change_ui_{};
//   ProgressTimeChangeUi* progress_time_change_ui_{};
//   DartStatusTimeChangeUi* dart_status_time_change_ui_{};
//   RotationTimeChangeUi* rotation_time_change_ui_{};
//   LaneLineTimeChangeGroupUi* lane_line_time_change_ui_{};
//   BalancePitchTimeChangeGroupUi* balance_pitch_time_change_group_ui_{};
//   PitchAngleTimeChangeUi* pitch_angle_time_change_ui_{};
//   JointPositionTimeChangeUi *engineer_joint1_time_change_ui{}, *engineer_joint2_time_change_ui{},
//       *engineer_joint3_time_change_ui{};
//   TargetDistanceTimeChangeUi* target_distance_time_change_ui_{};
//   StringTriggerChangeUi *step_name_trigger_change_ui_{}, *servo_mode_trigger_change_ui_{},
//       *reversal_state_trigger_change_ui_{}, *stone_num_trigger_change_ui_{}, *joint_temperature_trigger_change_ui_{},
//       *drag_state_trigger_change_ui_{}, *gripper_state_trigger_change_ui_{};

//   FixedUi* fixed_ui_{};

//   CoverFlashUi* cover_flash_ui_{};
//   SpinFlashUi* spin_flash_ui_{};
//   HeroHitFlashUi* hero_hit_flash_ui_{};

//   InteractiveSender* interactive_data_sender_{};
//   InteractiveSender* enemy_hero_state_sender_{};
//   InteractiveSender* sentry_state_sender_{};

//   GroupUiBase* graph_queue_sender_{};
//   std::deque<Graph> graph_queue_;
//   std::deque<Graph> character_queue_;

//   rclcpp::Time radar_interactive_data_last_send_;
//   rclcpp::Time sentry_interactive_data_last_send_;
//   rclcpp::Time sentry_cmd_data_last_send_, radar_cmd_data_last_send_;

//   Base& base_;
//   rclcpp::Timer add_ui_timer_, send_serial_data_timer_;
//   int add_ui_times_, add_ui_max_times_, add_ui_frequency_;
//   double send_ui_queue_delay_;
//   bool add_ui_flag_ = false, is_adding_ = false;
//   rclcpp::NodeHandle nh_;
//   std::string dbus_topic_;
};
}  // namespace rm_referee
