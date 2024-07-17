//
// Created by ljq on 2022/5/17.
//

#include "radar_referee/referee_comm.hpp"
#include <rclcpp/logging.hpp>

namespace radar_referee
{
RefereeComm::RefereeComm(rclcpp::Node::SharedPtr nh, SerialBase& base) : base_(base), nh_(nh)
{
  auto node = nh;
  clock_ = node->get_clock();
  radar_receive_sub_ = node->create_subscription<radar_interfaces::msg::ClientMapReceiveData>(
      "/rm_radar", 10, std::bind(&RefereeComm::radarReceiveCallback, this, std::placeholders::_1));
  manual_sub_ = node->create_subscription<radar_interfaces::msg::ManualToReferee>(
      "/manual_to_referee", 10, std::bind(&RefereeComm::manualDataCallBack, this, std::placeholders::_1));
  // sentry_deviate_sub_ = node->create_subscription<radar_interfaces::msg::SentryDeviate>(
  // "/deviate", 10, std::bind(&RefereeComm::sentryDeviateCallback, this, std::placeholders::_1));
  // radar_to_sentry_sub_ = node->create_subscription<radar_interfaces::msg::CurrentSentryPosData>(
  // "/radar_to_sentry", 10, std::bind(&RefereeComm::sendCurrentSentryCallback, this, std::placeholders::_1));
  //"/bullet_allowance_data"

  interactive_data_sender_ = new Sender(clock_, base_);
  RCLCPP_INFO(node->get_logger(), "RefereeComm initialized successfully!");
}

void RefereeComm::robotStatusDataCallBack(const radar_interfaces::msg::GameRobotStatus& data,
                                          const rclcpp::Time& last_get_data_time)
{
  (void)data;
  (void)last_get_data_time;
}

void RefereeComm::updateEnemyHeroState(const radar_interfaces::msg::GameRobotHp& game_robot_hp_data,
                                       const rclcpp::Time& last_get_data_time)
{
  std::wstring data;
  try
  {
    if (base_.robot_id_ < 100 && base_.robot_id_ != RED_SENTRY)
    {
      if (game_robot_hp_data.blue_1_robot_hp > 0)
        data = L"敌英雄存活:" + std::to_wstring(game_robot_hp_data.blue_1_robot_hp);
      else
        data = L"敌英雄死亡";
    }
    else if (base_.robot_id_ >= 100 && base_.robot_id_ != BLUE_SENTRY)
    {
      if (game_robot_hp_data.red_1_robot_hp > 0)
        data = L"敌英雄存活:" + std::to_wstring(game_robot_hp_data.red_1_robot_hp);
      else
        data = L"敌英雄死亡";
    }
    else
      return;
    std::wcout << L"EnemyHeroState: " << data << std::endl;
    (void)last_get_data_time;
    interactive_data_sender_->sendCustomInfoData(data);
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ashdk"), "Error in updateEnemyHeroState, %s", __FILE__);
  }
}

void RefereeComm::heroHpDataCallBack(const radar_interfaces::msg::GameRobotHp& game_robot_hp_data,
                                     const rclcpp::Time& last_get_data_time)
{
  (void)game_robot_hp_data;
  (void)last_get_data_time;
}
void RefereeComm::gameStatusDataCallBack(const radar_interfaces::msg::GameStatus& data,
                                         const rclcpp::Time& last_get_data_time)
{
  (void)data;
  (void)last_get_data_time;
}
void RefereeComm::capacityDataCallBack(const radar_interfaces::msg::PowerManagementSampleAndStatusData& data,
                                       rclcpp::Time& last_get_data_time)
{
  (void)data;
  (void)last_get_data_time;
}

void RefereeComm::manualDataCallBack(const radar_interfaces::msg::ManualToReferee::ConstSharedPtr data)
{
  (void)data;
}

void RefereeComm::powerHeatDataCallBack(const radar_interfaces::msg::PowerHeatData& data,
                                        const rclcpp::Time& last_get_data_time)
{
  (void)data;
  (void)last_get_data_time;
}
void RefereeComm::robotHurtDataCallBack(const radar_interfaces::msg::RobotHurt& data,
                                        const rclcpp::Time& last_get_data_time)
{
  (void)data;
  (void)last_get_data_time;
}
void RefereeComm::bulletRemainDataCallBack(const radar_interfaces::msg::BulletAllowance& bullet_allowance,
                                           const rclcpp::Time& last_get_data_time)
{
  (void)bullet_allowance;
  (void)last_get_data_time;
  // if (bullet_time_change_ui_ && !is_adding_)
  //   bullet_time_change_ui_->updateBulletData(bullet_allowance, last_get_data_time);
}
void RefereeComm::interactiveDataCallBack(const radar_referee::InteractiveData& data,
                                          const rclcpp::Time& last_get_data_time)
{
  (void)data;
  (void)last_get_data_time;
}
void RefereeComm::eventDataCallBack(const radar_interfaces::msg::EventData& data, const rclcpp::Time& last_get_data_time)
{
  (void)data;
  (void)last_get_data_time;
}

void RefereeComm::radarDataCallBack(const radar_interfaces::msg::TargetInfoArray data)
{
  (void)data;
  // if (radar_mark_ui_ && !is_adding_)
  //   radar_mark_ui_->updateRadarData(data);
}

// void RefereeComm::sentryDeviateCallback(const radar_interfaces::msg::SentryDeviateConstPtr& data)
// {
// }

void RefereeComm::radarReceiveCallback(const radar_interfaces::msg::ClientMapReceiveData::ConstSharedPtr data)
{
  
  radar_referee::ClientMapReceiveData radar_receive_data;
  radar_receive_data.hero_position_x = data->hero_position_x;
  radar_receive_data.hero_position_y = data->hero_position_y;
  radar_receive_data.engineer_position_x = data->engineer_position_x;
  radar_receive_data.engineer_position_y = data->engineer_position_y;
  radar_receive_data.infantry_3_position_x = data->infantry_3_position_x;
  radar_receive_data.infantry_3_position_y = data->infantry_3_position_y;
  radar_receive_data.infantry_4_position_x = data->infantry_4_position_x;
  radar_receive_data.infantry_4_position_y = data->infantry_4_position_y;
  radar_receive_data.infantry_5_position_x = data->infantry_5_position_x;
  radar_receive_data.infantry_5_position_y = data->infantry_5_position_y;
  radar_receive_data.sentry_position_x = data->sentry_position_x;
  radar_receive_data.sentry_position_y = data->sentry_position_y;

  if ((clock_->now() - radar_interactive_data_last_send_).seconds() <= 1.0 / 100)
  {
    RCLCPP_WARN(rclcpp::get_logger("radar_referee"), "radarReceiveCallback: time interval too short: %f hz",
                1.0 / (clock_->now() - radar_interactive_data_last_send_).seconds());
    return;
  }
  else
  {
    interactive_data_sender_->sendRadarInteractiveData(radar_receive_data);
    radar_interactive_data_last_send_ = clock_->now();
  }
}
// void RefereeComm::mapSentryCallback(const radar_interfaces::msg::MapSentryDataConstPtr& data)
// {
//   radar_referee::MapSentryData map_sentry_data;
//   map_sentry_data.intention = data->intention;
//   map_sentry_data.start_position_x = data->start_position_x;
//   map_sentry_data.start_position_y = data->start_position_y;
//   for (int i = 0; i < 49; i++)
//   {
//     map_sentry_data.delta_x[i] = data->delta_x[i];
//     map_sentry_data.delta_y[i] = data->delta_y[i];
//   }
//   if ((clock_->now() - sentry_interactive_data_last_send_).seconds() < 1.0)
//     return;
//   else
//   {
//     interactive_data_sender_->sendMapSentryData(map_sentry_data);
//     sentry_interactive_data_last_send_ = clock_->now();
//   }
// }

// void RefereeComm::sendCurrentSentryCallback(const radar_interfaces::msg::CurrentSentryPosDataConstPtr& data)
// {
//   interactive_data_sender_->sendCurrentSentryData(data);
// }

}  // namespace radar_referee
