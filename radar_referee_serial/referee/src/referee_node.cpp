/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by peter on 2021/5/17.
//
#include "radar_referee/referee_node.hpp"

namespace radar_referee
{
// read data from referee
void RefereeNode::read()
{
  if (!base_.serial_.available())
  {
    return;
  }

  rx_len_ = static_cast<int>(base_.serial_.available());
  base_.serial_.read(rx_buffer_, rx_len_);

  uint8_t temp_buffer[256] = { 0 };
  int frame_len;
  if ((this->now() - last_get_data_time_).seconds() > 0.1)
    base_.referee_data_is_online_ = false;
  if (rx_len_ < k_unpack_buffer_length_)
  {
    for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
      temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_];
    for (int k_i = 0; k_i < rx_len_; ++k_i)
      temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i];
    for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
      unpack_buffer_[k_i] = temp_buffer[k_i];
  }
  for (int k_i = 0; k_i < k_unpack_buffer_length_ - k_frame_length_; ++k_i)
  {
    if (unpack_buffer_[k_i] == 0xA5)
    {
      frame_len = unpack(&unpack_buffer_[k_i]);
      if (frame_len != -1)
        k_i += frame_len;
    }
  }
  getRobotInfo();
  clearRxBuffer();
}

int RefereeNode::unpack(uint8_t* rx_data)
{
  uint16_t cmd_id;
  int frame_len;
  radar_referee::FrameHeader frame_header;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (static_cast<bool>(base_.verifyCRC8CheckSum(rx_data, k_header_length_)))
  {
    if (frame_header.data_length > 256)  // temporary and inaccurate value
    {
      RCLCPP_INFO(this->get_logger(), "discard possible wrong frames, data length: %d", frame_header.data_length);
      return 0;
    }
    frame_len = frame_header.data_length + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
    if (base_.verifyCRC16CheckSum(rx_data, frame_len) == 1)
    {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);   
      switch (cmd_id)
      {
        case radar_referee::RefereeCmdId::GAME_STATUS_CMD:
        {
          radar_referee::GameStatus game_status_ref;
          radar_interfaces::msg::GameStatus game_status_data;
          memcpy(&game_status_ref, rx_data + 7, sizeof(radar_referee::GameStatus));

          game_status_data.game_type = game_status_ref.game_type;
          game_status_data.game_progress = game_status_ref.game_progress;
          game_status_data.stage_remain_time = game_status_ref.stage_remain_time;
          game_status_data.sync_time_stamp = game_status_ref.sync_time_stamp;
          game_status_data.header.stamp = last_get_data_time_;

          // referee_comm_->gameStatusDataCallBack(game_status_data, last_get_data_time_);
          game_status_pub_->publish(game_status_data);
          break;
        }
        case radar_referee::RefereeCmdId::GAME_RESULT_CMD:
        {
          radar_referee::GameResult game_result_ref;
          memcpy(&game_result_ref, rx_data + 7, sizeof(radar_referee::GameResult));
          break;
        }
        case radar_referee::RefereeCmdId::GAME_ROBOT_HP_CMD:
        {
          radar_referee::GameRobotHp game_robot_hp_ref;
          radar_interfaces::msg::GameRobotHp game_robot_hp_data;
          memcpy(&game_robot_hp_ref, rx_data + 7, sizeof(radar_referee::GameRobotHp));

          game_robot_hp_data.blue_1_robot_hp = game_robot_hp_ref.blue_1_robot_HP;
          game_robot_hp_data.blue_2_robot_hp = game_robot_hp_ref.blue_2_robot_HP;
          game_robot_hp_data.blue_3_robot_hp = game_robot_hp_ref.blue_3_robot_HP;
          game_robot_hp_data.blue_4_robot_hp = game_robot_hp_ref.blue_4_robot_HP;
          game_robot_hp_data.blue_5_robot_hp = game_robot_hp_ref.blue_5_robot_HP;
          game_robot_hp_data.blue_7_robot_hp = game_robot_hp_ref.blue_7_robot_HP;
          game_robot_hp_data.blue_outpost_hp = game_robot_hp_ref.blue_outpost_HP;
          game_robot_hp_data.blue_base_hp = game_robot_hp_ref.blue_base_HP;
          game_robot_hp_data.red_1_robot_hp = game_robot_hp_ref.red_1_robot_HP;
          game_robot_hp_data.red_2_robot_hp = game_robot_hp_ref.red_2_robot_HP;
          game_robot_hp_data.red_3_robot_hp = game_robot_hp_ref.red_3_robot_HP;
          game_robot_hp_data.red_4_robot_hp = game_robot_hp_ref.red_4_robot_HP;
          game_robot_hp_data.red_5_robot_hp = game_robot_hp_ref.red_5_robot_HP;
          game_robot_hp_data.red_7_robot_hp = game_robot_hp_ref.red_7_robot_HP;
          game_robot_hp_data.red_outpost_hp = game_robot_hp_ref.red_outpost_HP;
          game_robot_hp_data.red_base_hp = game_robot_hp_ref.red_base_HP;
          game_robot_hp_data.header.stamp = last_get_data_time_;

          // referee_comm_->updateEnemyHeroState(game_robot_hp_data, last_get_data_time_);
          // referee_comm_->heroHpDataCallBack(game_robot_hp_data, last_get_data_time_);
          game_robot_hp_pub_->publish(game_robot_hp_data);
          break;
        }
        case radar_referee::RefereeCmdId::DART_STATUS_CMD:
        {
          radar_referee::DartStatus dart_status_ref;
          radar_interfaces::msg::DartStatus dart_status_data;
          memcpy(&dart_status_ref, rx_data + 7, sizeof(radar_referee::DartStatus));

          dart_status_data.dart_belong = dart_status_ref.dart_belong;
          dart_status_data.stage_remaining_time = dart_status_ref.stage_remaining_time;
          dart_status_data.header.stamp = last_get_data_time_;

          dart_status_pub_->publish(dart_status_data);
          break;
        }
        case radar_referee::RefereeCmdId::ICRA_ZONE_STATUS_CMD:
        {
          radar_referee::IcraBuffDebuffZoneStatus icra_buff_debuff_zone_status_ref;
          radar_interfaces::msg::IcraBuffDebuffZoneStatus icra_buff_debuff_zone_status_data;
          memcpy(&icra_buff_debuff_zone_status_ref, rx_data + 7, sizeof(radar_referee::IcraBuffDebuffZoneStatus));

          icra_buff_debuff_zone_status_data.blue_1_bullet_left = icra_buff_debuff_zone_status_ref.blue_1_bullet_left;
          icra_buff_debuff_zone_status_data.blue_2_bullet_left = icra_buff_debuff_zone_status_ref.blue_2_bullet_left;
          icra_buff_debuff_zone_status_data.red_1_bullet_left = icra_buff_debuff_zone_status_ref.red_1_bullet_left;
          icra_buff_debuff_zone_status_data.red_2_bullet_left = icra_buff_debuff_zone_status_ref.red_2_bullet_left;
          icra_buff_debuff_zone_status_data.f_1_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_1_zone_buff_debuff_status;
          icra_buff_debuff_zone_status_data.f_1_zone_status = icra_buff_debuff_zone_status_ref.f_1_zone_status;
          icra_buff_debuff_zone_status_data.f_2_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_2_zone_buff_debuff_status;
          icra_buff_debuff_zone_status_data.f_2_zone_status = icra_buff_debuff_zone_status_ref.f_2_zone_status;
          icra_buff_debuff_zone_status_data.f_3_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_3_zone_buff_debuff_status;
          icra_buff_debuff_zone_status_data.f_3_zone_status = icra_buff_debuff_zone_status_ref.f_3_zone_status;
          icra_buff_debuff_zone_status_data.f_4_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_4_zone_buff_debuff_status;
          icra_buff_debuff_zone_status_data.f_4_zone_status = icra_buff_debuff_zone_status_ref.f_4_zone_status;
          icra_buff_debuff_zone_status_data.f_5_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_5_zone_buff_debuff_status;
          icra_buff_debuff_zone_status_data.f_5_zone_status = icra_buff_debuff_zone_status_ref.f_5_zone_status;
          icra_buff_debuff_zone_status_data.f_6_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_6_zone_buff_debuff_status;
          icra_buff_debuff_zone_status_data.f_6_zone_status = icra_buff_debuff_zone_status_ref.f_6_zone_status;
          icra_buff_debuff_zone_status_data.header.stamp = last_get_data_time_;

          icra_buff_debuff_zone_status_pub_->publish(icra_buff_debuff_zone_status_data);
          break;
        }
        case radar_referee::RefereeCmdId::FIELD_EVENTS_CMD:
        {
          radar_referee::EventData event_ref;
          radar_interfaces::msg::EventData event_data;
          memcpy(&event_ref, rx_data + 7, sizeof(radar_referee::EventData));

          event_data.event_data = event_ref.event_type;
          event_data.header.stamp = last_get_data_time_;

          event_data_pub_->publish(event_data);
          break;
        }
        case radar_referee::RefereeCmdId::SUPPLY_PROJECTILE_ACTION_CMD:
        {
          radar_referee::SupplyProjectileAction supply_projectile_action_ref;
          radar_interfaces::msg::SupplyProjectileAction supply_projectile_action_data;
          memcpy(&supply_projectile_action_ref, rx_data + 7, sizeof(radar_referee::SupplyProjectileAction));

          supply_projectile_action_data.reserved = supply_projectile_action_ref.reserved;
          supply_projectile_action_data.supply_projectile_num = supply_projectile_action_ref.supply_projectile_num;
          supply_projectile_action_data.supply_projectile_step = supply_projectile_action_ref.supply_projectile_step;
          supply_projectile_action_data.supply_robot_id = supply_projectile_action_ref.supply_robot_id;
          supply_projectile_action_data.header.stamp = last_get_data_time_;

          supply_projectile_action_pub_->publish(supply_projectile_action_data);
          break;
        }
        case radar_referee::RefereeCmdId::REFEREE_WARNING_CMD:
        {
          radar_referee::RefereeWarning referee_warning_ref;
          memcpy(&referee_warning_ref, rx_data + 7, sizeof(radar_referee::RefereeWarning));
          break;
        }
        case radar_referee::RefereeCmdId::DART_REMAINING_CMD:
        {
          radar_referee::DartRemainingTime dart_remaining_time_ref;
          radar_interfaces::msg::DartRemainingTime dart_remaining_time_data;
          memcpy(&dart_remaining_time_ref, rx_data + 7, sizeof(radar_referee::DartRemainingTime));

          dart_remaining_time_data.dart_remaining_time = dart_remaining_time_ref.dart_remaining_time;
          dart_remaining_time_data.dart_aim_state = dart_remaining_time_ref.dart_info;
          dart_remaining_time_data.header.stamp = last_get_data_time_;

          dart_remaining_time_pub_->publish(dart_remaining_time_data);
          break;
        }
        case radar_referee::RefereeCmdId::ROBOT_STATUS_CMD:
        {
          radar_referee::GameRobotStatus game_robot_status_ref;
          radar_interfaces::msg::GameRobotStatus game_robot_status_data;
          // pulse 7 is to jump the header and cmd_id
          memcpy(&game_robot_status_ref, rx_data + 7, sizeof(radar_referee::GameRobotStatus)); 

          game_robot_status_data.robot_id = game_robot_status_ref.robot_id;
          game_robot_status_data.robot_level = game_robot_status_ref.robot_level;
          game_robot_status_data.remain_hp = game_robot_status_ref.remain_hp;
          game_robot_status_data.max_hp = game_robot_status_ref.max_hp;
          game_robot_status_data.shooter_cooling_rate = game_robot_status_ref.shooter_cooling_rate;
          game_robot_status_data.shooter_cooling_limit = game_robot_status_ref.shooter_cooling_limit;
          game_robot_status_data.chassis_power_limit = game_robot_status_ref.chassis_power_limit;
          game_robot_status_data.mains_power_gimbal_output = game_robot_status_ref.mains_power_gimbal_output;
          game_robot_status_data.mains_power_chassis_output = game_robot_status_ref.mains_power_chassis_output;
          game_robot_status_data.mains_power_shooter_output = game_robot_status_ref.mains_power_shooter_output;
          base_.robot_id_ = game_robot_status_ref.robot_id;
          game_robot_status_data.header.stamp = last_get_data_time_;
          // referee_comm_->robotStatusDataCallBack(game_robot_status_data, last_get_data_time_);
          // RCLCPP_INFO(this->get_logger(), "base_id_: %d", game_robot_status_data);
          game_robot_status_pub_->publish(game_robot_status_data);
          break;
        }
        case radar_referee::RefereeCmdId::POWER_HEAT_DATA_CMD:
        {
          radar_referee::PowerHeatData power_heat_ref;
          radar_interfaces::msg::PowerHeatData power_heat_data;
          memcpy(&power_heat_ref, rx_data + 7, sizeof(radar_referee::PowerHeatData));

          power_heat_data.chassis_power_buffer = power_heat_ref.chassis_power_buffer;
          power_heat_data.chassis_power = power_heat_ref.chassis_power;
          power_heat_data.shooter_id_1_17_mm_cooling_heat = power_heat_ref.shooter_id_1_17_mm_cooling_heat;
          power_heat_data.shooter_id_2_17_mm_cooling_heat = power_heat_ref.shooter_id_2_17_mm_cooling_heat;
          power_heat_data.shooter_id_1_42_mm_cooling_heat = power_heat_ref.shooter_id_1_42_mm_cooling_heat;
          power_heat_data.chassis_volt = static_cast<uint16_t>(power_heat_ref.chassis_volt * 0.001);        // mV->V
          power_heat_data.chassis_current = static_cast<uint16_t>(power_heat_ref.chassis_current * 0.001);  // mA->A

          power_heat_data.header.stamp = last_get_data_time_;

          power_heat_data_pub_->publish(power_heat_data);
          break;
        }
        case radar_referee::RefereeCmdId::ROBOT_POS_CMD:
        {
          radar_referee::GameRobotPos game_robot_pos_ref;
          radar_interfaces::msg::GameRobotPosData game_robot_pos_data;
          memcpy(&game_robot_pos_ref, rx_data + 7, sizeof(radar_referee::GameRobotPos));

          game_robot_pos_data.x = game_robot_pos_ref.x;
          game_robot_pos_data.y = game_robot_pos_ref.y;
          game_robot_pos_data.yaw = game_robot_pos_ref.yaw;
          game_robot_pos_pub_->publish(game_robot_pos_data);
          break;
        }
        case radar_referee::RefereeCmdId::BUFF_CMD:
        {
          radar_referee::Buff referee_buff;
          memcpy(&referee_buff, rx_data + 7, sizeof(radar_referee::Buff));
          break;
        }
        case radar_referee::RefereeCmdId::AERIAL_ROBOT_ENERGY_CMD:
        {
          radar_referee::AerialRobotEnergy aerial_robot_energy_ref;
          memcpy(&aerial_robot_energy_ref, rx_data + 7, sizeof(radar_referee::AerialRobotEnergy));
          break;
        }
        case radar_referee::RefereeCmdId::ROBOT_HURT_CMD:
        {
          radar_referee::RobotHurt robot_hurt_ref;
          radar_interfaces::msg::RobotHurt robot_hurt_data;
          memcpy(&robot_hurt_ref, rx_data + 7, sizeof(radar_referee::RobotHurt));

          robot_hurt_data.armor_id = robot_hurt_ref.armor_id;
          robot_hurt_data.hurt_type = robot_hurt_ref.hurt_type;
          robot_hurt_data.header.stamp = last_get_data_time_;

          // referee_comm_->robotHurtDataCallBack(robot_hurt_data, last_get_data_time_);

          robot_hurt_pub_->publish(robot_hurt_data);
          break;
        }
        case radar_referee::RefereeCmdId::SHOOT_DATA_CMD:
        {
          radar_referee::ShootData shoot_data_ref;
          radar_interfaces::msg::ShootData shoot_data;

          memcpy(&shoot_data_ref, rx_data + 7, sizeof(radar_referee::ShootData));

          shoot_data.bullet_freq = shoot_data_ref.bullet_freq;
          shoot_data.bullet_speed = shoot_data_ref.bullet_speed;
          shoot_data.bullet_type = shoot_data_ref.bullet_type;
          shoot_data.shooter_id = shoot_data_ref.shooter_id;
          shoot_data.header.stamp = last_get_data_time_;

          shoot_data_pub_->publish(shoot_data);
          break;
        }
        case radar_referee::RefereeCmdId::BULLET_REMAINING_CMD:
        {
          radar_referee::BulletAllowance bullet_allowance_ref;
          radar_interfaces::msg::BulletAllowance bullet_allowance_data;
          memcpy(&bullet_allowance_ref, rx_data + 7, sizeof(radar_referee::BulletAllowance));

          bullet_allowance_data.bullet_allowance_num_17_mm = bullet_allowance_ref.bullet_allowance_num_17_mm;
          bullet_allowance_data.bullet_allowance_num_42_mm = bullet_allowance_ref.bullet_allowance_num_42_mm;
          bullet_allowance_data.coin_remaining_num = bullet_allowance_ref.coin_remaining_num;
          bullet_allowance_data.header.stamp = last_get_data_time_;
          // referee_comm_->bulletRemainDataCallBack(bullet_allowance_data, last_get_data_time_);

          bullet_allowance_pub_->publish(bullet_allowance_data);
          break;
        }
        case radar_referee::RefereeCmdId::ROBOT_RFID_STATUS_CMD:
        {
          radar_referee::RfidStatus rfid_status_ref;
          radar_interfaces::msg::RfidStatus rfid_status_data;
          memcpy(&rfid_status_ref, rx_data + 7, sizeof(radar_referee::RfidStatus));

          rfid_status_data.rfid_status = rfid_status_ref.rfid_status;
          rfid_status_data.header.stamp = last_get_data_time_;

          rfid_status_pub_->publish(rfid_status_data);
          break;
        }
        case radar_referee::RefereeCmdId::DART_CLIENT_CMD:
        {
          radar_referee::DartClientCmd dart_client_cmd_ref;
          radar_interfaces::msg::DartClientCmd dart_client_cmd_data;
          memcpy(&dart_client_cmd_ref, rx_data + 7, sizeof(radar_referee::DartClientCmd));

          dart_client_cmd_data.dart_launch_opening_status = dart_client_cmd_ref.dart_launch_opening_status;
          // dart_client_cmd_data.first_dart_speed = dart_client_cmd_ref.first_dart_speed;
          // dart_client_cmd_data.second_dart_speed = dart_client_cmd_ref.second_dart_speed;
          // dart_client_cmd_data.third_dart_speed = dart_client_cmd_ref.third_dart_speed;
          // dart_client_cmd_data.fourth_dart_speed = dart_client_cmd_ref.fourth_dart_speed;
          dart_client_cmd_data.last_dart_launch_time = dart_client_cmd_ref.latest_launch_cmd_time;
          // dart_client_cmd_data.operate_launch_cmd_time = dart_client_cmd_ref.operate_launch_cmd_time;
          dart_client_cmd_data.target_change_time = dart_client_cmd_ref.target_change_time;
          // dart_client_cmd_data.header.stamp = last_get_data_time_;

          dart_client_cmd_pub_->publish(dart_client_cmd_data);
          break;
        }
        case radar_referee::ROBOTS_POS_CMD:
        {
          radar_referee::RobotsPositionData robots_position_ref;
          radar_interfaces::msg::RobotsPositionData robots_position_data;
          memcpy(&robots_position_ref, rx_data + 7, sizeof(radar_referee::RobotsPositionData));

          robots_position_data.engineer_x = robots_position_ref.engineer_x;
          robots_position_data.engineer_y = robots_position_ref.engineer_y;
          robots_position_data.hero_x = robots_position_ref.hero_x;
          robots_position_data.hero_y = robots_position_ref.hero_y;
          robots_position_data.standard_3_x = robots_position_ref.standard_3_x;
          robots_position_data.standard_3_y = robots_position_ref.standard_3_y;
          robots_position_data.standard_4_x = robots_position_ref.standard_4_x;
          robots_position_data.standard_4_y = robots_position_ref.standard_4_y;
          robots_position_data.standard_5_x = robots_position_ref.standard_5_x;
          robots_position_data.standard_5_y = robots_position_ref.standard_5_y;
          robots_position_data.header.stamp = last_get_data_time_;

          robots_position_pub_->publish(robots_position_data);
          break;
        }
        case radar_referee::RADAR_MARK_CMD:
        {
          radar_referee::RadarMarkData radar_mark_ref;
          radar_interfaces::msg::RadarMarkData radar_mark_data;
          memcpy(&radar_mark_ref, rx_data + 7, sizeof(radar_referee::RadarMarkData));

          radar_mark_data.mark_engineer_progress = radar_mark_ref.mark_engineer_progress;
          radar_mark_data.mark_hero_progress = radar_mark_ref.mark_hero_progress;
          radar_mark_data.mark_sentry_progress = radar_mark_ref.mark_sentry_progress;
          radar_mark_data.mark_standard_3_progress = radar_mark_ref.mark_standard_3_progress;
          radar_mark_data.mark_standard_4_progress = radar_mark_ref.mark_standard_4_progress;
          radar_mark_data.mark_standard_5_progress = radar_mark_ref.mark_standard_5_progress;
          radar_mark_data.header.stamp = last_get_data_time_;

          radar_mark_pub_->publish(radar_mark_data);
          break;
        }
        case radar_referee::RefereeCmdId::INTERACTIVE_DATA_CMD:
        {
          radar_referee::InteractiveData interactive_data_ref;  // local variable temporarily before moving referee data
          memcpy(&interactive_data_ref, rx_data + 7, sizeof(radar_referee::InteractiveData));
          // TODO: case cmd_id
          if (interactive_data_ref.header_data.data_cmd_id == radar_referee::DataCmdId::CURRENT_SENTRY_POSITION_CMD)
          {
            radar_referee::CurrentSentryPosData current_sentry_pos_ref;
            radar_interfaces::msg::CurrentSentryPosData current_sentry_pos_data;
            memcpy(&current_sentry_pos_ref, rx_data + 7, sizeof(radar_referee::CurrentSentryPosData));
            current_sentry_pos_data.x = current_sentry_pos_ref.position_x;
            current_sentry_pos_data.y = current_sentry_pos_ref.position_y;
            current_sentry_pos_data.z = current_sentry_pos_ref.position_z;
            current_sentry_pos_data.yaw = current_sentry_pos_ref.position_yaw;

            current_sentry_pos_pub_->publish(current_sentry_pos_data);
          }
          break;
        }
        case radar_referee::CLIENT_MAP_CMD:
        {
          radar_referee::ClientMapReceiveData client_map_receive_ref;
          radar_interfaces::msg::ClientMapReceiveData client_map_receive_data;
          memcpy(&client_map_receive_ref, rx_data + 7, sizeof(radar_referee::ClientMapReceiveData));

          client_map_receive_data.engineer_position_x = client_map_receive_ref.engineer_position_x;
          client_map_receive_data.engineer_position_y = client_map_receive_ref.engineer_position_y;
          client_map_receive_data.hero_position_x = client_map_receive_ref.hero_position_x;
          client_map_receive_data.hero_position_y = client_map_receive_ref.hero_position_y;
          client_map_receive_data.infantry_3_position_x = client_map_receive_ref.infantry_3_position_x;
          client_map_receive_data.infantry_3_position_y = client_map_receive_ref.infantry_3_position_y;
          client_map_receive_data.infantry_4_position_x = client_map_receive_ref.infantry_4_position_x;
          client_map_receive_data.infantry_4_position_y = client_map_receive_ref.infantry_4_position_y;
          client_map_receive_data.infantry_5_position_x = client_map_receive_ref.infantry_5_position_x;
          client_map_receive_data.infantry_5_position_y = client_map_receive_ref.infantry_5_position_y;
          client_map_receive_data.sentry_position_x = client_map_receive_ref.sentry_position_x;
          client_map_receive_data.sentry_position_y = client_map_receive_ref.sentry_position_y;

          client_map_receive_pub_->publish(client_map_receive_data);
          break;
        }
        case radar_referee::CUSTOM_INFO_CMD:
        {
          radar_referee::CustomInfo custom_info;
          memcpy(&custom_info, rx_data + 7, sizeof(radar_referee::CustomInfo));
          break;
        }
        case radar_referee::TARGET_POS_CMD:
        {
          radar_referee::ClientMapSendData client_map_send_data_ref;
          radar_interfaces::msg::ClientMapSendData client_map_send_data;
          memcpy(&client_map_send_data_ref, rx_data + 7, sizeof(radar_referee::ClientMapSendData));

          client_map_send_data.target_position_x = client_map_send_data_ref.target_position_x;
          client_map_send_data.target_position_y = client_map_send_data_ref.target_position_y;
          // client_map_send_data.target_position_z = client_map_send_data_ref.target_position_z;
          client_map_send_data.command_keyboard = client_map_send_data_ref.command_keyboard;
          client_map_send_data.target_robot_id = client_map_send_data_ref.target_robot_ID;
          client_map_send_data.cmd_source = client_map_send_data_ref.cmd_source;
          client_map_send_data.header.stamp = last_get_data_time_;

          client_map_send_data_pub_->publish(client_map_send_data);
          break;
        }
        case radar_referee::SENTRY_INFO_CMD:
        {
          radar_referee::SentryInfo sentry_info;
          memcpy(&sentry_info, rx_data + 7, sizeof(radar_referee::SentryInfo));
          break;
        }
        case radar_referee::RADAR_INFO_CMD:
        {
          radar_referee::RadarInfo radar_info;
          memcpy(&radar_info, rx_data + 7, sizeof(radar_referee::RadarInfo));
          radar_interfaces::msg::RadarInfo msg;
          msg.radar_info = radar_info.radar_info;
          radar_info_pub_->publish(msg);
          break;
        }
        case radar_referee::POWER_MANAGEMENT_SAMPLE_AND_STATUS_DATA_CMD:
        {
          radar_interfaces::msg::PowerManagementSampleAndStatusData sample_and_status_pub_data;
          uint8_t data[sizeof(radar_referee::PowerManagementSampleAndStatusData)];
          memcpy(&data, rx_data + 7, sizeof(radar_referee::PowerManagementSampleAndStatusData));
          sample_and_status_pub_data.chassis_power = (static_cast<uint16_t>((data[0] << 8) | data[1]) / 100.);
          sample_and_status_pub_data.chassis_expect_power = (static_cast<uint16_t>((data[2] << 8) | data[3]) / 100.);
          sample_and_status_pub_data.capacity_recent_charge_power =
              (static_cast<uint16_t>((data[4] << 8) | data[5]) / 100.);
          sample_and_status_pub_data.capacity_remain_charge =
              (static_cast<uint16_t>((data[6] << 8) | data[7]) / 10000.);
          sample_and_status_pub_data.capacity_expect_charge_power = static_cast<uint8_t>(data[8]);
          sample_and_status_pub_data.state_machine_running_state = base_.capacity_recent_mode_ =
              static_cast<uint8_t>(data[9] >> 4);
          sample_and_status_pub_data.power_management_protection_info = static_cast<uint8_t>((data[9] >> 2) & 0x03);
          sample_and_status_pub_data.power_management_topology = static_cast<uint8_t>(data[9] & 0x03);
          sample_and_status_pub_data.header.stamp = last_get_data_time_;

          // referee_comm_->capacityDataCallBack(sample_and_status_pub_data, last_get_data_time_);

          power_management_sample_and_status_data_pub_->publish(sample_and_status_pub_data);
          break;
        }
        case radar_referee::POWER_MANAGEMENT_INITIALIZATION_EXCEPTION_CMD:
        {
          radar_referee::PowerManagementInitializationExceptionData initialization_exception_ref;
          radar_interfaces::msg::PowerManagementInitializationExceptionData initialization_exception_pub_data;
          memcpy(&initialization_exception_ref, rx_data + 7,
                 sizeof(radar_referee::PowerManagementInitializationExceptionData));

          initialization_exception_pub_data.error_code = initialization_exception_ref.error_code;
          initialization_exception_pub_data.string = initialization_exception_ref.string;
          initialization_exception_pub_data.header.stamp = last_get_data_time_;
          power_management_initialization_exception_pub_->publish(initialization_exception_pub_data);
          break;
        }
        case radar_referee::POWER_MANAGEMENT_SYSTEM_EXCEPTION_CMD:
        {
          unsigned char* tmp_rx_data_ptr = rx_data + 7;
          radar_interfaces::msg::PowerManagementSystemExceptionData system_exception_pub_data;

          system_exception_pub_data.r0 =
              tmp_rx_data_ptr[0] << 24 | tmp_rx_data_ptr[1] << 16 | tmp_rx_data_ptr[2] << 8 | tmp_rx_data_ptr[3];
          system_exception_pub_data.r1 =
              tmp_rx_data_ptr[4] << 24 | tmp_rx_data_ptr[5] << 16 | tmp_rx_data_ptr[6] << 8 | tmp_rx_data_ptr[7];
          system_exception_pub_data.r2 =
              tmp_rx_data_ptr[8] << 24 | tmp_rx_data_ptr[9] << 16 | tmp_rx_data_ptr[10] << 8 | tmp_rx_data_ptr[11];
          system_exception_pub_data.r3 =
              tmp_rx_data_ptr[12] << 24 | tmp_rx_data_ptr[13] << 16 | tmp_rx_data_ptr[14] << 8 | tmp_rx_data_ptr[15];
          system_exception_pub_data.r12 =
              tmp_rx_data_ptr[16] << 24 | tmp_rx_data_ptr[17] << 16 | tmp_rx_data_ptr[18] << 8 | tmp_rx_data_ptr[19];
          system_exception_pub_data.lr =
              tmp_rx_data_ptr[20] << 24 | tmp_rx_data_ptr[21] << 16 | tmp_rx_data_ptr[22] << 8 | tmp_rx_data_ptr[23];
          system_exception_pub_data.pc =
              tmp_rx_data_ptr[24] << 24 | tmp_rx_data_ptr[25] << 16 | tmp_rx_data_ptr[26] << 8 | tmp_rx_data_ptr[27];
          system_exception_pub_data.psr =
              tmp_rx_data_ptr[28] << 24 | tmp_rx_data_ptr[29] << 16 | tmp_rx_data_ptr[30] << 8 | tmp_rx_data_ptr[31];
          system_exception_pub_data.header.stamp = last_get_data_time_;
          power_management_system_exception_data_->publish(system_exception_pub_data);
          break;
        }
        case radar_referee::POWER_MANAGEMENT_PROCESS_STACK_OVERFLOW_CMD:
        {
          radar_referee::PowerManagementProcessStackOverflowData stack_overflow_ref;
          radar_interfaces::msg::PowerManagementProcessStackOverflowData stack_overflow_pub_data;
          memcpy(&stack_overflow_ref, rx_data + 7, sizeof(radar_referee::PowerManagementProcessStackOverflowData));

          stack_overflow_pub_data.process_name = stack_overflow_ref.process_name;
          stack_overflow_pub_data.header.stamp = last_get_data_time_;
          power_management_process_stack_overflow_pub_->publish(stack_overflow_pub_data);
          break;
        }
        case radar_referee::POWER_MANAGEMENT_UNKNOWN_EXCEPTION_CMD:
        {
          radar_referee::PowerManagementUnknownExceptionData unknown_exception_ref;
          radar_interfaces::msg::PowerManagementUnknownExceptionData unknown_exception_pub_data;
          memcpy(&unknown_exception_ref, rx_data + 7, sizeof(radar_referee::PowerManagementUnknownExceptionData));

          unknown_exception_pub_data.abnormal_reset_reason = unknown_exception_ref.abnormal_reset_reason;
          unknown_exception_pub_data.power_management_before_reset_topology =
              unknown_exception_ref.power_management_before_reset_topology;
          unknown_exception_pub_data.state_machine_before_reset_mode =
              unknown_exception_ref.state_machine_before_reset_mode;
          unknown_exception_pub_data.header.stamp = last_get_data_time_;
          power_management_unknown_exception_pub_->publish(unknown_exception_pub_data);
          break;
        }
        default:
          RCLCPP_WARN(this->get_logger(), "Referee command ID %d not found.", cmd_id);
          break;
      }
      base_.referee_data_is_online_ = true;
      last_get_data_time_ = this->now();
      return frame_len;
    }
  }
  return -1;
}

void RefereeNode::getRobotInfo()
{
  base_.robot_color_ = base_.robot_id_ >= 100 ? "blue" : "red";
  if (base_.robot_id_ != radar_referee::RobotId::BLUE_SENTRY && base_.robot_id_ != radar_referee::RobotId::RED_SENTRY)
  {
    switch (base_.robot_id_)
    {
      case radar_referee::RobotId::BLUE_HERO:
        base_.client_id_ = radar_referee::ClientId::BLUE_HERO_CLIENT;
        break;
      case radar_referee::RobotId::BLUE_ENGINEER:
        base_.client_id_ = radar_referee::ClientId::BLUE_ENGINEER_CLIENT;
        break;
      case radar_referee::RobotId::BLUE_STANDARD_3:
        base_.client_id_ = radar_referee::ClientId::BLUE_STANDARD_3_CLIENT;
        break;
      case radar_referee::RobotId::BLUE_STANDARD_4:
        base_.client_id_ = radar_referee::ClientId::BLUE_STANDARD_4_CLIENT;
        break;
      case radar_referee::RobotId::BLUE_STANDARD_5:
        base_.client_id_ = radar_referee::ClientId::BLUE_STANDARD_5_CLIENT;
        break;
      case radar_referee::RobotId::BLUE_AERIAL:
        base_.client_id_ = radar_referee::ClientId::BLUE_AERIAL_CLIENT;
        break;
      case radar_referee::RobotId::RED_HERO:
        base_.client_id_ = radar_referee::ClientId::RED_HERO_CLIENT;
        break;
      case radar_referee::RobotId::RED_ENGINEER:
        base_.client_id_ = radar_referee::ClientId::RED_ENGINEER_CLIENT;
        break;
      case radar_referee::RobotId::RED_STANDARD_3:
        base_.client_id_ = radar_referee::ClientId::RED_STANDARD_3_CLIENT;
        break;
      case radar_referee::RobotId::RED_STANDARD_4:
        base_.client_id_ = radar_referee::ClientId::RED_STANDARD_4_CLIENT;
        break;
      case radar_referee::RobotId::RED_STANDARD_5:
        base_.client_id_ = radar_referee::ClientId::RED_STANDARD_5_CLIENT;
        break;
      case radar_referee::RobotId::RED_AERIAL:
        base_.client_id_ = radar_referee::ClientId::RED_AERIAL_CLIENT;
        break;
    }
  }
}
}  // namespace radar_referee
