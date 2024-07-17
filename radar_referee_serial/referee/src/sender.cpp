#include "radar_referee/common/sender.hpp"

namespace radar_referee
{

void Sender::sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data)
{
  uint8_t tx_data[sizeof(InteractiveData)] = { 0 };
  auto student_interactive_data = (InteractiveData*)tx_data;

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  student_interactive_data->header_data.data_cmd_id = data_cmd_id;
  student_interactive_data->header_data.sender_id = base_.robot_id_;
  student_interactive_data->header_data.receiver_id = receiver_id;
  student_interactive_data->data = data;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(InteractiveData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(InteractiveData) + k_tail_length_);

  sendSerial(clock_->now(), sizeof(InteractiveData));
}

// void Sender::sendCurrentSentryData(const radar_interfaces::CurrentSentryPosData:: data)
// {
//   int data_len;
//   uint8_t tx_data[sizeof(CurrentSentryPosData)] = { 0 };
//   auto current_sentry_pos_data = (CurrentSentryPosData*)tx_data;
//   data_len = static_cast<int>(sizeof(radar_interfaces::CurrentSentryPosData));

//   for (int i = 0; i < 128; i++)
//     tx_buffer_[i] = 0;

//   current_sentry_pos_data->header_data.data_cmd_id = DataCmdId::CURRENT_SENTRY_POSITION_CMD;
//   current_sentry_pos_data->header_data.sender_id = base_.robot_id_;
//   current_sentry_pos_data->header_data.receiver_id = base_.robot_id_ < 100 ? RobotId::RED_SENTRY :
//   RobotId::BLUE_SENTRY; current_sentry_pos_data->position_x = data->x; current_sentry_pos_data->position_y = data->y;
//   current_sentry_pos_data->position_z = data->z;
//   current_sentry_pos_data->position_yaw = data->yaw;

//   pack(tx_buffer_, tx_data, radar_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(InteractiveData));
//   sendSerial(clock_->now(), data_len);
// }

void Sender::sendMapSentryData(const radar_referee::MapSentryData& data)
{
  uint8_t tx_data[sizeof(radar_referee::MapSentryData)] = { 0 };
  auto map_sentry_data = (radar_referee::MapSentryData*)tx_data;

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  map_sentry_data->intention = data.intention;
  map_sentry_data->start_position_x = data.start_position_x;
  map_sentry_data->start_position_y = data.start_position_y;
  for (int i = 0; i < 49; i++)
  {
    map_sentry_data->delta_x[i] = data.delta_x[i];
    map_sentry_data->delta_y[i] = data.delta_y[i];
  }
  map_sentry_data->sender_id = base_.robot_id_;
  pack(tx_buffer_, tx_data, radar_referee::RefereeCmdId::MAP_SENTRY_CMD, sizeof(radar_referee::MapSentryData));
  tx_len_ =
      k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(radar_referee::MapSentryData) + k_tail_length_);

  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (...)
  {
  }

  clearTxBuffer();
}

void Sender::sendCustomInfoData(std::wstring data)
{
  uint8_t tx_data[sizeof(radar_referee::CustomInfo)] = { 0 };
  auto custom_info = (radar_referee::CustomInfo*)tx_data;
  uint16_t characters[15];
  for (int i = 0; i < 15; i++)
  {
    if (i < static_cast<int>(data.size()))
      characters[i] = static_cast<uint16_t>(data[i]);
  }
  for (int i = 0; i < 15; i++)
  {
    custom_info->user_data[2 * i] = characters[i] & 0xFF;
    custom_info->user_data[2 * i + 1] = (characters[i] >> 8) & 0xFF;
  }
  custom_info->sender_id = base_.robot_id_;
  custom_info->receiver_id = base_.client_id_;
  pack(tx_buffer_, tx_data, radar_referee::RefereeCmdId::CUSTOM_INFO_CMD, sizeof(radar_referee::CustomInfo));
  tx_len_ = k_header_length_ + k_cmd_id_length_ +
            static_cast<int>(sizeof(radar_referee::ClientMapReceiveData) + k_tail_length_);

  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
    std::cout<<"Port not open: "<<e.what()<<std::endl;
  }
  clearTxBuffer();
}

void Sender::sendRadarInteractiveData(const radar_referee::ClientMapReceiveData& data)
{
  uint8_t tx_data[sizeof(radar_referee::ClientMapReceiveData)] = { 0 };
  auto radar_interactive_data = (radar_referee::ClientMapReceiveData*)tx_data;

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;

  radar_interactive_data->engineer_position_x = data.engineer_position_x;
  radar_interactive_data->engineer_position_y = data.engineer_position_y;
  radar_interactive_data->hero_position_x = data.hero_position_x;
  radar_interactive_data->hero_position_y = data.hero_position_y;
  radar_interactive_data->infantry_3_position_x = data.infantry_3_position_x;
  radar_interactive_data->infantry_3_position_y = data.infantry_3_position_y;
  radar_interactive_data->infantry_4_position_x = data.infantry_4_position_x;
  radar_interactive_data->infantry_4_position_y = data.infantry_4_position_y;
  radar_interactive_data->infantry_5_position_x = data.infantry_5_position_x;
  radar_interactive_data->infantry_5_position_y = data.infantry_5_position_y;
  radar_interactive_data->sentry_position_x = data.sentry_position_x;
  radar_interactive_data->sentry_position_y = data.sentry_position_y;
  // radar_interactive_data->target_robot_ID = data.target_robot_ID;
  // radar_interactive_data->target_position_x = data.target_position_x;
  // radar_interactive_data->target_position_y = data.target_position_y;
  pack(tx_buffer_, tx_data, radar_referee::RefereeCmdId::CLIENT_MAP_CMD, sizeof(radar_referee::ClientMapReceiveData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ +
            static_cast<int>(sizeof(radar_referee::ClientMapReceiveData) + k_tail_length_);
  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
    std::cout<<"Port not open: "<<e.what()<<std::endl;
  }
  clearTxBuffer();
}

void Sender::pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const
{
  memset(tx_buffer, 0, k_frame_length_);
  auto* frame_header = reinterpret_cast<FrameHeader*>(tx_buffer);

  frame_header->sof = 0xA5;
  frame_header->data_length = len;
  memcpy(&tx_buffer[k_header_length_], reinterpret_cast<uint8_t*>(&cmd_id), k_cmd_id_length_);
  base_.appendCRC8CheckSum(tx_buffer, k_header_length_);
  memcpy(&tx_buffer[k_header_length_ + k_cmd_id_length_], data, len);
  base_.appendCRC16CheckSum(tx_buffer, k_header_length_ + k_cmd_id_length_ + len + k_tail_length_);
}

void Sender::sendSerial(const rclcpp::Time& time, int data_len)
{
  tx_len_ = k_header_length_ + k_cmd_id_length_ + k_tail_length_ + data_len;
  last_send_ = time;
  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
  }
  clearTxBuffer();
}

void Sender::clearTxBuffer()
{
  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  tx_len_ = 0;
}
}  // namespace radar_referee
