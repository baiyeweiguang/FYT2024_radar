// Copyright (C) Qiayuan Liao.
// Copyright (C) FYT Vision Grpup. All rights reserved.

#ifndef RADAR_REFEREE_PROTOCOL_HPP_
#define RADAR_REFEREE_PROTOCOL_HPP_
#define __packed __attribute__((packed))

#include <cstdint>

namespace radar_referee
{
typedef enum
{
  GAME_STATUS_CMD = 0x0001,
  GAME_RESULT_CMD = 0x0002,
  GAME_ROBOT_HP_CMD = 0x0003,
  DART_STATUS_CMD = 0x0004,
  ICRA_ZONE_STATUS_CMD = 0x0005,
  FIELD_EVENTS_CMD = 0x0101,
  SUPPLY_PROJECTILE_ACTION_CMD = 0x0102,
  REFEREE_WARNING_CMD = 0x0104,
  DART_REMAINING_CMD = 0x0105,
  ROBOT_STATUS_CMD = 0x0201,
  POWER_HEAT_DATA_CMD = 0x0202,
  ROBOT_POS_CMD = 0x0203,
  BUFF_CMD = 0x0204,
  AERIAL_ROBOT_ENERGY_CMD = 0x0205,
  ROBOT_HURT_CMD = 0x0206,
  SHOOT_DATA_CMD = 0x0207,
  BULLET_REMAINING_CMD = 0x0208,
  ROBOT_RFID_STATUS_CMD = 0x0209,
  DART_CLIENT_CMD = 0x020A,
  ROBOTS_POS_CMD = 0X020B,
  RADAR_MARK_CMD = 0X020C,
  SENTRY_INFO_CMD = 0x020D,
  RADAR_INFO_CMD = 0x020E,
  INTERACTIVE_DATA_CMD = 0x0301,
  CUSTOM_CONTROLLER_CMD = 0x0302,  // controller
  TARGET_POS_CMD = 0x0303,         // send aerial->server
  ROBOT_COMMAND_CMD = 0x0304,      // controller
  CLIENT_MAP_CMD = 0x0305,
  CUSTOM_CLIENT_CMD = 0x0306,  // controller
  MAP_SENTRY_CMD = 0x0307,     // send sentry->aerial
  CUSTOM_INFO_CMD = 0x0308,
  POWER_MANAGEMENT_SAMPLE_AND_STATUS_DATA_CMD = 0X8301,
  POWER_MANAGEMENT_INITIALIZATION_EXCEPTION_CMD = 0X8302,
  POWER_MANAGEMENT_SYSTEM_EXCEPTION_CMD = 0X8303,
  POWER_MANAGEMENT_PROCESS_STACK_OVERFLOW_CMD = 0X8304,
  POWER_MANAGEMENT_UNKNOWN_EXCEPTION_CMD = 0X8305
} RefereeCmdId;

typedef enum
{
  ROBOT_INTERACTIVE_CMD_MIN = 0x0200,
  ROBOT_INTERACTIVE_CMD_MAX = 0x02FF,
  CLIENT_GRAPH_DELETE_CMD = 0x0100,
  CLIENT_GRAPH_SINGLE_CMD = 0x0101,
  CLIENT_GRAPH_DOUBLE_CMD = 0x0102,
  CLIENT_GRAPH_FIVE_CMD = 0x0103,
  CLIENT_GRAPH_SEVEN_CMD = 0x0104,
  CLIENT_CHARACTER_CMD = 0x0110,
  SENTRY_CMD = 0x0120,
  RADAR_CMD = 0x0121,
  CURRENT_SENTRY_POSITION_CMD = 0x0200  // send radar->sentry
} DataCmdId;

typedef enum
{
  RED_HERO = 1,
  RED_ENGINEER = 2,
  RED_STANDARD_3 = 3,
  RED_STANDARD_4 = 4,
  RED_STANDARD_5 = 5,
  RED_AERIAL = 6,
  RED_SENTRY = 7,
  RED_RADAR = 9,
  RED_OUTPOST = 10,
  RED_BASE = 11,
  BLUE_HERO = 101,
  BLUE_ENGINEER = 102,
  BLUE_STANDARD_3 = 103,
  BLUE_STANDARD_4 = 104,
  BLUE_STANDARD_5 = 105,
  BLUE_AERIAL = 106,
  BLUE_SENTRY = 107,
  BLUE_RADAR = 109,
  BLUE_OUTPOST = 110,
  BLUE_BASE = 111
} RobotId;

typedef enum
{
  RED_HERO_CLIENT = 0x0101,
  RED_ENGINEER_CLIENT = 0x0102,
  RED_STANDARD_3_CLIENT = 0x0103,
  RED_STANDARD_4_CLIENT = 0x0104,
  RED_STANDARD_5_CLIENT = 0x0105,
  RED_AERIAL_CLIENT = 0x0106,
  BLUE_HERO_CLIENT = 0x0165,
  BLUE_ENGINEER_CLIENT = 0x0166,
  BLUE_STANDARD_3_CLIENT = 0x0167,
  BLUE_STANDARD_4_CLIENT = 0x0168,
  BLUE_STANDARD_5_CLIENT = 0x0169,
  BLUE_AERIAL_CLIENT = 0x016A
} ClientId;

typedef enum
{
  ADD = 1,
  UPDATE = 2,
  DELETE = 3
} GraphOperation;

typedef enum
{
  MAIN_COLOR = 0,
  YELLOW = 1,
  GREEN = 2,
  ORANGE = 3,
  PURPLE = 4,
  PINK = 5,
  CYAN = 6,
  BLACK = 7,
  WHITE = 8
} GraphColor;

typedef enum
{
  LINE,
  RECTANGLE,
  CIRCLE,
  ELLIPSE,
  ARC,
  FLOAT_NUM,
  INT_NUM,
  STRING
} GraphType;

typedef enum
{
  ATTACK_IN = 1,
  DEFEND_IN = 2,
  MOVE_TO = 3
} SentryIntention;

typedef enum
{
  CHARGE = 0,
  BOOST = 1,
  NORMAL = 2,
  ALL_OFF = 3
} PowerManagementStateMachine;

typedef enum
{
  NO_PROBLEM = 0,
  HIGH_CURRENT = 1,
  REFEREE_POWER_DOWN = 2,
  REFEREE_DISCONNECT = 3
} PowerManagementProtectionInfo;

typedef enum
{
  POWER_ON = 1,
  EXTERNAL_BUTTON = 2,
  SOFT = 3,
  INDEPENDENT_WATCHDOG = 4,
  WINDOW_WATCHDOG = 5,
  LOW_VOLTAGE = 6,
  UNKNOWN = 7
} PowerManagementResetReason;

typedef enum
{
  PASS_THROUGH = 0,
  CHARGE_AND_BOOST = 1,
  SWITCHES_ALL_OFF = 2
} PowerManagementTopology;

/*****************************/

typedef struct
{
  uint8_t sof;
  uint16_t data_length;
  uint8_t seq;
  uint8_t crc_8;
} __packed FrameHeader;

// 官方文档中的game_status_t 0x0001
typedef struct
{
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t sync_time_stamp;
} __packed GameStatus;

// 官方文档中的game_result_t 0x0002
typedef struct
{
  uint8_t winner;
} __packed GameResult;

// 官方文档中的game_robot_HP_t 0x0003
typedef struct
{
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} __packed GameRobotHp;

// 官方文档中的event_data_t 0x0101
typedef struct
{
  uint32_t event_type;
} __packed EventData;

// 官方文档中的ext_supply_projectile_action_t 0x0102
typedef struct
{
  uint8_t reserved;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} __packed SupplyProjectileAction;

// 官方文档中的referee_warning_t 0x0104
typedef struct
{
  uint8_t level;
  uint8_t foul_robot_id;
  uint8_t count;
} __packed RefereeWarning;

// 官方文档中的dart_info_t 0x0105
typedef struct
{
  uint8_t dart_remaining_time;
  uint16_t dart_info;
} __packed DartRemainingTime;

// 官方文档中的robot_status_t 0x0201
typedef struct
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_hp;
  uint16_t max_hp;
  uint16_t shooter_cooling_rate;
  uint16_t shooter_cooling_limit;
  uint16_t chassis_power_limit;
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} __packed GameRobotStatus;

// 官方文档中的power_heat_data_t 0x0202
typedef struct
{
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t chassis_power_buffer;
  uint16_t shooter_id_1_17_mm_cooling_heat;
  uint16_t shooter_id_2_17_mm_cooling_heat;
  uint16_t shooter_id_1_42_mm_cooling_heat;
} __packed PowerHeatData;

// 官方文档中的robot_pos_t 0x0203
typedef struct
{
  float x;
  float y;
  float yaw;
} __packed GameRobotPos;

// 官方文档中的buff_t 0x0204
typedef struct
{
  uint8_t recovery_buff;
  uint8_t cooling_buff;
  uint8_t defence_buff;
  uint8_t vulnerability_buff;
  uint16_t attack_buff;
} __packed Buff;

// 官方文档中的air_support_data_t 0x0205
typedef struct
{
  uint8_t airforce_status;
  uint8_t attack_time;
} __packed AerialRobotEnergy;

// 官方文档中的hurt_data_t 0x0206
typedef struct
{
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} __packed RobotHurt;

// 官方文档中的shoot_data_t 0x0207
typedef struct
{
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} __packed ShootData;

// 官方文档中的projectile_allowance_t 0x0208
typedef struct
{
  uint16_t bullet_allowance_num_17_mm;
  uint16_t bullet_allowance_num_42_mm;
  uint16_t coin_remaining_num;
} __packed BulletAllowance;

// 官方文档中的rfid_status_t 0x0209
typedef struct
{
  uint32_t rfid_status;
} __packed RfidStatus;

// 官方文档中的dart_client_cmd_t 0x020A
typedef struct
{
  uint8_t dart_launch_opening_status;
  uint8_t reserved;
  uint16_t target_change_time;
  uint16_t latest_launch_cmd_time;
} __packed DartClientCmd;

// 官方文档中的ground_robot_position_t 0x020B
typedef struct
{
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
  float standard_5_x;
  float standard_5_y;
} __packed RobotsPositionData;

// 官方文档中的radar_mark_data_t 0x020C
typedef struct
{
  uint8_t mark_hero_progress;
  uint8_t mark_engineer_progress;
  uint8_t mark_standard_3_progress;
  uint8_t mark_standard_4_progress;
  uint8_t mark_standard_5_progress;
  uint8_t mark_sentry_progress;
} __packed RadarMarkData;

// 官方文档中的sentry_info_t 0x020D
typedef struct
{
  uint32_t sentry_info;
  uint16_t sentry_info_2; 
} __packed SentryInfo;

// 官方文档中的radar_info_t 0x020E
typedef struct
{
  uint8_t radar_info;
} __packed RadarInfo;

// 官方文档中的robot_interaction_data_t 0x0301
/*********************** Interactive data between robots----0x0301 ********************/
typedef struct
{
  uint16_t data_cmd_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  // x最大为113
  // uint8_t user_data[x];
} __packed InteractiveDataHeader;

// 官方文档中的interaction_layer_delete_t 0x0100
typedef struct
{
  uint8_t delete_type;
  uint8_t layer;
} __packed GraphicDelete;

// 官方文档中的interaction_figure_t 0x0101
struct GraphConfig
{
  uint8_t graphic_id[3];
  uint32_t operate_type : 3;
  uint32_t graphic_type : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t radius : 10;
  uint32_t end_x : 11;
  uint32_t end_y : 11;
  bool operator==(const GraphConfig& config)
  {
    return (graphic_id[0] == (uint32_t)config.graphic_id[0] && graphic_id[1] == (uint32_t)config.graphic_id[1] &&
            graphic_id[2] == (uint32_t)config.graphic_id[2] && operate_type == (uint32_t)config.operate_type &&
            graphic_type == (uint32_t)config.graphic_type && layer == (uint32_t)config.layer &&
            color == (uint32_t)config.color && start_angle == (uint32_t)config.start_angle &&
            end_angle == (uint32_t)config.end_angle && width == (uint32_t)config.width &&
            start_x == (uint32_t)config.start_x && start_y == (uint32_t)config.start_y &&
            radius == (uint32_t)config.radius && end_x == (uint32_t)config.end_x && end_y == (uint32_t)config.end_y);
  }
  GraphConfig& operator=(const GraphConfig& config)
  {
    graphic_id[0] = (uint32_t)config.graphic_id[0];
    graphic_id[1] = (uint32_t)config.graphic_id[1];
    graphic_id[2] = (uint32_t)config.graphic_id[2];
    operate_type = (uint32_t)config.operate_type;
    graphic_type = (uint32_t)config.graphic_type;
    layer = (uint32_t)config.layer;
    color = (uint32_t)config.color;
    start_angle = (uint32_t)config.start_angle;
    end_angle = (uint32_t)config.end_angle;
    width = (uint32_t)config.width;
    start_x = (uint32_t)config.start_x;
    start_y = (uint32_t)config.start_y;
    radius = (uint32_t)config.radius;
    end_x = (uint32_t)config.end_x;
    end_y = (uint32_t)config.end_y;
    return *this;
  }
} __packed;

typedef struct
{
  InteractiveDataHeader header;
  GraphConfig config;
  uint8_t content[30];
} __packed CharacterData;

typedef struct
{
  InteractiveDataHeader header;
  GraphConfig config;
} __packed SingleGraphData;


typedef struct
{
  InteractiveDataHeader header;
  GraphConfig config[5];
} __packed FiveGraphData;

typedef struct
{
  InteractiveDataHeader header;
  GraphConfig config[7];
} __packed SevenGraphData;

// 官方文档中的ext_client_custom_character_t 0x0110


// 官方文档中的sentry_cmd_t 0x0120
typedef struct
{
  uint32_t sentry_cmd;
} __packed SentryCmd;

// 官方文档中的radar_cmd_t 0x0121
typedef struct
{
  uint8_t radar_cmd;
} __packed RadarCmd;

// 官方文档中的map_command_t 0x0303
// 云台手在地图上发送的数据
typedef struct
{
  float target_position_x;
  float target_position_y;
  // float target_position_z;
  uint8_t command_keyboard;
  uint8_t target_robot_ID;
  uint8_t cmd_source;
} __packed ClientMapSendData;


// 官方文档中的map_robot_data_t 0x0305
//雷达发给裁判系统
typedef struct
{
  uint16_t hero_position_x;
  uint16_t hero_position_y;
  uint16_t engineer_position_x;
  uint16_t engineer_position_y;
  uint16_t infantry_3_position_x;
  uint16_t infantry_3_position_y;
  uint16_t infantry_4_position_x;
  uint16_t infantry_4_position_y;
  uint16_t infantry_5_position_x;
  uint16_t infantry_5_position_y;
  uint16_t sentry_position_x;
  uint16_t sentry_position_y; 
} __packed ClientMapReceiveData;

// 官方文档中的map_data_t 0x0307
// 哨兵机器人向队友告知路径的数据
typedef struct
{
  uint8_t intention;
  uint16_t start_position_x;
  uint16_t start_position_y;
  int8_t delta_x[49];
  int8_t delta_y[49];
  uint16_t sender_id;
} __packed MapSentryData;

// 官方文档中的custom_info_t 0x0308
typedef struct
{
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t user_data[30];
} __packed CustomInfo; 

// 官方文档中的custom_robot_data_t 0x0302
typedef struct
{
  uint8_t data[30];
} __packed CustomControllerData;

// 官方文档中的remote_control_t 0x0304
typedef struct
{
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int8_t left_button_down;
  int8_t right_button_down;
  uint16_t keyboard_value;
  uint16_t reserved;
} __packed RobotCommandData;

// 官方文档中的custom_client_data_t 0x0306
// 自定义控制器模拟键鼠
typedef struct
{
uint16_t key_value;
 uint16_t x_position:12;
 uint16_t mouse_left:4;
 uint16_t y_position:12;
 uint16_t mouse_right:4;
 uint16_t reserved;
} __packed CustomClientData;

// 以下是文档没提到的

typedef struct
{
  uint8_t dart_belong;
  uint16_t stage_remaining_time;
} __packed DartStatus;

typedef struct
{
  uint8_t f_1_zone_status : 1;
  uint8_t f_1_zone_buff_debuff_status : 3;
  uint8_t f_2_zone_status : 1;
  uint8_t f_2_zone_buff_debuff_status : 3;
  uint8_t f_3_zone_status : 1;
  uint8_t f_3_zone_buff_debuff_status : 3;
  uint8_t f_4_zone_status : 1;
  uint8_t f_4_zone_buff_debuff_status : 3;
  uint8_t f_5_zone_status : 1;
  uint8_t f_5_zone_buff_debuff_status : 3;
  uint8_t f_6_zone_status : 1;
  uint8_t f_6_zone_buff_debuff_status : 3;
  uint16_t red_1_bullet_left;
  uint16_t red_2_bullet_left;
  uint16_t blue_1_bullet_left;
  uint16_t blue_2_bullet_left;
} __packed IcraBuffDebuffZoneStatus;

typedef struct
{
  InteractiveDataHeader header_data;
  uint8_t data;
} __packed InteractiveData;

typedef struct
{
  InteractiveDataHeader header_data;
  float position_x;
  float position_y;
  float position_z;
  float position_yaw;
} __packed CurrentSentryPosData;


typedef struct
{
  uint8_t chassis_power_high_8_bit;
  uint8_t chassis_power_low_8_bit;
  uint8_t chassis_expect_power_high_8_bit;
  uint8_t chassis_expect_power_low_8_bit;
  uint8_t capacity_recent_charge_power_high_8_bit;
  uint8_t capacity_recent_charge_power_low_8_bit;
  uint8_t capacity_remain_charge_high_8_bit;
  uint8_t capacity_remain_charge_low_8_bit;
  uint8_t capacity_expect_charge_power;
  uint8_t power_management_topology : 2;
  uint8_t power_management_protection_info : 2;
  uint8_t state_machine_running_state : 4;
} __packed PowerManagementSampleAndStatusData;

typedef struct
{
  int8_t error_code;
  char string[31];
} __packed PowerManagementInitializationExceptionData;

typedef struct
{
  uint32_t r_0;
  uint32_t r_1;
  uint32_t r_2;
  uint32_t r_3;
  uint32_t r_12;
  uint32_t LR;
  uint32_t PC;
  uint32_t PSR;
} __packed PowerManagementSystemExceptionData;

typedef struct
{
  char process_name[32];
} __packed PowerManagementProcessStackOverflowData;

typedef struct
{
  uint8_t abnormal_reset_reason;
  uint8_t power_management_before_reset_topology : 4;
  uint8_t state_machine_before_reset_mode : 4;
} __packed PowerManagementUnknownExceptionData;

/***********************Frame tail(CRC8_CRC16)********************************************/
const uint8_t kCrc8Init = 0xff;
const uint8_t kCrc8Table[256] = {
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21,
  0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c,
  0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c,
  0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66,
  0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4,
  0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6,
  0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e, 0xed,
  0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
  0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1,
  0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf,
  0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57,
  0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
  0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9,
  0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
const uint16_t kCrc16Init = 0xffff;
const uint16_t wCRC_table[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5,
  0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52,
  0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3,
  0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9,
  0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f,
  0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862,
  0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb,
  0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948,
  0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226,
  0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497,
  0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704,
  0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb,
  0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
}  // namespace radar_referee
#endif
