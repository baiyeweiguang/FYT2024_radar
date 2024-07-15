cmake_minimum_required(VERSION 3.8)
project(radar_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
  "srv/PointTransform.srv"
  "msg/Point.msg"
  "msg/Point2d.msg" 
  "msg/BoundingBox.msg" 
  "msg/Detection.msg" 
  "msg/DetectionArray.msg" 
  "msg/CustomMsg.msg"
  "msg/CustomPoint.msg"
  "msg/TargetInfo.msg"
  "msg/TargetInfoArray.msg"
  "msg/referee/CapacityData.msg"
  "msg/referee/GameRobotStatus.msg"
  "msg/referee/GameStatus.msg"
  "msg/referee/StateCmd.msg"
  "msg/referee/EngineerUi.msg"
  "msg/referee/GameRobotHp.msg"
  "msg/referee/PowerHeatData.msg"
  "msg/referee/EventData.msg"
  "msg/referee/DartStatus.msg"
  "msg/referee/IcraBuffDebuffZoneStatus.msg"
  "msg/referee/SupplyProjectileAction.msg"
  "msg/referee/DartRemainingTime.msg"
  "msg/referee/RobotHurt.msg"
  "msg/referee/CurrentSentryPosData.msg"
  "msg/referee/ShootData.msg"
  "msg/referee/BulletAllowance.msg"
  "msg/referee/RfidStatus.msg"
  "msg/referee/DartClientCmd.msg"
  "msg/referee/ManualToReferee.msg"
  "msg/referee/RadarData.msg"
  "msg/referee/RadarMarkData.msg"
  "msg/referee/RobotsPositionData.msg"
  "msg/referee/ClientMapSendData.msg"
  "msg/referee/ClientMapReceiveData.msg"
  "msg/referee/MapSentryData.msg"
  "msg/referee/GameRobotPosData.msg"
  "msg/referee/SentryInfo.msg"
  "msg/referee/RadarInfo.msg"
  "msg/referee/Buff.msg"
  "msg/referee/PowerManagementSampleAndStatusData.msg"
  "msg/referee/PowerManagementInitializationExceptionData.msg"
  "msg/referee/PowerManagementProcessStackOverflowData.msg"
  "msg/referee/PowerManagementSystemExceptionData.msg"
  "msg/referee/PowerManagementUnknownExceptionData.msg"
  DEPENDENCIES
    std_msgs
)

# list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
# install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY livox_sdk_vendor DESTINATION ${CMAKE_INSTALL_PREFIX})
# ament_package(CONFIG_EXTRAS livox_sdk_vendor-extras.cmake)

ament_package()
