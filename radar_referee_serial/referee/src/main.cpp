//
// Created by ljq on 2022/5/17.
//
#include <rclcpp/rclcpp.hpp>
#include "radar_referee/referee_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<radar_referee::RefereeNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
