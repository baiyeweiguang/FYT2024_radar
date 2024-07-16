#include <QApplication>
#include <QMainWindow>

#include <rclcpp/rclcpp.hpp>

#include "radar_calibrator/calibrator_node.hpp"
#include "radar_calibrator/calibrator_widget.hpp"

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);

  auto widget = std::make_shared<CalibratorWidget>();
  widget->show();

  auto node = std::make_shared<CalibratorNode>(widget);

  std::thread([&]() {
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
  }).detach();

  int result = app.exec();
  rclcpp::shutdown();
  return result;
}
