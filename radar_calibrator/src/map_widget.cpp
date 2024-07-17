#include "radar_calibrator/map_widget.hpp"
#include "radar_calibrator/common.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QPainter>
#include <filesystem>

MapWidget::MapWidget(QWidget *parent) : QWidget(parent) {
  std::filesystem::path directory =
      ament_index_cpp::get_package_share_directory("radar_calibrator");
  image_ =
      cv::imread(directory / "resources/map_colorful.png", cv::IMREAD_COLOR);
  if (enemey_color == 1) {
    cv::flip(image_, image_, 0);
    cv::flip(image_, image_, 1);
  }
  setFixedSize(image_.cols, image_.rows);
  acceptable_radius_ = image_.rows / 28.0 * 0.8;
}

void MapWidget::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  cv::Mat final_image = image_.clone();
  for (const auto &target : targets_) {
    cv::Point2f p;
    p.x = target.y / 15.0 * final_image.cols;
    p.y = target.x / 28.0 * final_image.rows;
    if (enemey_color == 1) {
      p.x = final_image.cols - p.x;
      p.y = final_image.rows - p.y;
    }
    cv::Scalar color =
        target.robot_id > 100 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
    cv::circle(final_image, p, 5, color, -1);
    cv::circle(final_image, p, acceptable_radius_, color, 2);
    cv::putText(final_image, std::to_string(target.robot_id % 100),
                p - cv::Point2f(0, 15), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(255, 255, 255), 2);
  }
  cv::cvtColor(final_image, final_image, cv::COLOR_BGR2RGB);
  QImage qImage(final_image.data, final_image.cols, final_image.rows,
                final_image.cols * 3, QImage::Format_RGB888);
  painter.drawImage(QPoint(0, 0), qImage);
}