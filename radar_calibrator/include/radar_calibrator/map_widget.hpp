#ifndef RADAR_CALIBRATOR__MAP_WIDGET_HPP_
#define RADAR_CALIBRATOR__MAP_WIDGET_HPP_

#include <QImage>
#include <QPainter>
#include <QWidget>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "radar_interfaces/msg/client_map_receive_data.hpp"

class MapWidget : public QWidget {
public:
  MapWidget(QWidget *parent = nullptr);

  virtual ~MapWidget() {}

  void setTargets(
      const std::vector<radar_interfaces::msg::ClientMapReceiveData> &targets) {
    targets_ = targets;
    update();
  }

private:
  virtual void paintEvent(QPaintEvent *) override;

  std::vector<radar_interfaces::msg::ClientMapReceiveData> targets_;
  cv::Mat image_;
  int acceptable_radius_ = 10;
};
#endif