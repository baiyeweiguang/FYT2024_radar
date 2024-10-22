// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef RADAR_CALIBRATOR__CALIBRATOR_WIDGET_HPP_
#define RADAR_CALIBRATOR__CALIBRATOR_WIDGET_HPP_

// std
#include <cstddef>
#include <functional>
#include <iostream>
#include <numeric>
#include <opencv2/core/types.hpp>
#include <qnamespace.h>
#include <unordered_map>
// Qt
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QWidget>

// OpenCV
#include <opencv2/opencv.hpp>

#include "radar_calibrator/common.hpp"
#include "radar_calibrator/map_widget.hpp"

enum class State {
  INIT,
  CALIBRATING,
  PLANAR_MOVING,
};

class CalibratorWidget : public QWidget {
  // Q_OBJECT
public:
  CalibratorWidget(QWidget *parent = nullptr);

  virtual ~CalibratorWidget();

  void setCameraImage(const cv::Mat &image);

  cv::Point2f projectToMap(const cv::Rect &box);

  std::unique_ptr<MapWidget> map_widget;

protected:
  virtual void paintEvent(QPaintEvent *) override;

  virtual void keyPressEvent(QKeyEvent *event) override;

  virtual void mouseMoveEvent(QMouseEvent *event) override;

  virtual void mousePressEvent(QMouseEvent *event) override;

  virtual void mouseReleaseEvent(QMouseEvent *event) override;

private:
  cv::Mat addBorder(const cv::Mat &image);

  bool isNearControlPoint(int x, int y, int idx);

  void moveControlPoint(int idx, QPoint delta);

  void drawControlPoints(QPainter *painter);

  void updateProjectionMatrix();

  void updateMapImage();

  // border
  int top_;
  int bottom_;
  int left_;
  int right_;

  cv::Mat camera_image_;
  cv::Mat projection_vis_;
  cv::Mat rmuc_map_image_;
  cv::Mat terrain_mask_;
  QImage q_image_;

  State state_ = State::INIT;
  std::vector<cv::Point2f> init_points_;
  std::vector<cv::Point2f> projected_init_points_;
  std::vector<cv::Point2f> projected_key_points_A_;
  std::vector<cv::Point2f> projected_key_points_B_;
  cv::Point2f planar_moving_point_;
  // 先标定平面A，再平移得到平面B
  cv::Mat projection_A_;
  cv::Mat projection_B_;
  std::vector<cv::Mat> projection_matrixs_;
  int active_point_ = -1;
  QPoint mouse_press_pos_;
};

#endif // RADAR_CALIBRATOR__CALIBRATOR_WIDGET_HPP_