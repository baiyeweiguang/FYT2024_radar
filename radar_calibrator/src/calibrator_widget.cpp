#include "radar_calibrator/calibrator_widget.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <qnamespace.h>

const std::vector<cv::Point2f> key_points = {
    cv::Point2f(15, 141),  cv::Point2f(192, 216), cv::Point2f(181, 317),
    cv::Point2f(471, 214), cv::Point2f(622, 164), cv::Point2f(624, 345),
    cv::Point2f(63, 549),  cv::Point2f(65, 423),  cv::Point2f(129, 423),
    cv::Point2f(245, 548), cv::Point2f(483, 524), cv::Point2f(64, 705),
    cv::Point2f(259, 768), cv::Point2f(241, 830), cv::Point2f(344, 765),
    cv::Point2f(366, 829), cv::Point2f(442, 703), cv::Point2f(33, 876),
    cv::Point2f(199, 912)};

CalibratorWidget::CalibratorWidget(QWidget *parent) : QWidget(parent) {
  this->setMouseTracking(true);
  std::filesystem::path directory =
      ament_index_cpp::get_package_share_directory("radar_calibrator");
  rmuc_map_image_ = cv::imread(directory / "resources/map_contours.png");

  camera_image_ =
      cv::imread(directory / "resources/cover.png", cv::IMREAD_COLOR);

  terrain_mask_ = cv::imread(directory / "resources/terrain_mask.png",
                             cv::IMREAD_GRAYSCALE);

  planar_moving_point_ = cv::Point2f(0, 0);

  // float scale = static_cast<float>(camera_image_.rows) /
  // rmuc_map_image_.rows; cv::resize(rmuc_map_image_, rmuc_map_image_,
  // cv::Size(), scale, scale); initial_points_ = {cv::Point2f(0, 0),
  // cv::Point2f(0, rmuc_map_image_.rows),
  //                    cv::Point2f(rmuc_map_image_.cols,
  //                    rmuc_map_image_.rows),
  //                    cv::Point2f(rmuc_map_image_.cols, 0)};

  init_points_ = {key_points[0], key_points[4], key_points[18], key_points[16]};
  projected_init_points_ = init_points_;

  projected_key_points_A_ = key_points;
  projected_key_points_B_ = key_points;
  projection_A_ = cv::Mat::eye(3, 3, CV_64F);
  projection_B_ = cv::Mat::eye(3, 3, CV_64F);

  camera_image_ = addBorder(camera_image_);
  updateMapImage();

  map_widget = std::make_unique<MapWidget>();
  map_widget->show();
}

CalibratorWidget::~CalibratorWidget() { map_widget->close(); }

void CalibratorWidget::setCameraImage(const cv::Mat &image) {
  cv::Size old_size = camera_image_.size();
  camera_image_ = addBorder(image);
  if (old_size != camera_image_.size()) {
    updateMapImage();
  }
  update();
}

cv::Mat CalibratorWidget::addBorder(const cv::Mat &image) {
  // 定义边框的宽度，例如上1像素，下2像素，左3像素，右4像素
  int top = 50;
  int bottom = 400;
  int left = 800;
  int right = 800;
  // 复制图片并添加边框
  cv::Mat bordered;
  cv::copyMakeBorder(image, bordered, top, bottom, left, right,
                     cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  return bordered;
}

cv::Point2f CalibratorWidget::projectToMap(const cv::Rect &box) {
  const cv::Point2f offset = cv::Point2f(800, 50);
  float x = (box.tl().x + box.br().x) * 0.5;
  float y = box.tl().y * 0.08 + box.br().y * 0.92;

  cv::Point2f bottom = cv::Point2f(x, y) + offset;

  std::vector<cv::Point2f> in = {bottom};
  std::vector<cv::Point2f> out;
  cv::perspectiveTransform(in, out, projection_A_.inv());

  // 如果机器人在平面B上
  if (inImage(terrain_mask_, out[0]) && terrain_mask_.at<uchar>(out[0]) == 0) {
    cv::perspectiveTransform(in, out, projection_B_.inv());
  }

  cv::Point2f projected;
  projected.x = out[0].y / rmuc_map_image_.rows * 28.0;
  projected.y = out[0].x / rmuc_map_image_.cols * 15.0;
  if (enemey_color == 1) {
    projected.x = 28.0 - projected.x;
    projected.y = 15.0 - projected.y;
  }
  return projected;
}

void CalibratorWidget::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  cv::Mat final_image = camera_image_ | projection_vis_;
  setFixedSize(final_image.cols, final_image.rows);
  cv::cvtColor(final_image, final_image, cv::COLOR_BGR2RGB);
  QImage qImage(final_image.data, final_image.cols, final_image.rows,
                final_image.cols * 3, QImage::Format_RGB888);
  painter.drawImage(QPoint(0, 0), qImage);
  drawControlPoints(&painter);
}

void CalibratorWidget::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_C) {
    state_ = State::CALIBRATING;
    update();
  } else if (event->key() == Qt::Key_R) {
    state_ = State::INIT;
    projected_key_points_A_ = key_points;
    projected_key_points_B_ = key_points;
    update();
  } else if (event->key() == Qt::Key_V) {
    state_ = State::PLANAR_MOVING;
  }
}

void CalibratorWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    if (state_ == State::INIT) {
      planar_moving_point_ = cv::Point2f(0, 0);
      for (int i = 0; i < 4; ++i) {
        if (isNearControlPoint(event->x(), event->y(), i)) {
          active_point_ = i;
          mouse_press_pos_ = event->pos();
        }
      }
    } else if (state_ == State::CALIBRATING) {
      int size = projected_key_points_A_.size();
      for (int i = 0; i < size; ++i) {
        if (isNearControlPoint(event->x(), event->y(), i)) {
          active_point_ = i;
          mouse_press_pos_ = event->pos();
        }
      }
    } else if (state_ == State::PLANAR_MOVING) {
      active_point_ = 0;
      mouse_press_pos_ = event->pos();
    }
  }
}

void CalibratorWidget::mouseMoveEvent(QMouseEvent *event) {
  if (active_point_ != -1) {
    QPoint delta = event->pos() - mouse_press_pos_;
    moveControlPoint(active_point_, delta);
    mouse_press_pos_ = event->pos();
    updateProjectionMatrix();
    update();
  } else {
    if (state_ == State::INIT) {
      for (int i = 0; i < 4; ++i) {
        if (isNearControlPoint(event->x(), event->y(), i)) {
          QCursor::setPos(mapToGlobal(QPoint(projected_init_points_[i].x,
                                             projected_init_points_[i].y)));
        }
      }
    } else if (state_ == State::CALIBRATING) {
      int size = projected_key_points_A_.size();
      for (int i = 0; i < size; ++i) {
        if (isNearControlPoint(event->x(), event->y(), i)) {
          QCursor::setPos(mapToGlobal(QPoint(projected_key_points_A_[i].x,
                                             projected_key_points_A_[i].y)));
        }
      }
    }
  }
}

void CalibratorWidget::mouseReleaseEvent(QMouseEvent *event) {
  active_point_ = -1;
  (void)event;
}

bool CalibratorWidget::isNearControlPoint(int x, int y, int idx) {
  // 检查鼠标位置与控制点的距离是否足够近
  // 这里使用简单的距离计算，实际开发中可能需要更精细的逻辑
  cv::Point2f p = state_ == State::INIT ? projected_init_points_[idx]
                                        : projected_key_points_A_[idx];
  double tolerance = 20.0; // 容差值
  double dx = x - p.x;
  double dy = y - p.y;
  return sqrt(dx * dx + dy * dy) < tolerance;
}

void CalibratorWidget::moveControlPoint(int idx, QPoint delta) {
  if (state_ == State::PLANAR_MOVING) {
    planar_moving_point_.x += delta.x();
    planar_moving_point_.y += delta.y();
  } else {
    cv::Point2f &p = state_ == State::INIT ? projected_init_points_[idx]
                                           : projected_key_points_A_[idx];
    double x = p.x + delta.x();
    double y = p.y + delta.y();
    p.x = std::max(0.0, std::min(static_cast<double>(camera_image_.cols), x));
    p.y = std::max(0.0, std::min(static_cast<double>(camera_image_.rows), y));
  }
}

void CalibratorWidget::drawControlPoints(QPainter *painter) {
  // 绘制控制点
  if (state_ == State::INIT) {
    painter->setPen(QPen(Qt::red, 3));
    for (const auto &p : projected_init_points_) {
      painter->drawEllipse(QPoint(p.x, p.y), 5, 5);
    }
  } else {
    int size = projected_key_points_A_.size();
    for (int i = 0; i < size; ++i) {
      painter->setPen(QPen(Qt::green, 3));
      QPoint p =
          QPoint(projected_key_points_A_[i].x, projected_key_points_A_[i].y);
      painter->drawEllipse(p, 5, 5);
      std::string text = std::to_string(i + 1);
      painter->drawText(p - QPoint(5, 5), QString::fromStdString(text));

      if (state_ == State::PLANAR_MOVING) {
        painter->setPen(QPen(Qt::yellow, 3));
        QPoint p_new =
            QPoint(projected_key_points_B_[i].x, projected_key_points_B_[i].y);
        painter->drawLine(p, p_new);
        painter->drawEllipse(p_new, 5, 5);
      }
    }
  }
}

void CalibratorWidget::updateProjectionMatrix() {
  if (state_ == State::INIT) {
    projection_A_ =
        cv::getPerspectiveTransform(init_points_, projected_init_points_);
    cv::perspectiveTransform(key_points, projected_key_points_A_,
                             projection_A_);
    projection_B_ = projection_A_.clone();
    projected_key_points_B_ = projected_key_points_A_;
  } else {
    double min_projection_error = 10000000;
    int size = key_points.size();

    using Pair = std::pair<cv::Mat, double>;
    auto cmp = [](const Pair &a, const Pair &b) { return a.second < b.second; };
    std::priority_queue<Pair, std::vector<Pair>, decltype(cmp)> q(cmp);
    for (int i = 0; i < size; i++) {
      for (int j = i + 1; j < size; j++) {
        for (int k = j + 1; k < size; k++) {
          for (int l = k + 1; l < size; l++) {
            std::array<cv::Point2f, 4> pts1 = {key_points[i], key_points[j],
                                               key_points[k], key_points[l]};
            std::array<cv::Point2f, 4> pts2 = {
                projected_key_points_A_[i], projected_key_points_A_[j],
                projected_key_points_A_[k], projected_key_points_A_[l]};
            cv::Mat mat = cv::getPerspectiveTransform(pts1, pts2);
            std::vector<cv::Point2f> projected_points;
            cv::perspectiveTransform(key_points, projected_points, mat);
            double error = 0;
            for (int m = 0; m < size; m++) {
              double factor =
                  (2 - projected_key_points_A_[m].y / rmuc_map_image_.rows) * 2;
              error +=
                  cv::norm(projected_points[m] - projected_key_points_A_[m]) *
                  factor;
            }
            if (error < min_projection_error) {
              min_projection_error = error;
              // final_project_matirx_ = mat;
              q.push({mat, error});
              if (q.size() > 10) {
                q.pop();
              }
            }

            // sum += mat;
            // n++;
          }
        }
      }
    }
    cv::Mat sum = cv::Mat::zeros(3, 3, CV_64F);
    while (!q.empty()) {
      sum += q.top().first;
      q.pop();
    }
    projection_A_ = sum / 10;

    projection_B_ = projection_A_.clone();
    projection_B_.at<double>(0, 2) += planar_moving_point_.x;
    projection_B_.at<double>(1, 2) += planar_moving_point_.y;
    cv::perspectiveTransform(key_points, projected_key_points_B_,
                             projection_B_);
  }
}

void CalibratorWidget::updateMapImage() {
  cv::Mat projected_A, projected_B;
  cv::warpPerspective(rmuc_map_image_, projected_A, projection_A_,
                      camera_image_.size());
  cv::warpPerspective(rmuc_map_image_, projected_B, projection_B_,
                      camera_image_.size());

  projection_vis_ = (projected_A | projected_B) * 0.5;
}