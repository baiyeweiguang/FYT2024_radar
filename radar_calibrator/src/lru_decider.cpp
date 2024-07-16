#include "radar_calibrator/lru_decider.hpp"
#include "radar_calibrator/common.hpp"

LruDecider::LruDecider(rclcpp::Clock::SharedPtr clock) : clock_(clock) {
  marks_.resize(6);
  for (int i = 0; i < 6; i++) {
    marks_[i].detected = false;
    marks_[i].last_send_time = clock_->now();
  }
}

void LruDecider::update(
    const radar_interfaces::msg::ClientMapReceiveData &msg) {
  if ((enemey_color == 0 && msg.target_robot_id >= 100) ||
      (enemey_color == 1 && msg.target_robot_id < 100)) {
    return;
  }
  int idx = msg.target_robot_id % 100 - 1;
  if (idx < 0 || idx >= 6) {
    return;
  }
  marks_[idx].mark_info = msg;
  marks_[idx].detected = true;
}

std::optional<radar_interfaces::msg::ClientMapReceiveData>
LruDecider::getMarkInfo() {
  rclcpp::Time now = clock_->now();

  std::vector<decltype(marks_)::iterator> detected_marks;
  for (auto it = marks_.begin(); it != marks_.end(); it++) {
    if (it->detected) {
      detected_marks.push_back(it);
    }
  }

  if (detected_marks.empty()) {
    return std::nullopt;
  }

  auto oldest =
      *std::min_element(detected_marks.begin(), detected_marks.end(),
                        [now](const auto &a, const auto &b) {
                          return a->last_send_time < b->last_send_time;
                        });
  oldest->last_send_time = now;
  oldest->detected = false;

  auto mark = oldest->mark_info;
  mark.header.stamp = now;
  mark.header.frame_id = "map";

  return oldest->mark_info;
}