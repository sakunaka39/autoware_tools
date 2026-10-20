// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "perception_replayer.hpp"

namespace autoware::planning_debug_tools
{

PerceptionReplayer::PerceptionReplayer(
  const PerceptionReplayerCommonParam & param, const rclcpp::NodeOptions & node_options)
: PerceptionReplayerCommon(param, "perception_replayer", node_options)
{
  // Initialize widget with rosbag timestamps
  widget = new TimeManagerWidget(get_bag_start_time(), get_bag_end_timestamp());
  widget->show();

  // Connect signals
  for (auto * rate_button : widget->rate_buttons) {
    QObject::connect(rate_button, &QPushButton::clicked, [this, rate_button]() {
      on_set_rate(rate_button->text());
    });
  }

  QObject::connect(widget->pub_recorded_ego_pose_button, &QPushButton::clicked, [this]() {
    publish_recorded_ego_pose(widget->get_slider_timestamp());
  });

  QObject::connect(
    widget->pub_goal_pose_button, &QPushButton::clicked, [this]() { publish_goal_pose(); });

  QObject::connect(widget, &TimeManagerWidget::windowClosed, []() { rclcpp::shutdown(); });

  status_pub_ = this->create_publisher<::planning_debug_tools::msg::PerceptionReproducerStatus>(
    "/perception_replayer_common/status", 1);
    
  sub_enable_fast_forward_ = this->create_subscription<std_msgs::msg::Bool>(
    "/perception_replayer_common/enable_fast_forward", 1,
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      enable_fast_forward_ = msg->data;
      RCLCPP_INFO(get_logger(), "Fast forward is now %s", enable_fast_forward_ ? "ON" : "OFF");
    });

  // Start timer callback
  timer = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(static_cast<int>(delta_time * 1000)),
    std::bind(&PerceptionReplayer::on_timer, this), callback_group_check_perception_);

  RCLCPP_INFO(get_logger(), "Start timer callback");

  if (param_.publish_route) {
    publish_recorded_ego_pose(get_bag_start_time());
    // ルート生成待ちのためのスリープ
    rclcpp::sleep_for(std::chrono::seconds(2));
    publish_goal_pose();
  }
}

void PerceptionReplayer::on_timer()
{
  const auto current_timestamp = this->get_clock()->now();
  rclcpp::Time bag_timestamp = widget->get_slider_timestamp();

  // --- 1. シミュレータ自車の速度判定 ---
  double ego_speed = 0.0;
  const auto ego_odom_opt = get_latest_ego_odom();
  if (ego_odom_opt.has_value()) {
    ego_speed = utils::calculate_speed_2d(ego_odom_opt.value().twist.twist);
    if (ego_speed > 0.1) {
      last_moving_time_ = current_timestamp;
    } else if (!last_moving_time_.has_value()) {
      last_moving_time_ = current_timestamp;
    }
  }

  double stopped_duration = last_moving_time_.has_value() 
                              ? (current_timestamp - last_moving_time_.value()).seconds() 
                              : 0.0;

  double current_rate = rate; // GUIで設定された基本レート
  bool is_fast_forwarding = false;

  if (enable_fast_forward_ && stopped_duration > 3.0) {
    current_rate = rate * 10.0; // 3秒以上停止ならさらに10倍速にする
    is_fast_forwarding = true;
  }

  // --- 2. 時間の更新とパブリッシュ ---
  if (!widget->pause_button->isChecked()) {
    bag_timestamp += rclcpp::Duration::from_seconds(delta_time * current_rate);
    const auto bag_end_timestamp = get_bag_end_timestamp();
    if (bag_timestamp >= bag_end_timestamp) {
      bag_timestamp = bag_end_timestamp;
    }
    widget->set_slider_timestamp(bag_timestamp);
  }

  publish_topics_at_timestamp(bag_timestamp, current_timestamp, false);

  // --- 3. ステータスの計算とパネルへの配信 ---
  double current_playback_ratio = 0.0;
  if (last_real_time_.has_value() && last_bag_time_.has_value()) {
    double real_dt = (current_timestamp - last_real_time_.value()).seconds();
    double bag_dt = (bag_timestamp - last_bag_time_.value()).seconds();
    if (real_dt > 0.001) {
      current_playback_ratio = bag_dt / real_dt;
    }
  }
  
  last_bag_time_ = bag_timestamp;
  last_real_time_ = current_timestamp;

  smoothed_speed_ratio_ = 0.8 * smoothed_speed_ratio_ + 0.2 * current_playback_ratio;

  ::planning_debug_tools::msg::PerceptionReproducerStatus status_msg;
  status_msg.ego_speed = ego_speed;
  status_msg.current_speed_ratio = smoothed_speed_ratio_;
  
  if (!rosbag_ego_odom_data_.empty()) {
    auto start_time = rosbag_ego_odom_data_.front().first;
    auto end_time = rosbag_ego_odom_data_.back().first;
    
    status_msg.elapsed_time_sec = (bag_timestamp - start_time).seconds();
    status_msg.total_time_sec = (end_time - start_time).seconds();

    double progress = status_msg.elapsed_time_sec / status_msg.total_time_sec * 100.0;
    status_msg.bag_progress_percent = std::max(0.0, std::min(100.0, progress));

    status_msg.current_bag_index = static_cast<int>(status_msg.elapsed_time_sec / 60.0) + 1;
    status_msg.current_bag_progress_percent = 
      std::fmod(status_msg.elapsed_time_sec, 60.0) / 60.0 * 100.0f;
  }

  // ステータス文字の設定
  if (widget->pause_button->isChecked()) {
    status_msg.status_text = "Paused";
  } else if (is_fast_forwarding) {
    status_msg.status_text = "Fast Forwarding";
  } else if (stopped_duration > 0.0 && ego_speed <= 0.1) {
    status_msg.status_text = "Stopped (Waiting)";
  } else {
    status_msg.status_text = "Normal Playback";
  }

  status_pub_->publish(status_msg);
}

void PerceptionReplayer::on_set_rate(const QString & rate_text)
{
  bool ok;
  double new_rate = rate_text.toDouble(&ok);
  if (ok) {
    rate = new_rate;
  }
}

}  // namespace autoware::planning_debug_tools
