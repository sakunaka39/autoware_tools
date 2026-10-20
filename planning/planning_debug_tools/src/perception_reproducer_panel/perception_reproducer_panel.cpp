#include "perception_reproducer_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace planning_debug_tools
{
PerceptionReproducerPanel::PerceptionReproducerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto layout = new QVBoxLayout;
  status_label_ = new QLabel("Status: Unknown");
  speed_ratio_label_ = new QLabel("Playback Ratio: -");
  ego_speed_label_ = new QLabel("Ego Speed: -");
  progress_label_ = new QLabel("Bag Progress: -");
  current_bag_label_ = new QLabel("Current Bag: -");
  time_label_ = new QLabel("Time: - / -");
  fast_forward_checkbox_ = new QCheckBox("Enable Fast Forward (Stop > 3s)");

  // スタイル設定（見やすく太字にするなど）
  status_label_->setStyleSheet("font-weight: bold; font-size: 18px;"); // 大きくする
  
  QString normal_style = "font-size: 16px;";
  speed_ratio_label_->setStyleSheet(normal_style);
  ego_speed_label_->setStyleSheet(normal_style);
  progress_label_->setStyleSheet(normal_style);
  current_bag_label_->setStyleSheet(normal_style);
  time_label_->setStyleSheet(normal_style);
  fast_forward_checkbox_->setChecked(true);
  
  layout->addWidget(status_label_);
  layout->addWidget(speed_ratio_label_);
  layout->addWidget(ego_speed_label_);
  layout->addWidget(progress_label_);
  layout->addWidget(current_bag_label_);
  layout->addWidget(time_label_);
  layout->addWidget(fast_forward_checkbox_);
  setLayout(layout);

  connect(fast_forward_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(onFastForwardCheckboxChanged(int)));
}

void PerceptionReproducerPanel::onInitialize()
{
  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pub_enable_fast_forward_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/perception_replayer_common/enable_fast_forward", 1);
  sub_status_ = node_->create_subscription<msg::PerceptionReproducerStatus>(
    "/perception_replayer_common/status", 1,
    [this](const msg::PerceptionReproducerStatus::ConstSharedPtr msg) {
      status_label_->setText(QString("Status: %1").arg(msg->status_text.c_str()));
      speed_ratio_label_->setText(QString("Playback Ratio: x%1").arg(msg->current_speed_ratio, 0, 'f', 1));
      ego_speed_label_->setText(QString("Ego Speed: %1 m/s").arg(msg->ego_speed, 0, 'f', 2));
      progress_label_->setText(QString("Bag Progress: %1 %").arg(msg->bag_progress_percent, 0, 'f', 1));

      current_bag_label_->setText(QString("Current Bag: #%1 (%2 %)")
                                    .arg(msg->current_bag_index)
                                    .arg(msg->current_bag_progress_percent, 0, 'f', 1));

      int elapsed_m = msg->elapsed_time_sec / 60;
      int elapsed_s = static_cast<int>(msg->elapsed_time_sec) % 60;
      int total_m = msg->total_time_sec / 60;
      int total_s = static_cast<int>(msg->total_time_sec) % 60;
      time_label_->setText(QString("Time: %1:%2 / %3:%4")
        .arg(elapsed_m, 2, 10, QChar('0')).arg(elapsed_s, 2, 10, QChar('0'))
        .arg(total_m, 2, 10, QChar('0')).arg(total_s, 2, 10, QChar('0')));
      
      // 状態に応じて色を変える
      if (msg->status_text == "Fast Forwarding") {
        status_label_->setStyleSheet("color: orange; font-weight: bold; font-size: 18px;");
      } else if (msg->status_text == "Normal Playback") {
        status_label_->setStyleSheet("color: green; font-weight: bold; font-size: 18px;");
      } else {
        status_label_->setStyleSheet("color: gray; font-weight: bold; font-size: 18px;");
      }
    });
}

void PerceptionReproducerPanel::onFastForwardCheckboxChanged(int state)
{
  if (pub_enable_fast_forward_) {
    std_msgs::msg::Bool msg;
    msg.data = (state == Qt::Checked);
    pub_enable_fast_forward_->publish(msg);
  }
}

}  // namespace planning_debug_tools

PLUGINLIB_EXPORT_CLASS(planning_debug_tools::PerceptionReproducerPanel, rviz_common::Panel)