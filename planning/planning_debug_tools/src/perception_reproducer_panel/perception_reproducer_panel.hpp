#ifndef PERCEPTION_REPRODUCER_PANEL_HPP_
#define PERCEPTION_REPRODUCER_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <QLabel>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <planning_debug_tools/msg/perception_reproducer_status.hpp>
#include <QCheckBox>
#include <std_msgs/msg/bool.hpp>

namespace planning_debug_tools
{
class PerceptionReproducerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PerceptionReproducerPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<msg::PerceptionReproducerStatus>::SharedPtr sub_status_;

  QLabel * status_label_;
  QLabel * speed_ratio_label_;
  QLabel * ego_speed_label_;
  QLabel * progress_label_;
  QLabel * current_bag_label_;
  QLabel * time_label_;
  QCheckBox * fast_forward_checkbox_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_fast_forward_;

private Q_SLOTS:
  void onFastForwardCheckboxChanged(int state);
};
}  // namespace planning_debug_tools
#endif