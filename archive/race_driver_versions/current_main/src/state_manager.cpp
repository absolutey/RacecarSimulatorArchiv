#include "race_driver/state_manager.hpp"

#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace race_driver
{

void StateManager::updateFromOdom(const nav_msgs::msg::Odometry & msg)
{
  ego_state_.x = msg.pose.pose.position.x;
  ego_state_.y = msg.pose.pose.position.y;

  tf2::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  ego_state_.yaw = yaw;

  const double vx = msg.twist.twist.linear.x;
  const double vy = msg.twist.twist.linear.y;
  ego_state_.speed = std::sqrt(vx * vx + vy * vy);
  ego_state_.valid = true;
}

EgoState StateManager::getState() const
{
  return ego_state_;
}

}  // namespace race_driver
