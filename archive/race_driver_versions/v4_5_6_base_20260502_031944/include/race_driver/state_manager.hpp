#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "race_driver/types.hpp"

namespace race_driver
{

class StateManager
{
public:
  void updateFromOdom(const nav_msgs::msg::Odometry & msg);
  EgoState getState() const;

private:
  EgoState ego_state_;
};

}  // namespace race_driver
