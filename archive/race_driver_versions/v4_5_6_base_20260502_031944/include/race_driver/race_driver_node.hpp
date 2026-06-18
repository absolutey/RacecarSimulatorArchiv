#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "race_driver/boundary_corridor_planner.hpp"
#include "race_driver/controller.hpp"
#include "race_driver/obstacle_detector.hpp"
#include "race_driver/occupancy_grid_manager.hpp"
#include "race_driver/map_rollout_planner.hpp"
#include "race_driver/raceline_planner.hpp"
#include "race_driver/speed_planner.hpp"
#include "race_driver/state_manager.hpp"
#include "race_driver/track_manager.hpp"

namespace race_driver
{

class RaceDriverNode : public rclcpp::Node
{
public:
  RaceDriverNode();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void centerPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void leftBoundaryCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void rightBoundaryCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void controlLoop();
  void publishZeroCommand();
  void publishLocalPath(const LocalPath & local_path);
  void publishObstacleMarkers(const ObstacleList & obstacles);
  bool inputsFresh() const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool has_odom_{false};
  bool has_scan_{false};

  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_track_time_{0, 0, RCL_ROS_TIME};

  double odom_timeout_sec_{0.30};
  double scan_timeout_sec_{0.30};
  double track_timeout_sec_{1.00};
  bool stop_on_invalid_path_{true};
  bool debug_logging_{true};
  std::string planner_mode_{"raceline"};

  bool use_map_safety_check_{false};
  double map_vehicle_radius_m_{0.22};
  double map_safety_speed_cap_{3.2};

  bool map_rollout_fallback_to_boundary_{true};

  StateManager state_manager_;
  TrackManager track_manager_;
  ObstacleDetector obstacle_detector_;
  OccupancyGridManager occupancy_grid_manager_;
  MapRolloutPlanner map_rollout_planner_;
  BoundaryCorridorPlanner boundary_corridor_planner_;
  RacelinePlanner raceline_planner_;
  SpeedPlanner speed_planner_;
  Controller controller_;
};

}  // namespace race_driver
