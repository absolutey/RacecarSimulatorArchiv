#pragma once

#include "nav_msgs/msg/path.hpp"
#include "race_driver/types.hpp"

namespace race_driver
{

class TrackManager
{
public:
  void updateCenterline(const nav_msgs::msg::Path & msg);
  void updateLeftBoundary(const nav_msgs::msg::Path & msg);
  void updateRightBoundary(const nav_msgs::msg::Path & msg);

  TrackModel getTrackModel() const;
  bool isReady() const;

  int findNearestCenterIndex(double x, double y) const;
  double projectToCenterS(double x, double y, int * nearest_idx = nullptr) const;
  double computeSignedLateralOffset(double x, double y, int center_idx) const;
  bool frenetFromXY(double x, double y, double & s, double & d, int & nearest_idx) const;
  bool getCenterPointAtS(double query_s, TrackPoint & pt) const;

private:
  static std::vector<TrackPoint> convertPath(const nav_msgs::msg::Path & msg);
  void updateDerivedFields();
  static double distance(const TrackPoint & a, const TrackPoint & b);

  TrackModel track_model_;
};

}  // namespace race_driver
