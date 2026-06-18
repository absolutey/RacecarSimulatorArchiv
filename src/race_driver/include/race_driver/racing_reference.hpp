#pragma once

#include <string>
#include <vector>

#include "race_driver/types.hpp"

namespace race_driver
{

class RacingReference
{
public:
  bool loadFromCsv(const std::string & csv_path);

  bool valid() const;
  const std::string & sourcePath() const;
  const std::vector<TrackPoint> & points() const;

  // center_path의 한 점을 기준으로, reference가 좌/우로 얼마나 벗어나 있는지 d hint 계산
  bool lateralOffsetHint(
    const TrackPoint & center_point,
    double & offset_d) const;

private:
  static double normalizeAngle(double angle);
  static bool parseNumericCsvLine(
    const std::string & line,
    std::vector<double> & values);

  void computeGeometry();

  std::vector<TrackPoint> points_;
  std::string source_path_;
  bool valid_{false};
};

}  // namespace race_driver
