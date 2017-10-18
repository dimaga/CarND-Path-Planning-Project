#ifndef SRC_MAP_H_
#define SRC_MAP_H_

#include "Waypoint.h"
#include <istream>
#include <vector>

class Map {
 public:
  explicit Map(std::istream& is);

  double length() const;

  std::size_t ClosestWaypoint(const Eigen::Vector2d& pos) const;
  std::size_t NextWaypoint(const Eigen::Vector2d& pos) const;

  Eigen::Vector2d ToFrenet(const Eigen::Vector2d& cartesian) const;
  Eigen::Vector2d ToCartesian(const Eigen::Vector2d& frenet) const;

 private:
  std::vector<Waypoint> track_;
  double length_{0.0};
};

#endif  // SRC_MAP_H_
