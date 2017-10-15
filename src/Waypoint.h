#ifndef SRC_WAYPOINT_H_
#define SRC_WAYPOINT_H_

#include "Eigen-3.3/Eigen/Core"

struct Waypoint {
  Eigen::Vector2d pos_{0, 0};
  double s_{0};
  Eigen::Vector2d dir_{1, 0};
};

#endif  // SRC_WAYPOINT_H_
