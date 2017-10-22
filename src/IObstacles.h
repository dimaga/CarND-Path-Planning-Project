#ifndef SRC_IOBSTACLES_H_
#define SRC_IOBSTACLES_H_

#include "Eigen-3.3/Eigen/Core"
#include "ITrajectory.h"

class IObstacles {
 public:
  virtual ~IObstacles() = default;

  struct Obstacle {
    Eigen::Vector2d frenet_;
    Eigen::Vector2d frenet_vel_;

    inline double velocity_mph() const {
      return frenet_vel_[0] / ITrajectory::kMilesPerHour2MetersPerSecond;
    }
  };

  virtual const Obstacle* Forward(double s, int lane, double range) const = 0;
};

#endif  // SRC_IOBSTACLES_H_
