#ifndef SRC_IOBSTACLES_H_
#define SRC_IOBSTACLES_H_

#include "Eigen-3.3/Eigen/Core"
#include "ITrajectory.h"
#include <vector>

class IObstacles {
 public:
  virtual ~IObstacles() = default;

  static constexpr double kLength = 5.0;
  static constexpr double kWidth = 3.0;

  struct Obstacle {
    Eigen::Vector2d frenet_;
    Eigen::Vector2d frenet_vel_;

    inline double velocity_mph() const {
      return frenet_vel_[0] / ITrajectory::kMilesPerHour2MetersPerSecond;
    }
  };

  virtual const Obstacle* Forward(double future_time_sec,
                                  double future_ego_s,
                                  int lane,
                                  double range) const = 0;

  virtual bool IsCollided(const std::vector<double>& path_x,
                          const std::vector<double>& path_y) const = 0;

  virtual double min_distance(const std::vector<double>& path_x,
                              const std::vector<double>& path_y) const = 0;
};

#endif  // SRC_IOBSTACLES_H_
