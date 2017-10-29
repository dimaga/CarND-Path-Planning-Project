#ifndef SRC_ITRAJECTORY_H_
#define SRC_ITRAJECTORY_H_

#include <vector>
#include <initializer_list>
#include "Eigen-3.3/Eigen/Core"

class ITrajectory {
 public:
  static constexpr double kDtInSeconds {0.02};
  static constexpr double kMilesPerHour2MetersPerSecond{1.0 / 2.24};

  virtual ~ITrajectory() = default;

  virtual void set_car_pos(const Eigen::Vector2d& pos) = 0;
  virtual void set_car_yaw_rad(double yaw_rad) = 0;
  virtual void set_car_speed_mph(double speed_mph) = 0;

  virtual double recent_time_sec() const = 0;
  virtual Eigen::Vector2d recent_cartesian() const = 0;

  virtual void set_previous_path(const std::vector<double>& path_x,
                                 const std::vector<double>& path_y) = 0;

  virtual void Trace(double target_speed_mph,
                     std::initializer_list<Eigen::Vector2d> target_poses) = 0;

  virtual const std::vector<double>& path_x() const = 0;
  virtual const std::vector<double>& path_y() const = 0;
};

#endif  // SRC_ITRAJECTORY_H_
