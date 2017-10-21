#ifndef SRC_ITRAJECTORY_H_
#define SRC_ITRAJECTORY_H_

#include <vector>
#include <initializer_list>
#include "Eigen-3.3/Eigen/Core"

class ITrajectory {
 public:
  virtual ~ITrajectory() = default;

  virtual void set_car_pos(const Eigen::Vector2d& pos) = 0;
  virtual void set_car_yaw_rad(double yaw_rad) = 0;
  virtual void set_car_speed(double speed) = 0;

  virtual void set_previous_path(const std::vector<double>& path_x,
                                 const std::vector<double>& path_y) = 0;

  virtual void Trace(double target_vel,
                     std::initializer_list<Eigen::Vector2d> target_poses) = 0;

  virtual const std::vector<double>& path_x() const = 0;
  virtual const std::vector<double>& path_y() const = 0;
};

#endif  // SRC_ITRAJECTORY_H_
