#ifndef SRC_TRAJECTORY_H_
#define SRC_TRAJECTORY_H_

#include "ITrajectory.h"

class Trajectory : public ITrajectory {
 public:
  void set_car_pos(const Eigen::Vector2d& pos) override;
  void set_car_yaw_rad(double yaw_rad) override;
  void set_car_speed(double speed) override;

  void set_previous_path(const std::vector<double>& path_x,
                         const std::vector<double>& path_y) override;

  void Trace(double target_vel,
             std::initializer_list<Eigen::Vector2d> target_poses) override;

  const std::vector<double>& path_x() const override;
  const std::vector<double>& path_y() const override;

 private:
  Eigen::Vector2d car_pos_{};
  double car_yaw_rad_{0};
  double car_speed_{0};
  std::vector<double> previous_path_x_;
  std::vector<double> previous_path_y_;
  std::vector<double> path_x_;
  std::vector<double> path_y_;
};

#endif  // SRC_TRAJECTORY_H_
