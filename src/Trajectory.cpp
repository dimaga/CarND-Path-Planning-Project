#include "Trajectory.h"
#include "spline.h"
#include <cmath>
#include <cassert>

void Trajectory::set_car_pos(const Eigen::Vector2d& pos) {
  car_pos_ = pos;
}


void Trajectory::set_car_yaw_rad(double yaw_rad) {
  car_yaw_rad_ = yaw_rad;
}


void Trajectory::set_car_speed(double speed) {
  car_speed_ = speed;
}


void Trajectory::set_previous_path(const std::vector<double>& path_x,
                                   const std::vector<double>& path_y) {
  assert(path_x.size() == path_y.size());
  previous_path_x_ = path_x;
  previous_path_y_ = path_y;
}


void Trajectory::Trace(double target_vel,
                       std::initializer_list<Eigen::Vector2d> target_poses) {
  path_x_.clear();
  path_y_.clear();

  double ref_x = car_pos_.x();
  double ref_y = car_pos_.y();
  double ref_yaw = car_yaw_rad_;

  const std::size_t prev_size = previous_path_x_.size();

  if (prev_size < 2) {
    const double prev_car_x = car_pos_.x() - std::cos(car_yaw_rad_);
    const double prev_car_y = car_pos_.y() - std::sin(car_yaw_rad_);

    path_x_.push_back(prev_car_x);
    path_x_.push_back(car_pos_.x());

    path_y_.push_back(prev_car_y);
    path_y_.push_back(car_pos_.y());
  } else {
    ref_x = previous_path_x_.at(prev_size - 1);
    ref_y = previous_path_y_.at(prev_size - 1);

    const double ref_x_prev = previous_path_x_.at(prev_size - 2);
    const double ref_y_prev = previous_path_y_.at(prev_size - 2);
    ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    path_x_.push_back(ref_x_prev);
    path_x_.push_back(ref_x);

    path_y_.push_back(ref_y_prev);
    path_y_.push_back(ref_y);
  }

  for (const Eigen::Vector2d& pos : target_poses) {
    path_x_.push_back(pos.x());
    path_y_.push_back(pos.y());
  }

  const double cos_neg_yaw = std::cos(-ref_yaw);
  const double sin_neg_yaw = std::sin(-ref_yaw);

  // Convert in local reference frame of the car
  for (std::size_t i = 0, sz = path_x_.size(); i < sz; ++i) {
    const double shift_x = path_x_.at(i) - ref_x;
    const double shift_y = path_y_.at(i) - ref_y;

    path_x_.at(i) = shift_x * cos_neg_yaw - shift_y * sin_neg_yaw;
    path_y_.at(i) = shift_x * sin_neg_yaw + shift_y * cos_neg_yaw;
  }

  tk::spline s;
  s.set_points(path_x_, path_y_);

  path_x_ = previous_path_x_;
  path_y_ = previous_path_y_;

  const double target_x = 30.0;
  const double target_y = s(target_x);
  const double target_dist = Eigen::Vector2d{target_x, target_y}.norm();

  double x_add_on = 0;

  const double cos_yaw = std::cos(ref_yaw);
  const double sin_yaw = std::sin(ref_yaw);

  for (std::size_t i = previous_path_x_.size(); i < 50; ++i) {
    const double N = target_dist / (.02 * target_vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    const double x_ref = x_point;
    const double y_ref = y_point;

    x_point = x_ref * cos_yaw - y_ref * sin_yaw;
    y_point = x_ref * sin_yaw + y_ref * cos_yaw;

    x_point += ref_x;
    y_point += ref_y;

    path_x_.push_back(x_point);
    path_y_.push_back(y_point);
  }
}


const std::vector<double>& Trajectory::path_x() const {
  return path_x_;
}


const std::vector<double>& Trajectory::path_y() const {
  return path_y_;
}
