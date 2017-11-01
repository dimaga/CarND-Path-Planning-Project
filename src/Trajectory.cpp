#include "Trajectory.h"
#include "spline.h"
#include <cmath>
#include <cassert>
#include <iostream>

void Trajectory::set_car_pos(const Eigen::Vector2d& pos) {
  car_pos_ = pos;
}


void Trajectory::set_car_yaw_rad(double yaw_rad) {
  car_yaw_rad_ = yaw_rad;
}


void Trajectory::set_car_speed_mph(double speed_mph) {
  car_speed_mph_ = speed_mph;
}


double Trajectory::recent_time_sec() const {
  return previous_path_x_.size() * kDtInSeconds;
}


Eigen::Vector2d Trajectory::recent_cartesian() const {
  if (previous_path_x_.empty()) {
    return car_pos_;
  }

  return { previous_path_x_.back(), previous_path_y_.back() };
}


void Trajectory::set_previous_path(const std::vector<double>& path_x,
                                   const std::vector<double>& path_y) {
  assert(path_x.size() == path_y.size());
  previous_path_x_ = path_x;
  previous_path_y_ = path_y;
}


void Trajectory::Trace(double target_speed_mph,
                       std::initializer_list<Eigen::Vector2d> target_poses,
                       std::size_t samples) {
  path_x_.clear();
  path_y_.clear();

  using Eigen::Vector2d;
  Vector2d ref_pos = car_pos_;
  double ref_yaw = car_yaw_rad_;
  double ref_speed = car_speed_mph_ * kMilesPerHour2MetersPerSecond;

  const std::size_t prev_size = previous_path_x_.size();

  if (prev_size >= 2) {
    const Vector2d prev_pos {
      previous_path_x_.at(prev_size - 1),
      previous_path_y_.at(prev_size - 1)
    };

    const Vector2d prev_prev_pos {
      previous_path_x_.at(prev_size - 2),
      previous_path_y_.at(prev_size - 2)
    };

    const Vector2d dpos { prev_pos - prev_prev_pos };
    const double dpos_norm = dpos.norm();
    if (dpos_norm > 1e-5) {
      ref_pos = prev_pos;
      ref_yaw = std::atan2(dpos.y(), dpos.x());
      ref_speed = dpos_norm / kDtInSeconds;

      path_x_.push_back(prev_prev_pos.x());
      path_x_.push_back(prev_pos.x());

      path_y_.push_back(prev_prev_pos.y());
      path_y_.push_back(prev_pos.y());
    }
  }

  if (path_x_.empty()) {
    assert(path_y_.empty());

    path_x_.push_back(car_pos_.x() - std::cos(car_yaw_rad_));
    path_x_.push_back(car_pos_.x());

    path_y_.push_back(car_pos_.y() - std::sin(car_yaw_rad_));
    path_y_.push_back(car_pos_.y());
  }

  for (const Eigen::Vector2d& pos : target_poses) {
    path_x_.push_back(pos.x());
    path_y_.push_back(pos.y());
  }

  // Convert in local reference frame of the car
  const double cos_neg_yaw = std::cos(-ref_yaw);
  const double sin_neg_yaw = std::sin(-ref_yaw);

  for (std::size_t i = 0, sz = path_x_.size(); i < sz; ++i) {
    const double shift_x = path_x_.at(i) - ref_pos.x();
    const double shift_y = path_y_.at(i) - ref_pos.y();

    path_x_.at(i) = shift_x * cos_neg_yaw - shift_y * sin_neg_yaw;
    path_y_.at(i) = shift_x * sin_neg_yaw + shift_y * cos_neg_yaw;
  }

  const double last_spline_x = path_x_.back();

  tk::spline s;
  s.set_points(path_x_, path_y_);

  path_x_ = previous_path_x_;
  path_y_ = previous_path_y_;

  // Resample and convert back to global coordinates
  double x_add_on = 0.0;

  const double cos_yaw = std::cos(ref_yaw);
  const double sin_yaw = std::sin(ref_yaw);

  const double target_speed = target_speed_mph * kMilesPerHour2MetersPerSecond;
  const double avg_speed = 0.5 * (ref_speed + target_speed);
  const double total_time = last_spline_x / avg_speed;
  const double accel = (target_speed - ref_speed) / total_time;

  for (std::size_t prev_i = previous_path_x_.size();
       prev_i < samples;
       ++prev_i) {
    const std::size_t i = prev_i - previous_path_x_.size();
    const double time_elapsed_sec = i * kDtInSeconds;

    const double speed = ref_speed + accel * time_elapsed_sec;
    const double step_x = speed * kDtInSeconds;

    const Vector2d prev_point {x_add_on, s(x_add_on)};
    const Vector2d probe_point {x_add_on + step_x, s(x_add_on + step_x)};

    const double step_length = (probe_point - prev_point).norm();

    double x_point = x_add_on;
    if (step_length > 1e-7) {
      x_point += step_x * step_x / step_length;
    }

    const bool exceeds_final_target_point = x_point > last_spline_x;
    if (exceeds_final_target_point) {
      break;
    }

    const double y_point = s(x_point);

    x_add_on = x_point;

    path_x_.push_back(x_point * cos_yaw - y_point * sin_yaw + ref_pos.x());
    path_y_.push_back(x_point * sin_yaw + y_point * cos_yaw + ref_pos.y());
  }
}


const std::vector<double>& Trajectory::path_x() const {
  return path_x_;
}


const std::vector<double>& Trajectory::path_y() const {
  return path_y_;
}


double Trajectory::avg_speed() const {
  assert(path_x_.size() == path_y_.size());
  double sum = 0.0;
  int count = 0.0;

  const double kDiff2Mph = 1.0 / (kDtInSeconds * kMilesPerHour2MetersPerSecond);

  for (std::size_t i = 1, sz = path_x_.size(); i < sz; ++i) {
    using Eigen::Vector2d;

    const Vector2d p0 { path_x_.at(i - 1), path_y_.at(i - 1) };
    const Vector2d p1 { path_x_.at(i), path_y_.at(i) };

    const double speed_mph = (p1 - p0).norm() * kDiff2Mph;
    sum += speed_mph;
    ++count;
  }

  if (0 == count) {
    return 0;
  }

  return sum / count;
}
