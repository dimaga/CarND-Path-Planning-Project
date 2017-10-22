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


void Trajectory::set_previous_path(const std::vector<double>& path_x,
                                   const std::vector<double>& path_y) {
  assert(path_x.size() == path_y.size());
  previous_path_x_ = path_x;
  previous_path_y_ = path_y;
}


void Trajectory::Trace(double target_speed_mph,
                       std::initializer_list<Eigen::Vector2d> target_poses) {
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

  double x_add_on = 0.0;

  const double cos_yaw = std::cos(ref_yaw);
  const double sin_yaw = std::sin(ref_yaw);

  const double target_speed = target_speed_mph * kMilesPerHour2MetersPerSecond;
  const double avg_speed = 0.5 * (ref_speed + target_speed);
  const double total_time = last_spline_x / avg_speed;
  const double accel = (target_speed - ref_speed) / total_time;

  // Resample and convert back to global coordinates

  for (std::size_t prev_i = previous_path_x_.size(); prev_i < 50; ++prev_i) {
    const std::size_t i = prev_i - previous_path_x_.size();
    const double time_elapsed_sec = i * kDtInSeconds;

    const double speed = ref_speed + accel * time_elapsed_sec;
    const double step = speed * kDtInSeconds;

    const double prev_x_point = x_add_on;
    const double prev_y_point = s(prev_x_point);

    const double probe_x_point = x_add_on + step;
    const double probe_y_point = s(probe_x_point);
    const double probe_step = std::sqrt((probe_x_point - prev_x_point)*(probe_x_point - prev_x_point)
                                        + (probe_y_point - prev_y_point)*(probe_y_point - prev_y_point));

    double x_point = x_add_on;
    if (probe_step > 1e-7) {
      x_point += step * step / probe_step;
    }

    if (x_point > last_spline_x) {
      break;
    }

    double y_point = s(x_point);

    x_add_on = x_point;

    const double x_ref = x_point;
    const double y_ref = y_point;

    x_point = x_ref * cos_yaw - y_ref * sin_yaw;
    y_point = x_ref * sin_yaw + y_ref * cos_yaw;

    x_point += ref_pos.x();
    y_point += ref_pos.y();

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
