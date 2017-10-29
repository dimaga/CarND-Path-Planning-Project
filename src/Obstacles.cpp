#include "Obstacles.h"
#include <limits>

Obstacles::Obstacles(const IMap& map)
  : map_(map) {
}


void Obstacles::Clear() {
  obstacles_.clear();
}


void Obstacles::Add(const Eigen::Vector2d& cartesian,
                    const Eigen::Vector2d& vel) {
  obstacles_.emplace_back();
  obstacles_.back().frenet_ = map_.ToFrenet(cartesian);
  obstacles_.back().frenet_vel_ = map_.ToFrenetVel(cartesian, vel);
}


const Obstacles::Obstacle* Obstacles::Forward(double future_time_sec,
                                              double future_ego_s,
                                              int lane,
                                              double range) const {
  const Obstacles::Obstacle* result = nullptr;
  double min_dist = std::numeric_limits<double>::max();

  const double left_delim = (lane - 0.4) * IMap::kLaneW;
  const double right_delim = (1.4 + lane) * IMap::kLaneW;
  const double track_length = map_.length();

  future_ego_s = std::fmod(future_ego_s, track_length);
  if (future_ego_s < 0) {
    future_ego_s += track_length;
  }

  for (const auto& obstacle : obstacles_) {
    const double d = obstacle.frenet_[1];

    if (right_delim < d) {
      continue;
    }

    if (d < left_delim) {
      continue;
    }

    const double forward_speed = obstacle.frenet_vel_[0];

    double s = obstacle.frenet_[0] + future_time_sec * forward_speed;
    s = std::fmod(s, track_length);
    while (s < future_ego_s) {
      s += track_length;
    }

    const double dist = s - future_ego_s;
    if (dist < range && dist < min_dist) {
      min_dist = dist;
      result = &obstacle;
    }
  }

  return result;
}
