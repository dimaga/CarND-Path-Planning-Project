#include "Obstacles.h"
#include <limits>
#include <cassert>

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


bool Obstacles::IsCollided(const std::vector<double>& path_x,
                           const std::vector<double>& path_y) const {
  assert(path_x.size() == path_y.size());


  for (std::size_t i = 0, sz = path_x.size(); i < sz; ++i) {
    const double t = i * ITrajectory::kDtInSeconds;

    using Eigen::Vector2d;
    const Vector2d v { map_.ToFrenet({ path_x.at(i), path_y.at(i) })};

    for (const auto& obstacle : obstacles_) {
      const double forward_speed = obstacle.frenet_vel_[0];

      const Vector2d f { obstacle.frenet_[0] + forward_speed * t,
        obstacle.frenet_[1] };

      if (std::abs(f.x() - v.x()) < kLength &&
          std::abs(f.y() - v.y()) < kWidth) {
        return true;
      }
    }
  }

  return false;
}


double Obstacles::min_distance(const std::vector<double>& path_x,
                               const std::vector<double>& path_y) const {
  assert(path_x.size() == path_y.size());

  double result = 10.0;

  for (std::size_t i = 0, sz = path_x.size(); i < sz; ++i) {
    const double t = (i + 1) * ITrajectory::kDtInSeconds;

    using Eigen::Vector2d;
    const Vector2d v { map_.ToFrenet({ path_x.at(i), path_y.at(i) })};

    for (const auto& obstacle : obstacles_) {
      const double forward_speed = obstacle.frenet_vel_[0];

      const Vector2d f { obstacle.frenet_[0] + forward_speed * t,
                         obstacle.frenet_[1] };

      if (std::abs(v[1] - f[1]) > kWidth) {
        continue;
      }

      if (std::abs(v[0] - f[0]) > 3 * kLength) {
        continue;
      }

      const double dist = (f - v).norm();
      if (dist < result) {
        result = dist;
      }
    }
  }

  return result;
}
