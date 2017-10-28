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
  double min_s = std::numeric_limits<double>::max();

  for (const auto& obstacle : obstacles_) {
    const double d = obstacle.frenet_[1];

    if (IMap::kLaneW * (1 + lane) < d) {
      continue;
    }

    if(d < IMap::kLaneW * lane) {
      continue;
    }

    const double forward_speed = obstacle.frenet_vel_[0];

    const double s = obstacle.frenet_[0];
    const double check_car_s = s + future_time_sec * forward_speed;

    if (check_car_s > future_ego_s && check_car_s - future_ego_s < range) {
      if (check_car_s < min_s) {
        min_s = check_car_s;
        result = &obstacle;
      }
    }
  }

  return result;
}
