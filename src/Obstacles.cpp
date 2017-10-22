#include "Obstacles.h"

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


const Obstacles::Obstacle* Obstacles::Forward(double s,
                                              int lane,
                                              double range) const {
  return nullptr;
}
