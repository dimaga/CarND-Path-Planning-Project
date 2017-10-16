#include "Map.h"
#include <cstdio>
#include <sstream>
#include <limits>

Map::Map(std::istream& is) {
  std::string line;

  while (std::getline(is, line)) {
    std::istringstream iss(line);

    Waypoint wp;

    iss >> wp.pos_.x();
    iss >> wp.pos_.y();
    iss >> wp.s_;
    iss >> wp.dir_.x();
    iss >> wp.dir_.y();

    track_.push_back(wp);
  }

  if (!track_.empty()) {
    const double tail = (track_.back().pos_ - track_.front().pos_).norm();
    length_ = track_.back().s_ + tail;
  }
}


double Map::length() const {
  return length_;
}


std::size_t Map::ClosestWaypoint(const Eigen::Vector2d& pos) const {
  auto closest_dist_sq = std::numeric_limits<double>::max();
  std::size_t closest_waypoint = 0;

  for (std::size_t i = 0, size = track_.size(); i < size; ++i) {
    const Waypoint& wp = track_.at(i);

    const double dist_sq = (wp.pos_ - pos).squaredNorm();
    if (dist_sq < closest_dist_sq) {
      closest_dist_sq = dist_sq;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}


std::size_t Map::NextWaypoint(const Eigen::Vector2d& pos) const {
  auto idx = ClosestWaypoint(pos);

  const Waypoint& wp = track_.at(idx);
  const Eigen::Vector2d to_wp = pos - wp.pos_;

  if (to_wp.dot(wp.dir_) > 0) {
    idx = (idx + 1) % track_.size();
  }

  return idx;
}
