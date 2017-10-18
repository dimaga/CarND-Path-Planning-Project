#include "Map.h"
#include <cstdio>
#include <sstream>
#include <limits>
#include <algorithm>
#include <cassert>

Map::Map(std::istream& is) {
  std::string line;

  while (std::getline(is, line)) {
    std::istringstream iss(line);

    Waypoint wp;

    iss >> wp.pos_.x();
    iss >> wp.pos_.y();
    iss >> wp.s_;
    iss >> wp.normal_.x();
    iss >> wp.normal_.y();

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
  const auto idx = ClosestWaypoint(pos);
  const auto next_idx = (idx + 1) % track_.size();

  const Waypoint& wp = track_.at(idx);
  const Waypoint& next_wp = track_.at(next_idx);

  const Eigen::Vector2d to_pos = pos - wp.pos_;
  const Eigen::Vector2d to_next_wp = next_wp.pos_ - wp.pos_;

  const double dot = to_pos.dot(to_next_wp);
  if (dot > 0) {
    return next_idx;
  } else {
    return idx;
  }
}


Eigen::Vector2d Map::ToFrenet(const Eigen::Vector2d& cartesian) const {
  using std::size_t;
  const size_t next_idx = NextWaypoint(cartesian);
  const size_t prev_idx = next_idx ? (next_idx - 1) : (track_.size() - 1);

  const auto& next_wp = track_.at(next_idx);
  const auto& prev_wp = track_.at(prev_idx);

  using Eigen::Vector2d;
  const Vector2d to_next = next_wp.pos_ - prev_wp.pos_;
  const Vector2d to_pos = cartesian - prev_wp.pos_;

  // find the projection of x onto n
  const Vector2d proj = to_next * to_pos.dot(to_next) / to_next.squaredNorm();

  const double frenet_s = prev_wp.s_ + proj.norm();
  const double frenet_d = to_pos.dot(prev_wp.normal_);
  return {frenet_s, frenet_d};
}


Eigen::Vector2d Map::ToCartesian(const Eigen::Vector2d& frenet) const {
  assert(!track_.empty());

  double s = frenet[0];
  s -= length_ * static_cast<int>(s / length_);
  if (s < 0) {
    s += length_;
  }

  auto next_it = std::upper_bound(track_.cbegin(),
                                  track_.cend(),
                                  s,
                                  [](double s, const Waypoint& wp) {
                                    return s < wp.s_;
                                  });

  const auto it = (track_.cbegin() == next_it ? track_.cend() : next_it) - 1;
  if (track_.cend() == next_it) {
    next_it = track_.begin();
  }

  using Eigen::Vector2d;
  const Vector2d dir_to_next = (next_it->pos_ - it->pos_).normalized();

  const double seg_s = (s - it->s_);
  const double seg_x = it->pos_.x() + seg_s * dir_to_next.x();
  const double seg_y = it->pos_.y() + seg_s * dir_to_next.y();

  const double x = seg_x + frenet[1] * dir_to_next.y();
  const double y = seg_y - frenet[1] * dir_to_next.x();
  return {x, y};
}
