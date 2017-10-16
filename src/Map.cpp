#include "Map.h"
#include <cstdio>
#include <sstream>

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
